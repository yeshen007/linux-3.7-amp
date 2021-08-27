/*
 *
 *  Copyright (C) 2013-2014 Embest Technology co.,ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/completion.h>
#include <linux/dcache.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fcntl.h>
#include <linux/kref.h>
#include <linux/kthread.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/rwsem.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/freezer.h>
#include <linux/utsname.h>
#include <linux/delay.h>
#include <linux/smp.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <asm/mach/map.h>
#include <asm/amp_config.h>

#define DRIVER_NAME		"amp"
#define DRIVER_VERSION		"1 August 2013"


MODULE_DESCRIPTION("amp");
MODULE_AUTHOR("embest co.,ltd.yejc");
MODULE_LICENSE("Dual BSD/GPL");

static u32 terimate=0;
static u32 ipcflag;
static struct task_struct       *thread_handle[2];
static u32 *pbuf0 = (u32 *)AMP_SHARE_BUFFER_START;
//static u32 *pbuf1 = (u32 *)(AMP_SHARE_BUFFER_START + DRAM_BUF_SIZE);
static u32 *psram = (u32 *)AMP_SHARE_SRAM_STRART;

static DECLARE_COMPLETION(kthread_amp_done);
static DEFINE_MUTEX(amp_mutex);
static unsigned long useflags;
static unsigned long threadrun_flags;
static unsigned long preemptdis_flag = 0;

/*
***************************************************************************
*                       yeshen fix begin
*                       15895833516
***************************************************************************
*
*/

#define YESHEN_TEST

#define CPU0	(0x1)
#define CPU1	(0x2)
#define  GIC_SGI0						   (u16)( 0u)
#define  GIC_SGI1						   (u16)( 1u)
#define  GIC_SGI2						   (u16)( 2u)
#define  GIC_SGI3						   (u16)( 3u)
#define  GIC_SGI4						   (u16)( 4u)
#define  GIC_SGI5						   (u16)( 5u)
#define  GIC_SGI6						   (u16)( 6u)
#define  GIC_SGI7						   (u16)( 7u)
#define  GIC_SGI8						   (u16)( 8u)
#define  GIC_SGI9						   (u16)( 9u)
#define  GIC_SGI10						   (u16)( 10u)
#define  GIC_SGI11						   (u16)( 11u)
#define  GIC_SGI12						   (u16)( 12u)
#define  GIC_SGI13						   (u16)( 13u)
#define  GIC_SGI14						   (u16)( 14u)
#define  GIC_SGI15						   (u16)( 15u)



#define  GIC_INT_DIST_BASE         (u32)0xFFFED000

#define  GIC_INT_REG_ICDSGIR             (*(volatile u32 *)(GIC_INT_DIST_BASE + 0xF00))

static volatile int sgi13task_pending = 0;

static inline void gic_raise_interrupt(u32 target_cpu, u32 sgi)
{
	asm __volatile__("dsb" ::: "memory");
	GIC_INT_REG_ICDSGIR = (target_cpu << 16) | sgi;
}

static void delay_yeshen(void)
{
	volatile u32 i = 0xF0000;
	while(i>0) i--;

}


/*
***************************************************************************
*                       yeshen fix end
*                       15895833516
***************************************************************************
*
*/





static u32 walk_virt_to_phys(u32 addr)
{
        pmd_t *pmd;
        pte_t *ptep;

        pgd_t *pgd = pgd_offset(current->active_mm, addr);
        if (pgd_none(*pgd) || pgd_bad(*pgd))
                return 0;

        pmd = pmd_offset((pud_t *)pgd, addr);
        if (pmd_none(*pmd)) //allow section map pass yejc
                return 0;
	if(!pmd_bad(*pmd)) //page map
	{
        	ptep = pte_offset_map(pmd, addr);	//linux version pte
        	if (ptep && pte_present(*ptep))
		{
			ptep += 512;
                	return (PAGE_MASK & *ptep) | (~PAGE_MASK & addr);//hw version pte yejc
		}
	}
        return (pgd_val(*pgd)&SECTION_MASK) | (~SECTION_MASK & addr); //is section map yejc
}


/*
***************************************************************************
*                       Embest Tech co., ltd
*                        www.embest-tech.com
***************************************************************************
*
*two CPUs concurrenctly access the same memory adrress
*/
/*  在第一次amp_test_thread前linux_cmd_args应该为0.(未看到初始化,所以我在amp_test_store中初始化)
 *  之后的linux_cmd_args为前面的amp_test_thread留下的值
*/
static int amp_test_thread(void *arg)
{
	int cpu;
	int i, tmp;
	int sh_buf_test_cnt;
	enum test_sequence ts;

	/* 测试dram */
	/* 如果preemptdis_flag置1，则测试dram双核通信时关闭抢占 */
	if(preemptdis_flag)
		preempt_disable();
	
	sh_buf_test_cnt = 0;	//记录进行了多少次循环
	ts = eSH_DRAM;
	cpu = 1;
	printk(KERN_INFO"amp test begin\n");
	//ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args) = eSH_DRAM;
	
	asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args = eSH_DRAM;		//通知bm测试dram
#ifdef  YESHEN_TEST
	printk("yeshen amp test begin\n");
	sgi13task_pending = 1;
	while (1) { 
		if (sgi13task_pending) {
			delay_yeshen();
			sgi13task_pending = 0;
			gic_raise_interrupt(CPU1,GIC_SGI15);
		}

	}
#endif
	/* 通信第一轮开始前linux并没有验证数据，直接拷贝数据进共享内存然后发送sgi15给cpu1
     * 然后linux_cmd_status变为PENDING，进入空循环直到bm处理完发送sgi13，在sgi13handler中再次COMPLETE
	*/
	while(ts == eSH_DRAM) {
		if(ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_status) == COMPLETE) {	//如果sgi13handler验证数据通过
			//for(i = 0; i < (DRAM_BUF_SIZE/4); i++)
			//	*(pbuf0 + i) = CPU_DATA_PATERN0;
			memset_int(pbuf0, CPU_DATA_PATERN0, DRAM_BUF_SIZE);				//steps1 将64kb的CPU_DATA_PATERN0写入共享内存DRAM_BUF中
			ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_status) = PENDING;		//只有bm验证完毕后触发SGI13中断给linux后，linux才将PENDING变为COMPLETE
			sh_buf_test_cnt++;							//只有bm验证完毕后触发SGI13中断给linux后，linux才加1
			//request bm check buf
#ifdef YESHEN_TEST
			gic_raise_interrupt(CPU1, GIC_SGI15);
			printk("yehsne linux raise sgi15 1\n");
#else
			gic_raise_softirq(cpumask_of(cpu), SGI_LINUX_REQ_BM_CONSUME_BUF);			//steps2 触发中断SGI15给cpu1(bm)让cpu1开始验证共享内存数据
#endif
		}
		if(sh_buf_test_cnt >=  CORE_SHARE_BUFFER_TEST_COUNT)		//steps 5 当次数达到40000次后才跳出进入下一步
			break;
	}

	sh_buf_test_cnt = 0;		//计数从新归零
	
	/* 使能抢占 */
	if(preemptdis_flag)
    	preempt_enable();
	//give some time to last gic issue and process buffer data before move next test case;
	printk("\nDRAM share test finished(test DRAM share access and cache coherence)\n");
	msleep(10);

	/* 测试sram */
	/* 如果preemptdis_flag置1，则测试sram双核通信时关闭抢占 */
	if(preemptdis_flag)
    	preempt_disable();
	//ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args) = eSH_SRAM;
	asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args = eSH_SRAM;	//通知bm测试sram
	ts++;	//ts等于eSH_SRAM
	//ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args) = eSH_SRAM;
	while(ts == eSH_SRAM){
        if(ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_status) == COMPLETE){
            //for(i = 0; i < (DRAM_BUF_SIZE/4); i++)
            //      *(pbuf0 + i) = CPU_DATA_PATERN0;
            memset_int(psram, CPU_DATA_PATERN0, SRAM_BUF_SIZE);
            ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_status) = PENDING;
            sh_buf_test_cnt++;
            //request bm check buf
#ifdef YESHEN_TEST
			gic_raise_interrupt(CPU1, GIC_SGI15);
			printk("yehsne linux raise sgi15 2\n");
#else
            gic_raise_softirq(cpumask_of(cpu), SGI_LINUX_REQ_BM_CONSUME_BUF);
#endif
        }
        if(sh_buf_test_cnt >=  CORE_SHARE_BUFFER_TEST_COUNT)		//sram也是40000次
            break;
	}

	/* 使能抢占 */
	if(preemptdis_flag)
    	preempt_enable();
	printk("SRAM share test finished(test SRAM share access and cache coherence)\n");
    msleep(10);

	/* 测试spinlock */
	/* 如果preemptdis_flag置1，则测试spinlock双核通信时关闭抢占 */
	if(preemptdis_flag)
    	preempt_disable();
	ts++;
	asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args = eSPINLOCK;	//通知bm测试spinlock
	asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].bm_cmd_status = 0;	
	//isse gic notify BM do spinlock test
	for(i = 0; i < SPINLOCK_TEST_COUNT; i++) {
		raw_spin_lock(&asp->rslocks[0]);
		tmp = asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].bm_cmd_status;
		//dummy sh_buf_test_cnt++
		sh_buf_test_cnt++;
		asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].bm_cmd_status += 2;
		if((asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].bm_cmd_status - tmp) != 2)
			printk("Linux:spinlock test failed!\n");
		raw_spin_unlock(&asp->rslocks[0]);
		//dummy operation on sh_buf_test_cnt++ simulate the actual scenario to give another cpu chance
		//to take lock, reduce starvation situation
		sh_buf_test_cnt++;
	}
	printk("Linux spinlock test:%d\n", asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].bm_cmd_status);
	if(preemptdis_flag)
                preempt_enable();

	/* 测试dma */
	printk("waiting BM finish interrupt latency test and DMA test etc.\n");
	//preempt_disable();
	while(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args != -1)
	{
		//dummy, just put some interrupt load to bm
		//cpu = 1;
        	//gic_raise_softirq(cpumask_of(cpu), SGI_BRING_OTHER2CACHECOHERENCE);//yejc
		usleep_range(200, 210);
	}
	//preempt_enable();
	printk("BM finish BM finish interrupt latency test and DMA test\n");
	
	printk("use ""cat /sys/class/amp/amp/bm_log to check bmlog""\n");
	clear_bit(0, &threadrun_flags);	
	
	return 0;
}
static int thread_shced_test(void *arg)
{
        printk(KERN_INFO "++thread_shced_test\n");
        while(1) {
			//indicate linux activity
			*(u32 *)SOCFGA_GPIO1_VIRT_BASE ^= 0x8<<12;
            msleep(20);
        }
        printk(KERN_INFO "--thread_shced_test\n");
        return 0;
}


static irqreturn_t sgi13handler(int irq, void *dev_id)
{
#ifdef  YESHEN_TEST
		printk("get sgi13\n");
		sgi13task_pending = 1;
		return IRQ_HANDLED;
#endif

    int i;
	int size = 0;
	u32 *p = pbuf0;

	//TODO: u should move the check routine to BH in actual project
	//if(ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args) == eSH_DRAM)
	if(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args == eSH_DRAM)
	{
		p = pbuf0;
		size = DRAM_BUF_SIZE>>2;
	}
	else  if(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args == eSH_SRAM)
	//else  if(ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args) == eSH_SRAM)
	{
		p = psram;
		size = SRAM_BUF_SIZE>>2;
	}
	for(i = 0; i < size; i++)									//验证bm端写入共享内存的数据是否为CPU_DATA_PATERN3
		if(*(p + i) != CPU_DATA_PATERN3)
			printk("check buf from bm failed! addr=%08x, *addr=%08x~~~~\n", (u32)(p + i), *(p + i));
	
	//printk("check buf from bm finish!~~~~~\n");
	ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_status) = COMPLETE;		//收到bm的中断请求后将linux_cmd_status置为COMPLETE
        return IRQ_HANDLED;
}

static int amp_open(struct inode *inode, struct file *filp)
{
        int result = 0;

        mutex_lock(&amp_mutex);
        if (test_and_set_bit(0, &useflags)) {
                result = -EBUSY;
                /* this legacy device is always one per system and it doesn't
                 * know how to handle multiple concurrent clients.
                 */
                goto out;
        }
	/*
        result = request_irq(amp_interrupt, &amp_interrupt,
                             IRQF_DISABLED, "SGI", amp_interrupt);
        if (result == -EBUSY)
                printk(KERN_ERR "amp: Interrupt can't be reserved.\n");
	*/
out:
        mutex_unlock(&amp_mutex);
        return result;
}

static long amp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return 0;
}

static int amp_mmap(struct file *file, struct vm_area_struct *vma)
{
	size_t nm_size = vma->vm_end - vma->vm_start;
	size_t dma_size = 0;
	pgprot_t dma_page_prot;

	if(nm_size > (AMPMAP_SIZE - BM_CONTEXT_SIZE))
		return -EPERM;

	if(nm_size > (AMPMAP_SIZE - DMABUF_SIZE - BM_CONTEXT_SIZE)){ 
		//for security reason, don't expose bm ctx to user space.
		dma_size = nm_size - (AMPMAP_SIZE - DMABUF_SIZE - BM_CONTEXT_SIZE);
		nm_size = AMPMAP_SIZE - DMABUF_SIZE - BM_CONTEXT_SIZE;
	}

	/* Remap-pfn-range will mark the range VM_IO */
	if (remap_pfn_range(vma,
			    vma->vm_start,
			    (AMPPHY_START + BM_CONTEXT_SIZE)>>PAGE_SHIFT,
			    nm_size,
			    vma->vm_page_prot)) {
		return -EAGAIN;
	}
	if(dma_size > 0){
		dma_page_prot = pgprot_writecombine(vma->vm_page_prot);
		 if (remap_pfn_range(vma,
                            vma->vm_start + nm_size,
                            (AMPPHY_START + AMPMAP_SIZE - DMABUF_SIZE)>>PAGE_SHIFT,
                            dma_size,
                            dma_page_prot)) {
                	return -EAGAIN;
        	}
	}
	return 0;
}

static int amp_release(struct inode *inode, struct file *filp)
{
        //free_irq(amp_interrupt, amp_interrupt);
	clear_bit(0, &useflags);	

        return 0;
}

static const struct file_operations amp_fops = {
       	.mmap = amp_mmap,
		.open = amp_open,
		.unlocked_ioctl = amp_ioctl,
        .release = amp_release,
};

static struct class *amp_class;
int amp_major = 0;

/* 用户向属性amp_test写数据时触发，
 * 每次写相应的值给属性amp_test就会创建amp_test_thread线程
*/
static ssize_t amp_test_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int i;
	
	/* 看用户写入的数据前4位是否为test
     * 例如echo test > /sys/class/amp/amp/amp_test，
     * echo test_wpd > /sys/class/amp/amp/amp_test
	*/
	if(strncmp(buf, "test", 4) == 0) {
		/* 如果threadrun_flags已经置1说明amp_test_thread已经被创建，则返回-EBUSY，
		 * 如果没有置1，且写入amp_test属性的字符串前四位为test则置1
		*/
		if(test_and_set_bit(0, &threadrun_flags))	
			return -EBUSY;

		/* 如果写入test_wpd则表示关闭抢占 */
		if((size > 7) && (strncmp(buf, "test_wpd", 8) == 0))  //echo test_wpd > /sys/class/amp/amp/amp_test
			preemptdis_flag = 1;		//test with preempt disable
		else
			preemptdis_flag = 0;		//test with preempt enable

		/* 创建内核线程amp_test_thread，将task_struct保存到thread_handle[0]中，
		 * 因为thread_handle[1]用来保存thread_shced_test
		 * 尝试5次创建直到创建成功，创建成功后就唤醒调度amp_test_thread
		*/
		for(i = 0; i < 5; i++) {
			thread_handle[0] = kthread_create(amp_test_thread, NULL, "amp_test:0:0");		//创建线程amp_test_thread开始测试双核共享内存
			if (IS_ERR(thread_handle[0])) {
				msleep(30);
				continue;
        	}else{	//创建线程成功
				/* 应该要初始化linux_cmd_status */
				ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_status) = COMPLETE;
				wake_up_process(thread_handle[0]);	//调度amp_test_thread
				break;
			}
		}
		
		if(i >= 5) {
			size = PTR_ERR(thread_handle[0]);
			printk("create amp_test kthread failed 0x%08x\n", size);
			clear_bit(0, &threadrun_flags);	//线程创建不成功则将threadrun_flags清零
                        return size;
		}
	}

        return size;
}

/*  */
static ssize_t sgi_trigger_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int sgi_nb = -1;
	int cpu = 1;
	char *endp;

	endp = (char *)buf + size;
	
	sgi_nb = simple_strtol(buf, &endp, 10);
	if((sgi_nb >= 0) && (sgi_nb < 8))
	{
		gic_raise_softirq(cpumask_of(cpu), sgi_nb);
	}
	else
		printk("invalid sgi number\n");

	return size;
}

/* 将logbuf中的内容拷贝到用户buf */
static ssize_t
bm_log_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/*
	 * 如果logbuf 4kb益出,则先将未覆盖的部分拷贝到buf，再将覆盖的部分拷贝到buf
	 * 如果没有溢出,	则将logbuf中的内容拷贝到buf
	*/
	if(asp->logbuf_overlap){  
		printk("overlap:logindex %d\n", asp->logindex);
		memcpy(buf, (void *)((char *)asp->logbuf + asp->logindex), LOG_LEN - asp->logindex);
		memcpy(buf + LOG_LEN - asp->logindex, asp->logbuf, asp->logindex);
		return LOG_LEN - 1; //limit it little than 1 page
	}else{
		printk("logindex = %d\n", asp->logindex);
		memcpy(buf, asp->logbuf, asp->logindex);	//将asp->logbuf中的内容读到用户缓冲区
		return asp->logindex;
	}
}

/* 三个属性 */
static struct device_attribute amp_dev_attrs[] = {		
	__ATTR(amp_test, 0666, NULL, amp_test_store),		/* 写amp_test触发amp_test_store */
	__ATTR(sgi_trigger, 0600, NULL, sgi_trigger_store),	/* 写sgi_trigger触发sgi_trigger_store */
	__ATTR(bm_log, 0666, bm_log_show, NULL),			/* 读bm_log触发bm_log_show */
	__ATTR_NULL
};
	
/*
***************************************************************************
*                       Embest Tech co., ltd
*                        www.embest-tech.com
***************************************************************************
*
*request ipi irq, launch thread
*/
static int __init amp_init(void)
{
	int rc=0;

	walk_virt_to_phys(AMP_SHARE_DMABUF_START);

	/* step 1 注册字符设备amp进内核,返回主设备号rc */
	rc = register_chrdev(amp_major, "amp", &amp_fops);	
	if(rc < 0)
	{
        	printk(KERN_INFO"failed to register amp dev\n");
			return rc;
	}
	amp_major = rc;

	/* step 2 创建一个类，在/sys/class/amp */
    amp_class = class_create(THIS_MODULE, "amp");		
    if (IS_ERR(amp_class))
            return PTR_ERR(amp_class);
	
	/* step 2.1 创建amp类的属性和绑定属性读写函数              */
	amp_class->dev_attrs = amp_dev_attrs;

	/* step 2.2 在类下创建设备，即注册到sys文件系统/sys/class/amp/amp */
	if(!device_create(amp_class, NULL, MKDEV(amp_major, 0), NULL, "amp"))	
		printk(KERN_INFO"amp:device_create failed!\n");

	/* step 3 注册SGI13中断处理函数，虚拟中断号为512+13 */
	rc = request_irq(512 + SGI_BM_REQ_LINUX_CONSUME_BUF, sgi13handler, IRQF_SHARED, "SGI", &ipcflag);	
	if(rc)
	{
		printk(KERN_INFO"%s:request irq failed!\r\n", __FUNCTION__);
		return rc;
	}
	/*
	thread_handle[0] = kthread_create(amp_test_thread, NULL, "amp_test:0:0");
	//thread_task = kthread_create_on_cpu(ipi_demo_thread, &gIpiCnt, 0 , "amp_test:1:0");
        if (IS_ERR(thread_handle[0])) {
                rc = PTR_ERR(thread_handle[0]);
		return rc;
        }*/
        
    /* step 4 创建内核线程thread_shced_test,这个线程只是一个死循环点灯(但点的不是nano的灯,因此这是无用线程)；   
     * 当用户读写/sys/class/amp/amp/下的属性文件时就会调用属性的读写函数；
     * 例如，echo test > /sys/class/amp/amp/amp_test就会调用
     * amp_dev_attrs中的amp_test_store等函数来进行测试
    */
	thread_handle[1] = kthread_create(thread_shced_test, NULL, "sched_test:0:0");																					表示linux在活动
																					  
																					  
    if (IS_ERR(thread_handle[1])) {
            rc = PTR_ERR(thread_handle[1]);
            return rc;
    }

	//wake_up_process(thread_handle[0]);

	/* step 5 调度内核线程thread_shced_test */
	wake_up_process(thread_handle[1]);
	//kthread_unpark(thread_task);

	printk(KERN_INFO "--amp_init\n");
	return rc;
}
module_init(amp_init);


static void __exit amp_cleanup(void)
{
	terimate = 1;
}
module_exit(amp_cleanup);
