/*
* adxl345 i2c driver 
* copyright (C) leaf god 
*/


#include <linux/init.h>
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/mach/map.h>
#include <linux/err.h>
#include <linux/of.h>

 
#define ADXL345_I2C_ADDR 0x53 

#define AXS_X_REG 0x1e
#define AXS_Y_REG 0x1f
#define AXS_Z_REG 0x20

#define BLOCK 0
#define NONBLOCK 1

#define I2C_MSG_READ I2C_M_RD
#define I2C_MSG_WRITE 0

#define XYZ_ACTIVATE 0x70

struct adxl345 {
	struct i2c_client *i2c_client;
	int ofsx;
	int ofsy;
	int ofsz;
	int dev_addr;
};

struct adxl345 adxl345;

static int adxl345_i2c_read(int client_addr, char *reg, char *buf)
{
	int ret;
	struct i2c_msg msgs[6];

	msgs[0].addr = client_addr;
	msgs[0].flags = I2C_MSG_WRITE;
	msgs[0].len = 1;
	msgs[0].buf = &reg[0];

	msgs[1].addr = client_addr;
	msgs[1].flags = I2C_MSG_READ;
	msgs[1].len = 1;
	msgs[1].buf = &buf[0];

	msgs[2].addr = client_addr;
	msgs[2].flags = I2C_MSG_WRITE;
	msgs[2].len = 1;
	msgs[2].buf = &reg[1];

	msgs[3].addr = client_addr;
	msgs[3].flags = I2C_MSG_READ;
	msgs[3].len = 1;
	msgs[3].buf = &buf[1];
	
	msgs[4].addr = client_addr;
	msgs[4].flags = I2C_MSG_WRITE;
	msgs[4].len = 1;
	msgs[4].buf = &reg[2];

	msgs[5].addr = client_addr;
	msgs[5].flags = I2C_MSG_READ;
	msgs[5].len = 1;
	msgs[5].buf = &buf[2];

	
	ret = i2c_transfer(adxl345.i2c_client->adapter, msgs, 6);
	if (ret != 6) {
		printk(KERN_ERR "%s:read error\n", __FUNCTION__);	
		return -EIO;
	}

	return 0;
	
}

static int adxl345_i2c_write(int client_addr, char *reg, char *buf)
{
	int ret;
	struct i2c_msg msgs[6];


	msgs[0].addr = client_addr;
	msgs[0].flags = I2C_MSG_WRITE;
	msgs[0].len = 1;
	msgs[0].buf = &reg[0];

	msgs[1].addr = client_addr;
	msgs[1].flags = I2C_MSG_WRITE;
	msgs[1].len = 1;
	msgs[1].buf = &buf[0];

	msgs[2].addr = client_addr;
	msgs[2].flags = I2C_MSG_WRITE;
	msgs[2].len = 1;
	msgs[2].buf = &reg[1];

	msgs[3].addr = client_addr;
	msgs[3].flags = I2C_MSG_WRITE;
	msgs[3].len = 1;
	msgs[3].buf = &buf[1];
	
	msgs[4].addr = client_addr;
	msgs[4].flags = I2C_MSG_WRITE;
	msgs[4].len = 1;
	msgs[4].buf = &reg[2];

	msgs[5].addr = client_addr;
	msgs[5].flags = I2C_MSG_WRITE;
	msgs[5].len = 1;
	msgs[5].buf = &buf[2];


	ret = i2c_transfer(adxl345.i2c_client->adapter, msgs, 6);
	if (ret != 6) {
		printk(KERN_ERR "%s:write error\n", __FUNCTION__);	
		return -EIO;
	}
	
	return 0;
}



static long adxl345_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
		case BLOCK:
			filp->f_flags &= ~O_NONBLOCK;
			break;
		case NONBLOCK:
			filp->f_flags |= O_NONBLOCK;
			break;
		default:
			filp->f_flags |= O_NONBLOCK;
			return -ENOTTY;
	}

	return 0;
}


static int adxl345_open(struct inode *inode, struct file *filp)
{
	filp->private_data = (void *)&adxl345;
	filp->f_flags |= O_NONBLOCK;  //default nonblock
	return 0;
}


/* one x one y one z one time  */
static ssize_t adxl345_read(struct file *filp, char __user *buf,
							size_t size, loff_t *loff)
{
	char axs_buf[3];
	char reg[3];
	int i2c_dev_addr;
	int ret;
	
	reg[0] = adxl345.ofsx;
	reg[1] = adxl345.ofsy;
	reg[2] = adxl345.ofsz;
	i2c_dev_addr = adxl345.dev_addr;

	if (size != 3) {
		printk(KERN_ERR "%s:wrong read size\n", __FUNCTION__);
		return -EIO;
	}
		

	if (filp->f_flags & O_NONBLOCK) {  	/* non-blocked, default situation */
		ret = adxl345_i2c_read(i2c_dev_addr, reg, axs_buf);
		if (ret != 0) {
			printk(KERN_ERR "%s:i2c read error\n", __FUNCTION__);
			return -EIO;
		}
		ret = copy_to_user(buf, axs_buf, size);
		if (ret != 0) {
			printk(KERN_ERR "%s:copy to user error\n", __FUNCTION__);
			return -EIO;
		}
	} else {  							/* blocked */
		/* do it later */
		printk(KERN_CRIT "blocked read has not been realized\n");
		while (1) ;
	}

	return ret;
	
}



/* one x one y one z one time  */
static ssize_t adxl345_write(struct file *filp, const char __user *buf, 
									size_t size, loff_t *loff)
{
	char axs_buf[3];
	char reg[3];
	int i2c_dev_addr;
	int ret;
	
	reg[0] = adxl345.ofsx;
	reg[1] = adxl345.ofsy;
	reg[2] = adxl345.ofsz;
	i2c_dev_addr = adxl345.dev_addr;

	if (size != 3) {
		printk(KERN_ERR "%s:wrong write size\n", __FUNCTION__);
		return -EIO;
	}	

	if (filp->f_flags & O_NONBLOCK) {  	/* non-blocked, default situation */

		ret = copy_from_user(axs_buf, buf, size);
		if (ret != 0) {
			printk(KERN_ERR "%s:copy from user error\n", __FUNCTION__);
			return -EIO;
		}	
		ret = adxl345_i2c_write(i2c_dev_addr, reg, axs_buf);
		if (ret != 0) {
			printk(KERN_ERR "%s:i2c write error\n", __FUNCTION__);
			return -EIO;
		}	
	} else {  							/* blocked */

		/* do it later */
		printk(KERN_CRIT "blocked write has not been realized\n");
		while (1) ;
	}
	
	return ret;
}

static int adxl345_release(struct inode *inode, struct file *file)
{
	/* nothing to do */
	return 0;	
}



static struct file_operations adxl345_fops = {
	.owner   = THIS_MODULE,
	.open    = adxl345_open,
	.read    = adxl345_read,
	.write   = adxl345_write,
	.release = adxl345_release,
};


static struct miscdevice adxl345_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "adxl345_misc",
	.fops  = &adxl345_fops,
};


static int adxl345_probe(struct i2c_client *client,
						const struct i2c_device_id *id) 
{
	printk("adxl345 probe\n");
	int err;

	/*
	 * get informations from device tree
	 * do it later
	 */
	struct device_node *of_node;	// client->dev.of_node;
	
	
	/* register misc device */
	err = misc_register(&adxl345_misc);
	if (err) {
		printk(KERN_ERR "register adxl345 misc fail!\n");	
		goto register_err;
	}
	
	/* 
	 * register interrupt
	 * do it later with the blocked function
	 */



	/* fill the global value adxl345,
	 * and attach it to client->dev->p->driver_data, 
	 * but for now i do not use client->dev->p->driver_data,
	 * maybe i will use it later,
	 * and when i familiar with the device tree i wiil
	 * replace the follow code of filling
	 */
	adxl345.i2c_client = client;
	adxl345.ofsx = AXS_X_REG;
	adxl345.ofsy = AXS_Y_REG;
	adxl345.ofsz = AXS_Z_REG;
	adxl345.dev_addr = ADXL345_I2C_ADDR;
	i2c_set_clientdata(client, &adxl345);
	return 0;

interrupt_err:
	/* do it later */

register_err:
	err = misc_deregister(&adxl345_misc);
	if (err) {
		printk(KERN_ERR "deregister adxl345 misc fail!\n");	
		return err;
	}
	
	return 0;
}



static int adxl345_remove(struct i2c_client *client)
{
	printk("adxl345 remove\n");
	int err = 0;

	/*
	 * unregister interrupt
	 * do it later
	 */


	/* deregister misc device */
	err = misc_deregister(&adxl345_misc);
	if (err) 
		printk(KERN_ERR "deregister gsensor misc fail!\n");
		
	return err;
}
							


static const struct of_device_id adxl345_of_match[] = {
	{ .compatible = "analog,adxl345" },
	{ }
};

MODULE_DEVICE_TABLE(of, adxl345_of_match);

static const struct i2c_device_id adxl345_id[] = {
	{ "adxl345", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, adxl345_id);

static struct i2c_driver adxl345_driver = {
	.driver = {
		.name = "adxl345",
		.owner = THIS_MODULE,
		.of_match_table = adxl345_of_match,
	},
	.probe    = adxl345_probe,
	.remove   = adxl345_remove,
	.id_table = adxl345_id,
};

//module_i2c_driver(adxl345_driver);
static int __init adxl345_driver_init(void) 
{ 
	printk("adxl345 driver init\n");
	return i2c_add_driver(&adxl345_driver); 
} 
module_init(adxl345_driver_init); 

static void __exit adxl345_driver_exit(void) 
{ 
	printk("adxl345 driver exit\n");
	i2c_del_driver(&adxl345_driver); 
} 
module_exit(adxl345_driver_exit);

MODULE_AUTHOR("Leaf god <569242715@qq.com>");
MODULE_DESCRIPTION("ADXL345 Three-Axis Digital Accelerometer I2C Bus Driver");
MODULE_LICENSE("GPL");



