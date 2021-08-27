#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>

#define ADXL345_DEV "/dev/adxl345_misc"

#define ADXL345_BLOCK 0
#define ADXL345_NONBLOCK 1

#define ADXL345_ERROR -1

char adxwrite_buf[3];
char adxread_buf[3];


int  main(void)
{	
	int fd,ret;
	adxwrite_buf[0] = 0;
	adxwrite_buf[1] = 0;
	adxwrite_buf[2] = 0;
	
	fd = open(ADXL345_DEV,O_RDWR);
	if (fd < 0) {
		printf("open /dev/adxl345_misc fail!\n");
		return ADXL345_ERROR;
	}

	/* nonblock */
	ioctl(fd, ADXL345_NONBLOCK, NULL);

	/* write initial data to adxl345 */
	ret = write(fd, adxwrite_buf, sizeof(adxwrite_buf));
	if (ret < 0) {
		printf("adxl345 write initial data fail!\n");
		return ADXL345_ERROR;
	}


	/* read in poll situation*/
	while (1) {
		//usleep(200000);
		ret = read(fd, adxread_buf, sizeof(adxread_buf));	
		if (ret < 0) {
			printf("adxl345 read data not ready or error,please read again\n");
		} else if (adxread_buf[0] == 0) {
			;
		} else {
			printf("adxl345 read data: axs_x %x\n",adxread_buf[0]);
			printf("adxl345 read data: axs_y %x\n",adxread_buf[1]);
			printf("adxl345 read data: axs_z %x\n",adxread_buf[2]);
		}	
	}
	
	close(fd);
	return 0;
	
}

	
	
	


