/*
 *  gma30x.c - Linux kernel modules for gma302/gma303 3-Axis Orientation/Motion
 *  Detection Sensor 
 *
 * Copyright (c) 2014 Globalmems, Inc.  All rights reserved.
 *
 * This source is subject to the Globalmems Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of Globalmems Inc.
 * All other rights reserved.
 *
 * This code and information are provided "as is" without warranty of any
 * kind, either expressed or implied, including but not limited to the
 * implied warranties of merchantability and/or fitness for a
 * particular purpose.
 *
 * The following software/firmware and/or related documentation ("Globalmems Software")
 * have been modified by Globalmems Inc. All revisions are subject to any receiver's
 * applicable license agreements with Globalmems Inc.
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
//#define DEBUG	/**< if define : Enable gma->client->dev debug data .*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/input-polldev.h>
#include <linux/device.h>
#include <linux/init-input.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/kernel.h> 
#include <linux/unistd.h> 
#include <linux/sched.h> 
#include <linux/file.h> 
#include <linux/mm.h> 
#include "gma30x.h"
#include <linux/average.h>
#include <mach/hardware.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
//#define GMA_PERMISSION_THREAD
#include <linux/kthread.h>
#include <linux/syscalls.h>
#if defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_PM)
	#include <linux/pm.h>
#endif

/*
 * Defines
 */
#define assert(expr) printk(KERN_ERR "Assertion failed! %s,%d,%s,%s\n",\
			__FILE__, __LINE__, __func__, #expr);


#define POLL_INTERVAL_MAX        200
#define POLL_INTERVAL            20
#define INPUT_FUZZ               0
#define INPUT_FLAT               0

#define MODE_CHANGE_DELAY_MS 100

static struct device *hwmon_dev;

static struct gma_data {
	struct input_dev 		*input;
	struct i2c_client       *client;
	struct input_polled_dev *pollDev; 
	struct mutex interval_mutex;
	struct mutex init_mutex;
    struct workqueue_struct *gma30x_resume_wq;
	struct workqueue_struct *gma30x_init_wq;
	
	raw_data 		accel_data;	///< ACC Data
	raw_data 		offset;		///< Offset
	
	atomic_t	addr;	///< register address
	atomic_t 	enable;	///< HAL 1:enable	0:disable
	atomic_t 	calib;	///< 1:do calibration gsensor
	atomic_t	position;///< must int type ,for Kconfig setup
#ifdef SMA_FILTER
	int	sma_filter;		///< Set AVG sensor data :range(1~16)
	int	sum[SENSOR_DATA_SIZE];	///< SMA Filter sum
	int	bufferave[3][16];
#endif
#ifdef EWMA_FILTER
	int	ewma_filter[SENSOR_DATA_SIZE];	///< Set EWMA WEIGHT :value(2,4,8,16)
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
	volatile int suspend_indator;
#endif
};
static struct gma_data *s_gma;
/*static void gma_acc_set_data(struct gma_data *gma){
	s_gma = gma;
}*/
#ifdef EWMA_FILTER
struct ewma average[SENSOR_DATA_SIZE];
#endif
//char DIR_SENSOR[] = "/data/misc";				///< Need Create sensor folder
char GMA_Offset_TXT[] = "/data/misc/gsensor_offset.txt";	///< SAVE FILE PATH offset.txt
char SH_GSS[] = "system/bin/gss.sh";					///< shell script PATH
char EXEC_GMAD[] = "system/bin/gmad";					///< Execute file PATH
#ifdef GMA302
char CHAR_DEV[] = "/dev/gma302";						///< Character device PATH
#else
char CHAR_DEV[] = "/dev/gma303";						///< Character device PATH
#endif

static int __devinit gma_acc_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __devexit gma_acc_remove(struct i2c_client *client);

static int gma_i2c_rxdata(struct i2c_client *client, unsigned char *rxDat, int length);
static int gma_i2c_txdata(struct i2c_client *client, unsigned char *txData, int length);

static int gma_acc_measure(struct gma_data *gma, int *xyz);
static int gma_set_position(struct gma_data *gma, int);
static int gma_acc_calibration(struct gma_data *gma, int);
int gma30x_init(struct gma_data *gma);
static int sensor_close_dev(struct gma_data *gma);

static void gma30x_resume_events(struct work_struct *work);
static void gma30x_init_events(struct work_struct *work);
static DECLARE_WORK(gma30x_resume_work, gma30x_resume_events);
static DECLARE_WORK(gma30x_init_work, gma30x_init_events);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sensor_suspend(struct early_suspend *h);
static void sensor_resume(struct early_suspend *h);
#endif

static int gma_set_position(struct gma_data *gma, int position){
    dev_info(&gma->client->dev, "position=%d :%d\n", position, __LINE__);
	if (!((position >= -4) && (position <= 4)))
		return -1;
	atomic_set(&gma->position, position);
	//dev_info(&gma->client->dev, "gma->position=%d :%d\n",atomic_read(gma->position), __LINE__);
	return 0;
}

static int GMA_WriteCalibration(struct gma_data *gma, char * offset){
	char w_buf[20] = {0};
	struct file *fp;
	mm_segment_t fs;
	ssize_t ret;

	sprintf(w_buf,"%d %d %d", gma->offset.u.x, gma->offset.u.y, gma->offset.u.z);
	dev_err(&gma->client->dev, "%d %d %d", gma->offset.u.x, gma->offset.u.y, gma->offset.u.z);
	/* Set segment descriptor associated to kernel space */
	fp = filp_open(offset, O_RDWR | O_CREAT, 0666);
	if(IS_ERR(fp))
		dev_err(&gma->client->dev, "filp_open %s error!!.\n", offset);
	else{
		fs = get_fs();
		//set_fs(KERNEL_DS);
		set_fs(get_ds());
		dev_info(&gma->client->dev, "filp_open %s SUCCESS!!.\n", offset);
 		ret = fp->f_op->write(fp,w_buf,20,&fp->f_pos);
	    filp_close(fp,NULL);
		set_fs(fs);
	}
	//fs = get_fs();
	//set_fs(KERNEL_DS);
/*#ifdef GMA_PERMISSION_THREAD
	ret = sys_chmod(offset , 0666);
	ret = sys_fchmodat(AT_FDCWD, offset , 0666);
#endif*/
	

	return 0;
}

void GMA_ReadCalibration(struct gma_data *gma){
	unsigned int orgfs;
	char buffer[20];
	struct file *fp,*fp2;/* *fp open offset.txt , *fp2 open path2 offset.txt */
	orgfs = get_fs();
	/* Set segment descriptor associated to kernel space */
	set_fs(KERNEL_DS);

	fp = filp_open(GMA_Offset_TXT, O_RDWR , 0);
	if(IS_ERR(fp)){
		dev_err(&gma->client->dev, "Sorry,file open ERROR !\n");

		//dev_info(&gma->client->dev, "ABS(gma->accel_data.u.x)/y/z= %05d %05d %05d\n", 
				//ABS(gma->accel_data.u.x), ABS(gma->accel_data.u.y), ABS(gma->accel_data.u.z));
#if AutoZeroZ
		//if(1)ABS(gma->accel_data.u.x) < LevelValueRange_2_0 && ABS(gma->accel_data.u.y) < LevelValueRange_2_0)
		/* Calculate new offset */
		//if (atomic_read(&gma->position) < 0) {
		if (atomic_read(&gma->position) > 0) {
			gma_acc_calibration(gma, GRAVITY_ON_Z_NEGATIVE);
		} else 
			gma_acc_calibration(gma, GRAVITY_ON_Z_POSITIVE);
		
		//gma_acc_calibration(gma, GRAVITY_ON_Z_AUTO);
		/* save offset to file */
		GMA_WriteCalibration(gma , GMA_Offset_TXT);
#endif
	}
	else{
		dev_dbg(&gma->client->dev, "filp_open %s SUCCESS!!.\n",GMA_Offset_TXT);
		fp->f_op->read( fp, buffer, 20, &fp->f_pos); // read offset.txt
		sscanf(buffer,"%d %d %d",&gma->offset.u.x,&gma->offset.u.y,&gma->offset.u.z);
		dev_info(&gma->client->dev, "offset.u.x/offset.u.y/offset.u.z = %d/%d/%d\n",
							gma->offset.u.x, gma->offset.u.y, gma->offset.u.z);
		filp_close(fp,NULL);
	}
	set_fs(orgfs);
}

/***** I2C I/O function ***********************************************/
static int gma_i2c_rxdata( struct i2c_client *client, unsigned char *rxData, int length){
	struct i2c_msg msgs[] = 
	{
		{.addr = client->addr, .flags = 0, .len = 1, .buf = rxData,},
		{.addr = client->addr, .flags = I2C_M_RD, .len = length, .buf = rxData,},
	};
#ifdef GMA_DEBUG_DATA
	unsigned char addr = rxData[0];
#endif
	if (i2c_transfer(client->adapter, msgs, 2) < 0) {
		dev_err(&client->dev, "%s: transfer failed.", __func__);
		return -EIO;
	}
#ifdef GMA_DEBUG_DATA
	dev_dbg(&client->dev, "RxData: len=%02x, addr=%02x, data=%02x\n",
		length, addr, rxData[0]);
#endif
	return 0;
}

static int gma_i2c_txdata( struct i2c_client *client, unsigned char *txData, int length){
	struct i2c_msg msg[] = {
		{.addr = client->addr, .flags = 0, .len = length, .buf = txData,},
	};

	if (i2c_transfer(client->adapter, msg, 1) < 0) {
		dev_err(&client->dev, "%s: transfer failed.", __func__);
		return -EIO;
	}
#ifdef GMA_DEBUG_DATA
	dev_dbg(&client->dev, "TxData: len=%02x, addr=%02x data=%02x\n",
		length, txData[0], txData[1]);
#endif
	return 0;
}
/** 
 *	@brief get XYZ rawdata & Redefined 1G =1024
 *  @param gma_data description
 *  @param xyz_p
 *  @return 0
 */
static int gma_acc_measure(struct gma_data *gma, int *xyz_p){
	u8 buffer[11];
	s16 xyzTmp[SENSOR_DATA_SIZE];
	int i;
	/* get xyz high/low bytes, 0x04 */
	buffer[0] = GMA1302_REG_STADR;
	for(i = 0; i < SENSOR_DATA_SIZE; ++i){
		xyz_p[i] = 0;
		xyzTmp[i] = 0;
	}
	/* Read acceleration data */
	if (gma_i2c_rxdata(gma->client, buffer, 11)!= 0)
		dev_err(&gma->client->dev, "Read acceleration data fail\n");
	else{
		for(i = 0; i < SENSOR_DATA_SIZE; ++i){
			/* merge xyz high/low bytes(13bit) & 1g = 512 *2 = GMS_DEFAULT_SENSITIVITY */
			xyzTmp[i] = ((buffer[2*(i+2)] << 8) | buffer[2*(i+1)+1] ) << 1;
		}
	}
	/* enable ewma filter */ 
#ifdef EWMA_FILTER
	for(i = 0; i < SENSOR_DATA_SIZE; ++i){
		/* rawdata + EWMA_POSITIVE = unsigned long data */
	    ewma_add(&average[i], (unsigned long) (xyzTmp[i] + EWMA_POSITIVE));
	    xyz_p[i] = (int) (ewma_read(&average[i]) - EWMA_POSITIVE);
	}
#else
	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		xyz_p[i] = xyzTmp[i];
#endif
	return 0;
}
/* calculate delta offset */
static int gma_acc_calibration(struct gma_data *gma, int gAxis){
// add by Steve 20150202
// *****************************
	bool compare_avg=false;
	int avg_Z=0;
	int cycle=0;
	//int threshold=GMS_DEFAULT_SENSITIVITY/20; //about 0.05G
	int threshold=GMS_DEFAULT_SENSITIVITY/1; //about 1.00G 
// *****************************
	raw_data avg;
	int i, j, xyz[SENSOR_DATA_SIZE];
	long xyz_acc[SENSOR_DATA_SIZE];
	/* initialize the offset value */
	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		gma->offset.v[i] = 0;
// add by Steve 20150202
// *****************************
	
	for(cycle=0;cycle<3;cycle++)
	{		
// *****************************		  	
		/* initialize the accumulation buffer */
	  	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
			xyz_acc[i] = 0;
	
		for(i = 0; i < AVG_NUM; i++) {
			gma_acc_measure(gma, (int *)&xyz);
			for(j = 0; j < SENSOR_DATA_SIZE; ++j)
				xyz_acc[j] += xyz[j];
	  	}
		/* calculate averages */
	  	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
			avg.v[i] = xyz_acc[i] / AVG_NUM;
		
// add by Steve 20150202
// *****************************
		if(abs(avg.u.z-avg_Z) > threshold)
		{
			dev_info(&gma->client->dev, "Sensor unstable cycle %d , try again automatically: %d\n",cycle , __LINE__);
			compare_avg=false;
			avg_Z=avg.u.z;	
		}			
		else
		{
			dev_info(&gma->client->dev, "Sensor Stable in comparsion cycle %d : %d\n",cycle , __LINE__);
			compare_avg=true;
			break;	
		}
	}//end of for(c_cycle=0;c_cycle<2;c_cycle++)r	
	if(compare_avg==false)
	{
		dev_info(&gma->client->dev, "Sensor unstable,Please try again %d : %d\n", gAxis, __LINE__);
		return -1;
	}
// *****************************

	switch(gAxis){
		case GRAVITY_ON_Z_NEGATIVE:
			gma->offset.u.x =  avg.v[0];
			gma->offset.u.y =  avg.v[1];
			gma->offset.u.z =  avg.v[2] + GMS_DEFAULT_SENSITIVITY;
			dev_info(&gma->client->dev, "gAxis = %d : %d\n", gAxis, __LINE__);
			break;
		case GRAVITY_ON_X_POSITIVE:
			gma->offset.u.x =  avg.v[0] - GMS_DEFAULT_SENSITIVITY;    
			gma->offset.u.y =  avg.v[1];
			gma->offset.u.z =  avg.v[2];
			dev_info(&gma->client->dev, "gAxis = %d : %d\n", gAxis, __LINE__);
		 	break;
		case GRAVITY_ON_Z_POSITIVE:
			gma->offset.u.x =  avg.v[0] ;
			gma->offset.u.y =  avg.v[1] ;
			gma->offset.u.z =  avg.v[2] - GMS_DEFAULT_SENSITIVITY;
			dev_info(&gma->client->dev, "gAxis = %d : %d\n", gAxis, __LINE__);
		 	break;
		case GRAVITY_ON_X_NEGATIVE:
			gma->offset.u.x =  avg.v[0] + GMS_DEFAULT_SENSITIVITY;    
			gma->offset.u.y =  avg.v[1];
			gma->offset.u.z =  avg.v[2];
			dev_info(&gma->client->dev, "gAxis = %d : %d\n", gAxis, __LINE__);
		 	break;
		case GRAVITY_ON_Y_NEGATIVE:
			gma->offset.u.x =  avg.v[0];    
			gma->offset.u.y =  avg.v[1] - GMS_DEFAULT_SENSITIVITY;
			gma->offset.u.z =  avg.v[2];
			dev_info(&gma->client->dev, "gAxis = %d : %d\n", gAxis, __LINE__);
		 	break;
		case GRAVITY_ON_Y_POSITIVE:
			gma->offset.u.x =  avg.v[0];    
			gma->offset.u.y =  avg.v[1] + GMS_DEFAULT_SENSITIVITY;
			gma->offset.u.z =  avg.v[2];
			dev_info(&gma->client->dev, "gAxis = %d : %d\n", gAxis, __LINE__);
		 	break;
		case GRAVITY_ON_X_AUTO:
			if(avg.v[0] < 0){
				gma->offset.u.x =  avg.v[0] + GMS_DEFAULT_SENSITIVITY;
				gma->offset.u.y =  avg.v[1];
				gma->offset.u.z =  avg.v[2];
			}
			else{
				gma->offset.u.x =  avg.v[0] - GMS_DEFAULT_SENSITIVITY;
				gma->offset.u.y =  avg.v[1];
				gma->offset.u.z =  avg.v[2];
			}
			dev_info(&gma->client->dev, "gAxis = %d : %d\n", gAxis, __LINE__);
			break;
	    case GRAVITY_ON_Y_AUTO:
			if(avg.v[1] < 0){
				gma->offset.u.x =  avg.v[0];
				gma->offset.u.y =  avg.v[1] + GMS_DEFAULT_SENSITIVITY;
				gma->offset.u.z =  avg.v[2];
			}
			else{
				gma->offset.u.x =  avg.v[0];
				gma->offset.u.y =  avg.v[1] - GMS_DEFAULT_SENSITIVITY;
				gma->offset.u.z =  avg.v[2];
			}
			dev_info(&gma->client->dev, "gAxis = %d : %d\n", gAxis, __LINE__);
			break;
		case GRAVITY_ON_Z_AUTO: 
			if(avg.v[2] < 0){
				gma->offset.u.x =  avg.v[0];
				gma->offset.u.y =  avg.v[1];
				gma->offset.u.z =  avg.v[2] + GMS_DEFAULT_SENSITIVITY;
			}
			else{
				gma->offset.u.x =  avg.v[0];
				gma->offset.u.y =  avg.v[1];
				gma->offset.u.z =  avg.v[2] - GMS_DEFAULT_SENSITIVITY;
			}
			dev_info(&gma->client->dev, "gAxis = %d : %d\n", gAxis, __LINE__);
			break;
		default:  
			return -ENOTTY;
	}
	return 0;
}
/* Addresses to scan */
static const unsigned short normal_i2c[2] = {SENSOR_I2C_ADDR, I2C_CLIENT_END};
static __u32 twi_id = 0;
static struct sensor_config_info gsensor_info = {
	.input_type = GSENSOR_TYPE,
};

enum {
	DEBUG_INIT = 1U << 0,
	DEBUG_CONTROL_INFO = 1U << 1,
	DEBUG_DATA_INFO = 1U << 2,
	DEBUG_SUSPEND = 1U << 3,
};
static u32 debug_mask = 0;
#define dprintk(level_mask, fmt, arg...)	 \
	printk(KERN_DEBUG fmt , ## arg)

module_param_named(debug_mask, debug_mask, int, 0644);

//Function as i2c_master_send, and return 1 if operation is successful. 
static int i2c_write_bytes(struct i2c_client *client, uint8_t *data, uint16_t len)
{
	struct i2c_msg msg;
	int ret=-1;
	
	msg.flags = !I2C_M_RD;
	msg.addr = client->addr;
	msg.len = len;
	msg.buf = data;		
	
	ret=i2c_transfer(client->adapter, &msg,1);
	return ret;
}
static bool gsensor_i2c_test(struct i2c_client * client)
{
	int ret, retry;
	uint8_t test_data[1] = { 0 };	//only write a data address.
	
	for(retry=0; retry < 2; retry++)
	{
		ret =i2c_write_bytes(client, test_data, 1);	//Test i2c.
		if (ret == 1)
			break;
		msleep(5);
	}
	
	return ret==1 ? true : false;
}

/**
 * gsensor_detect - Device detection callback for automatic device creation
 * return value:  
 *                    = 0; success;
 *                    < 0; err
 */
static int gsensor_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	int ret;
	
	dprintk(DEBUG_INIT, "%s enter \n", __func__);
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
        return -ENODEV;
    
	if(twi_id == adapter->nr){
            pr_info("%s: addr= %x\n",__func__,client->addr);

            ret = gsensor_i2c_test(client);
        	if(!ret){
        		pr_info("%s:I2C connection might be something wrong or maybe the other gsensor equipment! \n",__func__);
        		return -ENODEV;
        	}else{           	    
            	pr_info("I2C connection sucess!\n");
            	strlcpy(info->type, GSENSOR_ID, I2C_NAME_SIZE);
    		    return 0;	
	             }

	}else{
		return -ENODEV;
	}
}
/** 
 *	@brief Simple moving average
 *  @param gma_data description
 *  @param int *xyz
 *  @return 0
 */
#ifdef SMA_FILTER
/* for Simple Moving Average */
static int SMA( struct gma_data *gma, int *xyz){
	int i, j;
	static s8	pointer = -1;				/* last update data */

	/* init gma->sum */
	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		gma->sum[i] = 0;

	pointer++;
	pointer %= gma->sma_filter;
	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		gma->bufferave[i][pointer] = xyz[i];

    for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		for(j = 0; j < gma->sma_filter; ++j)
			gma->sum[i] += gma->bufferave[i][j];

	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		xyz[i] = gma->sum[i] / gma->sma_filter;

	return 0;
}
#endif

static void gma_sysfs_update_active_status(struct gma_data *gma , 
											int enable)
{
	int err;
	//dev_dbg(&s_gma->client->dev, "enable=%d : %d\n", enable, __LINE__);
	if(enable){
		err = gma30x_init(gma);
/*#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
		if (gma->suspend_indator == 0)
#endif
			dev_dbg(&gma->client->dev, "gma->suspend_indator =%d :%d\n", gma->suspend_indator, __LINE__);
*/		s_gma->pollDev->input->open(gma->pollDev->input);
		dev_dbg(&gma->client->dev, "gma30x enable setting active :%d\n", __LINE__);
	}
	else{
		s_gma->pollDev->input->close(gma->pollDev->input);
		err = sensor_close_dev(gma);
		dev_dbg(&gma->client->dev, "gma30x enable setting inactive :%d\n", __LINE__);
	}
}

static bool get_value_as_int(char const *buf, 
								size_t size, 
								int *value)
{
	long tmp;
	if (size == 0)
		return false;
	/* maybe text format value */
	if ((buf[0] == '0') && (size > 1)) {
		if ((buf[1] == 'x') || (buf[1] == 'X')) {
			/* hexadecimal format */
			if (0 != strict_strtol(buf, 16, &tmp))
				return false;
		} else {
			/* octal format */
			if (0 != strict_strtol(buf, 8, &tmp))
				return false;
		}
	} else {
		/* decimal format */
		if (0 != strict_strtol(buf, 10, &tmp))
			return false;
	}

	if (tmp > INT_MAX)
		return false;

	*value = tmp;
	return true;
}
static bool get_value_as_int64(char const *buf, 
								size_t size, 
								long long *value)
{
	long long tmp;
	if (size == 0)
		return false;
	/* maybe text format value */
	if ((buf[0] == '0') && (size > 1)) {
		if ((buf[1] == 'x') || (buf[1] == 'X')) {
			/* hexadecimal format */
			if (0 != strict_strtoll(buf, 16, &tmp))
				return false;
		} else {
			/* octal format */
			if (0 != strict_strtoll(buf, 8, &tmp))
				return false;
		}
	} else {
		/* decimal format */
		if (0 != strict_strtoll(buf, 10, &tmp))
			return false;
	}

	if (tmp > LLONG_MAX)
		return false;

	*value = tmp;
	return true;
}

static ssize_t gma_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_polled_dev *polldev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", polldev->poll_interval);
}

static ssize_t gma_delay_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	unsigned long data;
	int err;

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;
	if (data > POLL_INTERVAL_MAX)
		data = POLL_INTERVAL_MAX;
    
	mutex_lock(&s_gma->interval_mutex);
	s_gma->pollDev->poll_interval = data;
	mutex_unlock(&s_gma->interval_mutex);

	return count;
}

static ssize_t gma_acc_private_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	raw_data accel;
	accel = s_gma->accel_data;
	return sprintf(buf, "%d %d %d\n", accel.v[0], accel.v[1], accel.v[2]);
}

static ssize_t gma_enable_show( struct device *dev, 
							struct device_attribute *attr, 
							char *buf)
{
	char str[2][16]={"ACC enable OFF","ACC enable ON"};
	int flag;
	flag=atomic_read(&s_gma->enable);
	return sprintf(buf, "%s\n", str[flag]);
	return 0;
}

static ssize_t gma_enable_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	//struct gma_data *gma = dev_get_drvdata(dev);
	unsigned long en;
	int err;

	err = strict_strtoul(buf, 10, &en);
	if (err)
		dev_err(&s_gma->client->dev, "strict_strtoul error : %d\n", __LINE__);
	
	en = en ? 1 : 0;
	dev_dbg(&s_gma->client->dev, "en=%ld : %d\n", en, __LINE__);
	atomic_set(&s_gma->enable,en);
	gma_sysfs_update_active_status(s_gma , en);
	return count;
}
/* sysfs position show & store */
static ssize_t gma_position_show( struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
    int data;
    data = atomic_read(&(s_gma->position));
    return sprintf(buf, "%d\n", data);
}

static ssize_t gma_position_store(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	unsigned long position;
	int ret;

	ret = strict_strtol(buf, 10, &position);
	if (ret < 0)
		return count;
	dev_dbg(&s_gma->client->dev, "position=%ld :%d\n", position, __LINE__);
	gma_set_position(s_gma, position);
	return count;
}
/* sysfs offset show & store */
static ssize_t gma_offset_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{

	return sprintf(buf, "%d %d %d\n", 
				s_gma->offset.u.x, 
				s_gma->offset.u.y, 
				s_gma->offset.u.z);
}

static ssize_t gma_offset_store( struct device *dev,
						struct device_attribute *attr,
						const char *buf,
						size_t count)
{
	sscanf(buf, "%d %d %d", (int *)&s_gma->offset.v[0], (int *)&s_gma->offset.v[1], (int *)&s_gma->offset.v[2]);
	GMA_WriteCalibration(s_gma, GMA_Offset_TXT);
	return count;
}
/* sysfs calibration show & store*/
static ssize_t gma_calib_show( struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&s_gma->calib));
}

static ssize_t gma_calib_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	int i ;
	unsigned long side;
	int ret;

	ret = strict_strtoul(buf, 10, &side);
    if (!((side >= 1) && (side <= 9)))
		return -1;
	gma_acc_calibration(s_gma, side);
	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		dev_info(&s_gma->client->dev, "side =%ld, offset.v[%d]=%d : %d\n", side , i, s_gma->offset.v[i], __LINE__);
	/* save file */
	GMA_WriteCalibration(s_gma, GMA_Offset_TXT);
	return count;
}
/* sysfs id show */
static ssize_t gma_chipinfo_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	char str[8]={GSENSOR_ID};
	return sprintf(buf, "%s\n", str);
}

static ssize_t gma_register_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
	int address, value;
	unsigned char buffer[2];
	
    sscanf(buf, "[0x%x]=0x%x", &address, &value);
    
	buffer[0] = address;
	buffer[1] = value;
	gma_i2c_txdata(s_gma->client, buffer, 2);

    return count;
}

static ssize_t gma_register_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    size_t count = 0;
    u8 reg[0x10];
    int i;
    
    // read reg: 0x00 0x01
	reg[0] = GMA1302_REG_PID;
	gma_i2c_rxdata(s_gma->client, reg, 1);
	count += sprintf(&buf[count], "0x%02x: 0x%02x\n", GMA1302_REG_PID, reg[0]);
	// read reg: 0x01
	reg[0] = GMA1302_REG_PD;
	gma_i2c_rxdata(s_gma->client, reg, 1);
	count += sprintf(&buf[count], "0x%02x: 0x%02x\n", GMA1302_REG_PD, reg[0]);
	// read reg: 0x02
	reg[0] = GMA1302_REG_ACTR;
	gma_i2c_rxdata(s_gma->client, reg, 1);
	count += sprintf(&buf[count], "0x%02x: 0x%02x\n", GMA1302_REG_ACTR, reg[0]);
	// read reg: 0x03
	reg[0] = GMA1302_REG_MTHR;
	gma_i2c_rxdata(s_gma->client, reg, 1);
	count += sprintf(&buf[count], "0x%02x: 0x%02x\n", GMA1302_REG_MTHR, reg[0]);
    // read reg:  0x04 [0x0055]
	reg[0] = GMA1302_REG_STADR;
    gma_i2c_rxdata(s_gma->client, reg, 1);
    count += sprintf(&buf[count], "0x%02x: 0x%02x\n", GMA1302_REG_STADR, reg[0]);
    // read reg: 0x05 ~ 0x0c
	reg[0] = GMA1302_REG_STATUS;
    gma_i2c_rxdata(s_gma->client, reg, 9);
    for (i = 0 ; i < 9; i++)
        count += sprintf(&buf[count], "0x%02x: 0x%02x\n", GMA1302_REG_STATUS+i, reg[i]);
	// read reg: 0x15 ~ 0x17
	reg[0] = GMA1302_REG_INTCR;
    gma_i2c_rxdata(s_gma->client, reg, 3);
    for (i = 0 ; i < 3; i++)
        count += sprintf(&buf[count], "0x%02x: 0x%02x\n", GMA1302_REG_INTCR+i, reg[i]);
	// read reg: 0x18
	reg[0] = GMA1302_REG_CONTR3;
    gma_i2c_rxdata(s_gma->client, reg, 1);
	count += sprintf(&buf[count], "0x%02x: 0x%02x\n", GMA1302_REG_CONTR3, reg[0]);
	// read reg: 0x38
	reg[0] = GMA1302_REG_OSM;
    gma_i2c_rxdata(s_gma->client, reg, 1);
	count += sprintf(&buf[count], "0x%02x: 0x%02x\n", GMA1302_REG_OSM, reg[0]);
    return count;
}
/* sysfs reg_read show & store */
static ssize_t gma_reg_read_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	int err;
	unsigned char i2c[6];

	i2c[0] = (unsigned char)atomic_read(&s_gma->addr);
	err = gma_i2c_rxdata(s_gma->client, i2c, 6);
	if (err < 0)
		return err;

	return sprintf(buf, "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
						i2c[0], i2c[1], i2c[2], i2c[3], i2c[4], i2c[5]);
}

static ssize_t gma_reg_read_store(
	struct device *dev,
	struct device_attribute *attr,
	char const *buf,
	size_t count)
{
	int addr = 0;

	if (NULL == buf)
		return -EINVAL;
	
	if (0 == count)
		return 0;

	if (false == get_value_as_int(buf, count, &addr))
		return -EINVAL;
	dev_info(&s_gma->client->dev, "addr=%d  \n", addr);
	if (addr < 0 || 128 < addr)
		return -EINVAL;

	atomic_set(&s_gma->addr, addr);

	return count;
}
/* sysfs reg_write show & store */
static ssize_t gma_reg_write_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	int err;
	unsigned char i2c[6];

	i2c[0] = (unsigned char)atomic_read(&s_gma->addr);
	err = gma_i2c_rxdata(s_gma->client, i2c, 6);
	if (err < 0)
		return err;

	return sprintf(buf, "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
						i2c[0], i2c[1], i2c[2], i2c[3], i2c[4], i2c[5]);
}

static ssize_t gma_reg_write_store(
	struct device *dev,
	struct device_attribute *attr,
	char const *buf,
	size_t count)
{
	int value = 0;
	unsigned char i2c[2];
	if (NULL == buf)
		return -EINVAL;
	
	if (0 == count)
		return 0;

	if (false == get_value_as_int(buf, count, &value))
		return -EINVAL;
	//dev_info(&s_gma->client->dev, "value=%d\n", value);
	if (value < 0 || 256 < value)
		return -EINVAL;

	/* set value to reg */
	i2c[0] = (unsigned char)atomic_read(&s_gma->addr);
	i2c[1] = value;
	gma_i2c_txdata(s_gma->client, i2c, 2);
	//dev_dbg(&s_gma->client->dev, "i2c[0]=%d,i2c[1]=%d\n",i2c[0],i2c[1]);
	return count;
}

/*********************************************************************
 *
 * SysFS attribute functions
 *
 * directory : /sys/class/input/inputX/
 * files :
 *  - enable	    [rw]    [t] : enable flag for accelerometer
 *  - delay		    [rw]	[t] : delay in nanosecond for accelerometer
 *  - position		[rw]	[t] : chip mounting position
 *  - offset		[rw]	[t] : show calibration offset
 *  - calibration	[rw]	[t] : G sensor calibration (1~9)
 *  - sma			[rw]	[t] : Simple Moving Average sensor data(1~16)
 *  - ewma			[rw]	[t] : Exponentially weighted moving average sensor data(2,4,8,16)
 *  - data			[r]	    [t] : sensor data
 *  - reg			[r]	    [t] : Displays the current register value
 *  - chipinfo		[r]		[t] : chip information
 *
 * debug :
 *  - reg_rx		[rw] 	[t] : cat: Read from register(show value) , echo : Setting the register to be read
 *  - reg_tx		[rw] 	[t] : cat: Read from register(show value) , echo : The value currently being written to the register
 *
 * [rw]= read/write
 * [r] = read only
 * [w] = write only
 * [b] = binary format
 * [t] = text format
 */

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
		gma_enable_show, gma_enable_store);

static DEVICE_ATTR(data, S_IRUGO|S_IWUSR|S_IWGRP,
		gma_acc_private_data_show, NULL);

static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
		gma_delay_show, gma_delay_store);

static DEVICE_ATTR(chipinfo, S_IRUGO,
		gma_chipinfo_show, NULL);
		   
static DEVICE_ATTR(offset, S_IRUGO|S_IWUGO,
		gma_offset_show, gma_offset_store);
		
static DEVICE_ATTR(position, S_IRUGO|S_IWUSR,
		gma_position_show, gma_position_store);
		
static DEVICE_ATTR(calibration, S_IRUGO|S_IWUGO,
		gma_calib_show, gma_calib_store);

static DEVICE_ATTR(reg, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		gma_register_show, gma_register_store);

static DEVICE_ATTR(reg_rx, S_IRUGO|S_IWUGO,
		gma_reg_read_show, gma_reg_read_store);
		
static DEVICE_ATTR(reg_tx, S_IRUGO|S_IWUGO,
		gma_reg_write_show, gma_reg_write_store);
		   
#ifdef SMA_FILTER
static DEVICE_ATTR(sma, S_IRUGO|S_IWUGO,
		gma_sma_show, gma_sma_store);
#endif
#ifdef EWMA_FILTER
static DEVICE_ATTR(ewma, S_IRUGO|S_IWUGO,
		gma_ewma_show, gma_ewma_store);
#endif
static struct attribute *gma_acc_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_offset.attr,
	&dev_attr_position.attr,
	&dev_attr_chipinfo.attr,
	&dev_attr_calibration.attr,
#ifdef SMA_FILTER
	&dev_attr_sma.attr,
#endif /* Simple Moving Average */
#ifdef EWMA_FILTER
	&dev_attr_ewma.attr,
#endif 
	&dev_attr_data.attr,
	&dev_attr_reg.attr,
	&dev_attr_reg_rx.attr,
	&dev_attr_reg_tx.attr,
	NULL
};

static struct attribute_group gma_acc_attribute_group = {
	.attrs = gma_acc_attributes
};

/*
 * Initialization function
 */
int gma30x_init(struct gma_data *gma){
	unsigned char buffer[7];
	mutex_lock(&gma->init_mutex);
	/* 1. Powerdown reset */
	buffer[0] = GMA1302_REG_PD;
	buffer[1] = GMA1302_MODE_RESET;
	gma_i2c_txdata(gma->client, buffer, 2);
	/* 2. check GMA1302_REG_PID(0x00) */
	buffer[0] = GMA1302_REG_PID;
	gma_i2c_rxdata(gma->client, buffer, 1);
	if(buffer[0] == GMA302_VAL_WMI)
		dev_info(&gma->client->dev, "%s: PID = 0x%x, GMA302 accelerometer.\n", __func__, buffer[0]);
	else if(buffer[0] == GMA303_VAL_WMI)
		dev_info(&gma->client->dev, "%s: PID = 0x%x, GMA303 accelerometer.\n", __func__, buffer[0]);
	else if(buffer[0] == GMA305_VAL_WMI)
		dev_info(&gma->client->dev, "%s: PID = 0x%x, GMA305 accelerometer.\n", __func__, buffer[0]);
	else if(buffer[0] == GMA303_VAL_WMI_RD)
		dev_info(&gma->client->dev, "%s: PID = 0x%x, GMA303_RD sample.\n", __func__, buffer[0]);
	else{
		dev_err(&gma->client->dev, "%s: PID = 0x%x, The device is not GlobalMems accelerometer.", __func__, buffer[0]);
		return -ENXIO;
	}
	/* 3. turn off the low-pass filter */
	buffer[0] = GMA1302_REG_CONTR1;
	buffer[1] = GMA1302_VAL_OFF;//GMA1302_VAL_LPF_ON;
	gma_i2c_txdata(gma->client, buffer, 2);
	/* 4. turn on the offset temperature compensation */
	buffer[0] = GMA1302_REG_CONTR3;
	buffer[1] = GMA1302_VAL_OFFSET_TC_ON;
	gma_i2c_txdata(gma->client, buffer, 2);
	/* 5. turn off the data ready interrupt and configure the INT pin to active high, push-pull type */
	buffer[0] = GMA1302_REG_INTCR;
	buffer[1] = GMA1302_VAL_OFF;//GMA1302_VAL_DATA_READY_ON;
	gma_i2c_txdata(gma->client, buffer, 2);
	/* 6. treshold set to max */
    buffer[0] = GMA1302_REG_MTHR;
    buffer[1] = GMA1302_VAL_TRESHOLD_MAX;
    gma_i2c_txdata(gma->client, buffer, 2);
	/* 7. Oversampling mode*/
/*	buffer[0] = GMA1302_REG_OSM;
	buffer[1] = GMA1302_VAL_LOW_NOISE; //Low noise
	gma_i2c_txdata(gma->client, buffer, 2);
*/	
	mutex_unlock(&gma->init_mutex);
	//mdelay(MODE_CHANGE_DELAY_MS);
#ifdef EWMA_FILTER
	ewma_init(&average[0], EWMA_FACTOR, EWMA_WEIGHT_X);
	ewma_init(&average[1], EWMA_FACTOR, EWMA_WEIGHT_Y);
	ewma_init(&average[2], EWMA_FACTOR, EWMA_WEIGHT_Z);
	gma->ewma_filter[0] = EWMA_WEIGHT_X;
	gma->ewma_filter[1] = EWMA_WEIGHT_Y;
	gma->ewma_filter[2] = EWMA_WEIGHT_Z;
	dev_info(&gma->client->dev, "EWMA_FILTER: %d %d %d\n", gma->ewma_filter[0], gma->ewma_filter[1], gma->ewma_filter[2]);
#endif
	dev_dbg(&gma->client->dev, "gma30x enable setting active :%d\n", __LINE__);
	//s_gma->pollDev->input->open(gma->pollDev->input);
	return 0;
}
/** 
 *	@brief check_horizontal_state
 *  @param gma_data description
 *  @return 0
 */
#if 0//AutoZeroZ	//run time calibration AutoZeroZ 
static int check_horizontal_state(struct gma_data *gma)
{
	static int MaxRange = LevelValueRange_0_0625;
	int i;
	raw_data offset; 		/* Record old offset */
	static int stable_flag =0; //for auto zero .Analyzing horizontal state
	/* 1.Condition: Analyzing horizontal state */
	if( ABS(gma->accel_data.u.x) < MaxRange && ABS(gma->accel_data.u.y) < MaxRange  && stable_flag < 10)
		stable_flag++;
	else{
		if(stable_flag < 0)
			stable_flag = 0;
		else if(stable_flag < 10)
			stable_flag--;
		else if(stable_flag > 10)
			stable_flag=11;
		else
			stable_flag++;
	}
	//dev_dbg(&gma->client->dev, "a.stable_flag= %d, (rawdata - offset) x/y/z = %03d %03d %03d , MaxRange/981 =%d(LSB)\n", stable_flag, gma->axis.x/981, gma->axis.y/981, gma->axis.z/981, MaxRange/981);
	if(stable_flag == 10){
		//dev_dbg(&gma->client->dev, "stable_flag=%d\n", stable_flag);
		/* 2.Condition: Analyzing horizontal state check again*/
		if( ABS(gma->accel_data.u.x) <= LevelValueRange_0_015625  && ABS(gma->accel_data.u.y) <= LevelValueRange_0_015625 ){
		//dev_dbg(&gma->client->dev, "b.(rawdata - offset) x/y/z = %03d %03d %03d , MaxRange/981 =%d(LSB)\n", gma->axis.x/981, gma->axis.y/981, gma->axis.z/981, MaxRange/981);
			/* 3.record last time offset */
			for(i = 0; i < SENSOR_DATA_SIZE; ++i)
				offset.v[i] = gma->offset.v[i];
			/* 4.Calculate new offset */
			gma_acc_calibration(gma, GRAVITY_ON_Z_AUTO);
			/* 5.offset(X& Y): |new - last_time| < LevelValueRange_0_0078125 */
			if(ABS(gma->offset.u.x - offset.u.x) <= LevelValueRange_0_0078125 && ABS(gma->offset.u.y - offset.u.y) <= LevelValueRange_0_0078125 ){
				for(i = 0; i < SENSOR_DATA_SIZE; ++i)
					gma->offset.v[i] = offset.v[i];	/* use last time offset */
			}
			else{
			/* 6.offset(Z): |last time| > 1g (Prevent calibration errors) */
				if(ABS(offset.u.z) > GMS_DEFAULT_SENSITIVITY)
					gma->offset.v[2] = offset.v[2];	/*(of.z) use last time offset */
					
				dev_dbg(&gma->client->dev, "d.gma->offset.v[2] =%d :%d\n", gma->offset.v[2], __LINE__);
				GMA_WriteCalibration(gma , GMA_Offset_TXT);
			}
			MaxRange /= 2;
			stable_flag = 0;
		}
	}
	
	return stable_flag;
}
#endif

static int gma_axis_remap(struct gma_data *gma)
{
    int swap;
    int position = atomic_read(&gma->position);
    switch (abs(position)) {
		case 1:
			swap = gma->accel_data.v[0];
            gma->accel_data.v[0] = -(gma->accel_data.v[1]);
            gma->accel_data.v[1] = -swap;
			gma->accel_data.v[2] = -(gma->accel_data.v[2]);
            break;
        case 2:
            gma->accel_data.v[0] = -(gma->accel_data.v[0]);
            gma->accel_data.v[1] = gma->accel_data.v[1];
			gma->accel_data.v[2] = -(gma->accel_data.v[2]);
            break;
        case 3:
			swap = gma->accel_data.v[0];
            gma->accel_data.v[0] = gma->accel_data.v[1];
            gma->accel_data.v[1] = swap;
			gma->accel_data.v[2] = -(gma->accel_data.v[2]);
            break;
            
        case 4:
            gma->accel_data.v[0] = gma->accel_data.v[0];
            gma->accel_data.v[1] = -(gma->accel_data.v[1]);
			gma->accel_data.v[2] = -(gma->accel_data.v[2]);
            break;
    }
    
    if (position < 0) {
        gma->accel_data.v[2] = -(gma->accel_data.v[2]);
        gma->accel_data.v[0] = -(gma->accel_data.v[0]);
    }
    //dev_dbg(&gma->client->dev, "gma->position= %d :%d\n", position, __LINE__);
    return 0;
}

static void gma_work_func(struct input_polled_dev *dev)
{
	static bool firsttime=true;
	raw_data xyz;
	int i;
	
	int ret = gma_acc_measure(s_gma, (int *)&xyz.v);
#ifdef SMA_FILTER
	if (xyz.v[0] != 0 && xyz.v[1] != 0 && xyz.v[2] != 0)
		SMA( s_gma, (int *)&xyz.v);
	//dev_dbg(&gma->client->dev, "SMA_FILTER  xyz.v: %3d , %3d , %3d\n", xyz.v[0], xyz.v[1], xyz.v[2]);
#endif
	/* data->accel_data = RawData - Offset) , redefine 1g = 1024 */
  	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
     	s_gma->accel_data.v[i] = xyz.v[i] - s_gma->offset.v[i];
	
	if (ret == 0) 
		gma_axis_remap(s_gma);
	dev_dbg(&s_gma->client->dev, "after gma_axis_remap X/Y/Z  %05d,%05d,%05d !\n"
			, s_gma->accel_data.v[0], s_gma->accel_data.v[1], s_gma->accel_data.v[2]);
	if(firsttime){
		GMA_ReadCalibration(s_gma);
	 	firsttime=false;
	}
	
#if 0//AutoZeroZ	//run time calibration AutoZeroZ
	int flag = 0;
	if(flag != 11)	//Check the level of state
		flag = check_horizontal_state(s_gma);
	dev_dbg(&s_gma->client->dev, "flag= %d , s_gma->accel_data.u.x/y/z = %5d  %5d  %5d :%d\n"
			, flag, s_gma->accel_data.v[0], s_gma->accel_data.v[1], s_gma->accel_data.v[2], __LINE__);
#endif
	//Analyzing horizontal state 2015-05-16 add
	if( ABS(s_gma->accel_data.v[0]) < 42 && ABS(s_gma->accel_data.v[1]) < 42)
	{
		s_gma->accel_data.v[0] = 0;
		s_gma->accel_data.v[1] = 0;
	}

	input_report_abs(s_gma->pollDev->input, ABS_X, s_gma->accel_data.v[0]);
	input_report_abs(s_gma->pollDev->input, ABS_Y, s_gma->accel_data.v[1]);
	input_report_abs(s_gma->pollDev->input, ABS_Z, s_gma->accel_data.v[2]);
	input_sync(s_gma->pollDev->input);

	//dprintk(DEBUG_DATA_INFO, "x= 0x%hx, y = 0x%hx, z = 0x%hx\n", x, y, z);
} 
#ifdef GMA_PERMISSION_THREAD
static struct task_struct *GMAPermissionThread = NULL;

static int gma_permission_thread(void *data)
{
	int ret = 0;
	int retry = 0;
	char input_number[2];
	char input_enable[30];
	char input_delay[30];
	char input_calibration[35];
	const char *path;
	mm_segment_t fs = get_fs();
	set_fs(KERNEL_DS);	
	/* check uid */
	//int uid  = sys_getuid();
	//dev_dbg(&s_gma->client->dev, "\nUID %d",uid);
	/* get input the registration number */
	path = kobject_get_path(&s_gma->input->dev.kobj, GFP_KERNEL);
	//dev_dbg(&s_gma->client->dev, "input_register_device SUCCESS %s :%d\n", path ? path : "N/A", __LINE__);
	ret = strlen(path);
	if(path[ret-2] == 't'){
		input_number[1] = path[ret-1];
		sprintf(input_enable, "/sys/class/input/input%c/enable", input_number[1]);
		sprintf(input_delay, "/sys/class/input/input%c/delay", input_number[1]);
		sprintf(input_calibration, "/sys/class/input/input%c/calibration", input_number[1]);
		dev_dbg(&s_gma->client->dev, "input_enable = %s\n""input_delay = %s\n""input_calibration = %s :%d\n"
									, input_enable, input_delay, input_calibration, __LINE__);
	}
	else if(path[ret-3] == 't'){
		input_number[0] = path[ret-2];
		input_number[1] = path[ret-1];
		sprintf(input_enable, "/sys/class/input/input%c%c/enable", input_number[1],input_number[0]);
		sprintf(input_delay, "/sys/class/input/input%c%c/enable", input_number[1],input_number[0]);
		sprintf(input_calibration, "/sys/class/input/input%c%c/calibration", input_number[1],input_number[0]);
		dev_dbg(&s_gma->client->dev, "input_enable = %s\n""input_delay = %s\n""input_calibration = %s:%d\n"
									, input_enable, input_delay, input_calibration, __LINE__);
	}
	else
		goto ERR;
		
	//msleep(5000);
	do{
	    msleep(2000);
		ret = sys_fchmodat(AT_FDCWD, CHAR_DEV , 0666);
		ret = sys_fchmodat(AT_FDCWD, SH_GSS , 0755);
		ret = sys_fchmodat(AT_FDCWD, EXEC_GMAD , 0755);
		ret = sys_chown(SH_GSS , 1000, 2000);	//system uid 1000 shell gid 2000
		ret = sys_chown(EXEC_GMAD , 1000, 2000);
		ret = sys_chown(input_enable , 1000, 1000);	
		ret = sys_chown(input_delay , 1000, 1000);
		ret = sys_mkdir(DIR_SENSOR , 01777);
		if(ret < 0)
			dev_err(&s_gma->client->dev, "fail to execute sys_mkdir,If the DIR exists, do not mind. ret = %d :%d\n"
					, ret, __LINE__);
		ret = sys_fchmodat(AT_FDCWD, input_calibration , 0666);
		ret = sys_fchmodat(AT_FDCWD, GMA_Offset_TXT , 0666);

		if(retry++ != 0)
			break;
	}while(ret == -ENOENT);
	set_fs(fs);
	//dev_dbg(&s_gma->client->dev, "%s exit, retry=%d\n", __func__, retry);
	return 0;
ERR:
	return ret;
}
#endif	/*	#ifdef GMA_PERMISSION_THREAD	*/

static void gma30x_init_events(struct work_struct *work)
{
	int result = 0;

	result = gma30x_init(s_gma);
	if(result != 0){
		dev_err(&s_gma->client->dev, "<%s> init err !", __func__);
		return;
	}
	dev_dbg(&s_gma->client->dev, "gma30x init events end\n");
}

/*  Input device interface */
static int gma_acc_input_init(struct gma_data *gma){
	struct input_dev *dev;
	int err;
	//const char *path;
	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;
	
	dev = s_gma->pollDev->input;
	dev->name = INPUT_NAME_ACC; /* Setup Input Device Name  */
	dev->id.bustype = BUS_I2C;
	dev->evbit[0] = BIT_MASK(EV_ABS);
	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_capability(dev, EV_ABS, ABS_RUDDER);
	input_set_abs_params(dev, ABS_X, GMS_GMA30x_ABSMAX_16G, -GMS_GMA30x_ABSMAX_16G, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(dev, ABS_Y, GMS_GMA30x_ABSMAX_16G, -GMS_GMA30x_ABSMAX_16G, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(dev, ABS_Z, GMS_GMA30x_ABSMAX_16G, -GMS_GMA30x_ABSMAX_16G, INPUT_FUZZ, INPUT_FLAT);
	input_set_drvdata(dev, s_gma);
	/* Register */
/*	err = input_register_device(dev);
	if (err < 0){
		dev_err(&gma->client->dev, "input_register_device ERROR !!\n");
		input_free_device(dev);
		return err;
	}
	
	gma->input = dev;
*/	//path = kobject_get_path(&gma->input->dev.kobj, GFP_KERNEL);
	//dev_info(&gma->client->dev, "input_register_device SUCCESS %d !!  %s \n",err, path ? path : "N/A");
	//strlen(path)
	return 0;//err;
}
/*
 * I2C init/probing/exit functions
 */

static int gma_acc_probe(struct i2c_client *client, const struct i2c_device_id *id){
	int err;
	struct i2c_adapter *adapter;
 
	dev_info(&client->dev, "start probing.");
	adapter = to_i2c_adapter(client->dev.parent);
 	err = i2c_check_functionality(adapter,
 					 I2C_FUNC_SMBUS_BYTE |
 					 I2C_FUNC_SMBUS_BYTE_DATA);
	if (err < 0){
	    dev_err(&client->dev, "%s: check_functionality failed.", __func__);
		goto ERR0;
	}
	/* Setup private data */
	s_gma = kzalloc(sizeof(struct gma_data), GFP_KERNEL);
	//memset(s_gma, 0, sizeof(struct gma_data));
	if (!s_gma) {
		dev_err(&client->dev, "%s: memory allocation failed.", __func__);
		err = -ENOMEM;
		goto ERR1;
	}
	//gma_acc_set_data(s_gma);

	hwmon_dev = hwmon_device_register(&client->dev);
	assert(!(IS_ERR(hwmon_dev)));
	dev_info(&client->dev, "build time %s %s\n", __DATE__, __TIME__);

	/*input poll device register */
	s_gma->pollDev = input_allocate_polled_device();
	if (!s_gma->pollDev) {
		dev_err(&client->dev, "alloc poll device failed!\n");
		err = -ENOMEM;
		return err;
	}
	s_gma->pollDev->poll = gma_work_func;
	s_gma->pollDev->poll_interval = POLL_INTERVAL;
	s_gma->pollDev->poll_interval_max = POLL_INTERVAL_MAX;
	/* Setup input device interface */
	err = gma_acc_input_init(s_gma);
	if (err < 0){
	    dev_err(&client->dev, "%s: input_dev register failed", __func__);
		goto ERR2;
	}
	mutex_init(&s_gma->interval_mutex);
	mutex_init(&s_gma->init_mutex);
	atomic_set(&s_gma->position, GMS_GMA30x_DEFAULT_POSITION);
	atomic_set(&s_gma->enable, 0);
	atomic_set(&s_gma->addr, 0);
	atomic_set(&s_gma->calib, 0);
	err = input_register_polled_device(s_gma->pollDev);
	if (err) {
		dev_err(&client->dev, "register poll device failed!\n");
		return err;
	}
	//s_gma->pollDev->input->close(s_gma->pollDev->input);

	err = sysfs_create_group(&s_gma->pollDev->input->dev.kobj, &gma_acc_attribute_group);
	if(err) 
		dev_err(&client->dev, "create sys failed\n");

	s_gma->client  = client;
	//s_gma->pollDev = s_gma->pollDev;
	i2c_set_clientdata(client, s_gma);

	s_gma->gma30x_init_wq = create_singlethread_workqueue("gma_acc_init");
	if (s_gma->gma30x_init_wq == NULL) {
		dev_err(&client->dev, "create gma30x_init_wq fail!\n");
		return -ENOMEM;
	}

	/* Initialize the GMA30x chip */
	queue_work(s_gma->gma30x_init_wq, &gma30x_init_work);

	s_gma->gma30x_resume_wq = create_singlethread_workqueue("gma_acc_resume");
	if (s_gma->gma30x_resume_wq == NULL) {
		dev_err(&client->dev, "create gma30x_resume_wq fail!\n");
		return -ENOMEM;
	}
#ifdef GMA_PERMISSION_THREAD
	GMAPermissionThread = kthread_run(gma_permission_thread,"gma","Permissionthread");
	if(IS_ERR(GMAPermissionThread))
		GMAPermissionThread = NULL;
#endif // GMA_PERMISSION_THREAD
#ifdef CONFIG_HAS_EARLYSUSPEND
	s_gma->early_suspend.level = 0x02;
	s_gma->early_suspend.suspend = sensor_suspend;
	s_gma->early_suspend.resume = sensor_resume;
	register_early_suspend(&s_gma->early_suspend);
	s_gma->suspend_indator = 0;
#endif
	dev_info(&client->dev, "gma30x probe end\n");
	return err;
ERR2:
	kfree(s_gma);
ERR1:
ERR0:
	return err;
}

static int gma_acc_remove(struct i2c_client *client)
{
	int result;
	
	mutex_lock(&s_gma->init_mutex);
	s_gma->suspend_indator = 1;
	gma_sysfs_update_active_status(s_gma , 0);
	result = sensor_close_dev(s_gma);
	mutex_unlock(&s_gma->init_mutex);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&s_gma->early_suspend);
#endif
	cancel_work_sync(&gma30x_resume_work);
	destroy_workqueue(s_gma->gma30x_resume_wq);
	sysfs_remove_group(&s_gma->pollDev->input->dev.kobj, &gma_acc_attribute_group);
	s_gma->pollDev->input->close(s_gma->pollDev->input);
	input_unregister_polled_device(s_gma->pollDev);
	input_free_polled_device(s_gma->pollDev);
	//hwmon_device_unregister(hwmon_dev);
	cancel_work_sync(&gma30x_init_work);
	destroy_workqueue(s_gma->gma30x_init_wq);
	i2c_set_clientdata(client, NULL);

	return result;
}
static void gma30x_resume_events (struct work_struct *work)
{
#if defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_PM)
	int result = 0;
	dev_dbg(&s_gma->client->dev, "gma30x device init\n");
	//if (SUPER_STANDBY == standby_type)  //Needs restoration
		//queue_work(gma->gma30x_resume_wq, &gma30x_resume_work);

	//if (NORMAL_STANDBY == standby_type) { //Needs restoration
		mutex_lock(&s_gma->init_mutex);
		s_gma->suspend_indator = 0;
		result = gma30x_init(s_gma);
		mutex_unlock(&s_gma->init_mutex);
	s_gma->pollDev->input->open(s_gma->pollDev->input);
	
	dev_dbg(&s_gma->client->dev, "gma30x device init end\n");
	return;
#endif
}

static int sensor_close_dev(struct gma_data *gma){
	char buffer[2];
	buffer[0] = GMA1302_REG_PD;
	buffer[1] = GMA1302_MODE_POWERDOWN;
	gma_i2c_txdata(gma->client, buffer, 2);
	return 0;
}
#ifdef CONFIG_HAS_EARLYSUSPEND
static void sensor_suspend(struct early_suspend *h){
	struct gma_data *gma = 
			container_of(h, struct gma_data, early_suspend);
	dev_dbg(&gma->client->dev, "gma30x early suspend\n");
	//flush_workqueue(gma->gma30x_resume_wq);
	mutex_lock(&gma->init_mutex);
	s_gma->suspend_indator = 1;
	gma_sysfs_update_active_status(gma , 0);
	mutex_unlock(&gma->init_mutex);

	sensor_close_dev(s_gma);
	return;
}

static void sensor_resume(struct early_suspend *h){
	struct gma_data *gma = 
			container_of(h, struct gma_data, early_suspend);
	int en = atomic_read(&gma->enable);
	
	if (SUPER_STANDBY == standby_type)  //Needs restoration
		queue_work(gma->gma30x_resume_wq, &gma30x_resume_work);
	if (NORMAL_STANDBY == standby_type) { //Needs restoration	
		dev_dbg(&gma->client->dev, "gma30x late resume, gma->enable=%d\n", en);
		gma30x_init(gma);
		atomic_set(&gma->enable,en);
		gma->pollDev->input->open(gma->pollDev->input);
	}
	return;
}
#else
#ifdef CONFIG_PM
static int gma_acc_resume(struct i2c_client *client)
{
	struct gma_data *gma = i2c_get_clientdata(client);
	int result = 0;
	dprintk(DEBUG_SUSPEND, "gma30x resume\n");
	
	if (SUPER_STANDBY == standby_type) {
		queue_work(s_gma->gma30x_resume_wq, &gma30x_resume_work);
	}
	if (SUPER_STANDBY == standby_type) {
		mutex_lock(&gma->init_mutex);
		gma->suspend_indator = 0;
		result = gma30x_init(gma);
		mutex_unlock(&gma->init_mutex);
		assert(result==0);
		s_gma->pollDev->input->open(s_gma->pollDev->input);
	}
	return result;
}

static int gma_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct gma_data *gma = i2c_get_clientdata(client);
	int result;
	dprintk(DEBUG_SUSPEND, "gma30x suspend\n");

	flush_workqueue(s_gma->gma30x_resume_wq);
	
	s_gma->pollDev->input->close(s_gma->pollDev->input);

	mutex_lock(&gma->init_mutex);
	gma->suspend_indator = 1;
	gma_sysfs_update_active_status(gma , 0);
	sensor_close_dev(gma);
	mutex_unlock(&gma->init_mutex);
	assert(result==0);
	return result;
}
#endif
#endif /* CONFIG_HAS_EARLYSUSPEND */

static const struct i2c_device_id gma_i2c_ids[] = {
	{ GSENSOR_ID, 1 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, gma_i2c_ids);

static struct i2c_driver gma_i2c_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name	= GSENSOR_ID,
		.owner	= THIS_MODULE,
	},
	.probe	= gma_acc_probe,
	.remove	= gma_acc_remove,
#ifdef CONFIG_HAS_EARLYSUSPEND
#else
#ifdef CONFIG_PM
	.suspend = gma_acc_suspend,
	.resume = gma_acc_resume,
#endif
#endif
	.id_table = gma_i2c_ids,
	.detect = gsensor_detect,
	.address_list	= normal_i2c,
};

static int __init gma_acc_init(void){
	if (input_fetch_sysconfig_para(&(gsensor_info.input_type))) {
		GSE_LOG("%s: err.\n", __func__);
		return -1;
	} else
		twi_id = gsensor_info.twi_id;
	GSE_LOG("%s i2c twi is %d \n", __func__, twi_id);

	//GSE_LOG("GMA30x ACC driver: initialize.\n");
	return i2c_add_driver(&gma_i2c_driver);
}

static void __exit gma_acc_exit(void){
	i2c_del_driver(&gma_i2c_driver);
	GSE_LOG("GMA30x ACC driver: release.\n");
#ifdef GMA_PERMISSION_THREAD
	if(GMAPermissionThread)
		GMAPermissionThread = NULL;
#endif // GMA_PERMISSION_THREAD
}

MODULE_AUTHOR("Bruce <bruce.yang@globalmems.com>");
MODULE_DESCRIPTION("GMA302/GMA303 3-Axis Orientation/Motion Detection Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

module_init(gma_acc_init);
module_exit(gma_acc_exit);

