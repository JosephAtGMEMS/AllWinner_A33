/* 
 * Copyright (c) 2014 Globalmems, Inc.  All rights reserved.
 *
 * This source is subject to the Globalmems Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of Globalmems Inc.
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef GMA30X_H
#define GMA30X_H

/* Switch options */
#define GMA302	/* Enable flag use gma302 setup ,Disable use gma303 setup*/
#define sensorfusion_test	0	/* Default Disable.	Stage : Test not ok ,Open no effect */
//#define GMA_DEBUG_DATA		/* Default Disable.	1:Enable Gsensor debug data. */

//#define EWMA_FILTER				/* Default:Enable EWMA */
//#define SMA_FILTER				/* Enable or Disable SMA. Default:Enable */

#define AutoZeroZ	1	/* Default Enable.	Z asix AutoZero (GRAVITY_ON_Z AUTO) */
#define AutoZeroY	0	/* Default Disable.	Y asix AutoZero (GRAVITY_ON_Y AUTO) */
#define AutoZeroX	0	/* Default Disable.	X asix AutoZero (GRAVITY_ON_X AUTO) */

/* Exponentially weighted moving average (EWMA) */
#ifdef EWMA_FILTER
	#define EWMA_POSITIVE	10000	/* The values ​​are all positive */
	#define EWMA_FACTOR		1024	/* Magnification 2^10=1024，All values ​​<< 10 */
	#define EWMA_WEIGHT_X	2		/* 2(α=0.5)，4(α=0.25)，8(α=0.125)，16(α=0.0625) */
	#define EWMA_WEIGHT_Y	2		/* 2(α=0.5)，4(α=0.25)，8(α=0.125)，16(α=0.0625) */
	#define EWMA_WEIGHT_Z	4		/* 2(α=0.5)，4(α=0.25)，8(α=0.125)，16(α=0.0625) */
#endif
/* for Simple Moving Average (SMA) */
#ifdef SMA_FILTER	/* Simple Moving Average */
	#define SMA_AVG	4	/* AVG sensor data */
#endif

#define GSE_TAG	"(GMA 30x)"
#define GSE_LOG(fmt, args...)	printk(KERN_INFO GSE_TAG fmt, ##args)

#define INPUT_NAME_ACC		"gma30x"	/* Input Device Name  */
#ifdef GMA302
#define GSENSOR_ID			"gma302"    /* Device name for GMA302 misc. device */
#else
#define GSENSOR_ID			"gma303"    /* Device name for GMA303 misc. device */
#endif

#define SENSOR_I2C_ADDR		0x18
/* Registers */
#define GMA1302_REG_PID 	0x00
#define GMA1302_REG_PD 		0x01
#define GMA1302_REG_ACTR 	0x02
#define GMA1302_REG_MTHR 	0x03
#define GMA1302_REG_STADR 	0x04
#define GMA1302_REG_STATUS 	0x05
#define GMA1302_REG_DX	 	0x06
#define GMA1302_REG_INTCR 	0x15
#define GMA1302_REG_CONTR1 	0x16
#define GMA1302_REG_CONTR2 	0x17
#define GMA1302_REG_CONTR3 	0x18
#define GMA1302_REG_OSM	 	0x38

#define GMA1302_MODE_RESET			0x02
#define GMA1302_MODE_POWERDOWN		0x05

#define GMA302_VAL_WMI				0x02
#define GMA303_VAL_WMI				0x03
#define GMA305_VAL_WMI				0x05
#define GMA303_VAL_WMI_RD			0x33
#define GMA1302_VAL_OFFSET_TC_ON	0x40
#define GMA1302_VAL_DATA_READY_ON	0x2a
#define GMA1302_VAL_OFF				0x00
#define GMA1302_VAL_LPF_ON			0x09 /* low-pass filter on*/
#define GMA1302_VAL_HPF_ON			0x1b /* high-pass filter on*/
#define GMA1302_VAL_TRESHOLD_MAX	0x1F /* treshold set to max*/
#define GMA1302_VAL_LOW_NOISE		0x5F /* Oversampling low noise*/
/* GlobalMems Sensor Default parameters */
#define GMS_GMA30x_DEFAULT_POSITION	1		/* Default sensorlayout parameters ,it have option (1) (2) (3) (4) (-1) (-2) (-3) (-4) ref gma_axis_remap() */
/* Transformation matrix for chip mounting position 
first pin top:Positive  bottom:Negative 
1: top/upper-left	(-y,-x,-z)
2: top/upper-right	(-x, y,-z)
3: top/lower-right	( y,-x,-z)
4: top/lower-left	( x,-y,-z)
-2: bottom/upper-left  $Equal position 2	( x,  y, z)
-1: bottom/upper-right $Equal position 1	( y, -x, z)
-4: bottom/lower-right $Equal position 4	(-x, -y, z)
-3: bottom/lower-left  $Equal position 3	(-y, -x, z)
*/
#define GRAVITY_ON_Z_NEGATIVE	1
#define GRAVITY_ON_Z_POSITIVE	2
#define GRAVITY_ON_Y_NEGATIVE	3
#define GRAVITY_ON_Y_POSITIVE	4
#define GRAVITY_ON_X_NEGATIVE	5
#define GRAVITY_ON_X_POSITIVE	6
#define GRAVITY_ON_X_AUTO		7
#define GRAVITY_ON_Y_AUTO		8
#define GRAVITY_ON_Z_AUTO		9

#define AVG_NUM 				8	/* for calibration */
#define SENSOR_DATA_SIZE 		3 

/* ABS axes parameter range [um/s^2] (for input event) */
#define GMS_DEFAULT_SENSITIVITY 1024	/* raw data sensitivity 256(LSB)*4 */
#define GMS_GMA30x_ABSMAX_16G	(GMS_DEFAULT_SENSITIVITY * 16)
#define LevelValueRange_0_0078125	(GMS_DEFAULT_SENSITIVITY/128)/* Level conditions about 0.0078125g,ps: gma30x 1g=512(LSB) |X| < 2(LSB) && |Y| < 2(LSB)*/
#define LevelValueRange_0_015625	(GMS_DEFAULT_SENSITIVITY/64)/* Level conditions about 0.015625g	, 4(LSB)	*/
#define LevelValueRange_0_03125		(GMS_DEFAULT_SENSITIVITY/32)/* Level conditions about 0.03125g	, 8(LSB)	*/
#define LevelValueRange_0_0625		(GMS_DEFAULT_SENSITIVITY/16)/* Level conditions about 0.0625g	, 16(LSB)	*/
#define LevelValueRange_0_1			(GMS_DEFAULT_SENSITIVITY/10)/* Level conditions about 0.1000g	, 25(LSB)	*/
#define LevelValueRange_0_125		(GMS_DEFAULT_SENSITIVITY/8)	/* Level conditions about 0.1250g	, 32(LSB)	*/
#define LevelValueRange_0_25		(GMS_DEFAULT_SENSITIVITY/4)	/* Level conditions about 0.2500g	, 64(LSB)	*/
#define LevelValueRange_0_5			(GMS_DEFAULT_SENSITIVITY/2)	/* Level conditions about 0.5000g	, 128(LSB)	*/
#define LevelValueRange_2_0			(GMS_DEFAULT_SENSITIVITY*2)	/* Level conditions about 2.0000g	*/

#define GMA_IOCTL	0x99
#define GMA_IOCTL_READ              	_IOWR(GMA_IOCTL, 0x01, char*)					//Not yet implemented(for sensor fusion)
#define GMA_IOCTL_WRITE					_IOW(GMA_IOCTL, 0x02, char*)					//Not yet implemented(for sensor fusion)
#define GMA_IOCTL_GETDATA				_IOW(GMA_IOCTL, 0x03, char[SENSOR_DATA_SIZE])	//Not yet implemented(for sensor fusion)
#define GMA_IOCTL_RESET					_IO(GMA_IOCTL, 0x04)							//ACC sensor reset
#define GMA_IOCTL_CALIBRATION			_IOWR(GMA_IOCTL, 0x05, int[SENSOR_DATA_SIZE])	//ACC sensor Calibration
#define GMA_IOCTL_GET_OFFSET			_IOR(GMA_IOCTL, 0x06, int[SENSOR_DATA_SIZE])	//GET ACC sensor offset
#define GMA_IOCTL_SET_OFFSET			_IOWR(GMA_IOCTL, 0x07, int[SENSOR_DATA_SIZE])	//SET ACC sensor offset
#define GMA_IOCTL_READ_ACCEL_RAW_XYZ	_IOR(GMA_IOCTL, 0x08, int[SENSOR_DATA_SIZE])	//read sensor rawdata
#define GMA_IOCTL_READ_ACCEL_XYZ		_IOR(GMA_IOCTL, 0x09, int[SENSOR_DATA_SIZE])	//read input report data
#define GMA_IOCTL_SETYPR				_IOW(GMA_IOCTL, 0x0A, int[SENSOR_DATA_SIZE])	//Not yet implemented(for sensor fusion) set Yaw-Pitch-Roll
#define GMA_IOCTL_GET_OPEN_STATUS		_IO(GMA_IOCTL, 0x0B)							//get sensor open status(for sensor fusion)
#define GMA_IOCTL_GET_CLOSE_STATUS		_IO(GMA_IOCTL, 0x0C)							//get sensor close status for sensor fusion)
#define GMA_IOCTL_GET_DELAY				_IOR(GMA_IOCTL, 0x0D, unsigned int*)			//get sensor delay 
#define GMA_IOCTL_GET_LAYOUT			_IOR(GMA_IOCTL, 0x0E, int)						//get sensor layout(for sensor fusion)
#define GMA_IOCTL_GET_TEMPERATURE		_IOR(GMA_IOCTL, 0x0F, int)						//get sensor temperature(for sensor fusion)
#define SENSOR_MAXNR	16

typedef union {
	struct {
		int	x;
		int	y;
		int	z;
	} u;
	int	v[SENSOR_DATA_SIZE];
} raw_data;

struct gma_platform_data {
	int layout;
	//int gpio_INT1;
	int gpio_RSTN;
};

#define ACC_DATA_FLAG		0
#define MAG_DATA_FLAG		1
#define ORI_DATA_FLAG		2
#define DMT_NUM_SENSORS		3

/* Wait timeout in millisecond */
#define GMA301_DRDY_TIMEOUT	100
#define ABS(a) ((a) < 0 ? -(a) : (a))

/**
* ewma_read() - Get average value
* @avg: Average structure
*
* Returns the average value held in @avg.
*/
/*struct ewma {
	unsigned long internal;
	unsigned long factor;
	unsigned long weight;
};

static inline unsigned long ewma_read(const struct ewma *avg)
{
	return avg->internal >> avg->factor;
}

static void ewma_init(struct ewma *avg, unsigned long factor, unsigned long weight)
{
	WARN_ON(!is_power_of_2(weight) || !is_power_of_2(factor));

	avg->weight = ilog2(weight);
	avg->factor = ilog2(factor);
	avg->internal = 0;
}

static struct ewma *ewma_add(struct ewma *avg, unsigned long val)
{
	unsigned long internal = ACCESS_ONCE(avg->internal);

	ACCESS_ONCE(avg->internal) = internal ?
	(((internal << avg->weight) - internal) + (val << avg->factor)) >> avg->weight :
	(val << avg->factor);
	return avg;
}
*/
#endif               
