/*  Date: 2011/11/02 17:00:00
 *  Revision: 2.9
 */

/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */


/* file bma253_bosch.c
   brief This file contains all function implementations for the BMA253 in linux

 */

#undef CONFIG_HAS_EARLYSUSPEND

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/wakelock.h>
#include <linux/of_gpio.h>

#include <linux/bma253.h>

//#include <mach/rpm-regulator.h>
#include <linux/regulator/consumer.h>

#define IS_MTK_PLATFORM 1

#ifdef IS_MTK_PLATFORM
#include <mach/mt_gpio.h>
#include <mach/eint.h>
#include <mach/mt_pm_ldo.h>
#include <linux/of_irq.h>
#endif

#define D(x...) printk(KERN_DEBUG "[GSNR][BMA253_BOSCH] " x)
#define I(x...) printk(KERN_INFO "[GSNR][BMA253_BOSCH] " x)
#define E(x...) printk(KERN_ERR "[GSNR][BMA253_BOSCH] " x)

#define HTC_ATTR 1
#define I2C_RETRY_COUNT		10

#define BMA253_USER_CALI_GRAVITY_EARTH (980)

#define CALIBRATION_DATA_PATH "/calibration_data"
#define G_SENSOR_FLASH_DATA "gs_flash"

static unsigned long debug_execute_point;
struct workqueue_struct *bma_wq;
static void debug_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(debug_work, debug_do_work);

struct bma253acc{
	s16	x,
		y,
		z;
} ;

static void bma253_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(polling_work, bma253_work_func);

#ifdef CONFIG_CIR_ALWAYS_READY
static void bma253_irq_work_func(struct work_struct *work);
static DECLARE_WORK(irq_work, bma253_irq_work_func);
#endif

struct bma253_data {
	struct i2c_client *bma253_client;
	atomic_t delay;
	atomic_t enable;
	atomic_t selftest_result;
	unsigned char mode;
	struct input_dev *input;
#ifdef CONFIG_CIR_ALWAYS_READY
	struct input_dev *input_cir;
	struct wake_lock cir_always_ready_wake_lock;
#endif
	struct bma253acc value;
	struct bma253acc cali_value;
	struct mutex value_mutex;
	struct mutex enable_mutex;
	struct mutex mode_mutex;
	struct delayed_work work;
	struct work_struct irq_work;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	int IRQ;
	int chip_layout;
#ifdef HTC_ATTR
	struct class *g_sensor_class;
	struct device *g_sensor_dev;
	struct device *g_sensor_user_cali_dev;
#endif /* HTC_ATTR */

	struct bma253_platform_data *pdata;
	short offset_buf[3];
	short user_offset_buf[3];
	int user_calibration_flag;

#ifdef IS_MTK_PLATFORM
	int nvram_gs_kvalue;
#endif

#ifndef IS_MTK_PLATFORM
	struct regulator	*sr_1v8;
	struct regulator	*sr_2v85;
#endif

	struct workqueue_struct *bma253_wq;
};

static struct bma253_data *gdata;

#ifdef CONFIG_CIR_ALWAYS_READY
#define BMA253_ENABLE_INT1 1
static int cir_flag = 0;
static int power_key_pressed = 0;
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
static void bma253_early_suspend(struct early_suspend *h);
static void bma253_late_resume(struct early_suspend *h);
#endif

static int bma253_smbus_read_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	if (dummy < 0)
		return -1;
	*data = dummy & 0x000000ff;

	return 0;
}

static int bma253_smbus_write_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	if (dummy < 0)
		return -1;
	return 0;
}

static int bma253_smbus_read_byte_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	s32 dummy;
	dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
	if (dummy < 0)
		return -1;
	return 0;
}

static int bma253_set_mode(struct i2c_client *client, unsigned char Mode)
{
	int comres = 0;
	unsigned char data1;

#ifdef CONFIG_CIR_ALWAYS_READY
	if(cir_flag && Mode == BMA253_MODE_SUSPEND) {
	    return 0;
	} else {
#endif

	if (Mode < 3) {
		comres = bma253_smbus_read_byte(client,
				BMA253_EN_LOW_POWER__REG, &data1);
		switch (Mode) {
		case BMA253_MODE_NORMAL:
			data1  = BMA253_SET_BITSLICE(data1,
					BMA253_EN_LOW_POWER, 0);
			data1  = BMA253_SET_BITSLICE(data1,
					BMA253_EN_SUSPEND, 0);
			break;
		case BMA253_MODE_LOWPOWER:
			data1  = BMA253_SET_BITSLICE(data1,
					BMA253_EN_LOW_POWER, 1);
			data1  = BMA253_SET_BITSLICE(data1,
					BMA253_EN_SUSPEND, 0);
			break;
		case BMA253_MODE_SUSPEND:
			data1  = BMA253_SET_BITSLICE(data1,
					BMA253_EN_LOW_POWER, 0);
			data1  = BMA253_SET_BITSLICE(data1,
					BMA253_EN_SUSPEND, 1);
			break;
		default:
			break;
		}

		comres += bma253_smbus_write_byte(client,
				BMA253_EN_LOW_POWER__REG, &data1);
	} else{
		comres = -1;
	}
#ifdef CONFIG_CIR_ALWAYS_READY
	}
#endif


	return comres;
}
#ifdef BMA253_ENABLE_INT1
static int bma253_set_int1_pad_sel(struct i2c_client *client, unsigned char
		int1sel)
{
	int comres = 0;
	unsigned char data;
	unsigned char state;
	state = 0x01;


	switch (int1sel) {
	case 0:
		comres = bma253_smbus_read_byte(client,
				BMA253_EN_INT1_PAD_LOWG__REG, &data);
		data = BMA253_SET_BITSLICE(data, BMA253_EN_INT1_PAD_LOWG,
				state);
		comres = bma253_smbus_write_byte(client,
				BMA253_EN_INT1_PAD_LOWG__REG, &data);
		break;
	case 1:
		comres = bma253_smbus_read_byte(client,
				BMA253_EN_INT1_PAD_HIGHG__REG, &data);
		data = BMA253_SET_BITSLICE(data, BMA253_EN_INT1_PAD_HIGHG,
				state);
		comres = bma253_smbus_write_byte(client,
				BMA253_EN_INT1_PAD_HIGHG__REG, &data);
		break;
	case 2:
		comres = bma253_smbus_read_byte(client,
				BMA253_EN_INT1_PAD_SLOPE__REG, &data);
		data = BMA253_SET_BITSLICE(data, BMA253_EN_INT1_PAD_SLOPE,
				state);
		comres = bma253_smbus_write_byte(client,
				BMA253_EN_INT1_PAD_SLOPE__REG, &data);
		break;
	case 3:
		comres = bma253_smbus_read_byte(client,
				BMA253_EN_INT1_PAD_DB_TAP__REG, &data);
		data = BMA253_SET_BITSLICE(data, BMA253_EN_INT1_PAD_DB_TAP,
				state);
		comres = bma253_smbus_write_byte(client,
				BMA253_EN_INT1_PAD_DB_TAP__REG, &data);
		break;
	case 4:
		comres = bma253_smbus_read_byte(client,
				BMA253_EN_INT1_PAD_SNG_TAP__REG, &data);
		data = BMA253_SET_BITSLICE(data, BMA253_EN_INT1_PAD_SNG_TAP,
				state);
		comres = bma253_smbus_write_byte(client,
				BMA253_EN_INT1_PAD_SNG_TAP__REG, &data);
		break;
	case 5:
		comres = bma253_smbus_read_byte(client,
				BMA253_EN_INT1_PAD_ORIENT__REG, &data);
		data = BMA253_SET_BITSLICE(data, BMA253_EN_INT1_PAD_ORIENT,
				state);
		comres = bma253_smbus_write_byte(client,
				BMA253_EN_INT1_PAD_ORIENT__REG, &data);
		break;
	case 6:
		comres = bma253_smbus_read_byte(client,
				BMA253_EN_INT1_PAD_FLAT__REG, &data);
		data = BMA253_SET_BITSLICE(data, BMA253_EN_INT1_PAD_FLAT,
				state);
		comres = bma253_smbus_write_byte(client,
				BMA253_EN_INT1_PAD_FLAT__REG, &data);
		break;
	default:
		break;
	}

	return comres;
}
#endif /* BMA253_ENABLE_INT1 */
#ifdef BMA253_ENABLE_INT2
static int bma253_set_int2_pad_sel(struct i2c_client *client, unsigned char
		int2sel)
{
	int comres = 0;
	unsigned char data;
	unsigned char state;
	state = 0x01;


	switch (int2sel) {
	case 0:
		comres = bma253_smbus_read_byte(client,
				BMA253_EN_INT2_PAD_LOWG__REG, &data);
		data = BMA253_SET_BITSLICE(data, BMA253_EN_INT2_PAD_LOWG,
				state);
		comres = bma253_smbus_write_byte(client,
				BMA253_EN_INT2_PAD_LOWG__REG, &data);
		break;
	case 1:
		comres = bma253_smbus_read_byte(client,
				BMA253_EN_INT2_PAD_HIGHG__REG, &data);
		data = BMA253_SET_BITSLICE(data, BMA253_EN_INT2_PAD_HIGHG,
				state);
		comres = bma253_smbus_write_byte(client,
				BMA253_EN_INT2_PAD_HIGHG__REG, &data);
		break;
	case 2:
		comres = bma253_smbus_read_byte(client,
				BMA253_EN_INT2_PAD_SLOPE__REG, &data);
		data = BMA253_SET_BITSLICE(data, BMA253_EN_INT2_PAD_SLOPE,
				state);
		comres = bma253_smbus_write_byte(client,
				BMA253_EN_INT2_PAD_SLOPE__REG, &data);
		break;
	case 3:
		comres = bma253_smbus_read_byte(client,
				BMA253_EN_INT2_PAD_DB_TAP__REG, &data);
		data = BMA253_SET_BITSLICE(data, BMA253_EN_INT2_PAD_DB_TAP,
				state);
		comres = bma253_smbus_write_byte(client,
				BMA253_EN_INT2_PAD_DB_TAP__REG, &data);
		break;
	case 4:
		comres = bma253_smbus_read_byte(client,
				BMA253_EN_INT2_PAD_SNG_TAP__REG, &data);
		data = BMA253_SET_BITSLICE(data, BMA253_EN_INT2_PAD_SNG_TAP,
				state);
		comres = bma253_smbus_write_byte(client,
				BMA253_EN_INT2_PAD_SNG_TAP__REG, &data);
		break;
	case 5:
		comres = bma253_smbus_read_byte(client,
				BMA253_EN_INT2_PAD_ORIENT__REG, &data);
		data = BMA253_SET_BITSLICE(data, BMA253_EN_INT2_PAD_ORIENT,
				state);
		comres = bma253_smbus_write_byte(client,
				BMA253_EN_INT2_PAD_ORIENT__REG, &data);
		break;
	case 6:
		comres = bma253_smbus_read_byte(client,
				BMA253_EN_INT2_PAD_FLAT__REG, &data);
		data = BMA253_SET_BITSLICE(data, BMA253_EN_INT2_PAD_FLAT,
				state);
		comres = bma253_smbus_write_byte(client,
				BMA253_EN_INT2_PAD_FLAT__REG, &data);
		break;
	default:
		break;
	}

	return comres;
}
#endif /* BMA253_ENABLE_INT2 */

static int bma253_set_Int_Enable(struct i2c_client *client, unsigned char
		InterruptType , unsigned char value)
{
	int comres = 0;
	unsigned char data1, data2;


	comres = bma253_smbus_read_byte(client, BMA253_INT_ENABLE1_REG, &data1);
	comres = bma253_smbus_read_byte(client, BMA253_INT_ENABLE2_REG, &data2);

	value = value & 1;
	switch (InterruptType) {
	case 0:
		/* Low G Interrupt  */
		data2 = BMA253_SET_BITSLICE(data2, BMA253_EN_LOWG_INT, value);
		break;
	case 1:
		/* High G X Interrupt */

		data2 = BMA253_SET_BITSLICE(data2, BMA253_EN_HIGHG_X_INT,
				value);
		break;
	case 2:
		/* High G Y Interrupt */

		data2 = BMA253_SET_BITSLICE(data2, BMA253_EN_HIGHG_Y_INT,
				value);
		break;
	case 3:
		/* High G Z Interrupt */

		data2 = BMA253_SET_BITSLICE(data2, BMA253_EN_HIGHG_Z_INT,
				value);
		break;
	case 4:
		/* New Data Interrupt  */

		data2 = BMA253_SET_BITSLICE(data2, BMA253_EN_NEW_DATA_INT,
				value);
		break;
	case 5:
		/* Slope X Interrupt */

		data1 = BMA253_SET_BITSLICE(data1, BMA253_EN_SLOPE_X_INT,
				value);
		break;
	case 6:
		/* Slope Y Interrupt */

		data1 = BMA253_SET_BITSLICE(data1, BMA253_EN_SLOPE_Y_INT,
				value);
		break;
	case 7:
		/* Slope Z Interrupt */

		data1 = BMA253_SET_BITSLICE(data1, BMA253_EN_SLOPE_Z_INT,
				value);
		break;
	case 8:
		/* Single Tap Interrupt */

		data1 = BMA253_SET_BITSLICE(data1, BMA253_EN_SINGLE_TAP_INT,
				value);
		break;
	case 9:
		/* Double Tap Interrupt */

		data1 = BMA253_SET_BITSLICE(data1, BMA253_EN_DOUBLE_TAP_INT,
				value);
		break;
	case 10:
		/* Orient Interrupt  */

		data1 = BMA253_SET_BITSLICE(data1, BMA253_EN_ORIENT_INT, value);
		break;
	case 11:
		/* Flat Interrupt */

		data1 = BMA253_SET_BITSLICE(data1, BMA253_EN_FLAT_INT, value);
		break;
	default:
		break;
	}
	comres = bma253_smbus_write_byte(client, BMA253_INT_ENABLE1_REG,
			&data1);
	comres = bma253_smbus_write_byte(client, BMA253_INT_ENABLE2_REG,
			&data2);

	return comres;
}


static int bma253_get_mode(struct i2c_client *client, unsigned char *Mode)
{
	int comres = 0;


	comres = bma253_smbus_read_byte(client,
			BMA253_EN_LOW_POWER__REG, Mode);
	*Mode  = (*Mode) >> 6;


	return comres;
}

static int bma253_set_range(struct i2c_client *client, unsigned char Range)
{
	int comres = 0;
	unsigned char data1;


	if (Range < 4) {
		comres = bma253_smbus_read_byte(client,
				BMA253_RANGE_SEL_REG, &data1);
		switch (Range) {
		case 0:
			data1  = BMA253_SET_BITSLICE(data1,
					BMA253_RANGE_SEL, 0);
			break;
		case 1:
			data1  = BMA253_SET_BITSLICE(data1,
					BMA253_RANGE_SEL, 5);
			break;
		case 2:
			data1  = BMA253_SET_BITSLICE(data1,
					BMA253_RANGE_SEL, 8);
			break;
		case 3:
			data1  = BMA253_SET_BITSLICE(data1,
					BMA253_RANGE_SEL, 12);
			break;
		default:
			break;
		}
		comres += bma253_smbus_write_byte(client,
				BMA253_RANGE_SEL_REG, &data1);
	} else{
		comres = -1;
	}


	return comres;
}

static int bma253_get_range(struct i2c_client *client, unsigned char *Range)
{
	int comres = 0;
	unsigned char data;


	comres = bma253_smbus_read_byte(client, BMA253_RANGE_SEL__REG,
			&data);
	data = BMA253_GET_BITSLICE(data, BMA253_RANGE_SEL);
	*Range = data;


	return comres;
}


static int bma253_set_bandwidth(struct i2c_client *client, unsigned char BW)
{
	int comres = 0;
	unsigned char data;
	int Bandwidth = 0;

	if (BW > 7 && BW < 16) {
		switch (BW) {
		case 8:
			Bandwidth = BMA253_BW_7_81HZ;
			break;
		case 9:
			Bandwidth = BMA253_BW_15_63HZ;
			break;
		case 10:
			Bandwidth = BMA253_BW_31_25HZ;
			break;
		case 11:
			Bandwidth = BMA253_BW_62_50HZ;
			break;
		case 12:
			Bandwidth = BMA253_BW_125HZ;
			break;
		case 13:
			Bandwidth = BMA253_BW_250HZ;
			break;
		case 14:
			Bandwidth = BMA253_BW_500HZ;
			break;
		case 15:
			Bandwidth = BMA253_BW_1000HZ;
			break;
		default:
			break;
		}
		comres = bma253_smbus_read_byte(client,
				BMA253_BANDWIDTH__REG, &data);
		data = BMA253_SET_BITSLICE(data, BMA253_BANDWIDTH,
				Bandwidth);
		comres += bma253_smbus_write_byte(client,
				BMA253_BANDWIDTH__REG, &data);
	} else{
		comres = -1;
	}


	return comres;
}

static int bma253_get_bandwidth(struct i2c_client *client, unsigned char *BW)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma253_smbus_read_byte(client, BMA253_BANDWIDTH__REG, &data);
	data = BMA253_GET_BITSLICE(data, BMA253_BANDWIDTH);
	*BW = data;

	return comres;
}

#if defined(BMA253_ENABLE_INT1) || defined(BMA253_ENABLE_INT2)
static int bma253_get_interruptstatus1(struct i2c_client *client, unsigned char
		*intstatus)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_STATUS1_REG, &data);
	*intstatus = data;

	return comres;
}

#if 0
static int bma253_get_HIGH_first(struct i2c_client *client, unsigned char
						param, unsigned char *intstatus)
{
	int comres = 0;
	unsigned char data;

	switch (param) {
	case 0:
		comres = bma253_smbus_read_byte(client,
				BMA253_STATUS_ORIENT_HIGH_REG, &data);
		data = BMA253_GET_BITSLICE(data, BMA253_HIGHG_FIRST_X);
		*intstatus = data;
		break;
	case 1:
		comres = bma253_smbus_read_byte(client,
				BMA253_STATUS_ORIENT_HIGH_REG, &data);
		data = BMA253_GET_BITSLICE(data, BMA253_HIGHG_FIRST_Y);
		*intstatus = data;
		break;
	case 2:
		comres = bma253_smbus_read_byte(client,
				BMA253_STATUS_ORIENT_HIGH_REG, &data);
		data = BMA253_GET_BITSLICE(data, BMA253_HIGHG_FIRST_Z);
		*intstatus = data;
		break;
	default:
		break;
	}

	return comres;
}

static int bma253_get_HIGH_sign(struct i2c_client *client, unsigned char
		*intstatus)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_STATUS_ORIENT_HIGH_REG,
			&data);
	data = BMA253_GET_BITSLICE(data, BMA253_HIGHG_SIGN_S);
	*intstatus = data;

	return comres;
}

static int bma253_get_slope_first(struct i2c_client *client, unsigned char
	param, unsigned char *intstatus)
{
	int comres = 0;
	unsigned char data;

	switch (param) {
	case 0:
		comres = bma253_smbus_read_byte(client,
				BMA253_STATUS_TAP_SLOPE_REG, &data);
		data = BMA253_GET_BITSLICE(data, BMA253_SLOPE_FIRST_X);
		*intstatus = data;
		break;
	case 1:
		comres = bma253_smbus_read_byte(client,
				BMA253_STATUS_TAP_SLOPE_REG, &data);
		data = BMA253_GET_BITSLICE(data, BMA253_SLOPE_FIRST_Y);
		*intstatus = data;
		break;
	case 2:
		comres = bma253_smbus_read_byte(client,
				BMA253_STATUS_TAP_SLOPE_REG, &data);
		data = BMA253_GET_BITSLICE(data, BMA253_SLOPE_FIRST_Z);
		*intstatus = data;
		break;
	default:
		break;
	}

	return comres;
}
static int bma253_get_slope_sign(struct i2c_client *client, unsigned char
		*intstatus)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_STATUS_TAP_SLOPE_REG,
			&data);
	data = BMA253_GET_BITSLICE(data, BMA253_SLOPE_SIGN_S);
	*intstatus = data;

	return comres;
}
static int bma253_get_orient_status(struct i2c_client *client, unsigned char
		*intstatus)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_STATUS_ORIENT_HIGH_REG,
			&data);
	data = BMA253_GET_BITSLICE(data, BMA253_ORIENT_S);
	*intstatus = data;

	return comres;
}

static int bma253_get_orient_flat_status(struct i2c_client *client, unsigned
		char *intstatus)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_STATUS_ORIENT_HIGH_REG,
			&data);
	data = BMA253_GET_BITSLICE(data, BMA253_FLAT_S);
	*intstatus = data;

	return comres;
}
#endif
#endif /* defined(BMA253_ENABLE_INT1)||defined(BMA253_ENABLE_INT2) */
static int bma253_set_Int_Mode(struct i2c_client *client, unsigned char Mode)
{
	int comres = 0;
	unsigned char data;


	comres = bma253_smbus_read_byte(client,
			BMA253_INT_MODE_SEL__REG, &data);
	data = BMA253_SET_BITSLICE(data, BMA253_INT_MODE_SEL, Mode);
	comres = bma253_smbus_write_byte(client,
			BMA253_INT_MODE_SEL__REG, &data);


	return comres;
}

static int bma253_get_Int_Mode(struct i2c_client *client, unsigned char *Mode)
{
	int comres = 0;
	unsigned char data;


	comres = bma253_smbus_read_byte(client,
			BMA253_INT_MODE_SEL__REG, &data);
	data  = BMA253_GET_BITSLICE(data, BMA253_INT_MODE_SEL);
	*Mode = data;


	return comres;
}
static int bma253_set_slope_duration(struct i2c_client *client, unsigned char
		duration)
{
	int comres = 0;
	unsigned char data;


	comres = bma253_smbus_read_byte(client,
			BMA253_SLOPE_DUR__REG, &data);
	data = BMA253_SET_BITSLICE(data, BMA253_SLOPE_DUR, duration);
	comres = bma253_smbus_write_byte(client,
			BMA253_SLOPE_DUR__REG, &data);


	return comres;
}

static int bma253_get_slope_duration(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;


	comres = bma253_smbus_read_byte(client,
			BMA253_SLOPE_DURN_REG, &data);
	data = BMA253_GET_BITSLICE(data, BMA253_SLOPE_DUR);
	*status = data;


	return comres;
}

static int bma253_set_slope_threshold(struct i2c_client *client,
		unsigned char threshold)
{
	int comres = 0;
	unsigned char data;


	comres = bma253_smbus_read_byte(client,
			BMA253_SLOPE_THRES__REG, &data);
	data = BMA253_SET_BITSLICE(data, BMA253_SLOPE_THRES, threshold);
	comres = bma253_smbus_write_byte(client,
			BMA253_SLOPE_THRES__REG, &data);


	return comres;
}

static int bma253_get_slope_threshold(struct i2c_client *client,
		unsigned char *status)
{
	int comres = 0;
	unsigned char data;


	comres = bma253_smbus_read_byte(client,
			BMA253_SLOPE_THRES_REG, &data);
	data = BMA253_GET_BITSLICE(data, BMA253_SLOPE_THRES);
	*status = data;


	return comres;
}
static int bma253_set_low_g_duration(struct i2c_client *client, unsigned char
		duration)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_LOWG_DUR__REG, &data);
	data = BMA253_SET_BITSLICE(data, BMA253_LOWG_DUR, duration);
	comres = bma253_smbus_write_byte(client, BMA253_LOWG_DUR__REG, &data);

	return comres;
}

static int bma253_get_low_g_duration(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_LOW_DURN_REG, &data);
	data = BMA253_GET_BITSLICE(data, BMA253_LOWG_DUR);
	*status = data;

	return comres;
}

static int bma253_set_low_g_threshold(struct i2c_client *client, unsigned char
		threshold)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_LOWG_THRES__REG, &data);
	data = BMA253_SET_BITSLICE(data, BMA253_LOWG_THRES, threshold);
	comres = bma253_smbus_write_byte(client, BMA253_LOWG_THRES__REG, &data);

	return comres;
}

static int bma253_get_low_g_threshold(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_LOW_THRES_REG, &data);
	data = BMA253_GET_BITSLICE(data, BMA253_LOWG_THRES);
	*status = data;

	return comres;
}

static int bma253_set_high_g_duration(struct i2c_client *client, unsigned char
		duration)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_HIGHG_DUR__REG, &data);
	data = BMA253_SET_BITSLICE(data, BMA253_HIGHG_DUR, duration);
	comres = bma253_smbus_write_byte(client, BMA253_HIGHG_DUR__REG, &data);

	return comres;
}

static int bma253_get_high_g_duration(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_HIGH_DURN_REG, &data);
	data = BMA253_GET_BITSLICE(data, BMA253_HIGHG_DUR);
	*status = data;

	return comres;
}

static int bma253_set_high_g_threshold(struct i2c_client *client, unsigned char
		threshold)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_HIGHG_THRES__REG, &data);
	data = BMA253_SET_BITSLICE(data, BMA253_HIGHG_THRES, threshold);
	comres = bma253_smbus_write_byte(client, BMA253_HIGHG_THRES__REG,
			&data);

	return comres;
}

static int bma253_get_high_g_threshold(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_HIGH_THRES_REG, &data);
	data = BMA253_GET_BITSLICE(data, BMA253_HIGHG_THRES);
	*status = data;

	return comres;
}


static int bma253_set_tap_duration(struct i2c_client *client, unsigned char
		duration)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_TAP_DUR__REG, &data);
	data = BMA253_SET_BITSLICE(data, BMA253_TAP_DUR, duration);
	comres = bma253_smbus_write_byte(client, BMA253_TAP_DUR__REG, &data);

	return comres;
}

static int bma253_get_tap_duration(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_TAP_PARAM_REG, &data);
	data = BMA253_GET_BITSLICE(data, BMA253_TAP_DUR);
	*status = data;

	return comres;
}

static int bma253_set_tap_shock(struct i2c_client *client, unsigned char setval)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_TAP_SHOCK_DURN__REG,
			&data);
	data = BMA253_SET_BITSLICE(data, BMA253_TAP_SHOCK_DURN, setval);
	comres = bma253_smbus_write_byte(client, BMA253_TAP_SHOCK_DURN__REG,
			&data);

	return comres;
}

static int bma253_get_tap_shock(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_TAP_PARAM_REG, &data);
	data = BMA253_GET_BITSLICE(data, BMA253_TAP_SHOCK_DURN);
	*status = data;

	return comres;
}

static int bma253_set_tap_quiet(struct i2c_client *client, unsigned char
		duration)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_TAP_QUIET_DURN__REG,
			&data);
	data = BMA253_SET_BITSLICE(data, BMA253_TAP_QUIET_DURN, duration);
	comres = bma253_smbus_write_byte(client, BMA253_TAP_QUIET_DURN__REG,
			&data);

	return comres;
}

static int bma253_get_tap_quiet(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_TAP_PARAM_REG, &data);
	data = BMA253_GET_BITSLICE(data, BMA253_TAP_QUIET_DURN);
	*status = data;

	return comres;
}

static int bma253_set_tap_threshold(struct i2c_client *client, unsigned char
		threshold)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_TAP_THRES__REG, &data);
	data = BMA253_SET_BITSLICE(data, BMA253_TAP_THRES, threshold);
	comres = bma253_smbus_write_byte(client, BMA253_TAP_THRES__REG, &data);

	return comres;
}

static int bma253_get_tap_threshold(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_TAP_THRES_REG, &data);
	data = BMA253_GET_BITSLICE(data, BMA253_TAP_THRES);
	*status = data;

	return comres;
}

static int bma253_set_tap_samp(struct i2c_client *client, unsigned char samp)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_TAP_SAMPLES__REG, &data);
	data = BMA253_SET_BITSLICE(data, BMA253_TAP_SAMPLES, samp);
	comres = bma253_smbus_write_byte(client, BMA253_TAP_SAMPLES__REG,
			&data);

	return comres;
}

static int bma253_get_tap_samp(struct i2c_client *client, unsigned char *status)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_TAP_THRES_REG, &data);
	data = BMA253_GET_BITSLICE(data, BMA253_TAP_SAMPLES);
	*status = data;

	return comres;
}

static int bma253_set_orient_mode(struct i2c_client *client, unsigned char mode)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_ORIENT_MODE__REG, &data);
	data = BMA253_SET_BITSLICE(data, BMA253_ORIENT_MODE, mode);
	comres = bma253_smbus_write_byte(client, BMA253_ORIENT_MODE__REG,
			&data);

	return comres;
}

static int bma253_get_orient_mode(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_ORIENT_PARAM_REG, &data);
	data = BMA253_GET_BITSLICE(data, BMA253_ORIENT_MODE);
	*status = data;

	return comres;
}

static int bma253_set_orient_blocking(struct i2c_client *client, unsigned char
		samp)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_ORIENT_BLOCK__REG,
			&data);
	data = BMA253_SET_BITSLICE(data, BMA253_ORIENT_BLOCK, samp);
	comres = bma253_smbus_write_byte(client, BMA253_ORIENT_BLOCK__REG,
			&data);

	return comres;
}

static int bma253_get_orient_blocking(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_ORIENT_PARAM_REG, &data);
	data = BMA253_GET_BITSLICE(data, BMA253_ORIENT_BLOCK);
	*status = data;

	return comres;
}

static int bma253_set_orient_hyst(struct i2c_client *client, unsigned char
		orienthyst)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_ORIENT_HYST__REG, &data);
	data = BMA253_SET_BITSLICE(data, BMA253_ORIENT_HYST, orienthyst);
	comres = bma253_smbus_write_byte(client, BMA253_ORIENT_HYST__REG,
			&data);

	return comres;
}

static int bma253_get_orient_hyst(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_ORIENT_PARAM_REG, &data);
	data = BMA253_GET_BITSLICE(data, BMA253_ORIENT_HYST);
	*status = data;

	return comres;
}
static int bma253_set_theta_blocking(struct i2c_client *client, unsigned char
		thetablk)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_THETA_BLOCK__REG, &data);
	data = BMA253_SET_BITSLICE(data, BMA253_THETA_BLOCK, thetablk);
	comres = bma253_smbus_write_byte(client, BMA253_THETA_BLOCK__REG,
			&data);

	return comres;
}

static int bma253_get_theta_blocking(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_THETA_BLOCK_REG, &data);
	data = BMA253_GET_BITSLICE(data, BMA253_THETA_BLOCK);
	*status = data;

	return comres;
}

static int bma253_set_theta_flat(struct i2c_client *client, unsigned char
		thetaflat)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_THETA_FLAT__REG, &data);
	data = BMA253_SET_BITSLICE(data, BMA253_THETA_FLAT, thetaflat);
	comres = bma253_smbus_write_byte(client, BMA253_THETA_FLAT__REG, &data);

	return comres;
}

static int bma253_get_theta_flat(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0 ;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_THETA_FLAT_REG, &data);
	data = BMA253_GET_BITSLICE(data, BMA253_THETA_FLAT);
	*status = data;

	return comres;
}

static int bma253_set_flat_hold_time(struct i2c_client *client, unsigned char
		holdtime)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_FLAT_HOLD_TIME__REG,
			&data);
	data = BMA253_SET_BITSLICE(data, BMA253_FLAT_HOLD_TIME, holdtime);
	comres = bma253_smbus_write_byte(client, BMA253_FLAT_HOLD_TIME__REG,
			&data);

	return comres;
}

static int bma253_get_flat_hold_time(struct i2c_client *client, unsigned char
		*holdtime)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_FLAT_HOLD_TIME_REG,
			&data);
	data  = BMA253_GET_BITSLICE(data, BMA253_FLAT_HOLD_TIME);
	*holdtime = data ;

	return comres;
}

static int bma253_write_reg(struct i2c_client *client, unsigned char addr,
		unsigned char *data)
{
	int comres = 0;
	comres = bma253_smbus_write_byte(client, addr, data);

	return comres;
}


static int bma253_set_offset_target_x(struct i2c_client *client, unsigned char
		offsettarget)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client,
			BMA253_COMP_TARGET_OFFSET_X__REG, &data);
	data = BMA253_SET_BITSLICE(data, BMA253_COMP_TARGET_OFFSET_X,
			offsettarget);
	comres = bma253_smbus_write_byte(client,
			BMA253_COMP_TARGET_OFFSET_X__REG, &data);

	return comres;
}

static int bma253_get_offset_target_x(struct i2c_client *client, unsigned char
		*offsettarget)
{
	int comres = 0 ;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_OFFSET_PARAMS_REG,
			&data);
	data = BMA253_GET_BITSLICE(data, BMA253_COMP_TARGET_OFFSET_X);
	*offsettarget = data;

	return comres;
}

static int bma253_set_offset_target_y(struct i2c_client *client, unsigned char
		offsettarget)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client,
			BMA253_COMP_TARGET_OFFSET_Y__REG, &data);
	data = BMA253_SET_BITSLICE(data, BMA253_COMP_TARGET_OFFSET_Y,
			offsettarget);
	comres = bma253_smbus_write_byte(client,
			BMA253_COMP_TARGET_OFFSET_Y__REG, &data);

	return comres;
}

static int bma253_get_offset_target_y(struct i2c_client *client, unsigned char
		*offsettarget)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_OFFSET_PARAMS_REG,
			&data);
	data = BMA253_GET_BITSLICE(data, BMA253_COMP_TARGET_OFFSET_Y);
	*offsettarget = data;

	return comres;
}

static int bma253_set_offset_target_z(struct i2c_client *client, unsigned char
		offsettarget)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client,
			BMA253_COMP_TARGET_OFFSET_Z__REG, &data);
	data = BMA253_SET_BITSLICE(data, BMA253_COMP_TARGET_OFFSET_Z,
			offsettarget);
	comres = bma253_smbus_write_byte(client,
			BMA253_COMP_TARGET_OFFSET_Z__REG, &data);

	return comres;
}

static int bma253_get_offset_target_z(struct i2c_client *client, unsigned char
		*offsettarget)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_OFFSET_PARAMS_REG,
			&data);
	data = BMA253_GET_BITSLICE(data, BMA253_COMP_TARGET_OFFSET_Z);
	*offsettarget = data;

	return comres;
}

static int bma253_get_cal_ready(struct i2c_client *client, unsigned char
		*calrdy)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_OFFSET_CTRL_REG, &data);
	data = BMA253_GET_BITSLICE(data, BMA253_FAST_COMP_RDY_S);
	*calrdy = data;

	return comres;
}

static int bma253_set_cal_trigger(struct i2c_client *client, unsigned char
		caltrigger)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_EN_FAST_COMP__REG,
			&data);
	data = BMA253_SET_BITSLICE(data, BMA253_EN_FAST_COMP, caltrigger);
	comres = bma253_smbus_write_byte(client, BMA253_EN_FAST_COMP__REG,
			&data);

	return comres;
}

static int bma253_set_selftest_st(struct i2c_client *client, unsigned char
		selftest)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_EN_SELF_TEST__REG,
			&data);
	data = BMA253_SET_BITSLICE(data, BMA253_EN_SELF_TEST, selftest);
	comres = bma253_smbus_write_byte(client, BMA253_EN_SELF_TEST__REG,
			&data);

	return comres;
}

static int bma253_set_selftest_stn(struct i2c_client *client, unsigned char stn)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client, BMA253_NEG_SELF_TEST__REG,
			&data);
	data = BMA253_SET_BITSLICE(data, BMA253_NEG_SELF_TEST, stn);
	comres = bma253_smbus_write_byte(client, BMA253_NEG_SELF_TEST__REG,
			&data);

	return comres;
}

static int bma253_set_ee_w(struct i2c_client *client, unsigned char eew)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client,
			BMA253_UNLOCK_EE_WRITE_SETTING__REG, &data);
	data = BMA253_SET_BITSLICE(data, BMA253_UNLOCK_EE_WRITE_SETTING, eew);
	comres = bma253_smbus_write_byte(client,
			BMA253_UNLOCK_EE_WRITE_SETTING__REG, &data);
	return comres;
}

static int bma253_set_ee_prog_trig(struct i2c_client *client)
{
	int comres = 0;
	unsigned char data;
	unsigned char eeprog;
	eeprog = 0x01;

	comres = bma253_smbus_read_byte(client,
			BMA253_START_EE_WRITE_SETTING__REG, &data);
	data = BMA253_SET_BITSLICE(data,
				BMA253_START_EE_WRITE_SETTING, eeprog);
	comres = bma253_smbus_write_byte(client,
			BMA253_START_EE_WRITE_SETTING__REG, &data);
	return comres;
}

static int bma253_get_eeprom_writing_status(struct i2c_client *client,
							unsigned char *eewrite)
{
	int comres = 0;
	unsigned char data;

	comres = bma253_smbus_read_byte(client,
				BMA253_EEPROM_CTRL_REG, &data);
	data = BMA253_GET_BITSLICE(data, BMA253_EE_WRITE_SETTING_S);
	*eewrite = data;

	return comres;
}

static int bma253_read_accel_x(struct i2c_client *client, short *a_x)
{
	int comres;
	unsigned char data[2];

	comres = bma253_smbus_read_byte_block(client, BMA253_ACC_X_LSB__REG,
			data, 2);

	*a_x = BMA253_GET_BITSLICE(data[0], BMA253_ACC_X_LSB) |
		(BMA253_GET_BITSLICE(data[1],
				     BMA253_ACC_X_MSB)<<BMA253_ACC_X_LSB__LEN);
	*a_x = *a_x <<
		(sizeof(short)*8-(BMA253_ACC_X_LSB__LEN+BMA253_ACC_X_MSB__LEN));
	*a_x = *a_x >>
		(sizeof(short)*8-(BMA253_ACC_X_LSB__LEN+BMA253_ACC_X_MSB__LEN));

	return comres;
}
static int bma253_read_accel_y(struct i2c_client *client, short *a_y)
{
	int comres;
	unsigned char data[2];

	comres = bma253_smbus_read_byte_block(client, BMA253_ACC_Y_LSB__REG,
			data, 2);
		
	*a_y = BMA253_GET_BITSLICE(data[0], BMA253_ACC_Y_LSB) |
		(BMA253_GET_BITSLICE(data[1],
				     BMA253_ACC_Y_MSB)<<BMA253_ACC_Y_LSB__LEN);
	*a_y = *a_y <<
		(sizeof(short)*8-(BMA253_ACC_Y_LSB__LEN+BMA253_ACC_Y_MSB__LEN));
	*a_y = *a_y >>
		(sizeof(short)*8-(BMA253_ACC_Y_LSB__LEN+BMA253_ACC_Y_MSB__LEN));

	return comres;
}

static int bma253_read_accel_z(struct i2c_client *client, short *a_z)
{
	int comres;
	unsigned char data[2];

	comres = bma253_smbus_read_byte_block(client, BMA253_ACC_Z_LSB__REG,
			data, 2);

	*a_z = BMA253_GET_BITSLICE(data[0], BMA253_ACC_Z_LSB) |
		BMA253_GET_BITSLICE(data[1],
				BMA253_ACC_Z_MSB)<<BMA253_ACC_Z_LSB__LEN;
	*a_z = *a_z <<
		(sizeof(short)*8-(BMA253_ACC_Z_LSB__LEN+BMA253_ACC_Z_MSB__LEN));
	*a_z = *a_z >>
		(sizeof(short)*8-(BMA253_ACC_Z_LSB__LEN+BMA253_ACC_Z_MSB__LEN));

	return comres;
}


static int bma253_read_accel_xyz(struct i2c_client *client,
		struct bma253acc *acc)
{
	int comres;
	unsigned char data[6];

	int i =0;
	comres = bma253_smbus_read_byte_block(client,
			BMA253_ACC_X_LSB__REG, data, 6);
	
	acc->x = BMA253_GET_BITSLICE(data[0], BMA253_ACC_X_LSB)
		|(BMA253_GET_BITSLICE(data[1],
				BMA253_ACC_X_MSB)<<BMA253_ACC_X_LSB__LEN);
	acc->x = acc->x << (sizeof(short)*8-(BMA253_ACC_X_LSB__LEN
				+ BMA253_ACC_X_MSB__LEN));
	acc->x = acc->x >> (sizeof(short)*8-(BMA253_ACC_X_LSB__LEN
				+ BMA253_ACC_X_MSB__LEN));
	acc->y = BMA253_GET_BITSLICE(data[2], BMA253_ACC_Y_LSB)
		| (BMA253_GET_BITSLICE(data[3],
				BMA253_ACC_Y_MSB)<<BMA253_ACC_Y_LSB__LEN);
	acc->y = acc->y << (sizeof(short)*8-(BMA253_ACC_Y_LSB__LEN
				+ BMA253_ACC_Y_MSB__LEN));
	acc->y = acc->y >> (sizeof(short)*8-(BMA253_ACC_Y_LSB__LEN
				+ BMA253_ACC_Y_MSB__LEN));

	acc->z = BMA253_GET_BITSLICE(data[4], BMA253_ACC_Z_LSB)
		| (BMA253_GET_BITSLICE(data[5],
				BMA253_ACC_Z_MSB)<<BMA253_ACC_Z_LSB__LEN);
	acc->z = acc->z << (sizeof(short)*8-(BMA253_ACC_Z_LSB__LEN
				+ BMA253_ACC_Z_MSB__LEN));
	acc->z = acc->z >> (sizeof(short)*8-(BMA253_ACC_Z_LSB__LEN
				+ BMA253_ACC_Z_MSB__LEN));

	return comres;
}

static void bma253_work_func(struct work_struct *work)
{
	struct bma253_data *bma253 = gdata;
	static struct bma253acc acc;
	static struct bma253acc cali_acc;
	unsigned long delay = msecs_to_jiffies(atomic_read(&bma253->delay));
	s16 data_x = 0, data_y = 0, data_z = 0;
	s16 hw_d[3] = {0};

	bma253_read_accel_xyz(bma253->bma253_client, &acc);

	if (1 == bma253->user_calibration_flag)
	{
		hw_d[0] = acc.x + bma253->user_offset_buf[0];
		hw_d[1] = acc.y + bma253->user_offset_buf[1];
		hw_d[2] = acc.z + bma253->user_offset_buf[2];
	}
	else
	{
		hw_d[0] = acc.x + bma253->offset_buf[0];
		hw_d[1] = acc.y + bma253->offset_buf[1];
		hw_d[2] = acc.z + bma253->offset_buf[2];
	}

	cali_acc.x = hw_d[0];
	cali_acc.y = hw_d[1];
	cali_acc.z = hw_d[2];

	data_x = ((bma253->pdata->negate_x) ? (-hw_d[bma253->pdata->axis_map_x])
		   : (hw_d[bma253->pdata->axis_map_x]));
	data_y = ((bma253->pdata->negate_y) ? (-hw_d[bma253->pdata->axis_map_y])
		   : (hw_d[bma253->pdata->axis_map_y]));
	data_z = ((bma253->pdata->negate_z) ? (-hw_d[bma253->pdata->axis_map_z])
		   : (hw_d[bma253->pdata->axis_map_z]));

	input_report_abs(bma253->input, ABS_X, data_x);
	input_report_abs(bma253->input, ABS_Y, data_y);
	input_report_abs(bma253->input, ABS_Z, data_z);
	input_sync(bma253->input);
	mutex_lock(&bma253->value_mutex);
	bma253->value = acc;
	bma253->cali_value = cali_acc;
	mutex_unlock(&bma253->value_mutex);
	queue_delayed_work(bma253->bma253_wq, &polling_work, delay);
}


static ssize_t bma253_register_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int address, value;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	sscanf(buf, "%d%d", &address, &value);

	if (bma253_write_reg(bma253->bma253_client, (unsigned char)address,
				(unsigned char *)&value) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma253_register_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	size_t count = 0;
	u8 reg[0x3d];
	int i;

	for (i = 0 ; i < 0x3d; i++) {
		bma253_smbus_read_byte(bma253->bma253_client, i, reg+i);

		count += sprintf(&buf[count], "0x%x: %d\n", i, reg[i]);
	}
	return count;


}
static ssize_t bma253_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_range(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma253_range_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma253_set_range(bma253->bma253_client, (unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma253_bandwidth_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_bandwidth(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma253_bandwidth_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma253_set_bandwidth(bma253->bma253_client,
				(unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static void debug_do_work(struct work_struct *w)
{
	E("%s: debug_execute_point = %lu\n", __func__, debug_execute_point);
}


static ssize_t bma253_chip_layout_show(struct device *dev,	/* show chip layou attribute */
		struct device_attribute *attr, char *buf)
{
	if (gdata == NULL) {
		E("%s: gdata == NULL\n", __func__);
		return 0;
	}

	return sprintf(buf, "chip_layout = %d\n"
			    "axis_map_x = %d, axis_map_y = %d,"
			    " axis_map_z = %d\n"
			    "negate_x = %d, negate_y = %d, negate_z = %d\n",
			    gdata->chip_layout,
			    gdata->pdata->axis_map_x, gdata->pdata->axis_map_y,
			    gdata->pdata->axis_map_z,
			    gdata->pdata->negate_x, gdata->pdata->negate_y,
			    gdata->pdata->negate_z);
}

static ssize_t bma253_chip_layout_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data = 0;
	int error = 0;

	/*D("%s++: debug: data = %lu\n", __func__, data);*/

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	debug_execute_point = data;
	if (debug_execute_point == 1) {
		queue_delayed_work(bma_wq, &debug_work,
			msecs_to_jiffies(2000));
	} else if (debug_execute_point == 2) {
		cancel_delayed_work(&debug_work);
	}

	/*D("%s: debug: data = %lu\n", __func__, data);*/

	return count;
}


static ssize_t bma253_get_raw_data_show(struct device *dev,	/* show chip layou attribute */
		struct device_attribute *attr, char *buf)
{
	struct bma253_data *bma253 = gdata;
	static struct bma253acc acc;

	if (bma253 == NULL) {
		E("%s: bma253 == NULL\n", __func__);
		return 0;
	}

	bma253_read_accel_xyz(bma253->bma253_client, &acc);

	return sprintf(buf, "x = %d, y = %d, z = %d\n",
			    acc.x, acc.y, acc.z);
}


static ssize_t bma253_set_k_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253 == NULL) {
		E("%s: bma253 == NULL\n", __func__);
		return 0;
	}

	return sprintf(buf, "gs_kvalue = 0x%x\n", bma253->pdata->gs_kvalue);
}

static ssize_t bma253_set_k_value_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);
	int i = 0;

	if (bma253 == NULL) {
		E("%s: bma253 == NULL\n", __func__);
		return count;
	}

	D("%s: Set buf = %s\n", __func__, buf);

	bma253->pdata->gs_kvalue = simple_strtoul(buf, NULL, 10);

	D("%s: bma253->pdata->gs_kvalue = 0x%x\n", __func__,
		bma253->pdata->gs_kvalue);

	if ((bma253->pdata->gs_kvalue & (0x67 << 24)) != (0x67 << 24)) {
		bma253->offset_buf[0] = 0;
		bma253->offset_buf[1] = 0;
		bma253->offset_buf[2] = 0;
	} else {
		bma253->offset_buf[0] = (bma253->pdata->gs_kvalue >> 16) & 0xFF;
		bma253->offset_buf[1] = (bma253->pdata->gs_kvalue >>  8) & 0xFF;
		bma253->offset_buf[2] =  bma253->pdata->gs_kvalue        & 0xFF;

		for (i = 0; i < 3; i++) {
			if (bma253->offset_buf[i] > 127) {
				bma253->offset_buf[i] =
					bma253->offset_buf[i] - 256;
			}
		}
		bma253->offset_buf[0] *= 4;
		bma253->offset_buf[1] *= 4;
		bma253->offset_buf[2] *= 4;
	}

	bma253->user_calibration_flag = 0;
	return count;
}

#ifdef IS_MTK_PLATFORM
static ssize_t get_nvram_cali_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);
	int ret;

	ret = sprintf(buf, "nvram_gs_kvalue = 0x%08x\n", bma253->nvram_gs_kvalue);
	return ret;
}

static ssize_t set_nvram_cali_data_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	sscanf(buf, "%x", &bma253->nvram_gs_kvalue);
	bma253->pdata->gs_kvalue = bma253->nvram_gs_kvalue;
	I("%s: nvram_gs_kvalue=0x%x\n", __func__, bma253->nvram_gs_kvalue);

	return count;
}
#endif

static ssize_t bma253_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_mode(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma253_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);
	I("bma253_mode_store\n");
	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma253_set_mode(bma253->bma253_client, (unsigned char) data) < 0)
	    return -EINVAL;

	return count;
}


static ssize_t bma253_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma253_data *bma253 = input_get_drvdata(input);
	struct bma253acc acc_value;

	mutex_lock(&bma253->value_mutex);
	acc_value = bma253->value;
	mutex_unlock(&bma253->value_mutex);

	return sprintf(buf, "%d %d %d\n", acc_value.x, acc_value.y,
			acc_value.z);
}

static ssize_t bma253_cali_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma253_data *bma253 = input_get_drvdata(input);

	static struct bma253acc acc;
	static struct bma253acc cali_acc;
	s16 hw_d[3] = {0};

	bma253_read_accel_xyz(bma253->bma253_client, &acc);

	if (1 == bma253->user_calibration_flag)
		{
			hw_d[0] = acc.x + bma253->user_offset_buf[0];
			hw_d[1] = acc.y + bma253->user_offset_buf[1];
			hw_d[2] = acc.z + bma253->user_offset_buf[2];
		}
		else
		{
			hw_d[0] = acc.x + bma253->offset_buf[0];
			hw_d[1] = acc.y + bma253->offset_buf[1];
			hw_d[2] = acc.z + bma253->offset_buf[2];
		}

		cali_acc.x = hw_d[0];
		cali_acc.y = hw_d[1];
		cali_acc.z = hw_d[2];

	return sprintf(buf, "%d %d %d\n", cali_acc.x, cali_acc.y,
			cali_acc.z);
}

static ssize_t bma253_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma253->delay));

}

static ssize_t bma253_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (data > BMA253_MAX_DELAY)
		data = BMA253_MAX_DELAY;
	atomic_set(&bma253->delay, (unsigned int) data);

	return count;
}


static ssize_t bma253_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma253->enable));

}

#ifndef IS_MTK_PLATFORM
static int bma253_sr_ldo_init(int init)
{
	int rc = 0;
	struct bma253_data *bma253 = gdata;

	if (bma253 == NULL) {
		E("%s: bma253 == NULL\n", __func__);
		return -1;
	}

	if (!init) {
		regulator_set_voltage(bma253->sr_1v8, 0, 1800000);
		regulator_set_voltage(bma253->sr_2v85, 0, 2850000);
		return 0;
	}

	bma253->sr_2v85 = devm_regulator_get(&bma253->bma253_client->dev, "SR_2v85");
	if (IS_ERR(bma253->sr_2v85)) {
		bma253->sr_2v85 = NULL;
		bma253->sr_1v8 = NULL;
		E("%s: Unable to get sr 2v85\n", __func__);
		return PTR_ERR(bma253->sr_2v85);
	}
	I("%s: bma253->sr_2v85 = 0x%p\n", __func__, bma253->sr_2v85);

	rc = regulator_set_voltage(bma253->sr_2v85, 2850000, 2850000);
	if (rc) {
		E("%s: unable to set voltage for sr 2v85\n", __func__);
		return rc;
	}

	bma253->sr_1v8 = devm_regulator_get(&bma253->bma253_client->dev, "SR_1v8");
	if (IS_ERR(bma253->sr_1v8)) {
		E("%s: unable to get sr 1v8\n", __func__);
		rc = PTR_ERR(bma253->sr_1v8);
		bma253->sr_1v8 = NULL;
		goto devote_2v85;
	}

	rc = regulator_set_voltage(bma253->sr_1v8, 1800000, 1800000);
	if (rc) {
		E("%s: unable to set voltage for sr 1v8\n", __func__);
		goto devote_2v85;
	}
	I("%s: bma253->sr_1v8 = 0x%p\n", __func__, bma253->sr_1v8);

	return 0;

devote_2v85:
	regulator_set_voltage(bma253->sr_2v85, 0, 2850000);

	return rc;
}

static int bma253_sr_lpm(int on)
{
	int rc = 0;
	struct bma253_data *bma253 = gdata;

	D("%s++: vreg (%s)\n", __func__, on ? "LPM" : "HPM");

	if ((bma253->sr_1v8 == NULL) || (bma253->sr_2v85 == NULL)) {
		I("%s: Regulator not available, return!!\n", __func__);
		return 0;
	}

	if (on) {
		rc = regulator_set_optimum_mode(bma253->sr_1v8, 100);
		if (rc < 0)
			E("Unable to set LPM of regulator sr_1v8\n");
		rc = regulator_enable(bma253->sr_1v8);
		if (rc)
			E("Unable to enable sr_1v8 111\n");
		D("%s: Set SR_1v8 to LPM--\n", __func__);

		rc = regulator_set_optimum_mode(bma253->sr_2v85, 100);
		if (rc < 0)
			E("Unable to set LPM of regulator sr_2v85\n");
		rc = regulator_enable(bma253->sr_2v85);
		if (rc)
			E("Unable to enable sr_2v85 111\n");
		D("%s: Set SR_2v85 to LPM--\n", __func__);
	} else {
		rc = regulator_set_optimum_mode(bma253->sr_1v8, 100000);
		if (rc < 0)
			E("Unable to set HPM of regulator sr_1v8\n");
		rc = regulator_enable(bma253->sr_1v8);
		if (rc)
			E("Unable to enable sr_1v8 222\n");
		D("%s: Set SR_1v8 to HPM--\n", __func__);

		rc = regulator_set_optimum_mode(bma253->sr_2v85, 100000);
		if (rc < 0)
			E("Unable to set HPM of regulator sr_2v85\n");
		rc = regulator_enable(bma253->sr_2v85);
		if (rc)
			E("Unable to enable sr_2v85 222\n");
		D("%s: Set SR_2v85 to HPM--\n", __func__);
	}

	return rc < 0 ? rc : 0;
}
#endif
static void bma253_set_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);
	int pre_enable = atomic_read(&bma253->enable);
	int i = 0;

	mutex_lock(&bma253->enable_mutex);
	if (enable) {
		if (bma253->pdata->power_LPM)
			bma253->pdata->power_LPM(0);

		if (pre_enable == 0) {
			bma253_set_mode(bma253->bma253_client,
					BMA253_MODE_NORMAL);
			queue_delayed_work(bma253->bma253_wq, &polling_work,
				msecs_to_jiffies(atomic_read(&bma253->delay)));
			atomic_set(&bma253->enable, 1);
		}

	} else {
		if (pre_enable == 1) {
			bma253_set_mode(bma253->bma253_client,
					BMA253_MODE_SUSPEND);
			cancel_delayed_work_sync(&polling_work);
			atomic_set(&bma253->enable, 0);
		}

#ifdef CONFIG_CIR_ALWAYS_READY
		if (bma253->pdata->power_LPM && !cir_flag)
#else

		if (bma253->pdata->power_LPM)
#endif
			bma253->pdata->power_LPM(1);
	}

	if ((bma253->pdata->gs_kvalue & (0x67 << 24)) != (0x67 << 24)) {
		bma253->offset_buf[0] = 0;
		bma253->offset_buf[1] = 0;
		bma253->offset_buf[2] = 0;
	} else {
		bma253->offset_buf[0] = (bma253->pdata->gs_kvalue >> 16) & 0xFF;
		bma253->offset_buf[1] = (bma253->pdata->gs_kvalue >>  8) & 0xFF;
		bma253->offset_buf[2] =  bma253->pdata->gs_kvalue        & 0xFF;

		for (i = 0; i < 3; i++) {
			if (bma253->offset_buf[i] > 127) {
				bma253->offset_buf[i] =
					bma253->offset_buf[i] - 256;
			}
		}
		bma253->offset_buf[0] *= 4;
		bma253->offset_buf[1] *= 4;
		bma253->offset_buf[2] *= 4;
	}

	mutex_unlock(&bma253->enable_mutex);

}

static ssize_t bma253_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if ((data == 0) || (data == 1))
		bma253_set_enable(dev, data);

	return count;
}

static ssize_t bma253_enable_int_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int type, value;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	sscanf(buf, "%d%d", &type, &value);

	if (bma253_set_Int_Enable(bma253->bma253_client, type, value) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma253_int_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_Int_Mode(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma253_int_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma253_set_Int_Mode(bma253->bma253_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma253_slope_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_slope_duration(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma253_slope_duration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma253_set_slope_duration(bma253->bma253_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma253_slope_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_slope_threshold(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma253_slope_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma253_set_slope_threshold(bma253->bma253_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma253_high_g_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_high_g_duration(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma253_high_g_duration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma253_set_high_g_duration(bma253->bma253_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma253_high_g_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_high_g_threshold(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma253_high_g_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma253_set_high_g_threshold(bma253->bma253_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma253_low_g_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_low_g_duration(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma253_low_g_duration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma253_set_low_g_duration(bma253->bma253_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma253_low_g_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_low_g_threshold(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma253_low_g_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma253_set_low_g_threshold(bma253->bma253_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma253_tap_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_tap_threshold(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma253_tap_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma253_set_tap_threshold(bma253->bma253_client, (unsigned char)data)
			< 0)
		return -EINVAL;

	return count;
}
static ssize_t bma253_tap_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_tap_duration(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma253_tap_duration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma253_set_tap_duration(bma253->bma253_client, (unsigned char)data)
			< 0)
		return -EINVAL;

	return count;
}
static ssize_t bma253_tap_quiet_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_tap_quiet(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma253_tap_quiet_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma253_set_tap_quiet(bma253->bma253_client, (unsigned char)data) <
			0)
		return -EINVAL;

	return count;
}

static ssize_t bma253_tap_shock_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_tap_shock(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma253_tap_shock_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma253_set_tap_shock(bma253->bma253_client, (unsigned char)data) <
			0)
		return -EINVAL;

	return count;
}

static ssize_t bma253_tap_samp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_tap_samp(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma253_tap_samp_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma253_set_tap_samp(bma253->bma253_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma253_orient_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_orient_mode(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma253_orient_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma253_set_orient_mode(bma253->bma253_client, (unsigned char)data) <
			0)
		return -EINVAL;

	return count;
}

static ssize_t bma253_orient_blocking_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_orient_blocking(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma253_orient_blocking_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma253_set_orient_blocking(bma253->bma253_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma253_orient_hyst_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_orient_hyst(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma253_orient_hyst_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma253_set_orient_hyst(bma253->bma253_client, (unsigned char)data) <
			0)
		return -EINVAL;

	return count;
}

static ssize_t bma253_orient_theta_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_theta_blocking(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma253_orient_theta_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma253_set_theta_blocking(bma253->bma253_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma253_flat_theta_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_theta_flat(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma253_flat_theta_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma253_set_theta_flat(bma253->bma253_client, (unsigned char)data) <
			0)
		return -EINVAL;

	return count;
}
static ssize_t bma253_flat_hold_time_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_flat_hold_time(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma253_flat_hold_time_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma253_set_flat_hold_time(bma253->bma253_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}


static ssize_t bma253_fast_calibration_x_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{


	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_offset_target_x(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma253_fast_calibration_x_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	unsigned char timeout = 0;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma253_set_offset_target_x(bma253->bma253_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	if (bma253_set_cal_trigger(bma253->bma253_client, 1) < 0)
		return -EINVAL;

	do {
		mdelay(2);
		bma253_get_cal_ready(bma253->bma253_client, &tmp);

	/*	I("wait 2ms cal ready flag is %d\n",tmp);*/
		timeout++;
		if (timeout == 50) {
			I("get fast calibration ready error\n");
			return -EINVAL;
		};

	} while (tmp == 0);

	I("x axis fast calibration finished\n");
	return count;
}

static ssize_t bma253_fast_calibration_y_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{


	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_offset_target_y(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma253_fast_calibration_y_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	unsigned char timeout = 0;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma253_set_offset_target_y(bma253->bma253_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	if (bma253_set_cal_trigger(bma253->bma253_client, 2) < 0)
		return -EINVAL;

	do {
		mdelay(2);
		bma253_get_cal_ready(bma253->bma253_client, &tmp);

	/*	I("wait 2ms cal ready flag is %d\n",tmp);*/
		timeout++;
		if (timeout == 50) {
			I("get fast calibration ready error\n");
			return -EINVAL;
		};

	} while (tmp == 0);

	I("y axis fast calibration finished\n");
	return count;
}

static ssize_t bma253_fast_calibration_z_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{


	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253_get_offset_target_z(bma253->bma253_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma253_fast_calibration_z_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	unsigned char timeout = 0;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma253_set_offset_target_z(bma253->bma253_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	if (bma253_set_cal_trigger(bma253->bma253_client, 3) < 0)
		return -EINVAL;

	do {
		mdelay(2);
		bma253_get_cal_ready(bma253->bma253_client, &tmp);

	/*	I("wait 2ms cal ready flag is %d\n",tmp);*/
		timeout++;
		if (timeout == 50) {
			I("get fast calibration ready error\n");
			return -EINVAL;
		};

	} while (tmp == 0);

	I("z axis fast calibration finished\n");
	return count;
}

static ssize_t bma253_selftest_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{


	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma253->selftest_result));

}

static ssize_t bma253_selftest_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{

	unsigned long data;
	unsigned char clear_value = 0;
	int error;
	short value1 = 0;
	short value2 = 0;
	short diff = 0;
	unsigned long result = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;


	if (data != 1)
		return -EINVAL;
	/* set to 2 G range */
	if (bma253_set_range(bma253->bma253_client, 0) < 0)
		return -EINVAL;

	bma253_write_reg(bma253->bma253_client, 0x32, &clear_value);

	bma253_set_selftest_st(bma253->bma253_client, 1); /* 1 for x-axis*/
	bma253_set_selftest_stn(bma253->bma253_client, 0); /* positive
							      direction*/
	mdelay(10);
	bma253_read_accel_x(bma253->bma253_client, &value1);
	bma253_set_selftest_stn(bma253->bma253_client, 1); /* negative
							      direction*/
	mdelay(10);
	bma253_read_accel_x(bma253->bma253_client, &value2);
	diff = value1-value2;

	I("diff x is %d,value1 is %d, value2 is %d\n", diff,
			value1, value2);

	if (abs(diff) < 204)
		result |= 1;

	bma253_set_selftest_st(bma253->bma253_client, 2); /* 2 for y-axis*/
	bma253_set_selftest_stn(bma253->bma253_client, 0); /* positive
							      direction*/
	mdelay(10);
	bma253_read_accel_y(bma253->bma253_client, &value1);
	bma253_set_selftest_stn(bma253->bma253_client, 1); /* negative
							      direction*/
	mdelay(10);
	bma253_read_accel_y(bma253->bma253_client, &value2);
	diff = value1-value2;
	I("diff y is %d,value1 is %d, value2 is %d\n", diff,
			value1, value2);
	if (abs(diff) < 204)
		result |= 2;


	bma253_set_selftest_st(bma253->bma253_client, 3); /* 3 for z-axis*/
	bma253_set_selftest_stn(bma253->bma253_client, 0); /* positive
							      direction*/
	mdelay(10);
	bma253_read_accel_z(bma253->bma253_client, &value1);
	bma253_set_selftest_stn(bma253->bma253_client, 1); /* negative
							      direction*/
	mdelay(10);
	bma253_read_accel_z(bma253->bma253_client, &value2);
	diff = value1-value2;

	I("diff z is %d,value1 is %d, value2 is %d\n", diff,
			value1, value2);
	if (abs(diff) < 102)
		result |= 4;

	atomic_set(&bma253->selftest_result, (unsigned int)result);

	I("self test finished\n");


	return count;
}


static ssize_t bma253_eeprom_writing_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	int timeout = 0;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;



	if (data != 1)
		return -EINVAL;

	/* unlock eeprom */
	if (bma253_set_ee_w(bma253->bma253_client, 1) < 0)
		return -EINVAL;

	I("unlock eeprom successful\n");

	if (bma253_set_ee_prog_trig(bma253->bma253_client) < 0)
		return -EINVAL;
	I("start update eeprom\n");

	do {
		mdelay(2);
		bma253_get_eeprom_writing_status(bma253->bma253_client, &tmp);

		I("wait 2ms eeprom write status is %d\n", tmp);
		timeout++;
		if (timeout == 1000) {
			I("get eeprom writing status error\n");
			return -EINVAL;
		};

	} while (tmp == 0);

	I("eeprom writing is finished\n");

	/* unlock eeprom */
	if (bma253_set_ee_w(bma253->bma253_client, 0) < 0)
		return -EINVAL;

	I("lock eeprom successful\n");
	return count;
}

#ifdef CONFIG_CIR_ALWAYS_READY
static ssize_t bma253_enable_interrupt(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long enable;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &enable);
		if (error)
		return error;
	I("bma253_enable_interrupt, power_key_pressed = %d\n", power_key_pressed);
	if(enable == 1 && !power_key_pressed){ // Slope interrupt

	    cir_flag = 1;

	    //Don't change to low power mode due to enabling interrupt mode
	    if(bma253->pdata->power_LPM)
		bma253->pdata->power_LPM(0);
	    /*Set the related parameters*/
	    error = bma253_set_Int_Mode(bma253->bma253_client, 1);/*latch interrupt 250ms*/

	    error += bma253_set_slope_duration(bma253->bma253_client, 0x01);//dur+1
	    error += bma253_set_slope_threshold(bma253->bma253_client, 0x07);//0x07 * 3.91  =

	    /*Enable the interrupts*/
	    error += bma253_set_Int_Enable(bma253->bma253_client, 5, 1);//Slope X
	    error += bma253_set_Int_Enable(bma253->bma253_client, 6, 1);//Slope Y
	    error += bma253_set_Int_Enable(bma253->bma253_client, 7, 0);//Slope Z
	    error += bma253_set_int1_pad_sel(bma253->bma253_client, PAD_SLOP);

	    error += bma253_set_mode(bma253->bma253_client, BMA253_MODE_NORMAL);

	    if (error)
		return error;
	    I("Always Ready enable = 1 \n");

	}  else if(enable == 0){

	    error += bma253_set_Int_Enable(bma253->bma253_client, 5, 0);//Slope X
	    error += bma253_set_Int_Enable(bma253->bma253_client, 6, 0);//Slope Y
	    error += bma253_set_Int_Enable(bma253->bma253_client, 7, 0);//Slope Z

	    power_key_pressed = 0;
	    cir_flag = 0;
	    if (error)
		return error;
	    I("Always Ready enable = 0 \n");

	} 	return count;
}
static ssize_t bma253_clear_powerkey_pressed(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long powerkey_pressed;
	int error;
	error = strict_strtoul(buf, 10, &powerkey_pressed);
	if (error)
	    return error;

	if(powerkey_pressed == 1) {
	    power_key_pressed = 1;
	}
	else if(powerkey_pressed == 0) {
	    power_key_pressed = 0;
	}
	return count;
}
static ssize_t bma253_get_powerkry_pressed(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", power_key_pressed);
}
static DEVICE_ATTR(enable_cir_interrupt, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		NULL, bma253_enable_interrupt);
static DEVICE_ATTR(clear_powerkey_flag, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		bma253_get_powerkry_pressed, bma253_clear_powerkey_pressed);
#endif

static DEVICE_ATTR(range, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_range_show, bma253_range_store);
static DEVICE_ATTR(bandwidth, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_bandwidth_show, bma253_bandwidth_store);
static DEVICE_ATTR(mode, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_mode_show, bma253_mode_store);
static DEVICE_ATTR(value, S_IRUGO,
		bma253_value_show, NULL);
static DEVICE_ATTR(cali_value, S_IRUGO,
		bma253_cali_value_show, NULL);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_delay_show, bma253_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_enable_show, bma253_enable_store);
static DEVICE_ATTR(enable_int, S_IWUSR|S_IWGRP|S_IWOTH,
		NULL, bma253_enable_int_store);
static DEVICE_ATTR(int_mode, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_int_mode_show, bma253_int_mode_store);
static DEVICE_ATTR(slope_duration, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_slope_duration_show, bma253_slope_duration_store);
static DEVICE_ATTR(slope_threshold, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_slope_threshold_show, bma253_slope_threshold_store);
static DEVICE_ATTR(high_g_duration, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_high_g_duration_show, bma253_high_g_duration_store);
static DEVICE_ATTR(high_g_threshold, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_high_g_threshold_show, bma253_high_g_threshold_store);
static DEVICE_ATTR(low_g_duration, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_low_g_duration_show, bma253_low_g_duration_store);
static DEVICE_ATTR(low_g_threshold, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_low_g_threshold_show, bma253_low_g_threshold_store);
static DEVICE_ATTR(tap_duration, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_tap_duration_show, bma253_tap_duration_store);
static DEVICE_ATTR(tap_threshold, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_tap_threshold_show, bma253_tap_threshold_store);
static DEVICE_ATTR(tap_quiet, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_tap_quiet_show, bma253_tap_quiet_store);
static DEVICE_ATTR(tap_shock, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_tap_shock_show, bma253_tap_shock_store);
static DEVICE_ATTR(tap_samp, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_tap_samp_show, bma253_tap_samp_store);
static DEVICE_ATTR(orient_mode, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_orient_mode_show, bma253_orient_mode_store);
static DEVICE_ATTR(orient_blocking, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_orient_blocking_show, bma253_orient_blocking_store);
static DEVICE_ATTR(orient_hyst, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_orient_hyst_show, bma253_orient_hyst_store);
static DEVICE_ATTR(orient_theta, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_orient_theta_show, bma253_orient_theta_store);
static DEVICE_ATTR(flat_theta, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_flat_theta_show, bma253_flat_theta_store);
static DEVICE_ATTR(flat_hold_time, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_flat_hold_time_show, bma253_flat_hold_time_store);
static DEVICE_ATTR(reg, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_register_show, bma253_register_store);
static DEVICE_ATTR(fast_calibration_x, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_fast_calibration_x_show,
		bma253_fast_calibration_x_store);
static DEVICE_ATTR(fast_calibration_y, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_fast_calibration_y_show,
		bma253_fast_calibration_y_store);
static DEVICE_ATTR(fast_calibration_z, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_fast_calibration_z_show,
		bma253_fast_calibration_z_store);
static DEVICE_ATTR(selftest, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma253_selftest_show, bma253_selftest_store);
static DEVICE_ATTR(eeprom_writing, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		NULL, bma253_eeprom_writing_store);

static DEVICE_ATTR(chip_layout,S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		bma253_chip_layout_show, bma253_chip_layout_store);

static DEVICE_ATTR(get_raw_data, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		bma253_get_raw_data_show, NULL);

static DEVICE_ATTR(set_k_value, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		bma253_set_k_value_show, bma253_set_k_value_store);

#ifdef IS_MTK_PLATFORM
static DEVICE_ATTR(nvram_cali_data, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		get_nvram_cali_data_show, set_nvram_cali_data_store);
#endif

static struct attribute *bma253_attributes[] = {
	&dev_attr_range.attr,
	&dev_attr_bandwidth.attr,
	&dev_attr_mode.attr,
	&dev_attr_value.attr,
	&dev_attr_cali_value.attr,
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_enable_int.attr,
	&dev_attr_int_mode.attr,
	&dev_attr_slope_duration.attr,
	&dev_attr_slope_threshold.attr,
	&dev_attr_high_g_duration.attr,
	&dev_attr_high_g_threshold.attr,
	&dev_attr_low_g_duration.attr,
	&dev_attr_low_g_threshold.attr,
	&dev_attr_tap_threshold.attr,
	&dev_attr_tap_duration.attr,
	&dev_attr_tap_quiet.attr,
	&dev_attr_tap_shock.attr,
	&dev_attr_tap_samp.attr,
	&dev_attr_orient_mode.attr,
	&dev_attr_orient_blocking.attr,
	&dev_attr_orient_hyst.attr,
	&dev_attr_orient_theta.attr,
	&dev_attr_flat_theta.attr,
	&dev_attr_flat_hold_time.attr,
	&dev_attr_reg.attr,
	&dev_attr_fast_calibration_x.attr,
	&dev_attr_fast_calibration_y.attr,
	&dev_attr_fast_calibration_z.attr,
	&dev_attr_selftest.attr,
	&dev_attr_eeprom_writing.attr,
	&dev_attr_chip_layout.attr,
	&dev_attr_get_raw_data.attr,
	&dev_attr_set_k_value.attr,
#ifdef IS_MTK_PLATFORM
	&dev_attr_nvram_cali_data.attr,
#endif
#ifdef CONFIG_CIR_ALWAYS_READY
	&dev_attr_enable_cir_interrupt.attr,
#endif
	NULL
};

static struct attribute_group bma253_attribute_group = {
	.attrs = bma253_attributes
};

static ssize_t set_g_sensor_user_offset(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct bma253_data *bma253 = dev_get_drvdata(dev);
	char *token;
	char *str_buf;
	char *running;
	long input_val[3] = {0};
	int rc, i;
	short temp_kvalue[3] = {0};
	short user_cali_offset_buf[3] = {0};
	bma253->user_calibration_flag = 0;
	str_buf = kstrndup(buf, count, GFP_KERNEL);
	if (str_buf == NULL) {
		E("%s: cannot allocate buffer\n", __func__);
		return -1;
	}
	running = str_buf;

	for (i = 0; i < 3; i++) {
		token = strsep(&running, " ");
		if (token == NULL) {
			E("%s: token = NULL, i = %d\n", __func__, i);
			break;
		}

		rc = kstrtol(token, 10, &input_val[i]);
		if (rc) {
			E("%s: kstrtol fails, rc = %d, i = %d\n",
			  __func__, rc, i);
			kfree(str_buf);
			return rc;
		}
	}
	kfree(str_buf);

	I("%s: User Calibration(x, y, z) = (%ld, %ld, %ld)\n", __func__,
	  input_val[0], input_val[1], input_val[2]);

	temp_kvalue[0] = (short)input_val[0];
	temp_kvalue[1] = (short)input_val[1];
	temp_kvalue[2] = (short)input_val[2];

	I("%s: temp_kvalue(x, y, z) = (0x%x, 0x%x, 0x%x)\n", __func__,
	  temp_kvalue[0] & 0xFFFF, temp_kvalue[1] & 0xFFFF, temp_kvalue[2] & 0xFFFF);

	for (i = 0; i < 3; i++) {
		user_cali_offset_buf[i]=temp_kvalue[i]*1024/BMA253_USER_CALI_GRAVITY_EARTH;
		I("%s: user_cali_offset_buf[%d] = %d\n", __func__, i, user_cali_offset_buf[i]);
	}

	bma253->user_offset_buf[0] = ((bma253->pdata->negate_x) ? (user_cali_offset_buf[bma253->pdata->axis_map_x])
		   : (-user_cali_offset_buf[bma253->pdata->axis_map_x]));
	bma253->user_offset_buf[1] = ((bma253->pdata->negate_y) ? (user_cali_offset_buf[bma253->pdata->axis_map_y])
		   : (-user_cali_offset_buf[bma253->pdata->axis_map_y]));
	bma253->user_offset_buf[2] = ((bma253->pdata->negate_z) ? (user_cali_offset_buf[bma253->pdata->axis_map_z])
		   : (-user_cali_offset_buf[bma253->pdata->axis_map_z]));

	bma253->user_calibration_flag = 1;
	return count;
}

static ssize_t bma253_sensor_user_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma253_data *bma253 = i2c_get_clientdata(client);

	if (bma253 == NULL) {
		E("%s: bma253 == NULL\n", __func__);
		return 0;
	}
	return sprintf(buf, "bma253->user_calibration_flag = %d, bma253->user_offset_buf[0] = %d, bma253->user_offset_buf[1] = %d, bma253->user_offset_buf[2] = %d\n",
			    bma253->user_calibration_flag, bma253->user_offset_buf[0], bma253->user_offset_buf[1],bma253->user_offset_buf[2]);
}

static DEVICE_ATTR(g_sensor_user_offset, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		bma253_sensor_user_offset_show, set_g_sensor_user_offset);


#ifdef CONFIG_CIR_ALWAYS_READY
#if defined(BMA253_ENABLE_INT1) || defined(BMA253_ENABLE_INT2)
unsigned char *orient_st[] = {"upward looking portrait upright",   \
	"upward looking portrait upside-down",   \
		"upward looking landscape left",   \
		"upward looking landscape right",   \
		"downward looking portrait upright",   \
		"downward looking portrait upside-down",   \
		"downward looking landscape left",   \
		"downward looking landscape right"};

static void bma253_irq_work_func(struct work_struct *work)
{
	struct bma253_data *bma253 = gdata;
	unsigned char status = 0;

	wake_lock_timeout(&(bma253->cir_always_ready_wake_lock), 1*HZ);
	bma253_get_interruptstatus1(bma253->bma253_client, &status);
	I("bma253_irq_work_func, status = 0x%x\n", status);
	input_report_rel(bma253->input_cir,
		SLOP_INTERRUPT,
		SLOPE_INTERRUPT_X_NEGATIVE_HAPPENED);
	input_report_rel(bma253->input_cir,
		SLOP_INTERRUPT,
		SLOPE_INTERRUPT_Y_NEGATIVE_HAPPENED);
	input_report_rel(bma253->input_cir,
		SLOP_INTERRUPT,
		SLOPE_INTERRUPT_X_HAPPENED);
	input_report_rel(bma253->input_cir,
		SLOP_INTERRUPT,
		SLOPE_INTERRUPT_Y_HAPPENED);
	input_sync(bma253->input_cir);
	enable_irq(bma253->IRQ);

}

static irqreturn_t bma253_irq_handler(int irq, void *handle)
{


	struct bma253_data *data = handle;

	if (data != NULL)
		disable_irq_nosync(data->IRQ);

	if (data == NULL)
		return IRQ_HANDLED;
	if (data->bma253_client == NULL)
		return IRQ_HANDLED;

	queue_work(data->bma253_wq, &irq_work);

	return IRQ_HANDLED;


}
#endif
#endif /* defined(BMA253_ENABLE_INT1)||defined(BMA253_ENABLE_INT2) */

#ifdef IS_MTK_PLATFORM
static int bma253_irq_registration(unsigned int irq, irq_handler_t handler, unsigned long flags, const char *name, void *dev)
{
  struct device_node *node = NULL;
  int ret = 0; 
  u32 ints[2] = {0,0};

  node = of_find_compatible_node(NULL, NULL, "mediatek, IRQ_BMA253-eint");
  if(node){
	of_property_read_u32_array(node , "debounce", ints, ARRAY_SIZE(ints));
    gpio_set_debounce(ints[0], ints[1]);

    irq = irq_of_parse_and_map(node, 0);

    ret = request_irq(irq, handler, flags, "IRQ_BMA253-eint", dev);
    if(ret > 0){
      ret = -1;
      E("%s: request_irq IRQ LINE NOT AVAILABLE!.\n", __func__);
    }
  }else{
    E("%s: request_irq can not find eint device node!.\n", __func__);
    ret = -1;
  }    
  I("%s: irq:%d, debounce:%d-%d: \n", __func__, irq, ints[0], ints[1]);
  return ret; 
}
#endif

static int bma253_parse_dt(struct device *dev, struct bma253_platform_data *pdata)
{
	struct property *prop = NULL;
	struct device_node *dt = dev->of_node;
	u32 buf = 0;
	uint32_t irq_gpio_flags = 0;

	struct device_node *offset = NULL;
	int cali_size = 0;
	unsigned char *cali_data = NULL;
	int i = 0;

	if (pdata == NULL) {
		E("%s: pdata is NULL\n", __func__);
		return -EINVAL;
	}

	prop = of_find_property(dt, "g_sensor_bma253,chip_layout", NULL);
	if (prop) {
		of_property_read_u32(dt, "g_sensor_bma253,chip_layout", &buf);
		pdata->chip_layout = buf;
		I("%s: chip_layout = %d", __func__, pdata->chip_layout);
	} else
		I("%s: g_sensor_bma253,chip_layout not found", __func__);

	prop = of_find_property(dt, "g_sensor_bma253,axis_map_x", NULL);
	if (prop) {
		of_property_read_u32(dt, "g_sensor_bma253,axis_map_x", &buf);
		pdata->axis_map_x = buf;
		I("%s: axis_map_x = %d", __func__, pdata->axis_map_x);
	} else
		I("%s: g_sensor_bma253,axis_map_x not found", __func__);

	prop = of_find_property(dt, "g_sensor_bma253,axis_map_y", NULL);
	if (prop) {
		of_property_read_u32(dt, "g_sensor_bma253,axis_map_y", &buf);
		pdata->axis_map_y = buf;
		I("%s: axis_map_y = %d", __func__, pdata->axis_map_y);
	} else
		I("%s: g_sensor_bma253,axis_map_y not found", __func__);

	prop = of_find_property(dt, "g_sensor_bma253,axis_map_z", NULL);
	if (prop) {
		of_property_read_u32(dt, "g_sensor_bma253,axis_map_z", &buf);
		pdata->axis_map_z = buf;
		I("%s: axis_map_z = %d", __func__, pdata->axis_map_z);
	} else
		I("%s: g_sensor_bma253,axis_map_z not found", __func__);

	prop = of_find_property(dt, "g_sensor_bma253,negate_x", NULL);
	if (prop) {
		of_property_read_u32(dt, "g_sensor_bma253,negate_x", &buf);
		pdata->negate_x = buf;
		I("%s: negate_x = %d", __func__, pdata->negate_x);
	} else
		I("%s: g_sensor_bma253,negate_x not found", __func__);

	prop = of_find_property(dt, "g_sensor_bma253,negate_y", NULL);
	if (prop) {
		of_property_read_u32(dt, "g_sensor_bma253,negate_y", &buf);
		pdata->negate_y = buf;
		I("%s: negate_y = %d", __func__, pdata->negate_y);
	} else
		I("%s: g_sensor_bma253,negate_y not found", __func__);

	prop = of_find_property(dt, "g_sensor_bma253,negate_z", NULL);
	if (prop) {
		of_property_read_u32(dt, "g_sensor_bma253,negate_z", &buf);
		pdata->negate_z = buf;
		I("%s: negate_z = %d", __func__, pdata->negate_z);
	} else
		I("%s: g_sensor_bma253,negate_z not found", __func__);

#ifdef IS_MTK_PLATFORM
	prop = of_find_property(dt, "g_sensor_bma253,intr", NULL);
	if (prop) {
		of_property_read_u32(dt, "g_sensor_bma253,intr", &irq_gpio_flags);
		pdata->intr = irq_gpio_flags;
		I("%s: pdata->intr = %d", __func__, pdata->intr);
	} else
		I("%s: g_sensor_bma253,intr not found", __func__);
	
	if (pdata->intr < 0) {
		E("%s: get g_sensor_bma253,intr fails: pdata->intr\n", __func__);
		return -EINVAL;
	}
#else
	pdata->intr = of_get_named_gpio_flags(dt,
					      "g_sensor_bma253,intr",
					      0,
					      &irq_gpio_flags);
	if (pdata->intr < 0) {
		E("%s: of_get_named_gpio_flags fails: pdata->intr\n", __func__);
		return -EINVAL;
	}
	I("%s: pdata->intr = %d", __func__, pdata->intr);
#endif

	pdata->gs_kvalue = 0;
	if ((offset = of_find_node_by_path(CALIBRATION_DATA_PATH))) {
		cali_data = (unsigned char*) of_get_property(offset, G_SENSOR_FLASH_DATA, &cali_size);
		I("%s: cali_size = %d", __func__, cali_size);
		if (cali_data) {
			for (i = 0; (i < cali_size) && (i < 4); i++) {
				I("cali_data[%d] = %02x ", i, cali_data[i]);
				pdata->gs_kvalue |= (cali_data[i] << (i * 8));
			}
		}
	} else
		I("%s: Calibration data offset not found", __func__);

#ifndef IS_MTK_PLATFORM
	pdata->power_LPM = bma253_sr_lpm;
#else
	pdata->power_LPM = NULL;
#endif

	return 0;
}

static int bma253_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	unsigned char tempvalue;
	struct bma253_data *data;
	struct input_dev *dev;
#ifdef CONFIG_CIR_ALWAYS_READY
	struct input_dev *dev_cir;
	struct class *bma253_powerkey_class = NULL;
	struct device *bma253_powerkey_dev = NULL;
	int res;
#endif

/*
	omap_mux_init_gpio(145, OMAP_PIN_INPUT);
	omap_mux_init_gpio(146, OMAP_PIN_INPUT);
*/

#ifdef IS_MTK_PLATFORM
#if 0
    hwPowerOn(MT6325_POWER_LDO_VIO28, VOL_2800, "V_SR_2V85");
	hwPowerOn(MT6325_POWER_LDO_VCAM_AF, VOL_1800, "V_SRIO_1V8");
#endif
#endif

#ifndef NO_GPIO_NFC_ENABLE
	// TODO NFC enable
	mt_set_gpio_mode(GPIO_NFC_ENABLE, 0);
	mt_set_gpio_dir(GPIO_NFC_ENABLE, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_NFC_ENABLE, GPIO_OUT_ONE);
#endif	

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		I("i2c_check_functionality error\n");
		goto exit;
	}
	data = kzalloc(sizeof(struct bma253_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}
	/* read chip id */
	tempvalue = i2c_smbus_read_byte_data(client, BMA253_CHIP_ID_REG);

	if (tempvalue == BMA253_CHIP_ID || tempvalue == 0xF8) {
		I("Bosch Sensortec Device detected! CHIP ID = 0x%x. "
				"BMA253 registered I2C driver!\n", tempvalue);
	} else{
		I("Bosch Sensortec Device not found"
				"i2c error %d \n", tempvalue);
		err = -ENODEV;
		goto kfree_exit;
	}
	i2c_set_clientdata(client, data);
	data->bma253_client = client;
	mutex_init(&data->value_mutex);
	mutex_init(&data->mode_mutex);
	mutex_init(&data->enable_mutex);
	bma253_set_bandwidth(client, BMA253_BW_SET);
	bma253_set_range(client, BMA253_RANGE_SET);

	data->pdata = kmalloc(sizeof(*data->pdata), GFP_KERNEL);
	if (data->pdata == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto pdata_kmalloc_fail;
	}

	if (client->dev.of_node) {
		I("Device Tree parsing.");

		err = bma253_parse_dt(&client->dev, data->pdata);
		if (err) {
			dev_err(&client->dev, "%s: bma253_parse_dt "
					"for pdata failed. err = %d",
					__func__, err);
			goto exit_bma253_parse_dt_fail;
		}
	} else {
		if (client->dev.platform_data != NULL) {
			memcpy(data->pdata, client->dev.platform_data,
			       sizeof(*data->pdata));
		}
	}

	if (data->pdata) {
		data->chip_layout = data->pdata->chip_layout;
		if (!(client->dev.of_node)) {
//			data->pdata->gs_kvalue = gs_kvalue;
			I("%s: Use ATAG Calibration data\n", __func__);
		}
	} else {
		data->chip_layout = 0;
		data->pdata->gs_kvalue = 0;
	}
	I("BMA253 G-sensor I2C driver: gs_kvalue = 0x%X\n",
		data->pdata->gs_kvalue);

	gdata = data;
	D("%s: layout = %d\n", __func__, gdata->chip_layout);

#if defined(BMA253_ENABLE_INT1) || defined(BMA253_ENABLE_INT2)
//	bma253_set_Int_Mode(client, 1);/*latch interrupt 250ms*/
#endif
	/*8,single tap
	  10,orient
	  11,flat*/
/*	bma253_set_Int_Enable(client,8, 1);
	bma253_set_Int_Enable(client,10, 1);
	bma253_set_Int_Enable(client,11, 1);
*/
#ifdef BMA253_ENABLE_INT1
	/* maps interrupt to INT1 pin */
	/*
	bma253_set_int1_pad_sel(client, PAD_LOWG);
	bma253_set_int1_pad_sel(client, PAD_HIGHG);
	bma253_set_int1_pad_sel(client, PAD_SLOP);
	bma253_set_int1_pad_sel(client, PAD_DOUBLE_TAP);
	bma253_set_int1_pad_sel(client, PAD_SINGLE_TAP);
	bma253_set_int1_pad_sel(client, PAD_ORIENT);
	bma253_set_int1_pad_sel(client, PAD_FLAT);*/
#endif


#ifdef BMA253_ENABLE_INT2
	/* maps interrupt to INT2 pin */
	bma253_set_int2_pad_sel(client, PAD_LOWG);
	bma253_set_int2_pad_sel(client, PAD_HIGHG);
	bma253_set_int2_pad_sel(client, PAD_SLOP);
	bma253_set_int2_pad_sel(client, PAD_DOUBLE_TAP);
	bma253_set_int2_pad_sel(client, PAD_SINGLE_TAP);
	bma253_set_int2_pad_sel(client, PAD_ORIENT);
	bma253_set_int2_pad_sel(client, PAD_FLAT);
#endif

#if defined(BMA253_ENABLE_INT1) || defined(BMA253_ENABLE_INT2)
#ifdef IS_MTK_PLATFORM
	err = bma253_irq_registration(client->irq, bma253_irq_handler,
					IRQF_TRIGGER_RISING, client->name, data);
	if (err < 0) {
		E("could not request irq\n");
	}
	data->IRQ = client->irq;
	enable_irq_wake(data->IRQ);
#else
	data->IRQ = client->irq;
	err = request_irq(data->IRQ, bma253_irq_handler, IRQF_TRIGGER_RISING,
			"bma253", data);
	enable_irq_wake(data->IRQ);
	if (err)
		E("could not request irq\n");
#endif

	INIT_WORK(&data->irq_work, bma253_irq_work_func);
#endif

	INIT_DELAYED_WORK(&data->work, bma253_work_func);
	atomic_set(&data->delay, BMA253_MAX_DELAY);
	atomic_set(&data->enable, 0);

	dev = input_allocate_device();
	if (!dev)
	    return -ENOMEM;

#ifdef CONFIG_CIR_ALWAYS_READY

	dev_cir = input_allocate_device();
	if (!dev_cir) {
	    kfree(data);
	    input_free_device(dev);//free the successful dev and return
	    return -ENOMEM;
	}
#endif
	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_I2C;
#ifdef CONFIG_CIR_ALWAYS_READY
	dev_cir->name = "CIRSensor";
	dev_cir->id.bustype = BUS_I2C;

	input_set_capability(dev_cir, EV_REL, SLOP_INTERRUPT);
	input_set_drvdata(dev_cir, data);
#endif
	input_set_capability(dev, EV_ABS, ORIENT_INTERRUPT);
	input_set_capability(dev, EV_ABS, FLAT_INTERRUPT);
	input_set_abs_params(dev, ABS_X, ABSMIN, ABSMAX, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN, ABSMAX, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN, ABSMAX, 0, 0);
	input_set_drvdata(dev, data);

	err = input_register_device(dev);

	if (err < 0) {
	    goto err_register_input_device;
	}


#ifdef CONFIG_CIR_ALWAYS_READY
	err = input_register_device(dev_cir);
	if (err < 0) {
	    goto err_register_input_cir_device;
	}
#endif

	data->input = dev;
#ifdef CONFIG_CIR_ALWAYS_READY
	data->input_cir = dev_cir;
#endif

#ifdef CONFIG_CIR_ALWAYS_READY
	bma253_powerkey_class = class_create(THIS_MODULE, "bma253_powerkey");
	if (IS_ERR(bma253_powerkey_class)) {
		err = PTR_ERR(bma253_powerkey_class);
		bma253_powerkey_class = NULL;
		E("%s: could not allocate bma253_powerkey_class\n", __func__);
		goto err_create_class;
	}

	bma253_powerkey_dev= device_create(bma253_powerkey_class,
				NULL, 0, "%s", "bma253");
	res = device_create_file(bma253_powerkey_dev, &dev_attr_clear_powerkey_flag);
	if (res) {
	        E("%s, create bma253_device_create_file fail!\n", __func__);
		goto err_create_bma253_device_file;
	}

	wake_lock_init(&(data->cir_always_ready_wake_lock), WAKE_LOCK_SUSPEND, "cir_always_ready");
#endif
	data->g_sensor_class = class_create(THIS_MODULE, "htc_g_sensor");
	if (IS_ERR(data->g_sensor_class)) {
		err = PTR_ERR(data->g_sensor_class);
		data->g_sensor_class = NULL;
		E("%s: could not allocate data->g_sensor_class\n", __func__);
		goto err_create_class;
	}

	data->g_sensor_dev = device_create(data->g_sensor_class,
				NULL, 0, "%s", "bma253");
	if (unlikely(IS_ERR(data->g_sensor_dev))) {
		err = PTR_ERR(data->g_sensor_dev);
		data->g_sensor_dev = NULL;
		E("%s: could not allocate data->g_sensor_dev\n", __func__);
		goto err_create_g_sensor_device;
	}

	dev_set_drvdata(data->g_sensor_dev, data);

	err = sysfs_create_group(&data->g_sensor_dev->kobj,
			&bma253_attribute_group);
	if (err < 0)
		goto error_sysfs;

	err = sysfs_create_group(&data->input->dev.kobj,
			&bma253_attribute_group);
	if (err < 0)
		goto error_sysfs;

	data->g_sensor_user_cali_dev = device_create(data->g_sensor_class,
				NULL, 0, "%s", "g_sensor");
	if (unlikely(IS_ERR(data->g_sensor_user_cali_dev))) {
		err = PTR_ERR(data->g_sensor_user_cali_dev);
		data->g_sensor_user_cali_dev = NULL;
		E("%s: could not allocate data->g_sensor_user_cali_dev\n", __func__);
		goto err_create_g_sensor_user_cali_device;
	}

	dev_set_drvdata(data->g_sensor_user_cali_dev, data);
	err = device_create_file(data->g_sensor_user_cali_dev, &dev_attr_g_sensor_user_offset);
	if (err) {
		E("%s: create g_sensor_user_offset fail!\n", __func__);
		goto err_dev_attr_g_sensor_user_offset;
	}

#ifndef IS_MTK_PLATFORM
	err = bma253_sr_ldo_init(1);
	if (err) {
		E("Sensor vreg configuration failed\n");
		data->pdata->power_LPM = NULL;
	}

	err = bma253_sr_lpm(0);
	if (err)
		E("%s: bma253_sr_lpm failed 111\n", __func__);
	err = bma253_sr_lpm(1);
	if (err)
		E("%s: bma253_sr_lpm failed 222\n", __func__);
#endif

	data->bma253_wq = create_singlethread_workqueue("bma253_wq");
	if (!data->bma253_wq) {
		E("%s: can't create workqueue\n", __func__);
		err = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = bma253_early_suspend;
	data->early_suspend.resume = bma253_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	mutex_init(&data->value_mutex);
	mutex_init(&data->mode_mutex);
	mutex_init(&data->enable_mutex);

	debug_execute_point = 0;

	bma_wq = create_singlethread_workqueue("bma253_debug_wq");
	if (!bma_wq) {
		E("%s: can't create workqueue\n", __func__);
		err = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}

	bma253_set_mode(client, BMA253_MODE_SUSPEND);

	I("%s: BMA253 BOSCH driver probe successful", __func__);

	return 0;

err_create_singlethread_workqueue:
	device_remove_file(data->g_sensor_user_cali_dev, &dev_attr_g_sensor_user_offset);
err_dev_attr_g_sensor_user_offset:
	device_unregister(data->g_sensor_user_cali_dev);
err_create_g_sensor_user_cali_device:
	sysfs_remove_group(&data->input->dev.kobj, &bma253_attribute_group);
error_sysfs:
	device_unregister(data->g_sensor_dev);
err_create_g_sensor_device:
	class_destroy(data->g_sensor_class);
#ifdef CONFIG_CIR_ALWAYS_READY
	device_remove_file(bma253_powerkey_dev, &dev_attr_clear_powerkey_flag);
err_create_bma253_device_file:
	class_destroy(bma253_powerkey_class);
#endif
err_create_class:
#ifdef CONFIG_CIR_ALWAYS_READY
	input_unregister_device(data->input_cir);
err_register_input_cir_device:
#endif
	input_unregister_device(data->input);
err_register_input_device:
#ifdef CONFIG_CIR_ALWAYS_READY
	input_free_device(dev_cir);
#endif
	input_free_device(dev);
exit_bma253_parse_dt_fail:
	if (client->dev.of_node && data->pdata)
		kfree(data->pdata);
pdata_kmalloc_fail:
kfree_exit:
	kfree(data);
exit:
	return err;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bma253_early_suspend(struct early_suspend *h)
{
	struct bma253_data *data =
		container_of(h, struct bma253_data, early_suspend);

	D("%s++\n", __func__);

	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable) == 1) {
	    I("suspend mode\n");
	    bma253_set_mode(data->bma253_client, BMA253_MODE_SUSPEND);
	    cancel_delayed_work_sync(&polling_work);
	}
	mutex_unlock(&data->enable_mutex);
}


static void bma253_late_resume(struct early_suspend *h)
{
	struct bma253_data *data =
		container_of(h, struct bma253_data, early_suspend);

	D("%s++\n", __func__);

	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable) == 1) {
		bma253_set_mode(data->bma253_client, BMA253_MODE_NORMAL);
		queue_delayed_work(data->bma253_wq, &polling_work,
				msecs_to_jiffies(atomic_read(&data->delay)));
	}
	mutex_unlock(&data->enable_mutex);
}
#endif

static int bma253_remove(struct i2c_client *client)
{
	struct bma253_data *data = i2c_get_clientdata(client);

	bma253_set_enable(&client->dev, 0);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif
	sysfs_remove_group(&data->input->dev.kobj, &bma253_attribute_group);
	input_unregister_device(data->input);
	device_remove_file(data->g_sensor_user_cali_dev, &dev_attr_g_sensor_user_offset);
	device_unregister(data->g_sensor_user_cali_dev);
	kfree(data);

	return 0;
}
#ifdef CONFIG_PM

static int bma253_suspend(struct device *dev)
{
	struct bma253_data *data = gdata;

	D("%s++\n", __func__);

	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable) == 1) {
	    I("suspend mode\n");
		bma253_set_mode(data->bma253_client, BMA253_MODE_SUSPEND);
		cancel_delayed_work_sync(&polling_work);
	}
	mutex_unlock(&data->enable_mutex);

#ifdef CONFIG_CIR_ALWAYS_READY
	//Add CIR Flag for always ready feature
	if ((data->pdata->power_LPM) && !cir_flag){
#else

	if (data->pdata->power_LPM){
#endif
	    I("suspend + power_LPM\n");
		data->pdata->power_LPM(1);
	}

	return 0;
}

static int bma253_resume(struct device *dev)
{
	struct bma253_data *data = gdata;

	D("%s++\n", __func__);

	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable) == 1) {

		bma253_set_mode(data->bma253_client, BMA253_MODE_NORMAL);
		queue_delayed_work(data->bma253_wq, &polling_work,
				msecs_to_jiffies(atomic_read(&data->delay)));
	}
	mutex_unlock(&data->enable_mutex);

	return 0;
}

#else

#define bma253_suspend		NULL
#define bma253_resume		NULL

#endif /* CONFIG_PM */

static const struct dev_pm_ops bma253_pm_ops = {
#ifdef CONFIG_PM
	.suspend = bma253_suspend,
	.resume  = bma253_resume,
#endif
};

static const struct i2c_device_id bma253_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bma253_id);
#ifdef CONFIG_OF
static struct of_device_id bma253_match_table[] = {
	{.compatible = "htc_g_sensor,bma253_bosch" },
	{},
};
#else
#define bma253_match_table NULL
#endif

static struct i2c_driver bma253_driver = {
#if 0
	.driver = {
		.owner	= THIS_MODULE,
		.name	= SENSOR_NAME,
	},
	.suspend	= bma253_suspend,
	.resume		= bma253_resume,
	.id_table	= bma253_id,
	.probe		= bma253_probe,
	.remove		= __devexit_p(bma253_remove),
#endif

	.driver = {
		.name           = SENSOR_NAME,
		.owner          = THIS_MODULE,
		.of_match_table = bma253_match_table,
#ifdef CONFIG_PM
		.pm             = &bma253_pm_ops,
#endif
	},
	.probe    = bma253_probe,
	.remove   = bma253_remove,
	.id_table = bma253_id,
};
module_i2c_driver(bma253_driver);

#if 0
static int __init BMA253_init(void)
{
	return i2c_add_driver(&bma253_driver);
}

static void __exit BMA253_exit(void)
{
	i2c_del_driver(&bma253_driver);
}

module_init(BMA253_init);
module_exit(BMA253_exit);
#endif

MODULE_DESCRIPTION("BMA253 accelerometer sensor driver");
MODULE_LICENSE("GPL");
