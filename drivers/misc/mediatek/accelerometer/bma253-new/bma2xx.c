/* BMA2XX motion sensor driver
 *
 *
 *
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
//#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

//#include <mach/mt_typedefs.h>
//#include <mach/mt_gpio.h>
//#include <mach/mt_pm_ldo.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#include <cust_acc.h>
#include "bma2xx.h"

#include <accel.h>
#include "batch.h"
#ifdef CUSTOM_KERNEL_SENSORHUB
#include <SCP_sensorHub.h>
#endif//#ifdef CUSTOM_KERNEL_SENSORHUB

/*----------------------------------------------------------------------------*/
#define I2C_DRIVERID_BMA2XX 253
/*----------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/

#define SW_CALIBRATION

/*----------------------------------------------------------------------------*/
#define BMA2XX_AXIS_X          0
#define BMA2XX_AXIS_Y          1
#define BMA2XX_AXIS_Z          2
#define BMA2XX_DATA_LEN        6
#define BMA2XX_DEV_NAME        "BMA2XX"
/*----------------------------------------------------------------------------*/

/*********/
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id bma2xx_i2c_id[] = {{BMA2XX_DEV_NAME,0},{}};
//static struct i2c_board_info __initdata i2c_BMA2XX={ I2C_BOARD_INFO(BMA2XX_DEV_NAME, 0x18)};

#ifdef CONFIG_OF
static const struct of_device_id accel_of_match[] = {
        {.compatible = "mediatek,gsensor"},
        {},
};
#endif

#define COMPATIABLE_NAME "mediatek,bma253_new"

/*----------------------------------------------------------------------------*/
static int bma2xx_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int bma2xx_i2c_remove(struct i2c_client *client);
#if !defined(CONFIG_HAS_EARLYSUSPEND) || !defined(USE_EARLY_SUSPEND)
static int bma2xx_suspend(struct i2c_client *client, pm_message_t msg);
static int bma2xx_resume(struct i2c_client *client);
#endif

static int gsensor_local_init(void);
static int  gsensor_remove(void);
#ifdef CUSTOM_KERNEL_SENSORHUB
static int gsensor_setup_irq(void);
#endif//#ifdef CUSTOM_KERNEL_SENSORHUB
static int gsensor_set_delay(u64 ns);
/*----------------------------------------------------------------------------*/
typedef enum {
	ADX_TRC_FILTER  = 0x01,
	ADX_TRC_RAWDATA = 0x02,
	ADX_TRC_IOCTL   = 0x04,
	ADX_TRC_CALI	= 0X08,
	ADX_TRC_INFO	= 0X10,
} ADX_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
	u8  whole;
	u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
	struct scale_factor scalefactor;
	int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
	s16 raw[C_MAX_FIR_LENGTH][BMA2XX_AXES_NUM];
	int sum[BMA2XX_AXES_NUM];
	int num;
	int idx;
};
/*----------------------------------------------------------------------------*/
struct bma2xx_i2c_data {
	struct i2c_client *client;
	struct acc_hw *hw;
	struct hwmsen_convert   cvt;
#ifdef CUSTOM_KERNEL_SENSORHUB
	struct work_struct	irq_work;
#endif//#ifdef CUSTOM_KERNEL_SENSORHUB
	int16_t sensor_type;
	/*misc*/
	struct data_resolution *reso;
	atomic_t                trace;
	atomic_t                suspend;
	atomic_t                selftest;
	atomic_t				filter;
	s16                     cali_sw[BMA2XX_AXES_NUM+1];

	/*data*/
	s8                      offset[BMA2XX_AXES_NUM+1];  /*+1: for 4-byte alignment*/
	s16                     data[BMA2XX_AXES_NUM+1];

#ifdef CUSTOM_KERNEL_SENSORHUB
	int                     SCP_init_done;
#endif//#ifdef CUSTOM_KERNEL_SENSORHUB


#if defined(CONFIG_BMA2XX_LOWPASS)
	atomic_t                firlen;
	atomic_t                fir_en;
	struct data_filter      fir;
#endif 
	/*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(USE_EARLY_SUSPEND)
	struct early_suspend    early_drv;
#endif   
	u8                      bandwidth;
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver bma2xx_i2c_driver = {
	.driver = {
		//        .owner          = THIS_MODULE,
		.name           = BMA2XX_DEV_NAME,
#ifdef CONFIG_OF
        	.of_match_table = accel_of_match,
#endif
	},
	.probe      		= bma2xx_i2c_probe,
	.remove    			= bma2xx_i2c_remove,
#ifndef USE_EARLY_SUSPEND
	.suspend            = bma2xx_suspend,
	.resume             = bma2xx_resume,
#endif
	.id_table = bma2xx_i2c_id,
	//	.address_data = &bma2xx_addr_data,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *bma2xx_i2c_client = NULL;
static struct bma2xx_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = true;
static int sensor_suspend = 0;
static struct GSENSOR_VECTOR3D gsensor_gain;
//static char selftestRes[8]= {0}; 
static DEFINE_MUTEX(gsensor_mutex);
static DEFINE_MUTEX(gsensor_scp_en_mutex);


static bool enable_status = false;

static int gsensor_init_flag =-1; // 0<==>OK -1 <==> fail
static struct acc_init_info bma2xx_init_info = {
	.name = BMA2XX_DEV_NAME,
	.init = gsensor_local_init,
	.uninit = gsensor_remove,
};

#define GSE_DEBUG
/*----------------------------------------------------------------------------*/
#ifdef GSE_DEBUG
//#define GSE_TAG                  "[Gsensor] "
#define GSE_TAG                  "[bma2xx] "
#define GSE_FUN(f)               printk(GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk(GSE_TAG fmt, ##args)
#else 
#define GSE_TAG                 
#define GSE_FUN(f)               
#define GSE_ERR(fmt, args...)    
#define GSE_LOG(fmt, args...) 
#endif

static struct acc_hw accel_cust;
static struct acc_hw *hw = &accel_cust;

static const struct bma2x2_type_map_t sensor_type_map[] = {
	{BMA222E_CHIP_ID, BMA222E_TYPE, "BMA222E/223"},
	{BMA250E_CHIP_ID, BMA250E_TYPE, "BMA250E"},
	{BMA255_CHIP_ID, BMA255_TYPE, "BMA255/254/253"},
	{BMA280_CHIP_ID, BMA280_TYPE, "BMA280"},
};

/*----------------------------------------------------------------------------*/
static struct data_resolution bma2xx_data_resolution[] = {
	/* combination by {FULL_RES,RANGE}*/
	{{ 15, 6}, 64},	/*bma222E/223: +/-2g  in 8-bit resolution:  15.6 mg/LSB*/
	{{ 3, 91}, 256},	/*bma250E: +/-2g  in 10-bit resolution:  3.91 mg/LSB*/
	{{ 0, 98}, 1024},	/*bma255/254/253: +/-2g  in 12-bit resolution:  0.975 mg/LSB*/
	{{ 0, 24}, 4096},	/*bma280: +/-2g  in 14-bit resolution:  0.244 mg/LSB*/
};
/*----------------------------------------------------------------------------*/
static struct data_resolution bma2xx_offset_resolution[] = {
	{{ 15, 6}, 64},	/*bma222E/223: +/-2g  in 8-bit resolution:  15.6 mg/LSB*/
	{{ 3, 91}, 256},	/*bma250E: +/-2g  in 10-bit resolution:  3.91 mg/LSB*/
	{{ 0, 98}, 1024},	/*bma255/254/253: +/-2g  in 12-bit resolution:  0.975 mg/LSB*/
	{{ 0, 24}, 4096},	/*bma280: +/-2g  in 14-bit resolution:  0.244 mg/LSB*/
};

/*----------------------------------------------------------------------------*/
static int bma_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	u8 beg = addr;
	int err;
	struct i2c_msg msgs[2]={{0},{0}};

	mutex_lock(&gsensor_mutex);

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len =1;
	msgs[0].buf = &beg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len =len;
	msgs[1].buf = data;

	if (!client)
	{
		mutex_unlock(&gsensor_mutex);
		return -EINVAL;
	}
	else if (len > C_I2C_FIFO_SIZE) 
	{
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&gsensor_mutex);
		return -EINVAL;
	}
	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) 
	{
		GSE_ERR("i2c_transfer error: (%d %p %d) %d\n",addr, data, len, err);
		err = -EIO;
	} 
	else 
	{
		err = 0;
	}
	mutex_unlock(&gsensor_mutex);
	return err;

}
EXPORT_SYMBOL(bma_i2c_read_block);
static int bma_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{   /*because address also occupies one byte, the maximum length for write is 7 bytes*/
	int err, idx, num;
	char buf[C_I2C_FIFO_SIZE];
	err =0;
	mutex_lock(&gsensor_mutex);
	if (!client)
	{
		mutex_unlock(&gsensor_mutex);
		return -EINVAL;
	}
	else if (len >= C_I2C_FIFO_SIZE) 
	{        
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&gsensor_mutex);
		return -EINVAL;
	}    

	num = 0;
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
	{
		buf[num++] = data[idx];
	}

	err = i2c_master_send(client, buf, num);
	if (err < 0)
	{
		GSE_ERR("send command error!!\n");
		mutex_unlock(&gsensor_mutex);
		return -EFAULT;
	} 
	mutex_unlock(&gsensor_mutex);
	return err;
}
EXPORT_SYMBOL(bma_i2c_write_block);

/*----------------------------------------------------------------------------*/
/*--------------------Add by Susan----------------------------------*/
#ifdef CUSTOM_KERNEL_SENSORHUB
int BMA2XX_SCP_SetPowerMode(bool enable, int sensorType)
{
	static bool gsensor_scp_en_status = false;
	static unsigned int gsensor_scp_en_map = 0;
	SCP_SENSOR_HUB_DATA req;
	int len;
	int err = 0;

	mutex_lock(&gsensor_scp_en_mutex);

	if (sensorType >= 32)
	{
		GSE_ERR("Out of index!\n");
		return -1;
	}

	if (true == enable)
	{
		gsensor_scp_en_map |= (1<<sensorType);
	}
	else
	{
		gsensor_scp_en_map &= ~(1<<sensorType);
	}

	if (0 == gsensor_scp_en_map)
		enable = false;
	else
		enable = true;

	if (gsensor_scp_en_status != enable)
	{
		gsensor_scp_en_status = enable;

		req.activate_req.sensorType = ID_ACCELEROMETER;
		req.activate_req.action = SENSOR_HUB_ACTIVATE;
		req.activate_req.enable = enable;
		len = sizeof(req.activate_req);
		err = SCP_sensorHub_req_send(&req, &len, 1);
		if (err)
		{
			GSE_ERR("SCP_sensorHub_req_send fail\n");
		}
	}

	mutex_unlock(&gsensor_scp_en_mutex);

	return err;
}
EXPORT_SYMBOL(BMA2XX_SCP_SetPowerMode);
#endif 
/*----------------------------------------------------------------------------*/

/*
bool __attribute__((weak)) hwPowerOn(MT65XX_POWER powerId, MT65XX_POWER_VOLTAGE powerVolt, char *mode_name)
{
	return 0;
}

bool __attribute__((weak)) hwPowerDown(MT65XX_POWER powerId, char *mode_name)
{
	return 0;
}
*/
/*--------------------BMA2XX power control function----------------------------------*/
static void BMA2XX_power(struct acc_hw *hw, unsigned int on) 
{
/*
#ifdef __USE_LINUX_REGULATOR_FRAMEWORK__
#else
#ifndef FPGA_EARLY_PORTING
	static unsigned int power_on = 0;

	if(hw->power_id != POWER_NONE_MACRO)		// have externel LDO
	{        
		GSE_LOG("power %s\n", on ? "on" : "off");
		if(power_on == on)	// power status not change
		{
			GSE_LOG("ignore power control: %d\n", on);
		}
		else if(on)	// power on
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "BMA2XX"))
			{
				GSE_ERR("power on fails!!\n");
			}
		}
		else	// power off
		{
			if (!hwPowerDown(hw->power_id, "BMA2XX"))
			{
				GSE_ERR("power off fail!!\n");
			}			  
		}
	}
	power_on = on;
#endif //#ifndef FPGA_EARLY_PORTING
#endif //__USE_LINUX_REGULATOR_FRAMEWORK__
*/
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int BMA2XX_SetDataResolution(struct bma2xx_i2c_data *obj)
{

	/*set g sensor dataresolution here*/

	/*BMA2XX only can set to 10-bit dataresolution, so do nothing in bma2xx driver here*/

	/*end of set dataresolution*/



	/*we set measure range from -2g to +2g in BMA2XX_SetDataFormat(client, BMA2XX_RANGE_2G), 
	  and set 10-bit dataresolution BMA2XX_SetDataResolution()*/

	/*so bma2xx_data_resolution[0] set value as {{ 3, 9}, 256} when declaration, and assign the value to obj->reso here*/  

	obj->reso = &bma2xx_data_resolution[obj->sensor_type];
	return 0;

	/*if you changed the measure range, for example call: BMA2XX_SetDataFormat(client, BMA2XX_RANGE_4G), 
	  you must set the right value to bma2xx_data_resolution*/

}
/*----------------------------------------------------------------------------*/
static int BMA2XX_ReadData(struct i2c_client *client, s16 data[BMA2XX_AXES_NUM])
{
	struct bma2xx_i2c_data *priv = i2c_get_clientdata(client);        
	int err = 0;
#if 0 //CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	u8 addr = BMA2XX_REG_DATAXLOW;
	u8 buf[BMA2XX_DATA_LEN] = {0};
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

#if 0 // CUSTOM_KERNEL_SENSORHUB
	req.get_data_req.sensorType = ID_ACCELEROMETER;
	req.get_data_req.action = SENSOR_HUB_GET_DATA;
	len = sizeof(req.get_data_req);
	err = SCP_sensorHub_req_send(&req, &len, 1);
	if (err)
	{
		GSE_ERR("SCP_sensorHub_req_send!\n");
		return err;
	}

	if (ID_ACCELEROMETER != req.get_data_rsp.sensorType ||
			SENSOR_HUB_GET_DATA != req.get_data_rsp.action ||
			0 != req.get_data_rsp.errCode)
	{
		GSE_ERR("error : %d\n", req.get_data_rsp.errCode);
		return req.get_data_rsp.errCode;
	}

	len -= offsetof(SCP_SENSOR_HUB_GET_DATA_RSP, int8_Data);

	if (6 == len)
	{
		data[BMA2XX_AXIS_X] = req.get_data_rsp.int16_Data[0];
		data[BMA2XX_AXIS_Y] = req.get_data_rsp.int16_Data[1];
		data[BMA2XX_AXIS_Z] = req.get_data_rsp.int16_Data[2];
	}
	else
	{
		GSE_ERR("data length fail : %d\n", len);
	}

	if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
	{
		//show data
	}
#endif//#ifdef CUSTOM_KERNEL_SENSORHUB
	if(NULL == client)
	{
		err = -EINVAL;
	}
	else if((err = bma_i2c_read_block(client, addr, buf, 6))!=0)
	{
		GSE_ERR("error: %d\n", err);
	}
	else
	{
		switch(priv->sensor_type) {
			case BMA222E_TYPE:
				data[BMA2XX_AXIS_X] =
					(s16)(((u16)buf[1]) << 8 | buf[0]) >> 8;
				data[BMA2XX_AXIS_Y] =
					(s16)(((u16)buf[3]) << 8 | buf[2]) >> 8;
				data[BMA2XX_AXIS_Z] =
					(s16)(((u16)buf[5]) << 8 | buf[4]) >> 8;
				break;
			case BMA250E_TYPE:
				/* 10 bit */
				data[BMA2XX_AXIS_X] =
					(s16)(((u16)buf[1]) << 8 | buf[0]) >> 6;
				data[BMA2XX_AXIS_Y] =
					(s16)(((u16)buf[3]) << 8 | buf[2]) >> 6;
				data[BMA2XX_AXIS_Z] =
					(s16)(((u16)buf[5]) << 8 | buf[4]) >> 6;
				break;
			case BMA255_TYPE:
				/* 12 bit */
				data[BMA2XX_AXIS_X] =
					(s16)(((u16)buf[1]) << 8 | buf[0]) >> 4;
				data[BMA2XX_AXIS_Y] =
					(s16)(((u16)buf[3]) << 8 | buf[2]) >> 4;
				data[BMA2XX_AXIS_Z] =
					(s16)(((u16)buf[5]) << 8 | buf[4]) >> 4;
				break;
			case BMA280_TYPE:
				/* 14 bit */
				data[BMA2XX_AXIS_X] =                                                          
					(s16)(((u16)buf[1]) << 8 | buf[0]) >> 2;                    
				data[BMA2XX_AXIS_Y] =                                                          
					(s16)(((u16)buf[3]) << 8 | buf[2]) >> 2;                    
				data[BMA2XX_AXIS_Z] =                                                          
					(s16)(((u16)buf[5]) << 8 | buf[4]) >> 2;
				break;
			default:
				GSE_ERR("Unsupport sensor_type %d, error!!\n ", priv->sensor_type);
		}

		if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
		{
			GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d] before\n", data[BMA2XX_AXIS_X], data[BMA2XX_AXIS_Y], data[BMA2XX_AXIS_Z],
					data[BMA2XX_AXIS_X], data[BMA2XX_AXIS_Y], data[BMA2XX_AXIS_Z]);
		}

		if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
		{
			GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d] after\n", data[BMA2XX_AXIS_X], data[BMA2XX_AXIS_Y], data[BMA2XX_AXIS_Z],
					data[BMA2XX_AXIS_X], data[BMA2XX_AXIS_Y], data[BMA2XX_AXIS_Z]);
		}
#ifdef CONFIG_BMA2XX_LOWPASS
		if(atomic_read(&priv->filter))
		{
			if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
			{
				int idx, firlen = atomic_read(&priv->firlen);   
				if(priv->fir.num < firlen)
				{                
					priv->fir.raw[priv->fir.num][BMA2XX_AXIS_X] = data[BMA2XX_AXIS_X];
					priv->fir.raw[priv->fir.num][BMA2XX_AXIS_Y] = data[BMA2XX_AXIS_Y];
					priv->fir.raw[priv->fir.num][BMA2XX_AXIS_Z] = data[BMA2XX_AXIS_Z];
					priv->fir.sum[BMA2XX_AXIS_X] += data[BMA2XX_AXIS_X];
					priv->fir.sum[BMA2XX_AXIS_Y] += data[BMA2XX_AXIS_Y];
					priv->fir.sum[BMA2XX_AXIS_Z] += data[BMA2XX_AXIS_Z];
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
								priv->fir.raw[priv->fir.num][BMA2XX_AXIS_X], priv->fir.raw[priv->fir.num][BMA2XX_AXIS_Y], priv->fir.raw[priv->fir.num][BMA2XX_AXIS_Z],
								priv->fir.sum[BMA2XX_AXIS_X], priv->fir.sum[BMA2XX_AXIS_Y], priv->fir.sum[BMA2XX_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				}
				else
				{
					idx = priv->fir.idx % firlen;
					priv->fir.sum[BMA2XX_AXIS_X] -= priv->fir.raw[idx][BMA2XX_AXIS_X];
					priv->fir.sum[BMA2XX_AXIS_Y] -= priv->fir.raw[idx][BMA2XX_AXIS_Y];
					priv->fir.sum[BMA2XX_AXIS_Z] -= priv->fir.raw[idx][BMA2XX_AXIS_Z];
					priv->fir.raw[idx][BMA2XX_AXIS_X] = data[BMA2XX_AXIS_X];
					priv->fir.raw[idx][BMA2XX_AXIS_Y] = data[BMA2XX_AXIS_Y];
					priv->fir.raw[idx][BMA2XX_AXIS_Z] = data[BMA2XX_AXIS_Z];
					priv->fir.sum[BMA2XX_AXIS_X] += data[BMA2XX_AXIS_X];
					priv->fir.sum[BMA2XX_AXIS_Y] += data[BMA2XX_AXIS_Y];
					priv->fir.sum[BMA2XX_AXIS_Z] += data[BMA2XX_AXIS_Z];
					priv->fir.idx++;
					data[BMA2XX_AXIS_X] = priv->fir.sum[BMA2XX_AXIS_X]/firlen;
					data[BMA2XX_AXIS_Y] = priv->fir.sum[BMA2XX_AXIS_Y]/firlen;
					data[BMA2XX_AXIS_Z] = priv->fir.sum[BMA2XX_AXIS_Z]/firlen;
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
								priv->fir.raw[idx][BMA2XX_AXIS_X], priv->fir.raw[idx][BMA2XX_AXIS_Y], priv->fir.raw[idx][BMA2XX_AXIS_Z],
								priv->fir.sum[BMA2XX_AXIS_X], priv->fir.sum[BMA2XX_AXIS_Y], priv->fir.sum[BMA2XX_AXIS_Z],
								data[BMA2XX_AXIS_X], data[BMA2XX_AXIS_Y], data[BMA2XX_AXIS_Z]);
					}
				}
			}
		}	
#endif         
	}

	return err;
}
/*----------------------------------------------------------------------------*/

static int BMA2XX_ReadOffset(struct i2c_client *client, s8 ofs[BMA2XX_AXES_NUM])
{    
	int err;
	err = 0;


#ifdef SW_CALIBRATION
	ofs[0]=ofs[1]=ofs[2]=0x0;
#else
	if((err = bma_i2c_read_block(client, BMA2XX_REG_OFSX, ofs, BMA2XX_AXES_NUM)))
	{
		GSE_ERR("error: %d\n", err);
	}
#endif
	//printk("offesx=%x, y=%x, z=%x",ofs[0],ofs[1],ofs[2]);

	return err;    
}

/*----------------------------------------------------------------------------*/
static int BMA2XX_ResetCalibration(struct i2c_client *client)
{
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	//u8 ofs[4]={0,0,0,0};
	int err = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA data;
	BMA2XX_CUST_DATA *pCustData;
	unsigned int len;
#endif

#ifdef GSENSOR_UT
	GSE_FUN();
#endif

#ifdef CUSTOM_KERNEL_SENSORHUB
	if (0 != obj->SCP_init_done)
	{
		pCustData = (BMA2XX_CUST_DATA *)&data.set_cust_req.custData;

		data.set_cust_req.sensorType = ID_ACCELEROMETER;
		data.set_cust_req.action = SENSOR_HUB_SET_CUST;
		pCustData->resetCali.action = BMA2XX_CUST_ACTION_RESET_CALI;
		len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->resetCali);
		SCP_sensorHub_req_send(&data, &len, 1);
	}
#endif
	printk("[bma2xx] %s\n", __FUNCTION__);
	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
	return err;    
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_ReadCalibration(struct i2c_client *client, int dat[BMA2XX_AXES_NUM])
{
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	int  err = 0;
	int mul;

	GSE_FUN();
#ifdef SW_CALIBRATION
	mul = 0;//only SW Calibration, disable HW Calibration
#else
	if ((err = BMA2XX_ReadOffset(client, obj->offset))) {
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}    
	mul = obj->reso->sensitivity/bma2xx_offset_resolution[obj->sensor_type].sensitivity;
#endif

	dat[obj->cvt.map[BMA2XX_AXIS_X]] = obj->cvt.sign[BMA2XX_AXIS_X]*(obj->offset[BMA2XX_AXIS_X]*mul + obj->cali_sw[BMA2XX_AXIS_X]);
	dat[obj->cvt.map[BMA2XX_AXIS_Y]] = obj->cvt.sign[BMA2XX_AXIS_Y]*(obj->offset[BMA2XX_AXIS_Y]*mul + obj->cali_sw[BMA2XX_AXIS_Y]);
	dat[obj->cvt.map[BMA2XX_AXIS_Z]] = obj->cvt.sign[BMA2XX_AXIS_Z]*(obj->offset[BMA2XX_AXIS_Z]*mul + obj->cali_sw[BMA2XX_AXIS_Z]);                        

	printk("[bma2xx] %s %d %d %d\n", __FUNCTION__, dat[obj->cvt.map[BMA2XX_AXIS_X]], dat[obj->cvt.map[BMA2XX_AXIS_Y]], dat[obj->cvt.map[BMA2XX_AXIS_Z]]);
	return err;
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_ReadCalibrationEx(struct i2c_client *client, int act[BMA2XX_AXES_NUM], int raw[BMA2XX_AXES_NUM])
{  
	/*raw: the raw calibration data; act: the actual calibration data*/
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	int mul;
	err = 0;


#ifdef SW_CALIBRATION
	mul = 0;//only SW Calibration, disable HW Calibration
#else
	if((err = BMA2XX_ReadOffset(client, obj->offset)))
	{
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}   
	mul = obj->reso->sensitivity/bma2xx_offset_resolution[obj->sensor_type].sensitivity;
#endif

	raw[BMA2XX_AXIS_X] = obj->offset[BMA2XX_AXIS_X]*mul + obj->cali_sw[BMA2XX_AXIS_X];
	raw[BMA2XX_AXIS_Y] = obj->offset[BMA2XX_AXIS_Y]*mul + obj->cali_sw[BMA2XX_AXIS_Y];
	raw[BMA2XX_AXIS_Z] = obj->offset[BMA2XX_AXIS_Z]*mul + obj->cali_sw[BMA2XX_AXIS_Z];

	act[obj->cvt.map[BMA2XX_AXIS_X]] = obj->cvt.sign[BMA2XX_AXIS_X]*raw[BMA2XX_AXIS_X];
	act[obj->cvt.map[BMA2XX_AXIS_Y]] = obj->cvt.sign[BMA2XX_AXIS_Y]*raw[BMA2XX_AXIS_Y];
	act[obj->cvt.map[BMA2XX_AXIS_Z]] = obj->cvt.sign[BMA2XX_AXIS_Z]*raw[BMA2XX_AXIS_Z];                        

	printk("[bma2xx] %s\n", __FUNCTION__);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_WriteCalibration(struct i2c_client *client, int dat[BMA2XX_AXES_NUM])
{
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[BMA2XX_AXES_NUM], raw[BMA2XX_AXES_NUM];
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA data;
	BMA2XX_CUST_DATA *pCustData;
	unsigned int len;
#endif


	if(0 != (err = BMA2XX_ReadCalibrationEx(client, cali, raw)))	/*offset will be updated in obj->offset*/
	{ 
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

	printk("[bma2xx] %s\n", __FUNCTION__);
	GSE_LOG("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
			raw[BMA2XX_AXIS_X], raw[BMA2XX_AXIS_Y], raw[BMA2XX_AXIS_Z],
			obj->offset[BMA2XX_AXIS_X], obj->offset[BMA2XX_AXIS_Y], obj->offset[BMA2XX_AXIS_Z],
			obj->cali_sw[BMA2XX_AXIS_X], obj->cali_sw[BMA2XX_AXIS_Y], obj->cali_sw[BMA2XX_AXIS_Z]);

#ifdef CUSTOM_KERNEL_SENSORHUB
	pCustData = (BMA2XX_CUST_DATA *)data.set_cust_req.custData;
	data.set_cust_req.sensorType = ID_ACCELEROMETER;
	data.set_cust_req.action = SENSOR_HUB_SET_CUST;
	pCustData->setCali.action = BMA2XX_CUST_ACTION_SET_CALI;
	pCustData->setCali.data[BMA2XX_AXIS_X] = dat[BMA2XX_AXIS_X];
	pCustData->setCali.data[BMA2XX_AXIS_Y] = dat[BMA2XX_AXIS_Y];
	pCustData->setCali.data[BMA2XX_AXIS_Z] = dat[BMA2XX_AXIS_Z];
	len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->setCali);
	SCP_sensorHub_req_send(&data, &len, 1);
#endif

	/*calculate the real offset expected by caller*/
	cali[BMA2XX_AXIS_X] += dat[BMA2XX_AXIS_X];
	cali[BMA2XX_AXIS_Y] += dat[BMA2XX_AXIS_Y];
	cali[BMA2XX_AXIS_Z] += dat[BMA2XX_AXIS_Z];

	GSE_LOG("UPDATE: (%+3d %+3d %+3d)\n", 
			dat[BMA2XX_AXIS_X], dat[BMA2XX_AXIS_Y], dat[BMA2XX_AXIS_Z]);

#ifdef SW_CALIBRATION
	obj->cali_sw[BMA2XX_AXIS_X] = obj->cvt.sign[BMA2XX_AXIS_X]*(cali[obj->cvt.map[BMA2XX_AXIS_X]]);
	obj->cali_sw[BMA2XX_AXIS_Y] = obj->cvt.sign[BMA2XX_AXIS_Y]*(cali[obj->cvt.map[BMA2XX_AXIS_Y]]);
	obj->cali_sw[BMA2XX_AXIS_Z] = obj->cvt.sign[BMA2XX_AXIS_Z]*(cali[obj->cvt.map[BMA2XX_AXIS_Z]]);	
#else
	int divisor = obj->reso->sensitivity/lsb;//modified
	obj->offset[BMA2XX_AXIS_X] = (s8)(obj->cvt.sign[BMA2XX_AXIS_X]*(cali[obj->cvt.map[BMA2XX_AXIS_X]])/(divisor));
	obj->offset[BMA2XX_AXIS_Y] = (s8)(obj->cvt.sign[BMA2XX_AXIS_Y]*(cali[obj->cvt.map[BMA2XX_AXIS_Y]])/(divisor));
	obj->offset[BMA2XX_AXIS_Z] = (s8)(obj->cvt.sign[BMA2XX_AXIS_Z]*(cali[obj->cvt.map[BMA2XX_AXIS_Z]])/(divisor));

	/*convert software calibration using standard calibration*/
	obj->cali_sw[BMA2XX_AXIS_X] = obj->cvt.sign[BMA2XX_AXIS_X]*(cali[obj->cvt.map[BMA2XX_AXIS_X]])%(divisor);
	obj->cali_sw[BMA2XX_AXIS_Y] = obj->cvt.sign[BMA2XX_AXIS_Y]*(cali[obj->cvt.map[BMA2XX_AXIS_Y]])%(divisor);
	obj->cali_sw[BMA2XX_AXIS_Z] = obj->cvt.sign[BMA2XX_AXIS_Z]*(cali[obj->cvt.map[BMA2XX_AXIS_Z]])%(divisor);

	GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
			obj->offset[BMA2XX_AXIS_X]*divisor + obj->cali_sw[BMA2XX_AXIS_X], 
			obj->offset[BMA2XX_AXIS_Y]*divisor + obj->cali_sw[BMA2XX_AXIS_Y], 
			obj->offset[BMA2XX_AXIS_Z]*divisor + obj->cali_sw[BMA2XX_AXIS_Z], 
			obj->offset[BMA2XX_AXIS_X], obj->offset[BMA2XX_AXIS_Y], obj->offset[BMA2XX_AXIS_Z],
			obj->cali_sw[BMA2XX_AXIS_X], obj->cali_sw[BMA2XX_AXIS_Y], obj->cali_sw[BMA2XX_AXIS_Z]);

	if((err = hwmsen_write_block(obj->client, BMA2XX_REG_OFSX, obj->offset, BMA2XX_AXES_NUM)))
	{
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
#endif
	mdelay(1);
	return err;
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[2]={0};    
	int res = 0;
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);


	res = bma_i2c_read_block(client,BMA2XX_REG_DEVID,databuf,0x1);
	if(res < 0)
	{
		goto exit_BMA2XX_CheckDeviceID;
	}

	switch(databuf[0])  {
		case BMA222E_CHIP_ID:
			obj->sensor_type = BMA222E_TYPE;
			break;
		case BMA250E_CHIP_ID:
			obj->sensor_type = BMA250E_TYPE;
			break;
		case BMA255_CHIP_ID:
			obj->sensor_type = BMA255_TYPE;
			break;
		case BMA280_CHIP_ID:
			obj->sensor_type = BMA280_TYPE;
			break;
		default:
			obj->sensor_type = UNSUPPORT_TYPE;
			goto exit_BMA2XX_CheckDeviceID;
	}

	GSE_LOG("BMA2XX_CheckDeviceID %d done, the sensor is %s\n ",
			databuf[0], sensor_type_map[obj->sensor_type].sensor_name);

exit_BMA2XX_CheckDeviceID:
	if (res < 0)
	{
		GSE_ERR("BMA2XX_CheckDeviceID %d failt!\n ", BMA2XX_ERR_I2C);
		return BMA2XX_ERR_I2C;
	}
	mdelay(1);
	return BMA2XX_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_SetPowerMode(struct i2c_client *client, bool enable)
{
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;

	u8 databuf[2];
	u8 addr = BMA2XX_REG_POWER_CTL;

	if(enable == sensor_power )
	{
		GSE_LOG("Sensor power status is newest!\n");
		return BMA2XX_SUCCESS;
	}

	if(bma_i2c_read_block(client, addr, databuf, 0x01))
	{
		GSE_ERR("read power ctl register err!\n");
		return BMA2XX_ERR_I2C;
	}
	GSE_LOG("set power mode value = 0x%x!\n",databuf[0]);
	mdelay(1);
	if(enable == 1)
	{
		databuf[0] &= ~BMA2XX_MEASURE_MODE;
	}
	else
	{
		databuf[0] |= BMA2XX_MEASURE_MODE;
	}

	res = bma_i2c_write_block(client,BMA2XX_REG_POWER_CTL,databuf,0x1);
	if(res < 0)
	{
		GSE_LOG("set power mode failed!\n");
		return BMA2XX_ERR_I2C;
	}
	else if(atomic_read(&obj->trace) & ADX_TRC_INFO)
	{
		GSE_LOG("set power mode ok %d!\n", databuf[1]);
	}


	sensor_power = enable;
	mdelay(1);
	//GSE_LOG("leave Sensor power status is sensor_power = %d\n",sensor_power);
	return BMA2XX_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10]={0};    
	int res = 0;

	if(bma_i2c_read_block(client, BMA2XX_REG_DATA_FORMAT, databuf, 0x01))
	{
		printk("bma2xx read Dataformat failt \n");
		return BMA2XX_ERR_I2C;
	}
	mdelay(1);
	databuf[0] &= ~BMA2XX_RANGE_MASK;
	databuf[0] |= dataformat;

	res = bma_i2c_write_block(client,BMA2XX_REG_DATA_FORMAT,databuf,0x1);
	if(res < 0)
	{
		return BMA2XX_ERR_I2C;
	}

	//printk("BMA2XX_SetDataFormat OK! \n");
	mdelay(1);
	return BMA2XX_SetDataResolution(obj);    
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	u8 databuf[10]={0};    
	int res = 0;

	if(bma_i2c_read_block(client, BMA2XX_REG_BW_RATE, databuf, 0x01))
	{
		printk("bma2xx read rate failt \n");
		return BMA2XX_ERR_I2C;
	}
	mdelay(1);
	databuf[0] &= ~BMA2XX_BW_MASK;
	databuf[0] |= bwrate;


	res = bma_i2c_write_block(client,BMA2XX_REG_BW_RATE,databuf,0x1);
	if(res < 0)
	{
		return BMA2XX_ERR_I2C;
	}
	mdelay(1);
	//printk("BMA2XX_SetBWRate OK! \n");

	return BMA2XX_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_SetIntEnable(struct i2c_client *client, u8 intenable)
{
	//u8 databuf[10];    
	int res = 0;

	res = hwmsen_write_byte(client, BMA2XX_INT_REG_1, 0x00);
	if(res != BMA2XX_SUCCESS) 
	{
		return res;
	}
	mdelay(1);
	res = hwmsen_write_byte(client, BMA2XX_INT_REG_2, 0x00);
	if(res != BMA2XX_SUCCESS) 
	{
		return res;
	}
	//printk("BMA2XX disable interrupt ...\n");

	/*for disable interrupt function*/
	mdelay(1);
	return BMA2XX_SUCCESS;	  
}

/*----------------------------------------------------------------------------*/
static int bma2xx_init_client(struct i2c_client *client, int reset_cali)
{
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;

	GSE_FUN();


	res = BMA2XX_CheckDeviceID(client); 
	if(res != BMA2XX_SUCCESS)
	{
		return res;
	}	
	//printk("BMA2XX_CheckDeviceID ok \n");

	res = BMA2XX_SetBWRate(client, BMA2XX_BW_100HZ);
	if(res != BMA2XX_SUCCESS ) 
	{
		return res;
	}
	//printk("BMA2XX_SetBWRate OK!\n");

	res = BMA2XX_SetDataFormat(client, BMA2XX_RANGE_2G);
	if(res != BMA2XX_SUCCESS) 
	{
		return res;
	}
	//printk("BMA2XX_SetDataFormat OK!\n");

	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;

#ifdef CUSTOM_KERNEL_SENSORHUB
	res = gsensor_setup_irq();
	if(res != BMA2XX_SUCCESS)
	{
		return res;
	}
#endif//#ifdef CUSTOM_KERNEL_SENSORHUB

	res = BMA2XX_SetIntEnable(client, 0x00);        
	if(res != BMA2XX_SUCCESS)
	{
		return res;
	}
	//printk("BMA2XX disable interrupt function!\n");

	res = BMA2XX_SetPowerMode(client, enable_status);//false);//
	if(res != BMA2XX_SUCCESS)
	{
		return res;
	}
	//printk("BMA2XX_SetPowerMode OK!\n");


	if(0 != reset_cali)
	{ 
		/*reset calibration only in power on*/
		res = BMA2XX_ResetCalibration(client);
		if(res != BMA2XX_SUCCESS)
		{
			return res;
		}
	}
	GSE_LOG("bma2xx_init_client OK!\n");
#ifdef CONFIG_BMA2XX_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif
	return BMA2XX_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];    

	memset(databuf, 0, sizeof(u8)*10);

	if((NULL == buf)||(bufsize<=30))
	{
		return -1;
	}

	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "BMA2XX Chip");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct bma2xx_i2c_data *obj = (struct bma2xx_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[BMA2XX_AXES_NUM];
	int res = 0;
	memset(databuf, 0, sizeof(u8)*10);

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	if(sensor_suspend == 1)
	{
		//GSE_LOG("sensor in suspend read not data!\n");
		return 0;
	}

	if((res = BMA2XX_ReadData(client, obj->data))!=0)
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
#if 0 // CUSTOM_KERNEL_SENSORHUB
		acc[BMA2XX_AXIS_X] = obj->data[BMA2XX_AXIS_X];
		acc[BMA2XX_AXIS_Y] = obj->data[BMA2XX_AXIS_Y];
		acc[BMA2XX_AXIS_Z] = obj->data[BMA2XX_AXIS_Z];
		//data has been calibrated in SCP side.
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
		//GSE_LOG("raw data x=%d, y=%d, z=%d \n",obj->data[BMA2XX_AXIS_X],obj->data[BMA2XX_AXIS_Y],obj->data[BMA2XX_AXIS_Z]);
		obj->data[BMA2XX_AXIS_X] += obj->cali_sw[BMA2XX_AXIS_X];
		obj->data[BMA2XX_AXIS_Y] += obj->cali_sw[BMA2XX_AXIS_Y];
		obj->data[BMA2XX_AXIS_Z] += obj->cali_sw[BMA2XX_AXIS_Z];

		//printk("cali_sw x=%d, y=%d, z=%d \n",obj->cali_sw[BMA2XX_AXIS_X],obj->cali_sw[BMA2XX_AXIS_Y],obj->cali_sw[BMA2XX_AXIS_Z]);

		/*remap coordinate*/
#ifdef CONFIG_BMA253_V36BML_SWAP_XY
		acc[obj->cvt.map[BMA2XX_AXIS_Y]] = obj->cvt.sign[BMA2XX_AXIS_X]*obj->data[BMA2XX_AXIS_X];
		acc[obj->cvt.map[BMA2XX_AXIS_X]] = obj->cvt.sign[BMA2XX_AXIS_Y]*obj->data[BMA2XX_AXIS_Y];
//		acc[BMA2XX_AXIS_X] = acc[BMA2XX_AXIS_X] + 268130;
#else
		acc[obj->cvt.map[BMA2XX_AXIS_X]] = obj->cvt.sign[BMA2XX_AXIS_X]*obj->data[BMA2XX_AXIS_X];
		acc[obj->cvt.map[BMA2XX_AXIS_Y]] = obj->cvt.sign[BMA2XX_AXIS_Y]*obj->data[BMA2XX_AXIS_Y];
#endif
		acc[obj->cvt.map[BMA2XX_AXIS_Z]] = obj->cvt.sign[BMA2XX_AXIS_Z]*obj->data[BMA2XX_AXIS_Z];
		//printk("cvt x=%d, y=%d, z=%d \n",obj->cvt.sign[BMA2XX_AXIS_X],obj->cvt.sign[BMA2XX_AXIS_Y],obj->cvt.sign[BMA2XX_AXIS_Z]);


		//GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[BMA2XX_AXIS_X], acc[BMA2XX_AXIS_Y], acc[BMA2XX_AXIS_Z]);

		//Out put the mg
		//printk("mg acc=%d, GRAVITY=%d, sensityvity=%d \n",acc[BMA2XX_AXIS_X],GRAVITY_EARTH_1000,obj->reso->sensitivity);
		acc[BMA2XX_AXIS_X] = acc[BMA2XX_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
//#ifdef CONFIG_BMA253_V36BML_SWAP_XY
#if 0
		acc[BMA2XX_AXIS_Y] = acc[BMA2XX_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity * (-1) - 268130;
#else
		acc[BMA2XX_AXIS_Y] = acc[BMA2XX_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
#endif
		acc[BMA2XX_AXIS_Z] = acc[BMA2XX_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;		
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB


		sprintf(buf, "%04x %04x %04x", acc[BMA2XX_AXIS_X], acc[BMA2XX_AXIS_Y], acc[BMA2XX_AXIS_Z]);
		if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)
		{
			GSE_LOG("gsensor data: %s!\n", buf);
		}
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_ReadRawData(struct i2c_client *client, char *buf)
{
	struct bma2xx_i2c_data *obj = (struct bma2xx_i2c_data*)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client)
	{
		return EINVAL;
	}

	if(0 != (res = BMA2XX_ReadData(client, obj->data)))
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return EIO;
	}
	else
	{
		sprintf(buf, "BMA2XX_ReadRawData %04x %04x %04x", obj->data[BMA2XX_AXIS_X], 
				obj->data[BMA2XX_AXIS_Y], obj->data[BMA2XX_AXIS_Z]);

	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma2xx_i2c_client;
	char strbuf[BMA2XX_BUFSIZE];
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	BMA2XX_ReadChipInfo(client, strbuf, BMA2XX_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}

#if 0
static ssize_t gsensor_init(struct device_driver *ddri, char *buf, size_t count)
{
	struct i2c_client *client = bma2xx_i2c_client;
	char strbuf[BMA2XX_BUFSIZE];

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	bma2xx_init_client(client, 1);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);			
}
#endif


/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma2xx_i2c_client;
	char strbuf[BMA2XX_BUFSIZE];

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	BMA2XX_ReadSensorData(client, strbuf, BMA2XX_BUFSIZE);
	//BMA2XX_ReadRawData(client, strbuf);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);            
}

#if 0
static ssize_t show_sensorrawdata_value(struct device_driver *ddri, char *buf, size_t count)
{
	struct i2c_client *client = bma2xx_i2c_client;
	char strbuf[BMA2XX_BUFSIZE];

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	//BMA2XX_ReadSensorData(client, strbuf, BMA2XX_BUFSIZE);
	BMA2XX_ReadRawData(client, strbuf);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);			
}
#endif

/*----------------------------------------------------------------------------*/
#if 1
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma2xx_i2c_client;
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	int err, len = 0, mul;
	int tmp[BMA2XX_AXES_NUM];

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);



	if(0 != (err = BMA2XX_ReadOffset(client, obj->offset)))
	{
		return -EINVAL;
	}
	else if(0 != (err = BMA2XX_ReadCalibration(client, tmp)))
	{
		return -EINVAL;
	}
	else
	{    
		mul = obj->reso->sensitivity/bma2xx_offset_resolution[obj->sensor_type].sensitivity;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,                        
				obj->offset[BMA2XX_AXIS_X], obj->offset[BMA2XX_AXIS_Y], obj->offset[BMA2XX_AXIS_Z],
				obj->offset[BMA2XX_AXIS_X], obj->offset[BMA2XX_AXIS_Y], obj->offset[BMA2XX_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1, 
				obj->cali_sw[BMA2XX_AXIS_X], obj->cali_sw[BMA2XX_AXIS_Y], obj->cali_sw[BMA2XX_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
				obj->offset[BMA2XX_AXIS_X]*mul + obj->cali_sw[BMA2XX_AXIS_X],
				obj->offset[BMA2XX_AXIS_Y]*mul + obj->cali_sw[BMA2XX_AXIS_Y],
				obj->offset[BMA2XX_AXIS_Z]*mul + obj->cali_sw[BMA2XX_AXIS_Z],
				tmp[BMA2XX_AXIS_X], tmp[BMA2XX_AXIS_Y], tmp[BMA2XX_AXIS_Z]);

		return len;
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = bma2xx_i2c_client;  
	int err, x, y, z;
	int dat[BMA2XX_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if(0 != (err = BMA2XX_ResetCalibration(client)))
		{
			GSE_ERR("reset offset err = %d\n", err);
		}	
	}
	else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
	{
		dat[BMA2XX_AXIS_X] = x;
		dat[BMA2XX_AXIS_Y] = y;
		dat[BMA2XX_AXIS_Z] = z;
		if(0 != (err = BMA2XX_WriteCalibration(client, dat)))
		{
			GSE_ERR("write calibration err = %d\n", err);
		}		
	}
	else
	{
		GSE_ERR("invalid format\n");
	}

	return count;
}
#endif

/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_BMA2XX_LOWPASS
	struct i2c_client *client = bma2xx_i2c_client;
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	if(atomic_read(&obj->firlen))
	{
		int idx, len = atomic_read(&obj->firlen);
		GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for(idx = 0; idx < len; idx++)
		{
			GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][BMA2XX_AXIS_X], obj->fir.raw[idx][BMA2XX_AXIS_Y], obj->fir.raw[idx][BMA2XX_AXIS_Z]);
		}

		GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[BMA2XX_AXIS_X], obj->fir.sum[BMA2XX_AXIS_Y], obj->fir.sum[BMA2XX_AXIS_Z]);
		GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[BMA2XX_AXIS_X]/len, obj->fir.sum[BMA2XX_AXIS_Y]/len, obj->fir.sum[BMA2XX_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_BMA2XX_LOWPASS
	struct i2c_client *client = bma2xx_i2c_client;  
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if(1 != sscanf(buf, "%d", &firlen))
	{
		GSE_ERR("invallid format\n");
	}
	else if(firlen > C_MAX_FIR_LENGTH)
	{
		GSE_ERR("exceeds maximum filter length\n");
	}
	else
	{ 
		atomic_set(&obj->firlen, firlen);
		if(NULL == firlen)
		{
			atomic_set(&obj->fir_en, 0);
		}
		else
		{
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif    
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct bma2xx_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bma2xx_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}	
	else
	{
		GSE_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
	}

	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;    
	struct bma2xx_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}	

	if(obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n", 
				obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);   
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_power_status_value(struct device_driver *ddri, char *buf)
{

	u8 databuf[2];    
	//int res = 0;
	u8 addr = BMA2XX_REG_POWER_CTL;
	struct bma2xx_i2c_data *obj = obj_i2c_data;
	if(bma_i2c_read_block(obj->client, addr, databuf, 0x01))
	{
		GSE_ERR("read power ctl register err!\n");
		return 1;
	}

	if(sensor_power)
		GSE_LOG("G sensor is in work mode, sensor_power = %d\n", sensor_power);
	else
		GSE_LOG("G sensor is in standby mode, sensor_power = %d\n", sensor_power);

	return snprintf(buf, PAGE_SIZE, "%x\n", databuf[0]);
}
static ssize_t show_chip_orientation(struct device_driver *ddri, char *pbBuf)
{
	ssize_t          _tLength = 0;

	GSE_LOG("[%s] default direction: %d\n", __FUNCTION__, hw->direction);

	_tLength = snprintf(pbBuf, PAGE_SIZE, "default direction = %d\n", hw->direction);

	return (_tLength);
}


static ssize_t store_chip_orientation(struct device_driver *ddri, const char *pbBuf, size_t tCount)
{
	int                       _nDirection = 0;
	struct bma2xx_i2c_data   *_pt_i2c_obj = obj_i2c_data;

	if (NULL == _pt_i2c_obj)
		return (0);

	if (1 == sscanf(pbBuf, "%d", &_nDirection))
	{
		if (hwmsen_get_convert(_nDirection, &_pt_i2c_obj->cvt))
			GSE_ERR("ERR: fail to set direction\n");
	}

	GSE_LOG("[%s] set direction: %d\n", __FUNCTION__, _nDirection);

	return (tCount);
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,   S_IWUSR | S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, show_cali_value,          store_cali_value);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, show_firlen_value,        store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value,         store_trace_value);
static DRIVER_ATTR(status,     S_IRUGO, show_status_value,        NULL);
static DRIVER_ATTR(powerstatus,               S_IRUGO, show_power_status_value,        NULL);
static DRIVER_ATTR(orientation, S_IWUSR | S_IRUGO, show_chip_orientation, store_chip_orientation);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *bma2xx_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,
	&driver_attr_powerstatus,
	&driver_attr_orientation,
};
/*----------------------------------------------------------------------------*/
static int bma2xx_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(bma2xx_attr_list)/sizeof(bma2xx_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(0 != (err = driver_create_file(driver, bma2xx_attr_list[idx])))
		{            
			GSE_ERR("driver_create_file (%s) = %d\n", bma2xx_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int bma2xx_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(bma2xx_attr_list)/sizeof(bma2xx_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}


	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, bma2xx_attr_list[idx]);
	}


	return err;
}

/*----------------------------------------------------------------------------*/
#ifdef CUSTOM_KERNEL_SENSORHUB
static void gsensor_irq_work(struct work_struct *work)
{
	struct bma2xx_i2c_data *obj = obj_i2c_data;
	struct scp_acc_hw scp_hw;
	BMA2XX_CUST_DATA *p_cust_data;
	SCP_SENSOR_HUB_DATA data;
	int max_cust_data_size_per_packet;
	int i;
	uint sizeOfCustData;
	uint len;
	char *p = (char *)&scp_hw;

	GSE_FUN();

	scp_hw.i2c_num = obj->hw->i2c_num;
	scp_hw.direction = obj->hw->direction;
	scp_hw.power_id = obj->hw->power_id;
	scp_hw.power_vol = obj->hw->power_vol;
	scp_hw.firlen = obj->hw->firlen;
	memcpy(scp_hw.i2c_addr, obj->hw->i2c_addr, sizeof(obj->hw->i2c_addr));
	scp_hw.power_vio_id = obj->hw->power_vio_id;
	scp_hw.power_vio_vol = obj->hw->power_vio_vol;
	scp_hw.is_batch_supported = obj->hw->is_batch_supported;

	p_cust_data = (BMA2XX_CUST_DATA *)data.set_cust_req.custData;
	sizeOfCustData = sizeof(scp_hw);
	max_cust_data_size_per_packet = sizeof(data.set_cust_req.custData) - offsetof(BMA2XX_SET_CUST, data);

	GSE_ERR("sizeOfCustData = %d, max_cust_data_size_per_packet = %d\n", sizeOfCustData, max_cust_data_size_per_packet);
	GSE_ERR("offset %d\n", offsetof(BMA2XX_SET_CUST, data));

	for (i=0;sizeOfCustData>0;i++)
	{
		data.set_cust_req.sensorType = ID_ACCELEROMETER;
		data.set_cust_req.action = SENSOR_HUB_SET_CUST;
		p_cust_data->setCust.action = BMA2XX_CUST_ACTION_SET_CUST;
		p_cust_data->setCust.part = i;
		if (sizeOfCustData > max_cust_data_size_per_packet)
		{
			len = max_cust_data_size_per_packet;
		}
		else
		{
			len = sizeOfCustData;
		}

		memcpy(p_cust_data->setCust.data, p, len);
		sizeOfCustData -= len;
		p += len;

		GSE_ERR("i= %d, sizeOfCustData = %d, len = %d \n", i, sizeOfCustData, len);        
		len += offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + offsetof(BMA2XX_SET_CUST, data);
		GSE_ERR("data.set_cust_req.sensorType= %d \n",  data.set_cust_req.sensorType);  		
		SCP_sensorHub_req_send(&data, &len, 1);

	}
	p_cust_data = (BMA2XX_CUST_DATA *)&data.set_cust_req.custData;
	data.set_cust_req.sensorType = ID_ACCELEROMETER;
	data.set_cust_req.action = SENSOR_HUB_SET_CUST;
	p_cust_data->resetCali.action = BMA2XX_CUST_ACTION_RESET_CALI;
	len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(p_cust_data->resetCali);
	SCP_sensorHub_req_send(&data, &len, 1);
	obj->SCP_init_done = 1;
}
/*----------------------------------------------------------------------------*/
static int gsensor_irq_handler(void* data, uint len)
{
	struct bma2xx_i2c_data *obj = obj_i2c_data;
	SCP_SENSOR_HUB_DATA_P rsp = (SCP_SENSOR_HUB_DATA_P)data;

	GSE_ERR("gsensor_irq_handler len = %d, type = %d, action = %d, errCode = %d\n", len, rsp->rsp.sensorType, rsp->rsp.action, rsp->rsp.errCode);
	if(!obj)
	{
		return -1;
	}

	switch(rsp->rsp.action)
	{
		case SENSOR_HUB_NOTIFY:
			switch(rsp->notify_rsp.event)
			{
				case SCP_INIT_DONE:
					schedule_work(&obj->irq_work);
					GSE_ERR("OK sensor hub notify\n");
					break;
				default:
					GSE_ERR("Error sensor hub notify\n");
					break;
			}
			break;
		default:
			GSE_ERR("Error sensor hub action\n");
			break;
	}

	return 0;
}

static int gsensor_setup_irq()
{
	int err = 0;



	err = SCP_sensorHub_rsp_registration(ID_ACCELEROMETER, gsensor_irq_handler);

	return err;
}
#endif//#ifdef CUSTOM_KERNEL_SENSORHUB
/****************************************************************************** 
 * Function Configuration
 ******************************************************************************/
static int bma2xx_open(struct inode *inode, struct file *file)
{
	file->private_data = bma2xx_i2c_client;

	if(file->private_data == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int bma2xx_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
//static int bma2xx_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
//       unsigned long arg)
static long bma2xx_unlocked_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)

{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct bma2xx_i2c_data *obj = (struct bma2xx_i2c_data*)i2c_get_clientdata(client);	
	char strbuf[BMA2XX_BUFSIZE];
	void __user *data;
	struct SENSOR_DATA sensor_data;
	long err = 0;
	int cali[3];

	//GSE_FUN(f);
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case GSENSOR_IOCTL_INIT:
			bma2xx_init_client(client, 0);	
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}

			BMA2XX_ReadChipInfo(client, strbuf, BMA2XX_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}				 
			break;	  

		case GSENSOR_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			BMA2XX_SetPowerMode(client,true);	
			BMA2XX_ReadSensorData(client, strbuf, BMA2XX_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;

		case GSENSOR_IOCTL_READ_GAIN:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			

			if(copy_to_user(data, &gsensor_gain, sizeof(struct GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case GSENSOR_IOCTL_READ_RAW_DATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			BMA2XX_ReadRawData(client, strbuf);
			if(copy_to_user(data, &strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;	  

		case GSENSOR_IOCTL_SET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;	  
			}
			if(atomic_read(&obj->suspend))
			{
				GSE_ERR("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			}
			else
			{
				cali[BMA2XX_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[BMA2XX_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[BMA2XX_AXIS_Z] = sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;			  
				err = BMA2XX_WriteCalibration(client, cali);			 
			}
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			err = BMA2XX_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(0 != (err = BMA2XX_ReadCalibration(client, cali)))
			{
				break;
			}

			sensor_data.x = cali[BMA2XX_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.y = cali[BMA2XX_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.z = cali[BMA2XX_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}		
			break;


		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;

	}

	return err;
}


#ifdef CONFIG_COMPAT
static long bma2xx_compat_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	long err = 0;

	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd)
	{
		case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
			if (arg32 == NULL)
			{
				err = -EINVAL;
				break;    
			}

			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg32);
			if (err){
				GSE_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
				return err;
			}
			break;
		case COMPAT_GSENSOR_IOCTL_SET_CALI:
			if (arg32 == NULL)
			{
				err = -EINVAL;
				break;    
			}

			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);
			if (err){
				GSE_ERR("GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed.");
				return err;
			}
			break;
		case COMPAT_GSENSOR_IOCTL_GET_CALI:
			if (arg32 == NULL)
			{
				err = -EINVAL;
				break;    
			}

			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg32);
			if (err){
				GSE_ERR("GSENSOR_IOCTL_GET_CALI unlocked_ioctl failed.");
				return err;
			}
			break;
		case COMPAT_GSENSOR_IOCTL_CLR_CALI:
			if (arg32 == NULL)
			{
				err = -EINVAL;
				break;    
			}

			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg32);
			if (err){
				GSE_ERR("GSENSOR_IOCTL_CLR_CALI unlocked_ioctl failed.");
				return err;
			}
			break;

		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;

	}

	return err;
}
#endif
/*----------------------------------------------------------------------------*/
static struct file_operations bma2xx_fops = {
	.owner = THIS_MODULE,
	.open = bma2xx_open,
	.release = bma2xx_release,
	.unlocked_ioctl = bma2xx_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = bma2xx_compat_ioctl,
#endif
};
/*----------------------------------------------------------------------------*/
static struct miscdevice bma2xx_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &bma2xx_fops,
};
/*----------------------------------------------------------------------------*/
#ifndef USE_EARLY_SUSPEND
/*----------------------------------------------------------------------------*/
static int bma2xx_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);    
	int err = 0;  
	mutex_lock(&gsensor_scp_en_mutex);
	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(obj == NULL)
		{
			GSE_ERR("null pointer!!\n");
			mutex_unlock(&gsensor_scp_en_mutex);
			return -EINVAL;
		}
		atomic_set(&obj->suspend, 1);
#ifdef CUSTOM_KERNEL_SENSORHUB
		if(0 != (err = BMA2XX_SCP_SetPowerMode(false, ID_ACCELEROMETER)))
#else
			if(0 != (err = BMA2XX_SetPowerMode(obj->client, false)))
#endif
			{
				GSE_ERR("write power control fail!!\n");
				mutex_unlock(&gsensor_scp_en_mutex);
				return -EINVAL;
			} 
#ifndef CUSTOM_KERNEL_SENSORHUB
		BMA2XX_power(obj->hw, 0);
#endif
	}
	mutex_unlock(&gsensor_scp_en_mutex);
	return err;
}
/*----------------------------------------------------------------------------*/
static int bma2xx_resume(struct i2c_client *client)
{
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);        
	int err;

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

#ifndef CUSTOM_KERNEL_SENSORHUB
	BMA2XX_power(obj->hw, 1);
#endif

#ifndef CUSTOM_KERNEL_SENSORHUB
	if(0 != (err = bma2xx_init_client(client, 0)))
#else
		if(0 != (err = BMA2XX_SCP_SetPowerMode(enable_status, ID_ACCELEROMETER)))
#endif
		{
			GSE_ERR("initialize client fail!!\n");

			return err;        
		}
	atomic_set(&obj->suspend, 0);


	return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void bma2xx_early_suspend(struct early_suspend *h) 
{
	struct bma2xx_i2c_data *obj = container_of(h, struct bma2xx_i2c_data, early_drv);   
	int err;

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend, 1); 

	GSE_FUN();  
	u8 databuf[2]; //for debug read power control register to see the value is OK
	if(bma_i2c_read_block(obj->client, BMA2XX_REG_POWER_CTL, databuf, 0x01))
	{
		GSE_ERR("read power ctl register err!\n");
		return BMA2XX_ERR_I2C;
	}
	if(databuf[0]==0xff)//if the value is ff the gsensor will not work anymore, any i2c operations won't be vaild
		GSE_LOG("before BMA2XX_SetPowerMode in suspend databuf = 0x%x\n",databuf[0]);
#ifndef CUSTOM_KERNEL_SENSORHUB
	if((err = BMA2XX_SetPowerMode(obj->client, false)))
#else
		if((err = BMA2XX_SCP_SetPowerMode(false, ID_ACCELEROMETER)))
#endif
		{
			GSE_ERR("write power control fail!!\n");

			return;
		}
	if(bma_i2c_read_block(obj->client, BMA2XX_REG_POWER_CTL, databuf, 0x01)) //for debug read power control register to see the value is OK
	{
		GSE_ERR("read power ctl register err!\n");

		return BMA2XX_ERR_I2C;
	}
	if(databuf[0]==0xff)//if the value is ff the gsensor will not work anymore, any i2c operations won't be vaild
		GSE_LOG("after BMA2XX_SetPowerMode suspend err databuf = 0x%x\n",databuf[0]);
	sensor_suspend = 1;

#ifndef CUSTOM_KERNEL_SENSORHUB
	BMA2XX_power(obj->hw, 0);
#endif

}
/*----------------------------------------------------------------------------*/
static void bma2xx_late_resume(struct early_suspend *h)
{
	struct bma2xx_i2c_data *obj = container_of(h, struct bma2xx_i2c_data, early_drv);         
	int err;

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}

#ifndef CUSTOM_KERNEL_SENSORHUB
	BMA2XX_power(obj->hw, 1);

#endif



	u8 databuf[2];//for debug read power control register to see the value is OK
	if(bma_i2c_read_block(obj->client, BMA2XX_REG_POWER_CTL, databuf, 0x01))
	{
		GSE_ERR("read power ctl register err!\n");		

		return BMA2XX_ERR_I2C;

	}
	if(databuf[0]==0xff)//if the value is ff the gsensor will not work anymore, any i2c operations won't be vaild

		GSE_LOG("before bma2xx_init_client databuf = 0x%x\n",databuf[0]);
#ifndef CUSTOM_KERNEL_SENSORHUB	
	if((err = bma2xx_init_client(obj->client, 0)))
#else
		if((err = BMA2XX_SCP_SetPowerMode(enable_status, ID_ACCELEROMETER)))
#endif
		{
			GSE_ERR("initialize client fail!!\n");

			return;        
		}

	if(bma_i2c_read_block(obj->client, BMA2XX_REG_POWER_CTL, databuf, 0x01)) //for debug read power control register to see the value is OK
	{
		GSE_ERR("read power ctl register err!\n");

		return BMA2XX_ERR_I2C;
	}

	if(databuf[0]==0xff)//if the value is ff the gsensor will not work anymore, any i2c operations won't be vaild
		GSE_LOG("after bma2xx_init_client databuf = 0x%x\n",databuf[0]);
	sensor_suspend = 0;

	atomic_set(&obj->suspend, 0);    
}
/*----------------------------------------------------------------------------*/
#endif /*USE_EARLY_SUSPEND*/
/*----------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int gsensor_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
/*----------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int gsensor_enable_nodata(int en)
{
	int err = 0;

	if(((en == 0) && (sensor_power == false)) ||((en == 1) && (sensor_power == true)))
	{
		enable_status = sensor_power;
		GSE_LOG("Gsensor device have updated!\n");
	}
	else
	{
		enable_status = !sensor_power;
		if (atomic_read(&obj_i2c_data->suspend) == 0)
		{
#ifdef CUSTOM_KERNEL_SENSORHUB
			err = BMA2XX_SCP_SetPowerMode(enable_status, ID_ACCELEROMETER);
			if (0 == err)
			{
				sensor_power = enable_status;
			}
#else
			err = BMA2XX_SetPowerMode(obj_i2c_data->client, enable_status);
#endif
			GSE_LOG("Gsensor not in suspend BMA2XX_SetPowerMode!, enable_status = %d\n",enable_status);
		}
		else
		{
			GSE_LOG("Gsensor in suspend and can not enable or disable!enable_status = %d\n",enable_status);
		}
	}


	if(err != BMA2XX_SUCCESS)
	{
		GSE_ERR("gsensor_enable_nodata fail!\n");
		return -1;
	}

	GSE_ERR("gsensor_enable_nodata OK!\n");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int gsensor_set_delay(u64 ns)
{
	int err = 0;
	int value;
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
#else//#ifdef CUSTOM_KERNEL_SENSORHUB
	int sample_delay;
#endif//#ifdef CUSTOM_KERNEL_SENSORHUB

	value = (int)ns/1000/1000;

#ifdef CUSTOM_KERNEL_SENSORHUB
	req.set_delay_req.sensorType = ID_ACCELEROMETER;
	req.set_delay_req.action = SENSOR_HUB_SET_DELAY;
	req.set_delay_req.delay = value;
	len = sizeof(req.activate_req);
	err = SCP_sensorHub_req_send(&req, &len, 1);
	if (err)
	{
		GSE_ERR("SCP_sensorHub_req_send!\n");
		return err;
	}
#else//#ifdef CUSTOM_KERNEL_SENSORHUB    
	if(value <= 5)
	{
		sample_delay = BMA2XX_BW_200HZ;
	}
	else if(value <= 10)
	{
		sample_delay = BMA2XX_BW_100HZ;
	}
	else
	{
		sample_delay = BMA2XX_BW_100HZ;
	}

	mutex_lock(&gsensor_scp_en_mutex);
	err = BMA2XX_SetBWRate(obj_i2c_data->client, sample_delay);
	mutex_unlock(&gsensor_scp_en_mutex);
	if(err != BMA2XX_SUCCESS ) //0x2C->BW=100Hz
	{
		GSE_ERR("Set delay parameter error!\n");
		return -1;
	}

	if(value >= 50)
	{
		atomic_set(&obj_i2c_data->filter, 0);
	}
	else
	{	
#if defined(CONFIG_BMA2XX_LOWPASS)
		priv->fir.num = 0;
		priv->fir.idx = 0;
		priv->fir.sum[BMA2XX_AXIS_X] = 0;
		priv->fir.sum[BMA2XX_AXIS_Y] = 0;
		priv->fir.sum[BMA2XX_AXIS_Z] = 0;
		atomic_set(&priv->filter, 1);
#endif
	}
#endif//#ifdef CUSTOM_KERNEL_SENSORHUB

	GSE_LOG("gsensor_set_delay (%d)\n",value);

	return 0;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static int gsensor_get_data(int* x ,int* y,int* z, int* status)
{
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
	int err = 0;
#else
	char buff[BMA2XX_BUFSIZE];
#endif

#ifdef CUSTOM_KERNEL_SENSORHUB
	req.get_data_req.sensorType = ID_ACCELEROMETER;
	req.get_data_req.action = SENSOR_HUB_GET_DATA;
	len = sizeof(req.get_data_req);
	err = SCP_sensorHub_req_send(&req, &len, 1);
	if (err)
	{
		GSE_ERR("SCP_sensorHub_req_send!\n");
		return err;
	}

	if (ID_ACCELEROMETER != req.get_data_rsp.sensorType ||
			SENSOR_HUB_GET_DATA != req.get_data_rsp.action ||
			0 != req.get_data_rsp.errCode)
	{
		GSE_ERR("error : %d\n", req.get_data_rsp.errCode);
		return req.get_data_rsp.errCode;
	}

	*x = (int)req.get_data_rsp.int16_Data[0]*GRAVITY_EARTH_1000/1000;
	*y = (int)req.get_data_rsp.int16_Data[1]*GRAVITY_EARTH_1000/1000;
	*z = (int)req.get_data_rsp.int16_Data[2]*GRAVITY_EARTH_1000/1000;
	GSE_ERR("x = %d, y = %d, z = %d\n", *x, *y, *z);

	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
#else
	mutex_lock(&gsensor_scp_en_mutex);
	BMA2XX_ReadSensorData(obj_i2c_data->client, buff, BMA2XX_BUFSIZE);
	mutex_unlock(&gsensor_scp_en_mutex);
	sscanf(buff, "%x %x %x", x, y, z);				
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static int bma2xx_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct bma2xx_i2c_data *obj;
	struct acc_control_path ctl={0};
	struct acc_data_path data={0};
	int err = 0;
	int retry = 0;
	GSE_FUN();

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(struct bma2xx_i2c_data));

	obj->hw = hw;

	if(0 != (err = hwmsen_get_convert(obj->hw->direction, &obj->cvt)))
	{
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}

#ifdef CUSTOM_KERNEL_SENSORHUB
	INIT_WORK(&obj->irq_work, gsensor_irq_work);
#endif//#ifdef CUSTOM_KERNEL_SENSORHUB

	obj_i2c_data = obj;
	obj->client = client;
#ifdef FPGA_EARLY_PORTING
	obj->client->timing = 100; 
#else
	obj->client->timing = 400;
#endif
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);

#ifdef CONFIG_BMA2XX_LOWPASS
	if(obj->hw->firlen > C_MAX_FIR_LENGTH)
	{
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	}	
	else
	{
		atomic_set(&obj->firlen, obj->hw->firlen);
	}

	if(atomic_read(&obj->firlen) > 0)
	{
		atomic_set(&obj->fir_en, 1);
	}

#endif

	bma2xx_i2c_client = new_client;	

	for(retry = 0; retry < 3; retry++){
		if(0 != (err = bma2xx_init_client(new_client, 1)))
		{
			GSE_ERR("bma2xx_device init cilent fail time: %d\n", retry);
			continue;
		}
	}
	if(err != 0)
		goto exit_init_failed;


	if(0 != (err = misc_register(&bma2xx_device)))
	{
		GSE_ERR("bma2xx_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if(0 != (err = bma2xx_create_attr(&bma2xx_init_info.platform_diver_addr->driver)))
	{
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	ctl.open_report_data= gsensor_open_report_data;
	ctl.enable_nodata = gsensor_enable_nodata;
	ctl.set_delay  = gsensor_set_delay;
	//ctl.batch = gsensor_set_batch;
	ctl.is_report_input_direct = false;

#ifdef CUSTOM_KERNEL_SENSORHUB
	ctl.is_support_batch = obj->hw->is_batch_supported;
#else
	ctl.is_support_batch = false;
#endif

	err = acc_register_control_path(&ctl);
	if(err)
	{
		GSE_ERR("register acc control path err\n");
		goto exit_kfree;
	}

	data.get_data = gsensor_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if(err)
	{
		GSE_ERR("register acc data path err\n");
		goto exit_kfree;
	}

	err = batch_register_support_info(ID_ACCELEROMETER,ctl.is_support_batch, 102, 0); //divisor is 1000/9.8
	if(err)
	{
		GSE_ERR("register gsensor batch support err = %d\n", err);
		goto exit_create_attr_failed;
	}

#ifdef USE_EARLY_SUSPEND
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
		obj->early_drv.suspend  = bma2xx_early_suspend,
		obj->early_drv.resume   = bma2xx_late_resume,    
		register_early_suspend(&obj->early_drv);
#endif 

	gsensor_init_flag =0;
	GSE_LOG("%s: OK\n", __func__);    
	return 0;

exit_create_attr_failed:
	misc_deregister(&bma2xx_device);
exit_misc_device_register_failed:
exit_init_failed:
	//i2c_detach_client(new_client);
exit_kfree:
	kfree(obj);
exit:
	GSE_ERR("%s: err = %d\n", __func__, err);
	gsensor_init_flag = -1;
	return err;
}

/*----------------------------------------------------------------------------*/
static int bma2xx_i2c_remove(struct i2c_client *client)
{
	int err = 0;	

	if(0 != (err = bma2xx_delete_attr(&bma2xx_init_info.platform_diver_addr->driver)))
	{
		GSE_ERR("bma2xx_delete_attr fail: %d\n", err);
	}

	if(0 != (err = misc_deregister(&bma2xx_device)))
	{
		GSE_ERR("misc_deregister fail: %d\n", err);
	}

	bma2xx_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
/*----------------------------------------------------------------------------*/
static int  gsensor_local_init(void)
{
	GSE_FUN();

	BMA2XX_power(hw, 1);
	if(i2c_add_driver(&bma2xx_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}
	if(-1 == gsensor_init_flag)
	{
		return -1;
	}
	//printk("fwq loccal init---\n");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int gsensor_remove()
{
	GSE_FUN();    
	BMA2XX_power(hw, 0);    
	i2c_del_driver(&bma2xx_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int __init bma2xx_init(void)
{
	GSE_FUN();
	hw = get_accel_dts_func(COMPATIABLE_NAME, hw);
	if (!hw)
		GSE_ERR("get dts info fail\n");
	GSE_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num);
	acc_driver_add(&bma2xx_init_info);
	return 0;    
}

/*----------------------------------------------------------------------------*/
static void __exit bma2xx_exit(void)
{
	GSE_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(bma2xx_init);
module_exit(bma2xx_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BMA2XX I2C driver");
MODULE_AUTHOR("Xiaoli.li@mediatek.com");
