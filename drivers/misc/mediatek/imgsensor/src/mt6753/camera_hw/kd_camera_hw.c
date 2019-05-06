#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <linux/xlog.h>

////
#include "kd_camera_typedef.h"

#include "../../../../include/mt-plat/mt_gpio.h"
//#include <mt-plat/mt_pm_ldo.h>
#include "cust_gpio_usage.h"
#include "gpio_const.h"
#define VOL_2800 2800000
#define VOL_1800 1800000
#define VOL_1500 1500000
#define VOL_1200 1200000
#define VOL_1000 1000000
////


#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, args...)    pr_debug(PFX  fmt, ##args)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG(fmt, args...)  pr_err(PFX fmt, ##args)  //PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         pr_err(PFX fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...) \
                do {    \
                   pr_debug(PFX  fmt, ##args); \
                } while(0)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#define PK_XLOG_INFO(fmt, args...)
#endif

/*
#ifndef BOOL
typedef unsigned char BOOL;
#endif
*/

/* Mark: need to verify whether ISP_MCLK1_EN is required in here //Jessy @2014/06/04*/
/*
extern void ISP_MCLK1_EN(BOOL En);
extern void ISP_MCLK2_EN(BOOL En);
extern void ISP_MCLK3_EN(BOOL En);
*/

/* GPIO Pin control*/
struct platform_device *cam_plt_dev = NULL;
struct pinctrl *camctrl = NULL;
struct pinctrl_state *cam0_pnd_h = NULL;
struct pinctrl_state *cam0_pnd_l = NULL;
struct pinctrl_state *cam0_rst_h = NULL;
struct pinctrl_state *cam0_rst_l = NULL;
struct pinctrl_state *cam1_pnd_h = NULL;
struct pinctrl_state *cam1_pnd_l = NULL;
struct pinctrl_state *cam1_rst_h = NULL;
struct pinctrl_state *cam1_rst_l = NULL;
struct pinctrl_state *cam_ldo0_h = NULL;
struct pinctrl_state *cam_ldo0_l = NULL;
struct pinctrl_state *cam_ldo1_h = NULL;
struct pinctrl_state *cam_ldo1_l = NULL;
struct pinctrl_state *cam_pin32 = NULL;

int mtkcam_gpio_set(int PinIdx, int PwrType, int Val)
{
	int ret = 0;
	printk("PinIdx(%d) PwrType(%d) val(%d)\n", PinIdx, PwrType, Val);
	switch (PwrType) {
	case CAMRST:
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam0_rst_l);
			else
				pinctrl_select_state(camctrl, cam0_rst_h);
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam1_rst_l);
			else
				pinctrl_select_state(camctrl, cam1_rst_h);
		}
		break;
	case CAMPDN:
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam0_pnd_l);
			else
				pinctrl_select_state(camctrl, cam0_pnd_h);
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam1_pnd_l);
			else
				pinctrl_select_state(camctrl, cam1_pnd_h);
		}

		break;
	case CAMLDO:
		if (Val == 0)
			pinctrl_select_state(camctrl, cam_ldo0_l);
		else
			pinctrl_select_state(camctrl, cam_ldo0_h);
		break;
	case CAMLDO1:
		if (Val == 0)
			pinctrl_select_state(camctrl, cam_ldo1_l);
		else
			pinctrl_select_state(camctrl, cam_ldo1_h);
		break;
	default:
		PK_DBG("PwrType(%d) is invalid !!\n", PwrType);
		break;
	};

	PK_DBG("PinIdx(%d) PwrType(%d) val(%d)\n", PinIdx, PwrType, Val);

	return ret;
}

int mtkcam_gpio_init(struct platform_device *pdev)
{
        int ret = 0;

        camctrl = devm_pinctrl_get(&pdev->dev);
        if (IS_ERR(camctrl)) {
                dev_err(&pdev->dev, "Cannot find camera pinctrl!");
                ret = PTR_ERR(camctrl);
        }
    /*Cam0 Power/Rst Ping initialization*/
        cam0_pnd_h = pinctrl_lookup_state(camctrl, "cam0_pnd1");
        if (IS_ERR(cam0_pnd_h)) {
                ret = PTR_ERR(cam0_pnd_h);
                pr_debug("%s : pinctrl err, cam0_pnd_h\n", __func__);
        }
#ifdef CONFIG_V36BML_CAMERA
	else {
		//pin7 must always up, used by sim cards too
		mtkcam_gpio_set(0, CAMPDN,1);
	}
#endif

        cam0_pnd_l = pinctrl_lookup_state(camctrl, "cam0_pnd0");
        if (IS_ERR(cam0_pnd_l)) {
                ret = PTR_ERR(cam0_pnd_l);
                pr_debug("%s : pinctrl err, cam0_pnd_l\n", __func__);
        }


        cam0_rst_h = pinctrl_lookup_state(camctrl, "cam0_rst1");
        if (IS_ERR(cam0_rst_h)) {
                ret = PTR_ERR(cam0_rst_h);
                pr_debug("%s : pinctrl err, cam0_rst_h\n", __func__);
        }

        cam0_rst_l = pinctrl_lookup_state(camctrl, "cam0_rst0");
        if (IS_ERR(cam0_rst_l)) {
                ret = PTR_ERR(cam0_rst_l);
                pr_debug("%s : pinctrl err, cam0_rst_l\n", __func__);
        }

    /*Cam1 Power/Rst Ping initialization*/
        cam1_pnd_h = pinctrl_lookup_state(camctrl, "cam1_pnd1");
        if (IS_ERR(cam1_pnd_h)) {
                ret = PTR_ERR(cam1_pnd_h);
                pr_debug("%s : pinctrl err, cam1_pnd_h\n", __func__);
        }

        cam1_pnd_l = pinctrl_lookup_state(camctrl, "cam1_pnd0");
        if (IS_ERR(cam1_pnd_l )) {
                ret = PTR_ERR(cam1_pnd_l );
                pr_debug("%s : pinctrl err, cam1_pnd_l\n", __func__);
        }
#ifdef CONFIG_V36BML_CAMERA
	else {
		// pin122 used only by ov5670v36
		mtkcam_gpio_set(1, CAMPDN,0);
	}
#endif

        cam1_rst_h = pinctrl_lookup_state(camctrl, "cam1_rst1");
        if (IS_ERR(cam1_rst_h)) {
                ret = PTR_ERR(cam1_rst_h);
                pr_debug("%s : pinctrl err, cam1_rst_h\n", __func__);
        }


        cam1_rst_l = pinctrl_lookup_state(camctrl, "cam1_rst0");
        if (IS_ERR(cam1_rst_l)) {
                ret = PTR_ERR(cam1_rst_l);
                pr_debug("%s : pinctrl err, cam1_rst_l\n", __func__);
        }
#ifndef CONFIG_V36BML_CAMERA
        /*externel LDO enable */
        cam_ldo0_h = pinctrl_lookup_state(camctrl, "cam_ldo0_1");
        if (IS_ERR(cam_ldo0_h)) {
                ret = PTR_ERR(cam_ldo0_h);
                pr_debug("%s : pinctrl err, cam_ldo0_h\n", __func__);
        }


        cam_ldo0_l = pinctrl_lookup_state(camctrl, "cam_ldo0_0");
        if (IS_ERR(cam_ldo0_l)) {
                ret = PTR_ERR(cam_ldo0_l);
                pr_debug("%s : pinctrl err, cam_ldo0_l\n", __func__);
        }
        /*externel LDO1 enable */ /* chyl add test */
        cam_ldo1_h = pinctrl_lookup_state(camctrl, "cam_ldo1_1");
        if (IS_ERR(cam_ldo1_h)) {
                ret = PTR_ERR(cam_ldo1_h);
                pr_debug("%s : pinctrl err, cam_ldo1_h\n", __func__);
        }


        cam_ldo1_l = pinctrl_lookup_state(camctrl, "cam_ldo1_0");
        if (IS_ERR(cam_ldo1_l)) {
                ret = PTR_ERR(cam_ldo1_l);
                pr_debug("%s : pinctrl err, cam_ldo1_l\n", __func__);
        }
#endif  
        return ret;
}

int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, bool On, char* mode_name)
{

u32 pinSetIdx = 0;//default main sensor

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4
#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3


u32 pinSet[3][8] = {
                        //for main sensor
                     {  CAMERA_CMRST_PIN, // The reset pin of main sensor uses GPIO10 of mt6306, please call mt6306 API to set
                        CAMERA_CMRST_PIN_M_GPIO,   /* mode */
                        GPIO_OUT_ONE,              /* ON state */
                        GPIO_OUT_ZERO,             /* OFF state */
                        CAMERA_CMPDN_PIN,
                        CAMERA_CMPDN_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     },
                     //for sub sensor
                     {  CAMERA_CMRST1_PIN,
                        CAMERA_CMRST1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                        CAMERA_CMPDN1_PIN,
                        CAMERA_CMPDN1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     },
                     //for main_2 sensor
                     {  GPIO_CAMERA_INVALID,
                        GPIO_CAMERA_INVALID,   /* mode */
                        GPIO_OUT_ONE,               /* ON state */
                        GPIO_OUT_ZERO,              /* OFF state */
                        GPIO_CAMERA_INVALID,
                        GPIO_CAMERA_INVALID,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     }
                   };



    if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
        pinSetIdx = 0;
    }
    else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
        pinSetIdx = 1;
    }
    else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx) {
        pinSetIdx = 2;
    }

    switch (SensorIdx)
	{
		case DUAL_CAMERA_MAIN_SENSOR:
			printk("[DUAL_CAMERA_MAIN_SENSOR] on = %d\n", On);
                        if (On) {
                                ISP_MCLK1_EN(1);
                                if (currSensorName && ((0 == strcmp(SENSOR_DRVNAME_OV13850V36_MIPI_RAW, currSensorName)) ||
				    (0 == strcmp(SENSOR_DRVNAME_S5K3L2XXV36_MIPI_RAW, currSensorName)))) {
                                        printk("[camdebug] %s poweron\n", currSensorName);
                                        // sub rst gpio 4 low
                                        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx+1][IDX_PS_CMRST]) {
                                                mtkcam_gpio_set(pinSetIdx+1, CAMRST,
                                                        pinSet[pinSetIdx+1][IDX_PS_CMRST + IDX_PS_OFF]);
                                        }
                                        // main rst gpio 46 low
                                        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                                                mtkcam_gpio_set(pinSetIdx, CAMRST,
                                                        pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
                                        }
                                        //VCAM_IO
                                        if(TRUE != _hwPowerOn(VCAMIO, VOL_1800)){
                                                PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
                                                goto _kdCISModulePowerOn_exit_;
                                        }
                                        mdelay(1);
                                        //VCAM_A
                                        if(TRUE != _hwPowerOn(VCAMA, VOL_2800)){
                                                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", VCAMA);
                                                goto _kdCISModulePowerOn_exit_;
                                        }
                                        mdelay(1);
                                        //VCAM_D
                                        if(TRUE != _hwPowerOn(VCAMD, VOL_1200)){
                                                PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %d \n", VCAMD);
                                                goto _kdCISModulePowerOn_exit_;
                                        }
                                        mdelay(2);
                                        //AF_VCC
                                        if(TRUE != _hwPowerOn(VCAMAF, VOL_2800)){
                                                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %d \n", VCAMAF);
                                                goto _kdCISModulePowerOn_exit_;
                                        }
                                        mdelay(5);
                                        // main rst gpio 46 high
                                        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                                                mtkcam_gpio_set(pinSetIdx, CAMRST,
                                                        pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
                                        }
                                        mdelay(10);
                                }
                                else
                                {
                                        goto _kdCISModulePowerOn_exit_;
                                }
                        } else { //power off
                                ISP_MCLK1_EN(0);
                                if (currSensorName && ((0 == strcmp(SENSOR_DRVNAME_OV13850V36_MIPI_RAW, currSensorName)) ||
                                    (0 == strcmp(SENSOR_DRVNAME_S5K3L2XXV36_MIPI_RAW, currSensorName)))) {
                                        // main rst gpio 46 low 
                                        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                                                mtkcam_gpio_set(pinSetIdx, CAMRST,
                                                        pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
                                        }               
                                        //AF_VCC
                                        if(TRUE != _hwPowerDown(VCAMAF)){
                                                PK_DBG("[CAMERA SENSOR] Fail to disable analog power (VCAM_AF), power id = %d \n", VCAMAF);
                                                goto _kdCISModulePowerOn_exit_;
                                        }
                                        //VCAM_D
                                        if(TRUE != _hwPowerDown(VCAMD)){
                                                PK_DBG("[CAMERA SENSOR] Fail to disable digital power (VCAM_D), power id = %d \n", VCAMD);
                                                goto _kdCISModulePowerOn_exit_;
                                        }
                                        //VCAM_A
                                        if(TRUE != _hwPowerDown(VCAMA)){
                                                PK_DBG("[CAMERA SENSOR] Fail to disable analog power (VCAM_A), power id = %d\n", VCAMA);
                                                goto _kdCISModulePowerOn_exit_;
                                        }
                                        //VCAM_IO
                                        if(TRUE != _hwPowerDown(VCAMIO)){
                                                PK_DBG("[CAMERA SENSOR] Fail to disable digital power (VCAM_IO), power id = %d \n", VCAMIO);
                                                goto _kdCISModulePowerOn_exit_;
                                        }
                                }
                                else
                                {
                                        goto _kdCISModulePowerOn_exit_;
                                }
                        }
			break;
		case DUAL_CAMERA_SUB_SENSOR:
			printk("[DUAL_CAMERA_SUB_SENSOR] on = %d\n", On);
			if (On) {
                                ISP_MCLK1_EN(1);
                                if (currSensorName && ((0 == strcmp(SENSOR_DRVNAME_S5K5E8YXV36_MIPI_RAW, currSensorName)) ||
                                    (0 == strcmp(SENSOR_DRVNAME_OV5670V36_MIPI_RAW, currSensorName)))) {
                                        printk("[camdebug] %s poweron\n", currSensorName);
                                        // main rst gpio 46 low 
                                        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx-1][IDX_PS_CMRST]) {
                                                mtkcam_gpio_set(pinSetIdx-1, CAMRST,
                                                        pinSet[pinSetIdx-1][IDX_PS_CMRST + IDX_PS_OFF]);
                                        }
                                        // sub rst gpio 4 low
                                        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                                                mtkcam_gpio_set(pinSetIdx, CAMRST,
                                                        pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
                                        }
                                        //VCAM_A
                                        if(TRUE != _hwPowerOn(VCAMA, VOL_2800)){
                                                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", VCAMA);
                                                goto _kdCISModulePowerOn_exit_;
                                        }
                                        mdelay(1);
                                        //VCAM_D
                                        if(TRUE != _hwPowerOn(VCAMD, VOL_1200)){
                                                PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %d \n", VCAMD);
                                                goto _kdCISModulePowerOn_exit_;
                                        }
                                        mdelay(2);
                                        //VCAM_IO
                                        if(TRUE != _hwPowerOn(VCAMIO, VOL_1800)){
                                                PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
                                                goto _kdCISModulePowerOn_exit_;
                                        }
                                        mdelay(2);
					if (currSensorName && ((0 == strcmp(SENSOR_DRVNAME_OV5670V36_MIPI_RAW, currSensorName)))) {
	                                        // sub pwr gpio 122 high
        	                                if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                	                                mtkcam_gpio_set(pinSetIdx, CAMPDN,
                        	                                pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]);
                                	        }
					}
                                        // sub rst gpio 4 high
                                        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                                                mtkcam_gpio_set(pinSetIdx, CAMRST,
                                                        pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
                                        }
                                        mdelay(10);
				}
				else 
				{
					goto _kdCISModulePowerOn_exit_;
				}
			} else { //power off
				ISP_MCLK1_EN(0);
                                if (currSensorName && ((0 == strcmp(SENSOR_DRVNAME_S5K5E8YXV36_MIPI_RAW, currSensorName)))) {
                                        printk("[camdebug] %s poweroff\n", currSensorName);
                                        // sub rst gpio 4 low 
                                        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                                                mtkcam_gpio_set(pinSetIdx, CAMRST,
                                                        pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
                                        }
                                        // sub power gpio 122 low 
                                        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                                                mtkcam_gpio_set(pinSetIdx, CAMPDN,
                                                        pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
                                        }
                                        //VCAM_IO
                                        if(TRUE != _hwPowerDown(VCAMIO)){
                                                PK_DBG("[CAMERA SENSOR] Fail to disable digital power (VCAM_IO), power id = %d \n", VCAMIO);
                                                goto _kdCISModulePowerOn_exit_;
                                        }
                                        //VCAM_D
                                        if(TRUE != _hwPowerDown(VCAMD)){
                                                PK_DBG("[CAMERA SENSOR] Fail to disable digital power (VCAM_D), power id = %d \n", VCAMD);
                                                goto _kdCISModulePowerOn_exit_;
                                        }
                                        //VCAM_A
                                        if(TRUE != _hwPowerDown(VCAMA)){
                                                PK_DBG("[CAMERA SENSOR] Fail to disable analog power (VCAM_A), power id = %d\n", VCAMA);
                                                goto _kdCISModulePowerOn_exit_;
                                        }
				}
                                else if (currSensorName && ((0 == strcmp(SENSOR_DRVNAME_OV5670V36_MIPI_RAW, currSensorName))))
				{
                                        printk("[camdebug] %s poweroff\n", currSensorName);
                                        // sub rst gpio 4 low 
                                        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                                                mtkcam_gpio_set(pinSetIdx, CAMRST,
                                                        pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
                                        }
                                        // sub power gpio 122 low 
                                        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                                                mtkcam_gpio_set(pinSetIdx, CAMPDN,
                                                        pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
                                        }
                                        //AF_VCC
/*
	                                if(TRUE != _hwPowerDown(VCAMAF)){
        	                                PK_DBG("[CAMERA SENSOR] Fail to disable analog power (VCAM_AF), power id = %d \n", VCAMAF);
                	                        goto _kdCISModulePowerOn_exit_;
					}
*/
                                        //VCAM_IO
                                        if(TRUE != _hwPowerDown(VCAMIO)){
                                                PK_DBG("[CAMERA SENSOR] Fail to disable digital power (VCAM_IO), power id = %d \n", VCAMIO);
                                                goto _kdCISModulePowerOn_exit_;
                                        }
                                        //VCAM_D
                                        if(TRUE != _hwPowerDown(VCAMD)){
                                                PK_DBG("[CAMERA SENSOR] Fail to disable digital power (VCAM_D), power id = %d \n", VCAMD);
                                                goto _kdCISModulePowerOn_exit_;
                                        }
                                        //VCAM_A
                                        if(TRUE != _hwPowerDown(VCAMA)){
                                                PK_DBG("[CAMERA SENSOR] Fail to disable analog power (VCAM_A), power id = %d\n", VCAMA);
                                                goto _kdCISModulePowerOn_exit_;
                                        }

				}
                                else
				{
                                        goto _kdCISModulePowerOn_exit_;
                                }
			}
			break;
		default:
			break;
	}

    return 0;

_kdCISModulePowerOn_exit_:
    return -EIO;

}

EXPORT_SYMBOL(kdCISModulePowerOn);

//!--
//


