#ifdef CONFIG_COMPAT

#include <linux/fs.h>
#include <linux/compat.h>

#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "mt_typedefs.h"
//#include <cust_gpio_usage.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
//#include <linux/xlog.h>
#include <linux/version.h>
#include <mt-plat/upmu_common.h>
//#include <mach/mt6333.h>
//HTC_START
#include <linux/htc_flashlight.h>
//HTC_END
#include "kd_flashlight.h"
/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "strobe_main_sid1_part1"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    printk(KERN_INFO TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        printk(KERN_WARNING TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      printk(KERN_NOTICE TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        printk(KERN_INFO TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              printk(TAG_NAME "<%s>\n", __FUNCTION__);
#define PK_TRC_VERBOSE(fmt, arg...) printk(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       printk(KERN_ERR TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif


#define HTC_TORCH_CURRENT_MAX 250

/******************************************************************************
 * local variables
******************************************************************************/
static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */
static struct work_struct workTimeOut;
static int g_timeOutTimeMs=0;
static u32 strobe_Res = 0;
/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);

//HTC_START
int gFlashDuty1=0;
int gbFlashOn1=0;
//extern int gFlashDuty2;
//extern int gbFlashOn2;
static int htc_flash_ind[10] = {150,175,200,220,275,350,425,500,575,650};
//HTC_END

static int FL_Enable(void)
{

//HTC_START
#if defined(CONFIG_HTC_FLASHLIGHT_COMMON)
  gbFlashOn1 = 1;
    //    if(gbFlashOn2==0){
    /*
    if(htc_flash_ind[gFlashDuty1] <=  HTC_TORCH_CURRENT_MAX)
      htc_torch_main(htc_flash_ind[gFlashDuty1],0);
    else
      htc_flash_main(htc_flash_ind[gFlashDuty1],0);

    PK_DBG(" %d:0\n",htc_flash_ind[gFlashDuty1]);
    */

  if(gFlashDuty1 == 0){
      htc_torch_main(htc_flash_ind[gFlashDuty1],0);
  } else{
      if(gFlashDuty1 > 9){
          gFlashDuty1 = 9;
      }
      PK_DBG("##### duty value  = %d  current value = %d \n",gFlashDuty1,htc_flash_ind[gFlashDuty1]);
      htc_flash_main(htc_flash_ind[gFlashDuty1],0);
  }
    //}
    //	else{
    //		htc_flash_main(htc_flash_ind[gFlashDuty1], htc_flash_ind[gFlashDuty2]);
    //		PK_DBG(" %d:%d\n",htc_flash_ind[gFlashDuty1], htc_flash_ind[gFlashDuty2]);
    //	}
#endif
//HTC_END
    return 0;
}

static int FL_Disable(void)
{
	//HTC_START
	#if defined(CONFIG_HTC_FLASHLIGHT_COMMON)

	if (gbFlashOn1==0){
	  //		PK_DBG("skip %d %d\n",gbFlashOn1, gbFlashOn2);
	  PK_DBG("skip %d\n",gbFlashOn1);
	  return 0;
	}
	gbFlashOn1 = 0;
	//gbFlashOn2 = 0;
	htc_flash_main(0,0);
	PK_DBG(" 0 0\n");

	#endif
	//HTC_END
    return 0;
}

static int FL_dim_duty(kal_uint32 duty)
{
    //HTC_START
    gFlashDuty1 = duty;
	PK_DBG("%d->%d\n",gFlashDuty1, htc_flash_ind[gFlashDuty1]);
    //HTC_END
    return 0;
}


static int FL_getPreOnTime(int duty)
{
    //return FL_getPreOnTime_6332_2(duty);
     return 0;
}

static int FL_preOn(void)
{
    //return FL_preOn_6332_2();
     return 0;
}

/*
static int g_lowPowerLevel=LOW_BATTERY_LEVEL_0;
static void lowPowerCB(LOW_BATTERY_LEVEL lev)
{
	g_lowPowerLevel=lev;
}*/

static int FL_Init(void)
{
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    INIT_WORK(&workTimeOut, work_timeOutFunc);
  //  register_low_battery_callback(&lowPowerCB, LOW_BATTERY_PRIO_FLASHLIGHT);
   // register_low_battery_notify(&lowPowerCB, LOW_BATTERY_PRIO_FLASHLIGHT);



    return 0;
}
static int FL_Uninit(void)
{
	FL_Disable();
    return 0;
}

static int FL_hasLowPowerDetect(void)
{

	return 1;
}

static int detLowPowerStart(void)
{

//	g_lowPowerLevel=LOW_BATTERY_LEVEL_0;
    return 0;

}


static int detLowPowerEnd(void)
{

	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/
static struct hrtimer g_timeOutTimer;

static int g_b1stInit=1;

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("ledTimeOut_callback\n");
}



static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}

static void timerInit(void)
{



	//mt6333_set_rg_chrwdt_en(0);

    //mt6333_set_rg_chrwdt_td(0); //4 sec
    //mt6333_set_rg_chrwdt_en(1);

    //mt6333_set_rg_chrwdt_en(0);

	if(g_b1stInit==1)
	{
		g_b1stInit=0;


	  	INIT_WORK(&workTimeOut, work_timeOutFunc);
		g_timeOutTimeMs=1000; //1s
		hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
		g_timeOutTimer.function=ledTimeOutCallback;
	}



}

static int gGetPreOnDuty=0;
static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int temp;
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	kdStrobeDrvArg kdArg;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%lu\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",(int)arg);
			g_timeOutTimeMs=arg;
		break;

		case FLASH_IOC_PRE_ON:
    		PK_DBG("FLASH_IOC_PRE_ON\n");
			FL_preOn();
    		break;

		 case FLASH_IOC_GET_PRE_ON_TIME_MS_DUTY:
            PK_DBG("FLASH_IOC_GET_PRE_ON_TIME_MS_DUTY: %d\n",(int)arg);
            gGetPreOnDuty = arg;
            break;

        case FLASH_IOC_GET_PRE_ON_TIME_MS:
    		PK_DBG("FLASH_IOC_GET_PRE_ON_TIME_MS: %d\n",(int)arg);
    		temp = FL_getPreOnTime(gGetPreOnDuty);
    		kdArg.arg = temp;
            if(copy_to_user((void __user *) arg , (void*)&kdArg , sizeof(kdStrobeDrvArg)))
            {
                PK_DBG(" ioctl copy to user failed\n");
				PK_DBG(" arg = %lx",arg);
                return -1;
            }
    		break;

    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",(int)arg);
    		FL_dim_duty(arg);
    		break;




    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",(int)arg);
    		if(arg==1)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
    			FL_Enable();
    		}
    		else
    		{
    			FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
/*
    	case FLASH_IOC_PRE_ON:
    		PK_DBG("FLASH_IOC_PRE_ON\n");
			FL_preOn();
    		break;
    	case FLASH_IOC_GET_PRE_ON_TIME_MS:
    		PK_DBG("FLASH_IOC_GET_PRE_ON_TIME_MS: %d\n",(int)arg);
    		temp=13;
    		if(copy_to_user((void __user *) arg , (void*)&temp , 4))
            {
                PK_DBG(" ioctl copy to user failed\n");
                return -1;
            }
    		break;
*/
        case FLASH_IOC_SET_REG_ADR:
            PK_DBG("FLASH_IOC_SET_REG_ADR: %d\n",(int)arg);
            //g_reg = arg;
            break;
        case FLASH_IOC_SET_REG_VAL:
            PK_DBG("FLASH_IOC_SET_REG_VAL: %d\n",(int)arg);
            //g_val = arg;
            break;
        case FLASH_IOC_SET_REG:
          //  PK_DBG("FLASH_IOC_SET_REG: %d %d\n",g_reg, g_val);

            break;

        case FLASH_IOC_GET_REG:
            PK_DBG("FLASH_IOC_GET_REG: %d\n",(int)arg);

            //i4RetValue = valTemp;
            break;

        case FLASH_IOC_HAS_LOW_POWER_DETECT:
    		PK_DBG("FLASH_IOC_HAS_LOW_POWER_DETECT");
    		temp=FL_hasLowPowerDetect();
    		if(copy_to_user((void __user *) arg , (void*)&temp , 4))
            {
                PK_DBG(" ioctl copy to user failed\n");
                return -1;
            }
    		break;
    	case FLASH_IOC_LOW_POWER_DETECT_START:
    		detLowPowerStart();
    		break;
    	case FLASH_IOC_LOW_POWER_DETECT_END:
    		i4RetValue = detLowPowerEnd();
    		break;

        default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;


        spin_unlock_irq(&g_strobeSMPLock);
    	FL_Uninit();
    }
    PK_DBG(" Done\n");
    return 0;
}


static FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};



MUINT32 strobeInit_main_sid1_part1(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}


