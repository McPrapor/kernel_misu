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
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
//#include <cust_gpio_usage.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
//#include <linux/xlog.h>
#include <linux/version.h>


#include <mt-plat/mt_gpio.h>


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif

#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/dma-mapping.h>
//#include <linux/htc_devices_dtb.h>
//#include <linux/htc_flashlight.h>
#include "htc_flashlight.h"

#define FLASH_sgm3785

#define FLASHLIGHT_NAME "flashlight"

#ifdef CONFIG_V36BML_CAMERA
#define GPIO_FLT_ENM (60 | 0x80000000)
#define GPIO_FLT_ENF (43 | 0x80000000)
#endif

struct mt6332_led_data {
	u8			num_leds;
	struct i2c_client	*client_dev;
	struct tps61310_data 	*tps61310;
	int status;
	struct led_classdev	cdev;
	int			max_current;
	int			id;
	u8			default_state;
	int                     torch_mode;
	struct mutex		lock;
	struct work_struct	work;
};

static struct led_classdev	this_fit;
struct delayed_work sgm3785_delayed_work;
static struct workqueue_struct *sgm3785_work_queue;

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
#define TAG_NAME "leds_strobe.c"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        xlog_printk(ANDROID_LOG_WARNING, TAG_NAME, KERN_WARNING  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_NOTICE  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        xlog_printk(ANDROID_LOG_INFO   , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME,  "<%s>\n", __FUNCTION__);
#define PK_TRC_VERBOSE(fmt, arg...) xlog_printk(ANDROID_LOG_VERBOSE, TAG_NAME,  fmt, ##arg)
#define PK_ERROR(fmt, arg...)       xlog_printk(ANDROID_LOG_ERROR  , TAG_NAME, KERN_ERR "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG printk
	#define PK_VER printk
	#define PK_ERR printk
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif



/******************************************************************************
 * local variables
******************************************************************************/

#define FLASHLIGHT_DEVNAME            "sgm3785"
struct flash_chip_data {
	struct led_classdev cdev_flash;
	struct led_classdev cdev_torch;

	struct mutex lock;

	int mode;
	int torch_level;

	int gpio_enm;
	int gpio_enf;
};

static struct flash_chip_data chipconf;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif

#define STROBE_DEVICE_ID 0x60

/*****************************************************************************
Functions
*****************************************************************************/
struct sgm3785_platform_data {
	u8 torch_pin_enable;    // 1:  TX1/TORCH pin isa hardware TORCH enable
	u8 pam_sync_pin_enable; // 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input
	u8 thermal_comp_mode_enable;// 1: LEDI/NTC pin in Thermal Comparator Mode
	u8 strobe_pin_disable;  // 1 : STROBE Input disabled
	u8 vout_mode_enable;  // 1 : Voltage Out Mode enable
};

struct sgm3785_chip_data {
	struct i2c_client *client;

	struct led_classdev cdev_flash;
	struct led_classdev cdev_torch;
	struct led_classdev cdev_indicator;

	struct sgm3785_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};

static int sgm3785_turn_off(void)
{
	struct flash_chip_data *chip;
	chip = &chipconf;
	printk("[FLT] %s\n", __func__);
	mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ZERO);
	mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ZERO);
	mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ZERO);

	return 0;
}

static void sgm3785_turn_off_work(struct work_struct *work)
{
	printk("[FLT] %s\n", __func__);
	sgm3785_turn_off();
}

static void led_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	//struct mt6332_led_data *led;

/*
	led = container_of(led_cdev, struct mt6332_led_data, cdev);
	if (value < LED_OFF || value > led->cdev.max_brightness) {
		printk(KERN_ERR "[FLT]"
				"Invalid brightness value\n");
		return;
	}
*/
	struct flash_chip_data *chip;
	chip = &chipconf;
	printk(KERN_INFO "[FLT] Flashlight set value %d.\n", value);


   	if(value >= 0)
   	{
   		switch(value)
   		{
   			//torch 1
   			case 125:
				mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ONE);
				mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ZERO);
				break;
			//torch 2
			case 126:
				mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ONE);
				mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ZERO);
				break;
			//torch 3
			case 127:
				mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ONE);
				mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ZERO);
				break;
			//pre-flash
			case 128:
				mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ONE);
				mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ZERO);
				break;
			//flash 56mA
			case 129:
				mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ONE);
				mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ONE);
				queue_delayed_work(sgm3785_work_queue, &sgm3785_delayed_work, msecs_to_jiffies(600));
				break;
			//flash 112mA
			case 130:
				mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ONE);
				mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ONE);
				queue_delayed_work(sgm3785_work_queue, &sgm3785_delayed_work, msecs_to_jiffies(600));
				break;
			//flash 168mA
			case 131:
				mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ONE);
				mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ONE);
				queue_delayed_work(sgm3785_work_queue, &sgm3785_delayed_work, msecs_to_jiffies(600));
				break;
			//flash 224mA
			case 132:
				mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ONE);
				mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ONE);
				queue_delayed_work(sgm3785_work_queue, &sgm3785_delayed_work, msecs_to_jiffies(600));
				break;
			//flash 280mA
			case 133:
				mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ONE);
				mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ONE);
				queue_delayed_work(sgm3785_work_queue, &sgm3785_delayed_work, msecs_to_jiffies(600));
				break;
			//flash 336mA
			case 134:
				mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ONE);
				mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ONE);
				queue_delayed_work(sgm3785_work_queue, &sgm3785_delayed_work, msecs_to_jiffies(600));
				break;
			//flash 392mA
			case 135:
				mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ONE);
				mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ONE);
				queue_delayed_work(sgm3785_work_queue, &sgm3785_delayed_work, msecs_to_jiffies(600));
				break;
			//flash 448mA
			case 136:
				mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ONE);
				mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ONE);
				queue_delayed_work(sgm3785_work_queue, &sgm3785_delayed_work, msecs_to_jiffies(600));
				break;
			//flash 504mA
			case 137:
				mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ONE);
				mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ONE);
				queue_delayed_work(sgm3785_work_queue, &sgm3785_delayed_work, msecs_to_jiffies(600));
				break;
			//flash 560mA
			case 138:
				mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ONE);
				mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ONE);
				queue_delayed_work(sgm3785_work_queue, &sgm3785_delayed_work, msecs_to_jiffies(600));
				break;
			//flash 616mA
			case 139:
				mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ONE);
				mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ONE);
				queue_delayed_work(sgm3785_work_queue, &sgm3785_delayed_work, msecs_to_jiffies(600));
				break;
			//flash 672mA
			case 140:
				mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ONE);
				mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ONE);
				queue_delayed_work(sgm3785_work_queue, &sgm3785_delayed_work, msecs_to_jiffies(600));
				break;
			//flash 728mA
			case 141:
				mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ONE);
				mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ONE);
				queue_delayed_work(sgm3785_work_queue, &sgm3785_delayed_work, msecs_to_jiffies(600));
				break;
			//flash 784mA
			case 142:
				mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ONE);
				mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ONE);
				queue_delayed_work(sgm3785_work_queue, &sgm3785_delayed_work, msecs_to_jiffies(600));
				break;
			//flash 784mA
			case 255:
				mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ONE);
				mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ONE);
				queue_delayed_work(sgm3785_work_queue, &sgm3785_delayed_work, msecs_to_jiffies(600));
				break;
			case LED_OFF:
		                mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ZERO);
		                mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ZERO);
                		mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ZERO);
				break;
   		}

		//printk("[flashchip] flash level = 1\n");
   	}
	else
	{
		printk("[FLT] Exception value:  0x%x\n",value);
	}

	//led->cdev.brightness = value;
	//schedule_work(&led->work);
}

static enum led_brightness led_get(struct led_classdev *led_cdev)
{
	struct mt6332_led_data *led;

	led = container_of(led_cdev, struct mt6332_led_data, cdev);

	printk("[FLT] Flashlight get value %d.\n", led->cdev.brightness);

	return led->cdev.brightness;
}


int sgm3785_flt_flash(struct led_classdev *flt, uint32_t mA)
{
	u8 lv = 0x0;
	uint32_t regval= 0x0;
	int ret = 0;

	struct flash_chip_data *chip;
	chip = &chipconf;

	lv=(u8)((int)mA/56);
	printk("[FLT] Flashlight sgm3785_flt_flash set %d mA, lv value 0x%x.\n", mA,lv);

	if (mA == 0) {
		mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ZERO);
		mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ZERO);
		mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ZERO);
	} else if(mA > 0 && mA <= 784) {
		mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ONE);
		mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ONE);
		mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ONE);
		queue_delayed_work(sgm3785_work_queue, &sgm3785_delayed_work, msecs_to_jiffies(600));
		printk("[FLT] Flashlight sgm3785_flt_flash set %d mA, reg val: %x, lv value %d.\n", mA,regval,lv);
	} else {
		printk("[FLT] Flashlight unsupport value.\n");
	}

	return ret;
}

int sgm3785_flt_torch(struct led_classdev *flt, uint32_t mA)
{
	u8 lv = 0x0;
	uint32_t regval= 0x0;
	int ret = 0;

	struct flash_chip_data *chip;
	chip = &chipconf;

	lv=(u8)((int)mA/28);

	printk("[FLT] Flashlight sgm3785_flt_torch set %d mA, lv value 0x%x.\n", mA,lv);

	if (mA == 0) {
		//sgm3785_write_reg(sgm3785_i2c_client, 0x10, 0x18); //ma torch output_en'
		mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ZERO);
		mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ZERO);
		mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ZERO);
	} else if(mA > 0 && mA <= 224) {
		//sgm3785_write_reg(sgm3785_i2c_client, 0xA0, lv); //ma torch output_en'
		//sgm3785_write_reg(sgm3785_i2c_client, 0x10, 0x1B); //ma flash output_en
		mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ONE);
		mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ONE);
		mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ZERO);
		printk("[FLT] Flashlight sgm3785_flt_torch set %d mA, reg val: %x, lv value %d.\n", mA,regval,lv);
	} else {
		printk("[FLT] Flashlight unsupport value.\n");
	}

    return ret;
}

#if defined(CONFIG_HTC_FLASHLIGHT_COMMON)

static int sgm3785_flt_flash_adapter(int mA1, int mA2){
	return sgm3785_flt_flash(&this_fit, mA1);
}

static int sgm3785_flt_torch_adapter(int mA1, int mA2){
	return sgm3785_flt_torch(&this_fit, mA1);
}
#endif


static int flashchip_probe(struct platform_device *dev)
{
	struct flash_chip_data *chip;

	int ret;
	static struct led_classdev	cdev;

	cdev.name			   = FLASHLIGHT_NAME;
	cdev.brightness_set    = led_set;
	cdev.brightness_get    = led_get;

	printk("[FLT] probe start\n");
	PK_ERR("[flashchip_probe] start\n");
	chip = &chipconf;
	chip->mode = 0;
	chip->torch_level = 0;
	
	mutex_init(&chip->lock);

#ifdef CONFIG_V36BML_CAMERA
		chip->gpio_enm = (60 | 0x80000000);
		chip->gpio_enf = (43 | 0x80000000);
		printk("[FLT]ENM GPIO = 0x%x, ENF GPIO = 0x%x\n", chip->gpio_enm, chip->gpio_enf);
#else	
		chip->gpio_enm = (GPIO60 | 0x80000000);
		chip->gpio_enf = (GPIO43 | 0x80000000);
		printk("[FLT]ENM GPIO = %d, ENF GPIO = %d hwid %d\n", GPIO60, GPIO43, of_machine_hwid());
#endif
	

	printk("[FLT] probe Done\n");

	ret = led_classdev_register(&dev->dev, &cdev);
	if (ret) {
		printk(KERN_ERR "[FLT] "
			"unable to register led rec=%d\n", ret);
		//goto fail_id_check;
	}

    PK_ERR("[flashchip_probe] Done\n");

#if defined(CONFIG_HTC_FLASHLIGHT_COMMON)
	printk("if defined(CONFIG_HTC_FLASHLIGHT_COMMON) %s %d\n",__FUNCTION__,__LINE__);
	htc_flash_main			= &sgm3785_flt_flash_adapter;
	htc_torch_main			= &sgm3785_flt_torch_adapter;
#endif

	INIT_DELAYED_WORK(&sgm3785_delayed_work, sgm3785_turn_off_work);
	sgm3785_work_queue = create_singlethread_workqueue("sgm3785_wq");
	if (!sgm3785_work_queue)
		goto err_chip_init;

	mt_set_gpio_mode(chip->gpio_enm, 0);
	mt_set_gpio_dir(chip->gpio_enm, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(chip->gpio_enm, GPIO_PULL_DISABLE);
	mt_set_gpio_out(chip->gpio_enm, GPIO_OUT_ZERO);

	mt_set_gpio_mode(GPIO_FLT_ENM, 0);
	mt_set_gpio_dir(GPIO_FLT_ENM, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_FLT_ENM, GPIO_OUT_ZERO);

	mt_set_gpio_mode(chip->gpio_enf, 0);
	mt_set_gpio_dir(chip->gpio_enf, GPIO_DIR_OUT);
	mt_set_gpio_out(chip->gpio_enf, GPIO_OUT_ZERO);

	return 0;

err_chip_init:
	printk(KERN_ERR "[flashchip_probe] is failed !\n");
	return -ENODEV;

}

static int flashchip_remove(struct platform_device *dev)
{
	struct flash_chip_data *chip = &chipconf;
    PK_DBG("[flashchip_remove] start\n");

	led_classdev_unregister(&chip->cdev_torch);
	led_classdev_unregister(&chip->cdev_flash);

//	sgm3785_exit();

    PK_DBG("[flashchip_remove] Done\n");
    return 0;
}

static struct platform_driver flashchip_platform_driver =
{
    .probe      = flashchip_probe,
    .remove     = flashchip_remove,
    .driver     = {
        .name = FLASHLIGHT_DEVNAME,
		.owner	= THIS_MODULE,
		//.of_match_table = of_match_ptr(flashlight_of_match),
    },
};



static struct platform_device flashchip_platform_device = {
    .name = FLASHLIGHT_DEVNAME,
    .id = 0,
    .dev = {
//    	.platform_data	= &chip,
    }
};

static int __init flashchip_init(void)
{
    int ret = 0;
    printk("[FLT][flashchip_init] start\n");

	ret = platform_device_register (&flashchip_platform_device);
	if (ret) {
        PK_ERR("[flashchip_init] platform_device_register fail\n");
        return ret;
	}

    ret = platform_driver_register(&flashchip_platform_driver);
	if(ret){
		PK_ERR("[flashchip_init] platform_driver_register fail\n");
		return ret;
	}

	printk("[FLT][flashchip_init] done!\n");
    return ret;
}

static void __exit flashchip_exit(void)
{
    printk("[FLT][flashchip_exit] start\n");
    platform_driver_unregister(&flashchip_platform_driver);
    printk("[FLT][flashchip_exit] done!\n");
}

/*****************************************************************************/
late_initcall(flashchip_init);
module_exit(flashchip_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("terry_yen@htc.com>");
MODULE_DESCRIPTION("sgm3785 flash control Driver");
