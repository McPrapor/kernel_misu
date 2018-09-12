#ifndef _SENTRAL_IIO_H_
#define _SENTRAL_IIO_H_

#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/notifier.h>
#include <linux/irq_work.h>
#include <linux/iio/iio.h>
#include <linux/miscdevice.h>
#include <linux/completion.h>
#include <linux/wait.h>
#include "sentral-version.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#ifdef CONFIG_AK8789_HALLSENSOR
#include <linux/hall_sensor.h>
#endif

// comment out the following to use printk logging instead of dyndbg
#define SENTRAL_LOG_DYNDBG 1
#define NVRAM_DATA_ONESHOT
#define CONFIG_MTK
// some build env may not have all levels available, so we define the level per
// log type here
#define SENTRAL_LOG_PRINTK_D KERN_DEBUG
#define SENTRAL_LOG_PRINTK_E KERN_ERR
#define SENTRAL_LOG_PRINTK_I KERN_INFO

#ifdef SENTRAL_LOG_DYNDBG
#define LOGD(dev, fmt, ...) dev_dbg(dev, fmt, ##__VA_ARGS__)
#define LOGE(dev, fmt, ...) dev_err(dev, fmt, ##__VA_ARGS__)
#define LOGI(dev, fmt, ...) dev_info(dev, fmt, ##__VA_ARGS__)
#else /* dyndbg not available, use printk */
#define LOGD(dev, fmt, ...) printk(SENTRAL_LOG_PRINTK_D "%s(): " fmt, __func__, ##__VA_ARGS__)
#define LOGE(dev, fmt, ...) printk(SENTRAL_LOG_PRINTK_E "%s(): " fmt, __func__, ##__VA_ARGS__)
#define LOGI(dev, fmt, ...) printk(SENTRAL_LOG_PRINTK_I "%s(): " fmt, __func__, ##__VA_ARGS__)
#endif /* SENTRAL_LOG_DYNDBG */

/* CW compat log */
#define D(x...) pr_debug("[S_HUB][sentral] " x)
#define I(x...) pr_info("[S_HUB][sentral] " x)
#define W(x...) pr_warn("[S_HUB][sentral] " x)
#define E(x...) pr_err("[S_HUB][sentral] " x)

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define ENSTR(b) (b ? "enable" : "disable")
#define TFSTR(b) (b ? "true" : "false")

#define SENTRAL_NAME "CwMcuSensor"
#define SENTRAL_HUB_CLASS_NAME "htc_sensorhub"
#define SENTRAL_HUB_DEVICE_NAME "sensor_hub"
#define SENTRAL_WORKQUEUE_NAME "sentral_mcu"

#ifdef CONFIG_MTK
#define SENTRAL_OF_PNI_INT_GPIO "pni,host_int_gpio"
#define SENTRAL_OF_PNI_FW "pni,firmware"
#define SENTRAL_OF_MTK_EINT "mediatek, HTC_SENSOR_HUB-eint"
#else
#define SENTRAL_OF_GPIO_IRQ "mcu,gpio-irq"
#define SENTRAL_OF_PNI_FW "mcu,firmware"
#endif /* CONFIG_MTK */
#define SENTRAL_OF_ALS_LEVELS "mcu,als_levels"
#define SENTRAL_OF_ALS_LEVEL_COUNT "mcu,als_level_count"
#define SENTRAL_OF_ALS_GOLDL "mcu,als_goldl"
#define SENTRAL_OF_ALS_GOLDH "mcu,als_goldh"
#define SENTRAL_OF_ALS_POLLING "mcu,als_polling"
#define SENTRAL_OF_ALS_LUX_RATIO_N "mcu,als_lux_ratio_n"
#define SENTRAL_OF_ALS_LUX_RATIO_D "mcu,als_lux_ratio_d"
#define SENTRAL_OF_PS_THD_ADD "mcu,ps_thd_add"
#define SENTRAL_OF_PS_THD_FIXED "mcu,ps_thd_fixed"

#ifdef CONFIG_MTK
#define SHUB_MTK_DMA (1)

#ifdef SHUB_MTK_DMA
// should be multiple of SENTRAL_FIFO_BLOCK_SIZE if > SENTRAL_FIFO_BLOCK_SIZE
#define I2C_BLOCK_SIZE_MAX (200)
#else
#define I2C_BLOCK_SIZE_MAX (7)
#endif /* SHUB_MTK_DMA */
#else
#define I2C_BLOCK_SIZE_MAX (I2C_SMBUS_BLOCK_MAX)
#endif /* CONFIG_MTK */

#define DATA_BUFFER_SIZE 16384
#define SENTRAL_FIFO_BLOCK_SIZE 50
#define SENTRAL_FIFO_WATERMARK_BUFFER 500
#define SENTRAL_SENSOR_TIMESTAMP_SCALE_NS 31250LL
#define SENTRAL_SHUB_LOG_COUNT (300)
#define SENTRAL_SHUB_LOG_BUF_SIZE (16384)
#define SEN_CW_MAG_MAX 26
// format meaning: <param number> <param size> <test param(not read or write)>  = data[0-7]
#define SENTRAL_PAGE_ITEM_IO_FORMAT "0x%02x,0x%02x,0x%02x=0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x"
#define SENTRAL_PAGE_UPDATE_FLAG_PARAM_NUMBER 0xAA
#define SENTRAL_PAGE_UPDATE_FLAG_PARAM_SIZE   0x55
#define SENTRAL_PAGE_UPDATE_FLAG_PARAM_TEST   0x00
#define SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_0 0xAA
#define SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_1 0x55
#define SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_2 0x00
#define SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_3 0xAA
#define SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_4 0x55
#define SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_5 0x00
#define SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_6 0xAA
#define SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_7 0x55

#define SEN_DBG_PIO 1
#define SEN_DBG_MUTEX 1
#define SEN_DBG_IIO 1

#define FW_IMAGE_SIGNATURE 0x652A
#define FW_CDS_SIGNATURE 0xC88B
#define SENTRAL_FW_VERSION_LEN 8
#define RAM_BUF_LEN 64

#define PARAM_READ_SIZE_MAX 16
#define PARAM_WRITE_SIZE_MAX 8
#define PARAM_MAX_RETRY 10
#define PARAM_SENSORS_ACTUAL_OFFSET 31

#define SENTRAL_PARAM_SIZE_MAX 8

#define SENTRAL_PT_MAX_RETRY 10

#define SENTRAL_WATCHDOG_WORK_MSECS 10000

#define SENTRAL_ALS_LEVELS_MAX 16

#define SENTRAL_ONE_SHOT_MAX_RETRY 10
#define SENTRAL_ONE_SHOT_DEFAULT_RATE_HZ 100
#define SENTRAL_ONE_SHOT_DELAY_MSECS_ACCEL 200
#define SENTRAL_ONE_SHOT_DELAY_MSECS_MAG 200
#define SENTRAL_ONE_SHOT_DELAY_MSECS_GYRO 200
#define SENTRAL_ONE_SHOT_DELAY_MSECS_LIGHT 500
#define SENTRAL_ONE_SHOT_DELAY_MSECS_PROX 200

#define SENTRAL_SNS_EA_RATE 5

#define SENTRAL_CAL_ACCEL_RATE 100
#define SENTRAL_CAL_LIGHT_RATE 12
#define SENTRAL_CAL_LIGHT_KADC_COUNT 100
#define SENTRAL_CAL_LIGHT_HEADER_MSB (0x6D)
#define SENTRAL_CAL_LIGHT_HEADER_LSB (0xA5)
#define SENTRAL_CAL_PS_RATE 12

#define SEN_SNS_STARTUP_FILT_ORIENT (3)
#define SEN_SNS_STARTUP_FILT_RV (3)
#define SEN_SNS_STARTUP_FILT_GMRV (3)

struct sentral_param_page_item {
    int param_size;
    const unsigned char *param_name;
    char param_data[PARAM_READ_SIZE_MAX];
    //struct device_attribute param_dev_attr;
};

enum sentral_param_page_mode {
    SEN_PARAM_PAGE_WARM_START_READ  = 1,
    SEN_PARAM_PAGE_WARM_START_WRITE,
    SEN_PARAM_PAGE_KNOBS_READ,
    SEN_PARAM_PAGE_KNOBS_WRITE,
    
#ifdef SENTRAL_CUSTOM_PARAMPAGE
    SEN_PARAM_PAGE_CUSTOM_READ,
    SEN_PARAM_PAGE_CUSTOM_WRITE,
#endif

};

enum sentral_registers {
	SR_FIFO_START =   0x00,
	SR_FIFO_FLUSH =   0x32,
	SR_CHIP_CONTROL = 0x34,
	SR_HOST_STATUS =  0x35,
	SR_INT_STATUS =   0x36,
	SR_CHIP_STATUS =  0x37,
	SR_FIFO_BYTES =   0x38,
	SR_PARAM_ACK =    0x3A,
	SR_PARAM_SAVE =   0x3B,
	SR_TC_HS_CNT =    0x4E,
	SR_TC_STATUS =    0x4F,
	SR_ERROR =        0x50,
	SR_PARAM_PAGE =   0x54,
	SR_HOST_CONTROL = 0x55,
	SR_PARAM_LOAD =   0x5C,
	SR_PARAM_REQ =    0x64,
	SR_CRASH_TEST =   0x65,
	SR_TC_HS =        0x66,
	SR_ROM_VERSION =  0x70,
	SR_PRODUCT_ID =   0x90,
	SR_REV_ID =       0x91,
	SR_UPLOAD_ADDR =  0x94,
	SR_UPLOAD_DATA =  0x96,
	SR_CRC_HOST =     0x97,
	SR_RESET_REQ =    0x9B,
	SR_PT_READY =     0x9E,
	SR_PT_CONFIG =    0xA0,
	SR_FIRST = SR_CHIP_CONTROL,
	SR_MAX = SR_PT_CONFIG,
};

enum sentral_host_control_flags {
	SEN_HOST_CTRL_ALGO_STANDBY =  1 << 0,
	SEN_HOST_CTRL_ABORT_XFER =    1 << 1,
	SEN_HOST_CTRL_UPDATE_XFER =   1 << 2,
	SEN_HOST_CTRL_INT_DISABLE =   1 << 3,
	SEN_HOST_CTRL_NED_MODE =      1 << 4,
	SEN_HOST_CTRL_AP_SUSPENDED =  1 << 5,
	SEN_HOST_CTRL_REQ_SELF_TEST = 1 << 6,
	SEN_HOST_CTRL_RESERVED =      1 << 7,
};

enum sentral_chip_control_flags {
	SEN_CHIP_CTRL_CPU_RUN =       1 << 0,
	SEN_CHIP_CTRL_UPLOAD_ENABLE = 1 << 1,
};

enum sentral_sensor_type {
	SST_NOP =                          0,
	SST_ACCELEROMETER =                1,
	SST_GEOMAGNETIC_FIELD =            2,
	SST_ORIENTATION =                  3,
	SST_GYROSCOPE =                    4,
	SST_LIGHT =                        5,
	SST_PRESSURE =                     6,
	SST_TEMPERATURE =                  7,
	SST_PROXIMITY =                    8,
	SST_GRAVITY =                      9,
	SST_LINEAR_ACCELERATION =         10,
	SST_ROTATION_VECTOR =             11,
	SST_RELATIVE_HUMIDITY =           12,
	SST_AMBIENT_TEMPERATURE =         13,
	SST_MAGNETIC_FIELD_UNCALIBRATED = 14,
	SST_GAME_ROTATION_VECTOR =        15,
	SST_GYROSCOPE_UNCALIBRATED =      16,
	SST_SIGNIFICANT_MOTION =          17,
	SST_STEP_DETECTOR =               18,
	SST_STEP_COUNTER =                19,
	SST_GEOMAGNETIC_ROTATION_VECTOR = 20,
	SST_SHUB_DEBUG =                  21,
	SST_TILT_DETECTOR =               22,
	SST_PICK_UP_GESTURE =             25,
	SST_VOLUME_UP =                   27,
	SST_VOLUME_DOWN =                 28,
	SST_TOUCH_GESTURE =               29,
	SST_MAX,
	SST_FIRST = SST_ACCELEROMETER,
	SST_ACCEL_OFFSET =                30,
	SST_PM =                          31,
	SST_DEBUG =                     0xF5,
	SST_TIMESTAMP_LSW =             0xFC,
	SST_TIMESTAMP_MSW =             0xFD,
	SST_META_EVENT =                0xFE,
	SST_ALL =                       0xFF,
};

enum sentral_param_page {
	SPP_SYS =                1,
	SPP_ALGO_WARM_START =    2,
	SPP_SENSORS =            3,
	SPP_ALGO_KNOBS =        13,
	SPP_HTC =               14,
};

enum sentral_param_number_fl {
	SP_FL_WARM_START_FIRST = 1,
	SP_FL_WARM_START_LAST = 50,
	SP_FL_ALGO_FIRST =       1,
	SP_FL_ALGO_LAST =       87,
	SP_FL_HTC_FIRST = 1,
	SP_FL_HTC_LAST = 47,
};

enum sentral_param_system {
	SP_SYS_META_EVENT_CONTROL =  1,
	SP_SYS_FIFO_CONTROL =        2,
	SP_SYS_SENSOR_STATUS_B0 =    3,
	SP_SYS_SENSOR_STATUS_B1 =    4,
	SP_SYS_HOST_IRQ_TS =        30,
	SP_SYS_PHYS_SENSOR_STATUS = 31,
};

enum sentral_param_htc {
	SP_HTC_EASY_ACC_01 =  1,
	SP_HTC_EASY_ACC_02 =  2,
	SP_HTC_EASY_ACC_03 =  3,
	SP_HTC_EASY_ACC_04 =  4,
	SP_HTC_EASY_ACC_05 =  5,
	SP_HTC_EASY_ACC_06 =  6,
	SP_HTC_EASY_ACC_07 =  7,
	SP_HTC_EASY_ACC_08 =  8,
	SP_HTC_EASY_ACC_09 =  9,
	SP_HTC_EASY_ACC_10 = 10,
	SP_HTC_EASY_ACC_11 = 11,
	SP_HTC_EASY_ACC_12 = 12,
	SP_HTC_EASY_ACC_29 = 13,
	SP_HTC_EASY_ACC_30 = 14,
	SP_HTC_EASY_ACC_31 = 15,
	SP_HTC_EASY_ACC_32 = 16,
	SP_HTC_EASY_ACC_33 = 17,
	SP_HTC_EASY_ACC_34 = 18,
	SP_HTC_EASY_ACC_COND = 19,
	SP_HTC_ACCEL_OFFSET = 20,
	SP_HTC_PS_THRD = 21,
	SP_HTC_PS_MIN_PS_VALUE = 22,
	SP_HTC_PS_CAL_OUT = 23,
	SP_HTC_PS_POCKET_FLAG = 24,
	SP_HTC_ALS_MODE = 25,
	SP_HTC_ALS_LEVEL_10_VAL_0_3 = 26,
	SP_HTC_ALS_LEVEL_10_VAL_4_7 = 27,
	SP_HTC_ALS_LEVEL_10_VAL_8_9 = 28,
	SP_HTC_ALS_LEVEL_16_VAL_0_3 = 29,
	SP_HTC_ALS_LEVEL_16_VAL_4_7 = 30,
	SP_HTC_ALS_LEVEL_16_VAL_8_11 = 31,
	SP_HTC_ALS_LEVEL_16_VAL_12_15 = 32,
	SP_HTC_GESTURE_MASK = 33,
	SP_HTC_PS_POLLING_MODE = 34,
	SP_HTC_ALS_GADC = 35,
	SP_HTC_ALS_KADC = 36,
	SP_HTC_PS_CANC = 37,
	SP_HTC_SMART_COVER_PS_THRD = 38,
	SP_HTC_SMART_COVER_COVERED = 39,
	SP_HTC_PS_GESTURE_THD = 40,
	SP_HTC_PS_GESTURE_EN = 41,
	SP_HTC_ACCEL_HIGH_G_EN = 42,
	SP_HTC_ACCEL_HIGH_G_THRD = 43,
	SP_HTC_PS_DIFF = 44,
	SP_HTC_PS_FIXED = 45,
	SP_HTC_PS_ADD = 46,
	SP_HTC_PS_AUTOK_ENABLE = 47,
	SP_HTC_FW_VERSION = 48,
	SP_HTC_PS_DATA = 49,
	SP_HTC_EASY_ACC_FIRST = SP_HTC_EASY_ACC_01,
	SP_HTC_EASY_ACC_LAST = SP_HTC_EASY_ACC_COND,
};

enum sentral_sensor_power_mode {
	SEN_SENSOR_POWER_MODE_NOT_PRESENT = 0x00,
	SEN_SENSOR_POWER_MODE_POWER_DOWN,
	SEN_SENSOR_POWER_MODE_SUSPEND,
	SEN_SENSOR_POWER_MODE_SELF_TEST,
	SEN_SENSOR_POWER_MODE_INT_MOTION,
	SEN_SENSOR_POWER_MODE_ONE_SHOT,
	SEN_SENSOR_POWER_MODE_LOW_POWER,
	SEN_SENSOR_POWER_MODE_ACTIVE,
};

enum sentral_iio_channel_scan_id {
	SEN_SCAN_U32_1,
	SEN_SCAN_U32_2,
	SEN_SCAN_U32_3,
	SEN_SCAN_U32_4,
	SEN_SCAN_TS,
};

enum sentral_meta_event {
	SEN_META_NA = 0,
	SEN_META_FLUSH_COMPLETE,
	SEN_META_SAMPLE_RATE_CHANGED,
	SEN_META_POWER_MODE_CHANGED,
	SEN_META_ERROR,
	SEN_META_MAG_TRANSIENT,
	SEN_META_CAL_STATUS_CHANGED,
	SEN_META_STILLNESS_CHANGED,
	SEN_META_ALGO_SLOWDOWN,
	SEN_META_CAL_STABLE,
	SEN_META_RESERVED,
	SEN_META_SENSOR_ERROR,
	SEN_META_FIFO_OVERFLOW,
	SEN_META_DYN_RANGE_CHANGED,
	SEN_META_FIFO_WATERMARK,
	SEN_META_SELF_TEST_RESULTS,
	SEN_META_INITIALIZED,
	SEN_META_MAX,
};

enum sentral_debug_type {
	SEN_DEBUG_STRING = 0,
	SEN_DEBUG_BINARY,
	SEN_DEBUG_RESERVED1,
	SEN_DEBUG_RESERVED2,
};

enum sentral_crash_test_type {
	SEN_CRASH_TEST_WATCHDOG     = 1,
	SEN_CRASH_TEST_MATH_ERROR   = 2,
	SEN_CRASH_TEST_SEG_FAULT    = 3,
	SEN_CRASH_TEST_WR_TO_RO_REG = 4,
	SEN_CRASH_TEST_MAX,
	SEN_CRASH_TEST_FIRST = SEN_CRASH_TEST_WATCHDOG,
};

enum sentral_pt_config_flags {
	SEN_PT_CONFIG_DISABLED =         0,
	SEN_PT_CONFIG_ENABLED =     1 << 0,
	SEN_PT_CONFIG_CLK_STRETCH = 1 << 1,
};

enum sentral_als_level_count {
	SEN_ALS_LEVEL_COUNT_10 = 10,
	SEN_ALS_LEVEL_COUNT_16 = 16,
};

enum sentral_acal_flags {
	SEN_ACAL_FLAG_COMPLETE =     1 << 0,
	SEN_ACAL_FLAG_X_RANGE_FAIL = 1 << 1,
	SEN_ACAL_FLAG_Y_RANGE_FAIL = 1 << 2,
	SEN_ACAL_FLAG_Z_RANGE_FAIL = 1 << 3,
	SEN_ACAL_FLAG_X_LEVEL_FAIL = 1 << 4,
	SEN_ACAL_FLAG_Y_LEVEL_FAIL = 1 << 5,
	SEN_ACAL_FLAG_Z_LEVEL_FAIL = 1 << 6,
	SEN_ACAL_COMPLETE_FAIL =     1 << 7,
};

enum sentral_debug_event_id {
	SEN_DEBUG_CODE_HIMAX_HANDSHAKE_TEST_PASSED = 0x28,
	SEN_DEBUG_CODE_HIMAX_HANDSHAKE_TEST_FAILED = 0x29,
	SEN_DEBUG_CODE_CAMERA_EVENT_REPORTED = 0x2A,
	SEN_DEBUG_CODE_PM_SENSOR_DISABLED_TOUCH = 0x30,
	SEN_DEBUG_CODE_PM_SENSOR_ENABLED_TOUCH = 0x31,
	SEN_DEBUG_CODE_HIMAX_MODE_NOT_ENABLED_ENABLING_NOW = 0x32,
	SEN_DEBUG_CODE_HIMAX_VALID_PACKET_DETECTED = 0x33,
	SEN_DEBUG_CODE_HIMAX_INVALID_PACKET_DETECTED = 0x34,
	SEN_DEBUG_CODE_PROXIMITY_FLAG_CHANGED = 0x35,
	SEN_DEBUG_CODE_HIMAX_GESTURE_MODE_PRE_ENABLED = 0x36,
	SEN_DEBUG_CODE_HIMAX_TIMEOUT_OCCURED = 0x37,
	SEN_DEBUG_CODE_HIMAX_NACK_DETECTED = 0x38 ,
	SEN_DEBUG_CODE_HIMAX_POWER_ON_COMPLETE = 0x39,
	SEN_DEBUG_CODE_HIMAX_POWER_DOWN = 0x40,
	SEN_DEBUG_CODE_PM_SENSOR_CONDITION_1_MATCH = 0x41,
	SEN_DEBUG_CODE_PM_SENSOR_CONDITION_2_MATCH = 0x42,
	SEN_DEBUG_CODE_PM_SENSOR_CONDITION_3_MATCH = 0x43,
	SEN_DEBUG_CODE_SIGNIFICANT_MOTION = 0x44,
	SEN_DEBUG_CODE_PM_SENSOR_PITCH_ROLL_RESET = 0x45,
	SEN_DEBUG_CODE_PM_SENSOR_INITIALIZATION = 0x46,
	SEN_DEBUG_CODE_PM_SENSOR_CAMERA_TIMEOUT = 0x47,
	SEN_DEBUG_CODE_PM_SENSOR_CONDITION_CAMERA_MATCH = 0x48,
	SEN_DEBUG_CODE_HIMAX_INTERRUPT_DETECTED = 0x49,
	SEN_DEBUG_CODE_PM_SENSOR_ACCEL_ANY_MOTION_ENABLE = 0x50,
	SEN_DEBUG_CODE_PM_SENSOR_ACCEL_ANY_MOTION_DISABLE = 0x51,
	SEN_DEBUG_CODE_PM_SENSOR_PROXIMITY_NEAR_DISBALE_TOUCH = 0x52,
	SEN_DEBUG_CODE_PM_SENSOR_TEARDOWN = 0x53,
	SEN_DEBUG_CODE_HIMAX_SENSE_OFF = 0x54,
	SEN_DEBUG_CODE_HIMAX_SENSE_ON = 0x55,
	SEN_DEBUG_CODE_HIMAX_SLEEP_IN = 0x56,
	SEN_DEBUG_CODE_HIMAX_SLEEP_OUT = 0x57,
	SEN_DEBUG_CODE_BETWEEN_DOUBLE_TAP_TIME_INCORRECT = 0xF0,
	SEN_DEBUG_CODE_PRESS_TIME_TOO_SHORT = 0xF1,
	SEN_DEBUG_CODE_WAIT_SECOND_PRESS_TOO_LONG = 0xF2,
	SEN_DEBUG_CODE_PRESS_TIME_TOO_LONG = 0xF3,
	SEN_DEBUG_CODE_SLIDE_DISTANCE_TOO_SHORT = 0xF4,
	SEN_DEBUG_CODE_MORE_THAN_ONE_FINGER = 0xF5,
	SEN_DEBUG_CODE_MAX,
	SEN_DEBUG_CODE_FIRST = SEN_DEBUG_CODE_HIMAX_HANDSHAKE_TEST_PASSED,
};

enum sentral_event_code {
	SEN_EVENT_SYSFS_REG_WRITE = 0,
	SEN_EVENT_SYSFS_ALGO_STANDBY_ENABLE,
	SEN_EVENT_SYSFS_PM_ENABLE,
	SEN_EVENT_SYSFS_SELF_TEST,
	SEN_EVENT_SYSFS_PT_ENABLE,
	SEN_EVENT_SYSFS_RESET,
	SEN_EVENT_SYSFS_FIFO_READ,
	SEN_EVENT_SYSFS_EA_PARAM_SET,
	SEN_EVENT_SYSFS_SMART_COVER_THRD_SET,
	SEN_EVENT_SYSFS_SENSOR_ENABLE,
	SEN_EVENT_SYSFS_SENSOR_DELAY_MS,
	SEN_EVENT_SYSFS_SENSOR_BATCH,
	SEN_EVENT_SYSFS_SENSOR_FLUSH,
	SEN_EVENT_SYSFS_CAL_ENABLE,
	SEN_EVENT_SYSFS_CAL_DATA_ACCEL_SET,
	SEN_EVENT_SYSFS_CAL_DATA_LIGHT_SET,
	SEN_EVENT_SYSFS_CAL_DATA_PROX_SET,
	SEN_EVENT_SYSFS_GESTURE_MASK_SET,
	SEN_EVENT_SYSFS_CRASH_TEST,
	SEN_EVENT_SYSFS_HIGH_G_EN,
	SEN_EVENT_SYSFS_HIGH_G_THRD,
	SEN_EVENT_SENSOR_CONFIG,
	SEN_EVENT_AP_SUSPEND_ENABLE,
	SEN_EVENT_ALS_LEVELS,
	SEN_EVENT_ALS_SETTING,
};

struct sentral_sensor_data_rv {
	s16 x, y, z, w;
	s16 accuracy;
} __attribute__((__packed__));

struct sentral_sensor_data_amg {
	s16 x, y, z;
	u8 accuracy;
} __attribute__((__packed__));

struct sentral_sensor_data_uncal {
	s16 x, y, z;
	s16 bias_x, bias_y, bias_z;
	u8 accuracy;
} __attribute__((__packed__));

struct sentral_sensor_data_meta {
	u8 number, type, value;
} __attribute__((__packed__));

struct sentral_sensor_data_u24 {
	u8 bytes[3];
} __attribute__((__packed__));

struct sentral_sensor_data_u16 {
	u16 value;
} __attribute__((__packed__));

struct sentral_sensor_data_s16 {
	s16 value;
} __attribute__((__packed__));

struct sentral_sensor_data_u8 {
	u8 value;
} __attribute__((__packed__));

struct sentral_sensor_data_als {
	u16 value;
	u8 level;
	u16 raw_adc;
	u16 kadc;
} __attribute__((__packed__));

struct sentral_sensor_data_ps {
	struct {
		u16 value:12;
		u16 reserved:2;
		u16 flag_int:1;
		u16 flag_far:1;
	} data;
	u16 ps_thdl;
	u16 ps_thdh;
	u16 min_ps;
};

struct sentral_data_meta {
	u8 event_id;
	u8 byte_1;
	u8 byte_2;
};

#define SEN_DATA_DBG_MAXLEN 12

struct sentral_data_debug {
	struct {
		u8 length:6;
		u8 type:2;
	} attr;
	char value[SEN_DATA_DBG_MAXLEN];
};

struct sentral_data_shub_log {
	u8 code;
	u16 tc;
	u16 accel[3];
	u16 ps;
	u8 far;
} __attribute__((__packed__));

struct sentral_param_fifo_control {
	union {
		u64 bytes;
		struct {
			u16 watermark;
			u16 size;
		} fifo;
	};
} __attribute__((__packed__));

struct sentral_param_sensor_status {
	union {
		u8 byte;
		struct {
			u8 data_available:1;
			u8 i2c_nack:1;
			u8 device_id_error:1;
			u8 transient_error:1;
			u8 data_lost:1;
			u8 power_mode:3;
		} bits;
	};
};

struct sentral_param_phys_sensor_status {
	u16 sample_rate;
	u16 dynamic_range;
	union {
		u8 byte;
		struct {
			u8 int_enable:1;
			u8 reserved:4;
			u8 power_mode:3;
		} bits;
	} flags;
} __attribute__((__packed__));

struct sentral_param_phys_sensor_status_page {
	struct sentral_param_phys_sensor_status accel;
	struct sentral_param_phys_sensor_status gyro;
	struct sentral_param_phys_sensor_status mag;
} __attribute__((__packed__));

struct sentral_param_sensor_info {
	u8 sensor_type;
	u8 driver_id;
	u8 driver_version;
	u8 power;
	u16 max_range;
	u16 resolution;
	u16 max_rate;
	u16 fifo_reserved;
	u16 fifo_max;
	u8 event_size;
	u8 reserved;
} __attribute__((__packed__));

struct sentral_param_sensor_config {
	u16 sample_rate;
	u16 max_report_latency;
	u16 change_sensitivity;
	u16 dynamic_range;
} __attribute__((__packed__));

struct sentral_param_timestamp {
	u32 int_stime;
	u32 current_stime;
};

// STATUS

struct sentral_chip_control {
	union {
		u8 byte;
		struct {
			u8 cpu_run:1;
			u8 upload_enable:1;
		} bits;
	};
};

struct sentral_host_status {
	union {
		u8 byte;
		struct {
			u8 cpu_reset:1;
			u8 algo_standby:1;
			u8 host_iface_id:3;
			u8 algo_id:3;
		} bits;
	};
};

struct sentral_htc_als_setting {
	union {
		u8 byte;
		struct {
			u8 polling:1;
			u8 level16:1;
			u8 fast_poll:1;
		} bits;
	};
};

struct sentral_fw_flags {
	u16 eeprom_no_exec:1;
	u16 reserved1:7;
	u16 i2c_clock_speed:3;
	u16 rom_ver_exp:4;
	u16 reserved2:1;
};

struct sentral_fw_header {
	u16 signature;
	struct sentral_fw_flags flags;
	u32 text_crc;
	u32 reserved1;
	u16 text_length;
	u16 reserved2;
} __attribute__((__packed__));

struct sentral_fw_cds {
	u16 signature;
	u8 reserved[6];
	u16 ram_version;
	u8 cds_version;
	u8 boot_protocol;
	u8 pin_selection[8];
	u8 pull_selection[8];
	u8 device_name[16];
} __attribute__((__packed__));

struct sentral_platform_data {
#ifdef CONFIG_MTK
	unsigned int host_int_gpio;
	unsigned int host_int_irq;
#else
	unsigned int gpio_irq;
#endif /* CONFIG_MTK */
	const char *firmware;
	u32 als_level_count;
	u32 als_levels[SENTRAL_ALS_LEVELS_MAX];
	u32 als_goldl;
	u32 als_goldh;
	u32 als_polling;
	u32 als_lux_ratio_n;
	u32 als_lux_ratio_d;
	u32 ps_thd_add;
	u32 ps_thd_fixed;
};

struct sentral_hw_id {
	u8 product_id;
	u8 revision_id;
} __attribute__((__packed__));

struct sentral_ts_ref {
	s64 system_nanos;
	u32 hub_stime;
};

struct sentral_cal_info_accel {
	s16 offset_x, offset_y, offset_z;
} __attribute__((__packed__));

struct sentral_cal_info_als {
	u16 gadc;
	u16 kadc;
} __attribute__((__packed__));

struct sentral_cal_info_ps {
	u16 ps_thrd_diff;
	u16 ps_thrd_l;
	u16 ps_canc;
} __attribute__((__packed__));

struct sentral_cal_info {
	struct sentral_cal_info_accel accel;
	struct sentral_cal_info_als als;
	struct sentral_cal_info_ps ps;
};

struct sentral_shub_log_event {
	struct sentral_data_shub_log data;
	s64 ts;
};

struct sentral_shub_log {
	struct sentral_shub_log_event *buffer;
	size_t size;
	size_t index;
	size_t count;
};

struct sentral_error_state {
	u8 error_value;
	u8 int_state;
	u8 debug_value;
	u8 debug_state;
} __attribute__((__packed__));

struct sentral_device {
	struct i2c_client *client;
	const struct i2c_device_id *device_id;
	struct sentral_platform_data platform_data;
	u32 fw_crc;
	int irq;
	struct dentry *debugfs_dir;
	struct class *hub_class;
	struct device *hub_device;
	struct class *bma253_class;
	struct device *bma253_dev;
	struct iio_dev *indio_dev;
	struct workqueue_struct *sentral_wq;
	struct work_struct work_startup;
	struct work_struct work_fifo_read;
	struct delayed_work work_watchdog;
	struct mutex lock;
	struct mutex lock_i2c;
	struct mutex lock_pio;
	struct mutex lock_ts;
	struct wake_lock w_lock;
	struct notifier_block nb;
	u8 *data_buffer;
	bool init_complete;
	struct sentral_ts_ref ts_ref;
	unsigned long ts_ref_reset;
	u32 ts_sensor_stime;
	s64 ts_sensor_nanos;
	s64 ts_sensor_millis;
	unsigned long enabled_mask;
	unsigned long cal_status_mask;
	int cal_sensor;
	u32 cal_als_kadc;
	int cal_als_count;
	u32 crash_count;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend sentral_early_suspend;
#endif
	s32 vibrate_ms;
	s32 iio_data[6];


#ifdef NVRAM_DATA_ONESHOT
	struct work_struct	one_shot_work;
//	struct workqueue_struct *mcu_wq;
	u32 gs_kvalue;
	s16 gs_kvalue_R1;
	s16 gs_kvalue_R2;
	s16 gs_kvalue_R3;
	s16 gs_kvalue_L1;
	s16 gs_kvalue_L2;
	s16 gs_kvalue_L3;
	u32 gy_kvalue;
	u32 als_kvalue;
	u32 ps_kvalue;
	u32 ps_kheader;
	u32 bs_kvalue;
	u8  bs_kheader;
	u8  gs_calibrated;
	u8  ps_calibrated;
	u8  ls_calibrated;
	u8  gy_calibrated;
#endif
	
	struct sentral_param_sensor_config sensor_config[SST_MAX];
	unsigned long data_ready_mask;
	struct sentral_sensor_data_als data_light_last;
	struct sentral_sensor_data_ps data_prox_last;
	struct sentral_sensor_data_amg cal_data_accel;
	struct sentral_param_sensor_config config_light_prev;
	struct sentral_param_sensor_config config_ps_prev;
	struct sentral_htc_als_setting als_setting;
	struct sentral_htc_als_setting als_setting_prev;
	struct sentral_cal_info cal_info;
	u8 ps_polling;
	u16 smart_cover_ps_thrd;
	struct sentral_shub_log shub_log;
	struct miscdevice misc_shub_log;
	char *shub_log_buf;
	size_t shub_log_size;
	size_t shub_log_pos;
	struct completion complete_shub_log;
	u8 smart_cover_ps_en;
	bool user_calibration_flag;
	int user_offset_buf[3];
	bool pm_sensor_enabled;
	u8 sensor_startup_filter[SST_MAX];
	wait_queue_head_t wq_one_shot;
	unsigned long wait_one_shot_mask;
	u64 step_count;
	u16 step_count_data_prev;
	unsigned long sensors_batched_mask;
	u16 sensors_batched_value[SST_MAX];
	u16 fifo_watermark;
	u8 touch_handshake;
	int warm_start_ready;
};

// CW compat

enum sentral_cw_sensor_id {
	CW_ACCELERATION =                   0,
	CW_MAGNETIC =                       1,
	CW_GYRO =                           2,
	CW_LIGHT =                          3,
	CW_PROXIMITY =                      4,
	CW_PRESSURE =                       5,
	CW_ORIENTATION =                    6,
	CW_ROTATIONVECTOR =                 7,
	CW_LINEARACCELERATION =             8,
	CW_GRAVITY =                        9,
	CW_TEMPERATURE                   = 10,
	CW_AMBIENT_TEMPERATURE           = 11,
	CW_MAGNETIC_UNCALIBRATED =         16,
	CW_GYROSCOPE_UNCALIBRATED =        17,
	CW_GAME_ROTATION_VECTOR =          18,
	CW_GEOMAGNETIC_ROTATION_VECTOR =   19,
	CW_SIGNIFICANT_MOTION =            20,
	CW_STEP_DETECTOR =                 21,
	CW_STEP_COUNTER =                  22,
	HTC_SHUB_DEBUG =                   23,
	HTC_GESTURE_MOTION =               24,
	HTC_PICK_UP_GESTURE =              25,
	HTC_ANY_MOTION =                   28,
	CW_ACCELERATION_W =                32,
	CW_MAGNETIC_W =                    33,
	CW_GYRO_W =                        34,
	CW_PRESSURE_W =                    37,
	CW_ORIENTATION_W =                 38,
	CW_ROTATIONVECTOR_W =              39,
	CW_LINEARACCELERATION_W =          40,
	CW_GRAVITY_W =                     41,
	CW_MAGNETIC_UNCALIBRATED_W =       48,
	CW_GYROSCOPE_UNCALIBRATED_W =      49,
	CW_GAME_ROTATION_VECTOR_W =        50,
	CW_GEOMAGNETIC_ROTATION_VECTOR_W = 51,
	CW_STEP_DETECTOR_W =               53,
	CW_STEP_COUNTER_W =                54,
	CW_SENSORS_ID_TOTAL =              55,
	TIME_DIFF_EXHAUSTED =              97,
	CW_TIME_BASE =                     98,
	CW_META_DATA =                     99,
	CW_MAGNETIC_UNCALIBRATED_BIAS =   100,
	CW_GYROSCOPE_UNCALIBRATED_BIAS =  101,
};

enum sentral_htc_gesture {
	HTC_GESTURE_MOTION_TYPE_SWIPE_UP = 2,
	HTC_GESTURE_MOTION_TYPE_SWIPE_DOWN = 3,
	HTC_GESTURE_MOTION_TYPE_SWIPE_LEFT = 4,
	HTC_GESTURE_MOTION_TYPE_SWIPE_RIGHT = 5,
	HTC_GESTURE_MOTION_TYPE_LAUNCH_CAMERA = 6,
	HTC_GESTURE_MOTION_TYPE_DOUBLE_TAP = 15,
};

enum sentral_htc_cal_sensors {
	HTC_CAL_SENSOR_DISABLE =     0,
	HTC_CAL_SENSOR_ACCEL =       1,
	HTC_CAL_SENSOR_MAG =         2,
	HTC_CAL_SENSOR_GYRO =        3,
	HTC_CAL_SENSOR_LIGHT =       7,
	HTC_CAL_SENSOR_PROX =        8,
	HTC_CAL_SENSOR_ACCEL_X_R =   9,
	HTC_CAL_SENSOR_ACCEL_X_L =  10,
	HTC_CAL_SENSOR_ACCEL_Z =    11,
	HTC_CAL_SENSOR_STEP_RESET = 12,
};

enum sentral_touch_gestures {
	SENTRAL_TOUCH_GESTURE_RIGHT = 0,
	SENTRAL_TOUCH_GESTURE_UP =    1,
	SENTRAL_TOUCH_GESTURE_DOWN =  2,
	SENTRAL_TOUCH_GESTURE_LEFT =  3,
	SENTRAL_TOUCH_GESTURE_DTAP =  4,
};

enum sentral_htc_gesture_flags {
	HTC_GESTURE_FLAG_SWIPE_RIGHT = 1 << 0,
	HTC_GESTURE_FLAG_SWIPE_UP =    1 << 1,
	HTC_GESTURE_FLAG_SWIPE_DOWN =  1 << 2,
	HTC_GESTURE_FLAG_SWIPE_LEFT =  1 << 3,
	HTC_GESTURE_FLAG_DTAP =        1 << 4,
	HTC_GESTURE_FLAG_VOLUME =      1 << 5,
};

enum sentral_htc_touch_status_flags {
	HTC_TOUCH_STATUS_HANDSHAKE_PASS =   1 << 0,
	HTC_TOUCH_STATUS_HANDSHAKE_FAIL =   1 << 1,
	HTC_TOUCH_STATUS_SUSPEND_COMPLETE = 1 << 2,
	HTC_TOUCH_STATUS_RESUME_COMPLETE =  1 << 3,
	HTC_TOUCH_STATUS_TC_NACK =          1 << 4,
	HTC_TOUCH_STATUS_BSLEEPOUT =        1 << 5,
	HTC_TOUCH_STATUS_RESERVED1 =        1 << 6,
	HTC_TOUCH_STATUS_RESERVED2 =        1 << 7,
};

struct sentral_htc_touch_status {
	u8 handshake_count;
	u8 status;
};

struct sentral_htc_ps_data {
	u8 flag_far;
	u16 adc;
	u16 min_adc;
	u16 autok_thd;
	u8 flag_pocket;
} __attribute__((__packed__));

// 24 bytes.
struct sentral_sensors_event {
	u8 sensor_id;
	union {
		struct {
			u16 x, y, z;
			u16 bias_x, bias_y, bias_z;
		} values;
		struct {
			u32 lsw;
			u16 na1;
			u32 msw;
			u16 na2;
		} __attribute__((__packed__)) step_count;
		u8 bytes[12];
	} data;
	u64 ts_ms;
    u8 reserved[3];
} __attribute__((__packed__));

struct sentral_sysfs_attr {
	const char *name;
	int command;
};

ssize_t sentral_sysfs_warm_data_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);

ssize_t sentral_sysfs_warm_data_show(struct device *dev,
		struct device_attribute *attr, char *buf);

int sentral_parameter_page_handle_internal(struct sentral_device *sentral
    , struct sentral_param_page_item items[], int mode);
    
int sentral_read_byte(struct sentral_device *sentral, u8 reg);
    
int sentral_write_byte(struct sentral_device *sentral, u8 reg, u8 value);

int sentral_write_block(struct sentral_device *sentral, u8 reg,
		void *buffer, size_t count);

int sentral_read_block(struct sentral_device *sentral, u8 reg,
		void *buffer, size_t count);

ssize_t sentral_sysfs_warm_data_enable_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);
        
void sentral_warmstart_restore(struct sentral_device *sentral);


ssize_t sentral_sysfs_warm_data_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern struct sentral_param_page_item warm_start_page_items[];

#endif /* _SENTRAL_IIO_H_ */
