#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/notifier.h>
#include <linux/suspend.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/dma-mapping.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/shub_ctrl.h>
#include <linux/completion.h>
#include <asm/uaccess.h>

#include <linux/htc_flags.h>


#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "sentral-core.h"
#include "sentral-iio-debug.h"

#ifdef CONFIG_MTK
#include <linux/vibtrig.h>
#endif /* CONFIG_MTK */

#ifdef CONFIG_MTK
#include <mach/mt_gpio.h>
#include <cust_eint.h>
#endif /* CONFIG_MTK */

static const char *sentral_meta_event_strings[SEN_META_MAX] = {
	[SEN_META_FLUSH_COMPLETE] = "Flush Complete",
	[SEN_META_SAMPLE_RATE_CHANGED] = "Sample Rate Changed",
	[SEN_META_POWER_MODE_CHANGED] = "Power Mode Changed",
	[SEN_META_ERROR] = "Error",
	[SEN_META_MAG_TRANSIENT] = "Magnetic Transient",
	[SEN_META_CAL_STATUS_CHANGED] = "Cal Status Changed",
	[SEN_META_STILLNESS_CHANGED] = "Stillness Changed",
	[SEN_META_ALGO_SLOWDOWN] = "Algorithm Slowdown",
	[SEN_META_CAL_STABLE] = "Calibration Stable",
	[SEN_META_RESERVED] = "Reserved",
	[SEN_META_SENSOR_ERROR] = "Sensor Error",
	[SEN_META_FIFO_OVERFLOW] = "FIFO Overflow",
	[SEN_META_DYN_RANGE_CHANGED] = "Dynamic Range Changed",
	[SEN_META_FIFO_WATERMARK] = "FIFO Watermark",
	[SEN_META_SELF_TEST_RESULTS] = "Self-Test Results",
	[SEN_META_INITIALIZED] = "Initialized",
};

static const char *sentral_sensor_type_strings[SST_MAX] = {
	[SST_NOP] = "NOP",
	[SST_ACCELEROMETER] = "ACCELEROMETER",
	[SST_GEOMAGNETIC_FIELD] = "GEOMAGNETIC_FIELD",
	[SST_ORIENTATION] = "ORIENTATION",
	[SST_GYROSCOPE] = "GYROSCOPE",
	[SST_LIGHT] = "LIGHT",
	[SST_PRESSURE] = "PRESSURE",
	[SST_TEMPERATURE] = "TEMPERATURE",
	[SST_PROXIMITY] = "PROXIMITY",
	[SST_GRAVITY] = "GRAVITY",
	[SST_LINEAR_ACCELERATION] = "LINEAR_ACCELERATION",
	[SST_ROTATION_VECTOR] = "ROTATION_VECTOR",
	[SST_RELATIVE_HUMIDITY] = "RELATIVE_HUMIDITY",
	[SST_AMBIENT_TEMPERATURE] = "AMBIENT_TEMPERATURE",
	[SST_MAGNETIC_FIELD_UNCALIBRATED] = "MAGNETIC_FIELD_UNCALIBRATED",
	[SST_GAME_ROTATION_VECTOR] = "GAME_ROTATION_VECTOR",
	[SST_GYROSCOPE_UNCALIBRATED] = "GYROSCOPE_UNCALIBRATED",
	[SST_SIGNIFICANT_MOTION] = "SIGNIFICANT_MOTION",
	[SST_STEP_DETECTOR] = "STEP_DETECTOR",
	[SST_STEP_COUNTER] = "STEP_COUNTER",
	[SST_GEOMAGNETIC_ROTATION_VECTOR] = "GEOMAGNETIC_ROTATION_VECTOR",
	[SST_SHUB_DEBUG] = "TC_DEBUG",
	[SST_TILT_DETECTOR] = "TILT_DETECTOR",
	[SST_PICK_UP_GESTURE] = "PICK_UP_GESTURE",
	[SST_VOLUME_UP] = "VOLUME_UP",
	[SST_VOLUME_DOWN] = "VOLUME_DOWN",
	[SST_TOUCH_GESTURE] = "TOUCH_GESTURE",
};

static const char *sentral_debug_event_id_strings[SEN_DEBUG_CODE_MAX] = {
	[SEN_DEBUG_CODE_HIMAX_HANDSHAKE_TEST_FAILED] = "DEBUG_CODE_HIMAX_HANDSHAKE_TEST_FAILED",
	[SEN_DEBUG_CODE_HIMAX_HANDSHAKE_TEST_PASSED] = "DEBUG_CODE_HIMAX_HANDSHAKE_TEST_PASSED",
	[SEN_DEBUG_CODE_CAMERA_EVENT_REPORTED] = "DEBUG_CODE_CAMERA_EVENT_REPORTED",
	[SEN_DEBUG_CODE_PM_SENSOR_DISABLED_TOUCH] = "DEBUG_CODE_PM_SENSOR_DISABLED_TOUCH",
	[SEN_DEBUG_CODE_PM_SENSOR_ENABLED_TOUCH] = "DEBUG_CODE_PM_SENSOR_ENABLED_TOUCH",
	[SEN_DEBUG_CODE_HIMAX_MODE_NOT_ENABLED_ENABLING_NOW] = "DEBUG_CODE_HIMAX_MODE_NOT_ENABLED_ENABLING_NOW",
	[SEN_DEBUG_CODE_HIMAX_VALID_PACKET_DETECTED] = "DEBUG_CODE_HIMAX_VALID_PACKET_DETECTED",
	[SEN_DEBUG_CODE_HIMAX_INVALID_PACKET_DETECTED] = "DEBUG_CODE_HIMAX_INVALID_PACKET_DETECTED",
	[SEN_DEBUG_CODE_PROXIMITY_FLAG_CHANGED] = "DEBUG_CODE_PROXIMITY_FLAG_CHANGED",
	[SEN_DEBUG_CODE_HIMAX_GESTURE_MODE_PRE_ENABLED] = "DEBUG_CODE_HIMAX_GESTURE_MODE_PRE_ENABLED",
	[SEN_DEBUG_CODE_HIMAX_TIMEOUT_OCCURED] = "DEBUG_CODE_HIMAX_TIMEOUT_OCCURED",
	[SEN_DEBUG_CODE_HIMAX_NACK_DETECTED] = "DEBUG_CODE_HIMAX_NACK_DETECTED", 
	[SEN_DEBUG_CODE_HIMAX_POWER_ON_COMPLETE] = "DEBUG_CODE_HIMAX_POWER_ON_COMPLETE",
	[SEN_DEBUG_CODE_HIMAX_POWER_DOWN] = "DEBUG_CODE_HIMAX_POWER_DOWN",
	[SEN_DEBUG_CODE_PM_SENSOR_CONDITION_1_MATCH] = "DEBUG_CODE_PM_SENSOR_CONDITION_1_MATCH",
	[SEN_DEBUG_CODE_PM_SENSOR_CONDITION_2_MATCH] = "DEBUG_CODE_PM_SENSOR_CONDITION_2_MATCH",
	[SEN_DEBUG_CODE_PM_SENSOR_CONDITION_3_MATCH] = "DEBUG_CODE_PM_SENSOR_CONDITION_3_MATCH",
	[SEN_DEBUG_CODE_SIGNIFICANT_MOTION] = "DEBUG_CODE_SIGNIFICANT_MOTION",
	[SEN_DEBUG_CODE_PM_SENSOR_PITCH_ROLL_RESET] = "DEBUG_CODE_PM_SENSOR_PITCH_ROLL_RESET",
	[SEN_DEBUG_CODE_PM_SENSOR_INITIALIZATION] = "DEBUG_CODE_PM_SENSOR_INITIALIZATION",
	[SEN_DEBUG_CODE_PM_SENSOR_CAMERA_TIMEOUT] = "DEBUG_CODE_PM_SENSOR_CAMERA_TIMEOUT",
	[SEN_DEBUG_CODE_PM_SENSOR_CONDITION_CAMERA_MATCH] = "DEBUG_CODE_PM_SENSOR_CONDITION_CAMERA_MATCH",
	[SEN_DEBUG_CODE_HIMAX_INTERRUPT_DETECTED] = "DEBUG_CODE_HIMAX_INTERRUPT_DETECTED",
	[SEN_DEBUG_CODE_PM_SENSOR_ACCEL_ANY_MOTION_ENABLE] = "DEBUG_CODE_PM_SENSOR_ACCEL_ANY_MOTION_ENABLE",
	[SEN_DEBUG_CODE_PM_SENSOR_ACCEL_ANY_MOTION_DISABLE] = "DEBUG_CODE_PM_SENSOR_ACCEL_ANY_MOTION_DISABLE",
	[SEN_DEBUG_CODE_PM_SENSOR_PROXIMITY_NEAR_DISBALE_TOUCH] = "DEBUG_CODE_PM_SENSOR_PROXIMITY_NEAR_DISBALE_TOUCH",
	[SEN_DEBUG_CODE_PM_SENSOR_TEARDOWN] = "DEBUG_CODE_PM_SENSOR_TEARDOWN",
	[SEN_DEBUG_CODE_HIMAX_SENSE_OFF] = "DEBUG_CODE_HIMAX_SENSE_OFF",
	[SEN_DEBUG_CODE_HIMAX_SENSE_ON] = "DEBUG_CODE_HIMAX_SENSE_ON",
	[SEN_DEBUG_CODE_HIMAX_SLEEP_IN] = "DEBUG_CODE_HIMAX_SLEEP_IN",
	[SEN_DEBUG_CODE_HIMAX_SLEEP_OUT] = "DEBUG_CODE_HIMAX_SLEEP_OUT",
	[SEN_DEBUG_CODE_BETWEEN_DOUBLE_TAP_TIME_INCORRECT] = "DEBUG_CODE_BETWEEN_DOUBLE_TAP_TIME_INCORRECT",
	[SEN_DEBUG_CODE_PRESS_TIME_TOO_SHORT] = "DEBUG_CODE_PRESS_TIME_TOO_SHORT",
	[SEN_DEBUG_CODE_WAIT_SECOND_PRESS_TOO_LONG] = "DEBUG_CODE_WAIT_SECOND_PRESS_TOO_LONG",
	[SEN_DEBUG_CODE_PRESS_TIME_TOO_LONG] = "DEBUG_CODE_PRESS_TIME_TOO_LONG",
	[SEN_DEBUG_CODE_SLIDE_DISTANCE_TOO_SHORT] = "DEBUG_CODE_SLIDE_DISTANCE_TOO_SHORT",
	[SEN_DEBUG_CODE_MORE_THAN_ONE_FINGER] = "DEBUG_CODE_MORE_THAN_ONE_FINGER",
};

#ifdef SHUB_MTK_DMA
static u8 *sentral_i2c_dma_va = NULL;
static u32 sentral_i2c_dma_pa = NULL;
#endif /* SHUB_MTK_DMA */

static struct sentral_device *priv_sentral;
#ifdef CONFIG_MTK
static struct vib_trigger *vib_trigger = NULL;
#endif /* CONFIG_MTK */

#define BMA253_USER_CALI_GRAVITY_EARTH (980)

static int sentral_easyaccess_enable(struct sentral_device *sentral, bool enable);

// I2C
int sentral_read_byte(struct sentral_device *sentral, u8 reg)
{
	int rc;

	LOGD(&sentral->client->dev, "read byte: reg: 0x%02X\n", reg);

	rc = i2c_smbus_read_byte_data(sentral->client, reg);
	return rc;
}

int sentral_write_byte(struct sentral_device *sentral, u8 reg, u8 value)
{
	int rc;

	LOGD(&sentral->client->dev, "write byte: reg: 0x%02X, value: 0x%02X\n",
			reg, value);

	rc = i2c_smbus_write_byte_data(sentral->client, reg, value);
	return rc;
}

int sentral_write_block(struct sentral_device *sentral, u8 reg,
		void *buffer, size_t count)
{
#ifdef SHUB_MTK_DMA

	int ret;
	s32 retry = 0;
	u8 *wr_buf = sentral_i2c_dma_va;
	struct i2c_client *client = sentral->client;


	struct i2c_msg msg =
	{
		.addr = (client->addr & I2C_MASK_FLAG),
		.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		.flags = 0,
		.buf = sentral_i2c_dma_pa,
		.len = 1 + count,
		.timing = 400
	};
#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] lock lock_i2c before\n");
#endif /* SEN_DBG_MUTEX */
	mutex_lock(&sentral->lock_i2c);
#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] lock lock_i2c after\n");
#endif /* SEN_DBG_MUTEX */

	wr_buf[0] = reg;
    memcpy(wr_buf+1, buffer, count);

	for (retry = 0; retry < 5; ++retry)
	{
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0)
		{
			continue;
		}
#ifdef SEN_DBG_MUTEX
		LOGD(&sentral->client->dev, "[MTX DBG] unlock lock_i2c before\n");
#endif /* SEN_DBG_MUTEX */
		mutex_unlock(&sentral->lock_i2c);
#ifdef SEN_DBG_MUTEX
		LOGD(&sentral->client->dev, "[MTX DBG] unlock lock_i2c after\n");
#endif /* SEN_DBG_MUTEX */
		return count;
	}

	LOGE(&sentral->client->dev, "error (%d) writing data\n", ret);
#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] unlock lock_i2c before\n");
#endif /* SEN_DBG_MUTEX */
	mutex_unlock(&sentral->lock_i2c);
#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] unlock lock_i2c after\n");
#endif /* SEN_DBG_MUTEX */
	return ret;

#else
	u8 buf[1 + count];
	struct i2c_msg msg[] = {
		{
			.addr = sentral->client->addr,
			.flags = 0,
			.len = sizeof(buf),
			.buf = buf,
		},
	};
	int rc;

	if (!count)
		return count;

	buf[0] = reg;
	memcpy(&buf[1], buffer, count);

#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] lock lock_i2c before\n");
#endif /* SEN_DBG_MUTEX */
	mutex_lock(&sentral->lock_i2c);
#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] lock lock_i2c after\n");
#endif /* SEN_DBG_MUTEX */

	rc = i2c_transfer(sentral->client->adapter, msg, 1);

#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] unlock lock_i2c before\n");
#endif /* SEN_DBG_MUTEX */
	mutex_unlock(&sentral->lock_i2c);
#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] unlock lock_i2c after\n");
#endif /* SEN_DBG_MUTEX */

	if (rc != 1) {
		LOGE(&sentral->client->dev, "error (%d) writing data\n", rc);
		return rc;
	}
	return count;
#endif /* SHUB_MTK_DMA */
}

int sentral_read_block(struct sentral_device *sentral, u8 reg,
		void *buffer, size_t count)
{
#ifdef SHUB_MTK_DMA
	int ret;
	s32 retry = 0;
	u8 *data = (u8 *)buffer;
	struct i2c_client *client = sentral->client;
	struct i2c_msg msg[2] =
	{
		{
			.addr = (client->addr & I2C_MASK_FLAG),
			.flags = 0,
			.buf = &reg,
			.len = 1,
			.timing = 400
		},
		{
			.addr = (client->addr & I2C_MASK_FLAG),
			.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
			.flags = I2C_M_RD,
			.buf = sentral_i2c_dma_pa,
			.len = count,
			.timing = 400
		},
	};

#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] lock lock_i2c before\n");
#endif /* SEN_DBG_MUTEX */
	mutex_lock(&sentral->lock_i2c);
#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] lock lock_i2c after\n");
#endif /* SEN_DBG_MUTEX */

	for (retry = 0; retry < 5; ++retry)
	{
		ret = i2c_transfer(client->adapter, &msg[0], 2);
		if (ret < 0)
		{
			continue;
		}
        memcpy(data, sentral_i2c_dma_va, count);
#ifdef SEN_DBG_MUTEX
		LOGD(&sentral->client->dev, "[MTX DBG] unlock lock_i2c before\n");
#endif /* SEN_DBG_MUTEX */
		mutex_unlock(&sentral->lock_i2c);
#ifdef SEN_DBG_MUTEX
		LOGD(&sentral->client->dev, "[MTX DBG] unlock lock_i2c after\n");
#endif /* SEN_DBG_MUTEX */
		return count;
	}

	LOGE(&sentral->client->dev, "error (%d) reading data\n", ret);
#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] unlock lock_i2c before\n");
#endif /* SEN_DBG_MUTEX */
	mutex_unlock(&sentral->lock_i2c);
#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] unlock lock_i2c after\n");
#endif /* SEN_DBG_MUTEX */
	return ret;
#else
	struct i2c_msg msg[] = {
		{
			.addr = sentral->client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = sentral->client->addr,
			.flags = I2C_M_RD,
			.len = count,
			.buf = buffer,
		},
	};
	int rc;

	if (!count)
		return count;

#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] lock lock_i2c before\n");
#endif /* SEN_DBG_MUTEX */
	mutex_lock(&sentral->lock_i2c);
#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] lock lock_i2c after\n");
#endif /* SEN_DBG_MUTEX */

	rc = i2c_transfer(sentral->client->adapter, msg, 2);
#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] unlock lock_i2c before\n");
#endif /* SEN_DBG_MUTEX */
	mutex_unlock(&sentral->lock_i2c);
#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] unlock lock_i2c after\n");
#endif /* SEN_DBG_MUTEX */

	if (rc != 2) {
		LOGE(&sentral->client->dev, "error (%d) reading data\n", rc);
		return rc;
	}
	return count;
#endif /* SHUB_MTK_DMA */
}


// misc

static s64 sentral_get_boottime_ns(void)
{
	struct timespec t;
	t.tv_sec = t.tv_nsec = 0;
	get_monotonic_boottime(&t);
	return (s64)(t.tv_sec) * NSEC_PER_SEC + t.tv_nsec;
}

static void sentral_get_timeofday(struct tm *result)
{
	struct timeval t;
	t.tv_sec = t.tv_usec = 0;
	do_gettimeofday(&t);

	time_to_tm(t.tv_sec, 0, result);
}

static bool sentral_sensor_id_is_continuous_sensor(int id)
{
	switch (id) {
	case SST_ACCELEROMETER:
	case SST_GEOMAGNETIC_FIELD:
	case SST_ORIENTATION:
	case SST_GYROSCOPE:
	case SST_GRAVITY:
	case SST_LINEAR_ACCELERATION:
	case SST_ROTATION_VECTOR:
	case SST_MAGNETIC_FIELD_UNCALIBRATED:
	case SST_GAME_ROTATION_VECTOR:
	case SST_GYROSCOPE_UNCALIBRATED:
	case SST_GEOMAGNETIC_ROTATION_VECTOR:
		return true;
	default:
		return false;
	}
}

static int sentral_sensor_id_cw_to_sentral(int id)
{
	switch (id) {
	case CW_ACCELERATION:
	case CW_ACCELERATION_W:
		return SST_ACCELEROMETER;

	case CW_MAGNETIC:
	case CW_MAGNETIC_W:
		return SST_GEOMAGNETIC_FIELD;

	case CW_GYRO:
	case CW_GYRO_W:
		return SST_GYROSCOPE;

	case CW_LIGHT:
		return SST_LIGHT;

	case CW_PROXIMITY:
		return SST_PROXIMITY;

	case CW_PRESSURE:
	case CW_PRESSURE_W:
		return SST_PRESSURE;

	case CW_ORIENTATION:
	case CW_ORIENTATION_W:
		return SST_ORIENTATION;

	case CW_ROTATIONVECTOR:
	case CW_ROTATIONVECTOR_W:
		return SST_ROTATION_VECTOR;

	case CW_LINEARACCELERATION:
	case CW_LINEARACCELERATION_W:
		return SST_LINEAR_ACCELERATION;

	case CW_GRAVITY:
	case CW_GRAVITY_W:
		return SST_GRAVITY;

	case CW_MAGNETIC_UNCALIBRATED:
	case CW_MAGNETIC_UNCALIBRATED_W:
		return SST_MAGNETIC_FIELD_UNCALIBRATED;

	case CW_GYROSCOPE_UNCALIBRATED:
	case CW_GYROSCOPE_UNCALIBRATED_W:
		return SST_GYROSCOPE_UNCALIBRATED;

	case CW_GAME_ROTATION_VECTOR:
	case CW_GAME_ROTATION_VECTOR_W:
		return SST_GAME_ROTATION_VECTOR;

	case CW_GEOMAGNETIC_ROTATION_VECTOR:
	case CW_GEOMAGNETIC_ROTATION_VECTOR_W:
		return SST_GEOMAGNETIC_ROTATION_VECTOR;

	case HTC_ANY_MOTION:
	case CW_SIGNIFICANT_MOTION:
		return SST_SIGNIFICANT_MOTION;

	case CW_STEP_DETECTOR:
	case CW_STEP_DETECTOR_W:
		return SST_STEP_DETECTOR;

	case CW_STEP_COUNTER:
	case CW_STEP_COUNTER_W:
		return SST_STEP_COUNTER;

	case HTC_GESTURE_MOTION:
		return SST_TOUCH_GESTURE;

	case CW_TEMPERATURE:
		return SST_TEMPERATURE;

	case CW_AMBIENT_TEMPERATURE:
		return SST_AMBIENT_TEMPERATURE;

	case HTC_PICK_UP_GESTURE:
		return SST_PICK_UP_GESTURE;

	case HTC_SHUB_DEBUG:
		return SST_SHUB_DEBUG;

	case SST_ALL:
		return SST_ALL;

	default:
		return -EINVAL;
	}
}

static int sentral_sensor_id_sentral_to_cw(int id)
{
	switch (id) {
	case SST_ACCELEROMETER:
		return CW_ACCELERATION;

	case SST_GEOMAGNETIC_FIELD:
		return CW_MAGNETIC;

	case SST_ORIENTATION:
		return CW_ORIENTATION;

	case SST_GYROSCOPE:
		return CW_GYRO;

	case SST_LIGHT:
		return CW_LIGHT;

	case SST_PRESSURE:
		return CW_PRESSURE;

	case SST_PROXIMITY:
		return CW_PROXIMITY;

	case SST_GRAVITY:
		return CW_GRAVITY;

	case SST_LINEAR_ACCELERATION:
		return CW_LINEARACCELERATION;

	case SST_ROTATION_VECTOR:
		return CW_ROTATIONVECTOR;

	case SST_MAGNETIC_FIELD_UNCALIBRATED:
		return CW_MAGNETIC_UNCALIBRATED;

	case SST_GAME_ROTATION_VECTOR:
		return CW_GAME_ROTATION_VECTOR;

	case SST_GYROSCOPE_UNCALIBRATED:
		return CW_GYROSCOPE_UNCALIBRATED;

	case SST_SIGNIFICANT_MOTION:
		return CW_SIGNIFICANT_MOTION;

	case SST_STEP_DETECTOR:
		return CW_STEP_DETECTOR;

	case SST_STEP_COUNTER:
		return CW_STEP_COUNTER;

	case SST_GEOMAGNETIC_ROTATION_VECTOR:
		return CW_GEOMAGNETIC_ROTATION_VECTOR;

	case SST_TILT_DETECTOR:
		return HTC_ANY_MOTION;

	case SST_PICK_UP_GESTURE:
		return HTC_PICK_UP_GESTURE;

	case SST_VOLUME_UP:
	case SST_VOLUME_DOWN:
	case SST_TOUCH_GESTURE:
		return HTC_GESTURE_MOTION;

	case SST_META_EVENT:
		return CW_META_DATA;

	case SST_TEMPERATURE:
		return CW_TEMPERATURE;

	case SST_AMBIENT_TEMPERATURE:
		return CW_AMBIENT_TEMPERATURE;

	case SST_SHUB_DEBUG:
		return HTC_SHUB_DEBUG;

	case SST_DEBUG:
	case SST_TIMESTAMP_LSW:
	case SST_TIMESTAMP_MSW:
	case SST_RELATIVE_HUMIDITY:
		return -EINVAL;

	case SST_ALL:
		return SST_ALL;

	}

	return -EINVAL;
}

static int sentral_gesture_to_cw_gesture(struct sentral_device *sentral,
		int sensor_id, int value)
{
	int gesture_id = -EINVAL;

	if ((sensor_id == SST_VOLUME_DOWN) || (sensor_id == SST_VOLUME_UP)) {
		gesture_id = HTC_GESTURE_MOTION_TYPE_LAUNCH_CAMERA;
	} else if (sensor_id == SST_TOUCH_GESTURE) {
		switch (value) {

		case SENTRAL_TOUCH_GESTURE_RIGHT:
			gesture_id = HTC_GESTURE_MOTION_TYPE_SWIPE_RIGHT;
			break;

		case SENTRAL_TOUCH_GESTURE_UP:
			gesture_id = HTC_GESTURE_MOTION_TYPE_SWIPE_UP;
			break;

		case SENTRAL_TOUCH_GESTURE_DOWN:
			gesture_id = HTC_GESTURE_MOTION_TYPE_SWIPE_DOWN;
			break;

		case SENTRAL_TOUCH_GESTURE_LEFT:
			gesture_id = HTC_GESTURE_MOTION_TYPE_SWIPE_LEFT;
			break;

		case SENTRAL_TOUCH_GESTURE_DTAP:
			gesture_id = HTC_GESTURE_MOTION_TYPE_DOUBLE_TAP;
			break;

		default:
			gesture_id = -EINVAL;
			break;
		}
	}
	return gesture_id;
}

static int sentral_handle_shub_log(struct sentral_device *sentral, void *data)
{
	struct sentral_data_shub_log *shub_log_data = (struct sentral_data_shub_log *)data;
	struct sentral_shub_log *shub_log = &sentral->shub_log;
	size_t shub_log_index = 0;

	// copy to shub log buffer
	shub_log_index = (shub_log->index++) % shub_log->size;
	shub_log->index %= shub_log->size;
	memcpy(&shub_log->buffer[shub_log_index].data, shub_log_data,
			sizeof(*shub_log_data));

	shub_log->buffer[shub_log_index].ts = sentral->ts_sensor_nanos;
	if (shub_log->count < shub_log->size)
		shub_log->count++;

	complete(&sentral->complete_shub_log);

	return sizeof(*shub_log_data);
}

static int sentral_error_get(struct sentral_device *sentral)
{
	return sentral_read_byte(sentral, SR_ERROR);
}

static int sentral_error_state_log(struct sentral_device *sentral)
{
	struct sentral_error_state error_state = { 0 };
	int rc;

	rc = sentral_read_block(sentral, SR_ERROR, (void *)&error_state,
			sizeof(error_state));

	if (rc < 0) {
		LOGE(&sentral->client->dev, "[ERR] error (%d) reading error state\n", rc);
		return rc;
	}

	LOGE(&sentral->client->dev,
			"[ERR] state: { 0x%02X, 0x%02X, 0x%02X, 0x%02X }\n",
			error_state.error_value, error_state.int_state,
			error_state.debug_value, error_state.debug_state);

	return 0;
}

static int sentral_touch_handshake_get(struct sentral_device *sentral, u8 *value)
{
	int rc = sentral_read_byte(sentral, SR_TC_HS);

	I("[TC] handshake: { 0x%02X }\n", rc);

	if (rc < 0)
		return rc;

	*value = rc;
	return 0;
}

static int sentral_touch_handshake_set(struct sentral_device *sentral, u8 value)
{
	I("[TC] set handshake: { 0x%02X }\n", value);

	return sentral_write_byte(sentral, SR_TC_HS, value);
}

static int sentral_parameter_read(struct sentral_device *sentral,
		u8 page_number, u8 param_number, void *param, size_t size)
{
	int rc;
	int i;
#ifdef SEN_DBG_PIO
	u8 dbuf[size * 5 + 1];
	size_t dcount = 0;
	u8 *dparam = (u8 *)param;
#endif /* SEN_DBG_PIO */

	if (size > PARAM_READ_SIZE_MAX)
		return -EINVAL;

#ifdef SEN_DBG_PIO
	LOGD(&sentral->client->dev,
			"[PIO Read] page: %u, number: %u, count: %zu\n",
			page_number, param_number, size);
#endif /* SEN_DBG_PIO */

#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] lock lock_pio before\n");
#endif /* SEN_DBG_MUTEX */
	mutex_lock(&sentral->lock_pio);
#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] lock lock_pio after\n");
#endif /* SEN_DBG_MUTEX */

	if (size < PARAM_READ_SIZE_MAX)
		page_number = (size << 4) | (page_number & 0x0F);

	// select page
	rc = sentral_write_byte(sentral, SR_PARAM_PAGE, page_number);
	if (rc) {
		LOGE(&sentral->client->dev, "error (%d) selecting parameter page: %u\n",
				rc, page_number);

		goto exit_error_page;
	}

	// select param number
	rc = sentral_write_byte(sentral, SR_PARAM_REQ, param_number);
	if (rc) {
		LOGE(&sentral->client->dev,
				"error (%d) selecting parameter number: %u\n", rc,
				param_number);

		goto exit_error_param;
	}

	// wait for ack
	for (i = 0; i < PARAM_MAX_RETRY; i++) {
		usleep_range(8000, 10000);
		rc = sentral_read_byte(sentral, SR_PARAM_ACK);
#ifdef SEN_DBG_PIO
		LOGD(&sentral->client->dev,
				"[PIO Read] ack: 0x%02X, expected: 0x%02X\n", rc, param_number);
#endif /* SEN_DBG_PIO */
		if (rc < 0) {
			LOGE(&sentral->client->dev, "error (%d) reading parameter ack\n",
					rc);

			goto exit;
		}

		if (rc == param_number)
			goto acked;
	}
	LOGE(&sentral->client->dev, "parameter ack retries (%d) exhausted\n",
			PARAM_MAX_RETRY);

	(void)sentral_error_state_log(sentral);

	rc = -EIO;
	goto exit;

acked:
	// read values
	rc = sentral_read_block(sentral, SR_PARAM_SAVE, param, size);
	if (rc < 0) {
		LOGE(&sentral->client->dev, "error (%d) reading parameter data\n", rc);
		goto exit;
	}
	rc = 0;

#ifdef SEN_DBG_PIO
	for (i = 0; i < size; i++) {
		dcount += scnprintf(dbuf + dcount, PAGE_SIZE - dcount, "0x%02X ", *(dparam + i));
	}
	LOGD(&sentral->client->dev, "[PIO Read] bytes: %s\n", dbuf);
#endif /* SEN_DBG_PIO */

exit:
	(void)sentral_write_byte(sentral, SR_PARAM_PAGE, 0);
exit_error_param:
	(void)sentral_write_byte(sentral, SR_PARAM_REQ, 0);
exit_error_page:
#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] unlock lock_pio before\n");
#endif /* SEN_DBG_MUTEX */
	mutex_unlock(&sentral->lock_pio);
#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] unlock lock_pio after\n");
#endif /* SEN_DBG_MUTEX */
	return rc;
}

static int sentral_parameter_write(struct sentral_device *sentral,
		u8 page_number, u8 param_number, void *param, size_t size)
{
	int rc;
	int i;
#ifdef SEN_DBG_PIO
	u8 dbuf[size * 5 + 1];
	size_t dcount = 0;
	u8 *dparam = (u8 *)param;
#endif /* SEN_DBG_PIO */

	if (size > PARAM_WRITE_SIZE_MAX)
		return -EINVAL;

#ifdef SEN_DBG_PIO
	LOGD(&sentral->client->dev,
			"[PIO Write] page: %u, number: %u, count: %zu\n",
			page_number, param_number, size);

	for (i = 0; i < size; i++) {
		dcount += scnprintf(dbuf + dcount, PAGE_SIZE - dcount, "0x%02X ", *(dparam + i));
	}
	LOGD(&sentral->client->dev, "[PIO Write] bytes: %s\n", dbuf);
#endif /* SEN_DBG_PIO */

#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] lock lock_pio before\n");
#endif /* SEN_DBG_MUTEX */
	mutex_lock(&sentral->lock_pio);
#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] lock lock_pio after\n");
#endif /* SEN_DBG_MUTEX */

	if (size < PARAM_WRITE_SIZE_MAX)
		page_number = (size << 4) | (page_number & 0x0F);

	// select page
	rc = sentral_write_byte(sentral, SR_PARAM_PAGE, page_number);
	if (rc) {
		LOGE(&sentral->client->dev, "error (%d) selecting parameter page: %u\n",
			rc, page_number);

		goto exit_error_page;
	}

	// write values
	rc = sentral_write_block(sentral, SR_PARAM_LOAD, param, size);
	if (rc < 0) {
		LOGE(&sentral->client->dev, "error (%d) writing parameter data\n", rc);
		goto exit_error_page;
	}

	// select param number
	param_number |= 0x80;
	rc = sentral_write_byte(sentral, SR_PARAM_REQ, param_number);
	if (rc) {
		LOGE(&sentral->client->dev,
				"error (%d) selecting parameter number: %u\n", rc,
				param_number);

		goto exit_error_param;
	}

	// wait for ack
	for (i = 0; i < PARAM_MAX_RETRY; i++) {
		usleep_range(8000, 10000);
		rc = sentral_read_byte(sentral, SR_PARAM_ACK);
#ifdef SEN_DBG_PIO
		LOGD(&sentral->client->dev,
				"[PIO Write] ack: 0x%02X, expected: 0x%02X\n", rc, param_number);
#endif /* SEN_DBG_PIO */
		if (rc < 0) {
			LOGE(&sentral->client->dev, "error (%d) reading parameter ack\n",
					rc);

			goto exit;
		}

		if (rc == param_number)
			goto acked;
	}
	LOGE(&sentral->client->dev, "parameter ack retries (%d) exhausted\n",
			PARAM_MAX_RETRY);

	(void)sentral_error_state_log(sentral);

	rc = -EIO;
	goto exit;

acked:
	rc = 0;

exit:
	(void)sentral_write_byte(sentral, SR_PARAM_PAGE, 0);
exit_error_param:
	(void)sentral_write_byte(sentral, SR_PARAM_REQ, 0);
exit_error_page:
#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] unlock lock_pio before\n");
#endif /* SEN_DBG_MUTEX */
	mutex_unlock(&sentral->lock_pio);
#ifdef SEN_DBG_MUTEX
	LOGD(&sentral->client->dev, "[MTX DBG] unlock lock_pio after\n");
#endif /* SEN_DBG_MUTEX */
	return rc;
}

static size_t sentral_parameter_page_dump(struct sentral_device *sentral,
		char *buf, size_t sz, u8 page_number, size_t first, size_t last)
{
	u8 values[SENTRAL_PARAM_SIZE_MAX];
	size_t i, j;
	int rc;
	size_t count = 0;

	for (i = first; i <= last; i++) {
		memset(values, 0, sizeof(values));

		count += scnprintf(buf + count, sz - count, "%-2zu:", i);
		rc = sentral_parameter_read(sentral, page_number, i, (void *)values,
				sizeof(values));
		if (rc) {
			count += scnprintf(buf + count, sz - count,
					" error (%d) reading parameter\n", rc);
			continue;
		}

		for (j = 0; j < SENTRAL_PARAM_SIZE_MAX; j++) {
			count += scnprintf(buf + count, sz - count, " 0x%02X",
					values[j]);
		}
		count += scnprintf(buf + count, sz - count, "\n");
	}
	return count;
}

// Sensors

static int sentral_sensor_config_read(struct sentral_device *sentral, u8 id,
		struct sentral_param_sensor_config *config)
{
	int rc;

	rc = sentral_parameter_read(sentral, SPP_SENSORS,
			id + PARAM_SENSORS_ACTUAL_OFFSET,
			(void *)config, sizeof(*config));

	if (rc) {
		LOGE(&sentral->client->dev,
				"error (%d) reading sensor parameter for sensor id: %u\n",
				rc, id);

		return rc;
	}

	LOGD(&sentral->client->dev,
			"read config: %u = { 0x%04X, 0x%04X, 0x%04X, 0x%04X }\n", id,
			config->sample_rate, config->max_report_latency,
			config->change_sensitivity, config->dynamic_range);

	return 0;
}

static int sentral_sensor_config_write(struct sentral_device *sentral, u8 id,
		struct sentral_param_sensor_config *config)
{
	int rc;

	rc = sentral_parameter_write(sentral, SPP_SENSORS,
			id + PARAM_SENSORS_ACTUAL_OFFSET,
			(void *)config, sizeof(*config));

	if (rc) {
		LOGE(&sentral->client->dev,
				"error (%d) writing sensor parameter for sensor id: %u\n",
				rc, id);

		return rc;
	}

	LOGD(&sentral->client->dev,
			"wrote config: %u = { 0x%04X, 0x%04X, 0x%04X, 0x%04X }\n", id,
			config->sample_rate, config->max_report_latency,
			config->change_sensitivity, config->dynamic_range);

	// update current sensor config
	memcpy(&sentral->sensor_config[id], config,
			sizeof(*config));

	if (id < SST_MAX) {
		sentral->sensors_batched_value[id] = config->max_report_latency;
		clear_bit(id, &sentral->sensors_batched_mask);
		if (config->max_report_latency)
			set_bit(id, &sentral->sensors_batched_mask);
	}
	return 0;
}

static int sentral_pm_enable(struct sentral_device *sentral, bool enable)
{
	struct sentral_param_sensor_config sensor_config;
	int rc;

	LOGD(&sentral->client->dev, "setting pm to %s\n", ENSTR(enable));

	rc = sentral_sensor_config_read(sentral, SST_PM, &sensor_config);
	if (rc)
		return rc;

	sensor_config.sample_rate = !!enable;
	rc = sentral_sensor_config_write(sentral, SST_PM, &sensor_config);

	sentral->pm_sensor_enabled = !!enable;

	return rc;
}

static int sentral_sensor_config_restore(struct sentral_device *sentral)
{
	int i;
	int rc = 0;

	if (sentral->enabled_mask) {
		for (i = 0; i < SST_MAX; i++) {
			if (!test_bit(i, &sentral->enabled_mask))
				continue;

			if ((i == SST_SHUB_DEBUG) || (i == SST_VOLUME_UP)
					|| (i == SST_VOLUME_DOWN) || (i == SST_TOUCH_GESTURE)) {
				continue;
			}

			LOGI(&sentral->client->dev, "restoring state for sensor id: %d\n",
					i);

			rc = sentral_sensor_config_write(sentral, i,
					&sentral->sensor_config[i]);

			if (rc) {
				LOGE(&sentral->client->dev,
						"error (%d) restoring sensor config for sensor id: %d\n",
						rc, i);
				continue;
			}
		}
	}

	if (test_bit(SST_TOUCH_GESTURE, &sentral->enabled_mask)) {
		if (sentral->touch_handshake) {
			rc = sentral_touch_handshake_set(sentral, sentral->touch_handshake);
			if (rc) {
				LOGE(&sentral->client->dev,
						"error (%d) setting touch handshake register to 0x%02X\n",
						rc, sentral->touch_handshake);
			}
		}
		return sentral_easyaccess_enable(sentral, true);
	}

	return 0;
}

static int sentral_hw_id_get(struct sentral_device *sentral,
		struct sentral_hw_id *hw_id)
{
	int rc = sentral_read_block(sentral, SR_PRODUCT_ID, (void *)hw_id,
			sizeof(*hw_id));
	return (rc < 0 ? rc : 0);
}

static int sentral_fw_crc_get(struct sentral_device *sentral, u32 *crc)
{
	int rc = sentral_read_block(sentral, SR_CRC_HOST, (void *)crc, sizeof(*crc));
	return (rc < 0 ? rc : 0);
}

static int sentral_fw_version_get(struct sentral_device *sentral, char *buf,
		size_t length)
{
	if (length > SENTRAL_FW_VERSION_LEN)
		return -EINVAL;

	return sentral_parameter_read(sentral, SPP_HTC, SP_HTC_FW_VERSION,
			(void *)buf, length);
}

static int sentral_stime_current_get(struct sentral_device *sentral, u32 *value)
{
	struct sentral_param_timestamp ts = { 0 };
	int rc;

	rc = sentral_parameter_read(sentral, SPP_SYS, SP_SYS_HOST_IRQ_TS,
			(void *)&ts, sizeof(ts));
	if (rc)
		return rc;

	*value = ts.current_stime;

	return 0;
}

static int sentral_ts_ref_get(struct sentral_device *sentral,
		struct sentral_ts_ref *ts_ref)
{
	int rc;

	rc = sentral_stime_current_get(sentral, &ts_ref->hub_stime);
	if (rc)
		return rc;

	ts_ref->system_nanos = sentral_get_boottime_ns();

	return 0;
}

static int sentral_fifo_ctrl_get(struct sentral_device *sentral,
		struct sentral_param_fifo_control *fifo_ctrl)
{
	return sentral_parameter_read(sentral, SPP_SYS, SP_SYS_FIFO_CONTROL,
			(void *)fifo_ctrl, sizeof(struct sentral_param_fifo_control));
}

static int sentral_fifo_ctrl_set(struct sentral_device *sentral,
		struct sentral_param_fifo_control *fifo_ctrl)
{
	return sentral_parameter_write(sentral, SPP_SYS, SP_SYS_FIFO_CONTROL,
			(void *)fifo_ctrl, sizeof(struct sentral_param_fifo_control));
}

static int sentral_fifo_watermark_set(struct sentral_device *sentral,
		u16 watermark)
{
	struct sentral_param_fifo_control fifo_ctrl = {{ 0 }};
	int rc;

	D("[FIFO] set watermark value to %u\n", watermark);

	rc = sentral_fifo_ctrl_get(sentral, &fifo_ctrl);
	if (rc)
		return rc;

	fifo_ctrl.fifo.watermark = watermark;

	return sentral_fifo_ctrl_set(sentral, &fifo_ctrl);
}

static int sentral_fifo_watermark_enable(struct sentral_device *sentral,
		bool enable)
{
	int rc;

	D("[FIFO] set watermark enable to %s\n", TFSTR(enable));

	if (enable) {
		rc = sentral_fifo_watermark_set(sentral, sentral->fifo_watermark);
	} else {
		rc = sentral_fifo_watermark_set(sentral, 0);
	}
	return rc;
}

static int sentral_fifo_watermark_autoset(struct sentral_device *sentral)
{
	struct sentral_param_fifo_control fifo_ctrl = {{ 0 }};
	int rc;

	// get FIFO size
	rc = sentral_fifo_ctrl_get(sentral, &fifo_ctrl);
	if (rc) {
		LOGE(&sentral->client->dev, "error (%d) getting FIFO control\n", rc);
		return rc;
	}

	// set the watermark below FIFO size
	sentral->fifo_watermark = 0;
	if (fifo_ctrl.fifo.size > SENTRAL_FIFO_WATERMARK_BUFFER) {
		sentral->fifo_watermark = fifo_ctrl.fifo.size
				- SENTRAL_FIFO_WATERMARK_BUFFER;
	}

	if (sentral->fifo_watermark)
		return sentral_fifo_watermark_enable(sentral, true);

	return 0;
}

// easy access

static int sentral_htc_easyaccess_get(struct sentral_device *sentral, u8 id,
		int *value)
{
	int rc;

	switch (id) {
	// s16
	case SP_HTC_EASY_ACC_01:
	case SP_HTC_EASY_ACC_02:
	case SP_HTC_EASY_ACC_03:
	case SP_HTC_EASY_ACC_04:
	case SP_HTC_EASY_ACC_05:
	case SP_HTC_EASY_ACC_06:
	case SP_HTC_EASY_ACC_07:
	case SP_HTC_EASY_ACC_08:
	case SP_HTC_EASY_ACC_09:
	case SP_HTC_EASY_ACC_10:
	case SP_HTC_EASY_ACC_29:
	case SP_HTC_EASY_ACC_30:
	case SP_HTC_EASY_ACC_31:
	case SP_HTC_EASY_ACC_32:
		{
			s16 data;
			rc = sentral_parameter_read(sentral, SPP_HTC, id, (void *)&data,
					sizeof(data));
			if (rc)
				return rc;

			*value = (int)data;
		}
		break;

	// u16
	case SP_HTC_EASY_ACC_11:
	case SP_HTC_EASY_ACC_12:
	case SP_HTC_EASY_ACC_33:
	case SP_HTC_EASY_ACC_34:
		{
			u16 data;
			rc = sentral_parameter_read(sentral, SPP_HTC, id, (void *)&data,
					sizeof(data));
			if (rc)
				return rc;

			*value = (int)data;
		}
		break;

	// u8
	case SP_HTC_EASY_ACC_COND:
		{
			u8 data;
			rc = sentral_parameter_read(sentral, SPP_HTC, id, (void *)&data,
					sizeof(data));
			if (rc)
				return rc;

			*value = (int)data;
		}
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int sentral_htc_easyaccess_set(struct sentral_device *sentral, u8 id,
		int value)
{
	int rc;

	switch (id) {
	// s16
	case SP_HTC_EASY_ACC_01:
	case SP_HTC_EASY_ACC_02:
	case SP_HTC_EASY_ACC_03:
	case SP_HTC_EASY_ACC_04:
	case SP_HTC_EASY_ACC_05:
	case SP_HTC_EASY_ACC_06:
	case SP_HTC_EASY_ACC_07:
	case SP_HTC_EASY_ACC_08:
	case SP_HTC_EASY_ACC_09:
	case SP_HTC_EASY_ACC_10:
	case SP_HTC_EASY_ACC_29:
	case SP_HTC_EASY_ACC_30:
	case SP_HTC_EASY_ACC_31:
	case SP_HTC_EASY_ACC_32:
		{
			s16 data = (s16)value;
			rc = sentral_parameter_write(sentral, SPP_HTC, id, (void *)&data,
					sizeof(data));
			if (rc)
				return rc;

		}
		break;

	// u16
	case SP_HTC_EASY_ACC_11:
	case SP_HTC_EASY_ACC_12:
	case SP_HTC_EASY_ACC_33:
	case SP_HTC_EASY_ACC_34:
		{
			u16 data = (u16)value;
			rc = sentral_parameter_write(sentral, SPP_HTC, id, (void *)&data,
					sizeof(data));
			if (rc)
				return rc;

		}
		break;

	// u8
	case SP_HTC_EASY_ACC_COND:
		{
			u8 data = (u8)value;
			rc = sentral_parameter_write(sentral, SPP_HTC, id, (void *)&data,
					sizeof(data));
			if (rc)
				return rc;

		}
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

// PS

static int sentral_ps_polling_enable(struct sentral_device *sentral,
		bool enable)
{
	sentral->ps_polling = !!enable;

	LOGI(&sentral->client->dev,
			"setting PS polling mode to %s\n", ENSTR(enable));

	return sentral_parameter_write(sentral, SPP_HTC, SP_HTC_PS_POLLING_MODE,
			(void *)&sentral->ps_polling, sizeof(sentral->ps_polling));
}

#ifdef CONFIG_AK8789_HALLSENSOR
static int sentral_smart_cover_covered_set(struct sentral_device *sentral,
		bool covered)
{
	u8 value = !!covered;

	LOGI(&sentral->client->dev,
			"setting Smart Cover covered: %s\n", TFSTR(covered));
	return sentral_parameter_write(sentral, SPP_HTC, SP_HTC_SMART_COVER_COVERED,
			(void *)&value, sizeof(value));
}
#endif /* CONFIG_MTK */

static int sentral_smart_cover_ps_thrd_get(struct sentral_device *sentral,
		u16 *thrd)
{
	return sentral_parameter_read(sentral, SPP_HTC, SP_HTC_SMART_COVER_PS_THRD,
			(void *)thrd, sizeof(u16));
}

static int sentral_smart_cover_ps_thrd_set(struct sentral_device *sentral,
		u16 thrd)
{
	LOGI(&sentral->client->dev,
			"setting Smart Cover PS threshold: %u\n", thrd);

	sentral->smart_cover_ps_thrd = thrd;

	return sentral_parameter_write(sentral, SPP_HTC, SP_HTC_SMART_COVER_PS_THRD,
			(void *)&thrd, sizeof(thrd));
}

static int sentral_smart_cover_covered_get(struct sentral_device *sentral,
		u8 *covered)
{
	return sentral_parameter_read(sentral, SPP_HTC, SP_HTC_SMART_COVER_COVERED,
			(void *)covered, sizeof(u8));
}

// accel calibration info

static int sentral_cal_info_accel_set(struct sentral_device *sentral,
		s16 offset_x, s16 offset_y, s16 offset_z)
{
	int rc;

	LOGI(&sentral->client->dev,
			"setting cal info accel offset to { %d, %d, %d }\n",
			offset_x, offset_y, offset_z);

	sentral->cal_info.accel.offset_x = offset_x;
	sentral->cal_info.accel.offset_y = offset_y;
	sentral->cal_info.accel.offset_z = offset_z;

	set_bit(HTC_CAL_SENSOR_ACCEL, &sentral->cal_status_mask);

	rc = sentral_parameter_write(sentral, SPP_HTC, SP_HTC_ACCEL_OFFSET,
			(void *)&sentral->cal_info.accel, sizeof(sentral->cal_info.accel));

	if (rc) {
		LOGE(&sentral->client->dev, "failed to set cal info accel offset\n");
		return rc;
	}

	return 0;
}

// ALS calibration info

static int sentral_cal_info_als_set(struct sentral_device *sentral, u16 gadc,
		u16 kadc)
{
	int rc;

	LOGI(&sentral->client->dev,
			"setting cal info als: { gadc: %u, kadc: %u }\n", gadc, kadc);

	sentral->cal_info.als.gadc = gadc;
	sentral->cal_info.als.kadc = kadc;

	set_bit(HTC_CAL_SENSOR_LIGHT, &sentral->cal_status_mask);

	// set GADC
	rc = sentral_parameter_write(sentral, SPP_HTC, SP_HTC_ALS_GADC,
			(void *)&sentral->cal_info.als.gadc,
			sizeof(sentral->cal_info.als.gadc));

	if (rc) {
		LOGE(&sentral->client->dev, "failed to set cal info gadc\n");
		return rc;
	}

	// set KADC
	rc = sentral_parameter_write(sentral, SPP_HTC, SP_HTC_ALS_KADC,
			(void *)&sentral->cal_info.als.kadc,
			sizeof(sentral->cal_info.als.kadc));

	if (rc) {
		LOGE(&sentral->client->dev, "failed to set cal info kadc\n");
		return rc;
	}

	return 0;
}

// PS calibration info

static int sentral_cal_info_ps_set(struct sentral_device *sentral,
		u16 ps_thrd_diff, u16 ps_thrd_l, u16 ps_canc)
{
	int rc;
//	u16 ps_thrd = ps_thrd_diff << 8 | ps_thrd_l;

	LOGI(&sentral->client->dev,
			"setting cal info ps: { ps_thrd_diff: %u, ps_thrd_l: %u, ps_canc: %u }\n",
			ps_thrd_diff, ps_thrd_l, ps_canc);

	sentral->cal_info.ps.ps_thrd_diff = ps_thrd_diff;
	sentral->cal_info.ps.ps_thrd_l = ps_thrd_l;
	sentral->cal_info.ps.ps_canc = ps_canc;

	set_bit(HTC_CAL_SENSOR_PROX, &sentral->cal_status_mask);

	// set PS_THRD
	rc = sentral_parameter_write(sentral, SPP_HTC, SP_HTC_PS_THRD,
			(void *)&ps_thrd_l, sizeof(ps_thrd_l));

	if (rc) {
		LOGE(&sentral->client->dev, "failed to set cal info ps_thrd\n");
		return rc;
	}

	//set PS_DIFF
	rc = sentral_parameter_write(sentral, SPP_HTC, SP_HTC_PS_DIFF,
			(void *)&ps_thrd_diff, sizeof(ps_thrd_diff));

	if (rc) {
		LOGE(&sentral->client->dev, "failed to set cal info ps_diff\n");
		return rc;
	}

	// set PS_CANC
	rc = sentral_parameter_write(sentral, SPP_HTC, SP_HTC_PS_CANC,
			(void *)&sentral->cal_info.ps.ps_canc,
			sizeof(sentral->cal_info.ps.ps_canc));

	if (rc) {
		LOGE(&sentral->client->dev, "failed to set cal info ps_canc\n");
		return rc;
	}


	// set PS_ADD
	rc = sentral_parameter_write(sentral, SPP_HTC, SP_HTC_PS_ADD,
			(void *)&sentral->platform_data.ps_thd_add,
			sizeof(sentral->platform_data.ps_thd_add));
	if (rc) {
		LOGE(&sentral->client->dev, "failed to set cal info ps_add\n");
		return rc;
	}

	// set PS_FIXED
	rc = sentral_parameter_write(sentral, SPP_HTC, SP_HTC_PS_FIXED,
			(void *)&sentral->platform_data.ps_thd_fixed,
			sizeof(sentral->platform_data.ps_thd_fixed));
	if (rc) {
		LOGE(&sentral->client->dev, "failed to set cal info ps_fixed\n");
		return rc;
	}



	return 0;
}

static int sentral_cal_info_load(struct sentral_device *sentral)
{
	int rc;

	// load accel cal info
	if (test_bit(HTC_CAL_SENSOR_ACCEL, &sentral->cal_status_mask)) {
		rc = sentral_cal_info_accel_set(sentral, sentral->cal_info.accel.offset_x,
				sentral->cal_info.accel.offset_y, sentral->cal_info.accel.offset_z);

		if (rc) {
			LOGE(&sentral->client->dev, "failed to load cal info accel\n");
			return rc;
		}
	}

	// load als cal info
	if (test_bit(HTC_CAL_SENSOR_LIGHT, &sentral->cal_status_mask)) {
		rc = sentral_cal_info_als_set(sentral, sentral->cal_info.als.gadc,
				sentral->cal_info.als.kadc);

		if (rc) {
			LOGE(&sentral->client->dev, "failed to load cal info ALS\n");
			return rc;
		}
	}

	// load ps cal info
	if (test_bit(HTC_CAL_SENSOR_PROX, &sentral->cal_status_mask)) {
		rc = sentral_cal_info_ps_set(sentral, sentral->cal_info.ps.ps_thrd_diff,
				sentral->cal_info.ps.ps_thrd_l, sentral->cal_info.ps.ps_canc);

		if (rc) {
			LOGE(&sentral->client->dev, "failed to load cal info PS\n");
			return rc;
		}
	}

	return 0;
}

// ALS config

static int sentral_config_als_lux_ratio_set(struct sentral_device *sentral,
		u32 lux_ratio_n, u32 lux_ratio_d)
{
	LOGI(&sentral->client->dev, "setting ALS lux ratio: { N: %u, D: %u }\n",
			lux_ratio_n, lux_ratio_d);

	return 0;
}

static int sentral_config_als_gold_set(struct sentral_device *sentral,
		u32 goldh, u32 goldl)
{
	LOGI(&sentral->client->dev, "setting ALS gold value: { H: %u, L: %u }\n",
			goldh, goldl);

	return 0;
}

static int sentral_config_als_levels_set(struct sentral_device *sentral,
		u32 *levels, u8 level_count)
{
	int rc;
	int i;
	u16 values[4];
	u8 param_number = 0;
	size_t sz;

	switch (level_count) {
	case SEN_ALS_LEVEL_COUNT_10:
		param_number = SP_HTC_ALS_LEVEL_10_VAL_0_3;
		break;

	case SEN_ALS_LEVEL_COUNT_16:
		param_number = SP_HTC_ALS_LEVEL_16_VAL_0_3;
		break;
	}

	while (level_count) {
		memset(values, 0, sizeof(values));
		sz = 0;
		for (i = 0; (i < 4) && level_count; i++) {
			values[i] = *levels++;
			level_count--;
			sz += sizeof(u16);
		}

		LOGI(&sentral->client->dev,
				"setting ALS table param %u to { %u, %u, %u, %u }\n",
				param_number, values[0], values[1], values[2], values[3]);

		rc = sentral_parameter_write(sentral, SPP_HTC, param_number,
				(void *)&values, sz);

		if (rc) {
			LOGE(&sentral->client->dev,
					"error (%d) writing ALS config levels parameter\n", rc);
			return rc;
		}

		param_number++;
	}
	return 0;

}

static int sentral_config_als_mode_get(struct sentral_device *sentral,
		struct sentral_htc_als_setting *als_setting)
{
	return sentral_parameter_read(sentral, SPP_HTC, SP_HTC_ALS_MODE,
			(void *)&als_setting->byte, sizeof(als_setting->byte));
}

static int sentral_config_als_mode_set(struct sentral_device *sentral,
		bool polling, u8 level_count, bool fast_poll)
{
	struct sentral_htc_als_setting als_setting;
	int rc;

	als_setting.byte = 0;

	als_setting.bits.polling = !!polling;
	als_setting.bits.fast_poll = !!fast_poll;

	switch (level_count) {
	case SEN_ALS_LEVEL_COUNT_10:
		als_setting.bits.level16 = 0;
		break;

	case SEN_ALS_LEVEL_COUNT_16:
		als_setting.bits.level16 = 1;
		break;

	default:
		LOGE(&sentral->client->dev, "invalid ALS level count: %u\n",
				level_count);
		return -EINVAL;
	}

	LOGI(&sentral->client->dev,
			"setting ALS config mode: { polling: %s, levels: %u, fast_poll: %s }\n",
			TFSTR(!!polling), level_count, TFSTR(!!fast_poll));

	rc = sentral_parameter_write(sentral, SPP_HTC, SP_HTC_ALS_MODE,
			(void *)&als_setting.byte, sizeof(als_setting.byte));

	if (rc) {
		LOGE(&sentral->client->dev,
				"error (%d) writing ALS config mode parameter\n", rc);
		return rc;
	}

	return 0;
}

static int sentral_config_load_als(struct sentral_device *sentral)
{
	int rc;

	// als mode
	rc = sentral_config_als_mode_set(sentral,
			(sentral->platform_data.als_polling > 0),
			sentral->platform_data.als_level_count, false);

	if (rc) {
		LOGE(&sentral->client->dev, "error (%d) setting ALS config mode", rc);
		return rc;
	}

	// als levels
	rc = sentral_config_als_levels_set(sentral,
			sentral->platform_data.als_levels,
			sentral->platform_data.als_level_count);

	// als gold value
	rc = sentral_config_als_gold_set(sentral, sentral->platform_data.als_goldh,
			sentral->platform_data.als_goldl);

	// als lux ratio
	rc = sentral_config_als_lux_ratio_set(sentral,
			sentral->platform_data.als_lux_ratio_n,
			sentral->platform_data.als_lux_ratio_d);

	return 0;
}

// PS config

static int sentral_config_ps_thd_add_set(struct sentral_device *sentral,
		u16 value)
{
	LOGI(&sentral->client->dev, "setting PS THD_ADD to: %u\n",
			value);

	return sentral_parameter_write(sentral, SPP_HTC, SP_HTC_PS_ADD,
			(void *)&value, sizeof(value));
}

static int sentral_config_ps_thd_fixed_set(struct sentral_device *sentral,
		u16 value)
{
	LOGI(&sentral->client->dev, "setting PS THD_FIXED to: %u\n",
			value);

	return sentral_parameter_write(sentral, SPP_HTC, SP_HTC_PS_FIXED,
			(void *)&value, sizeof(value));
}

static int sentral_config_ps_autok_get(struct sentral_device *sentral,
		u8 *value)
{
	return sentral_parameter_read(sentral, SPP_HTC, SP_HTC_PS_AUTOK_ENABLE,
			(void *)value, sizeof(*value));
}

static int sentral_config_ps_autok_set(struct sentral_device *sentral,
		u8 enable)
{
	u8 value = !!enable;

	LOGI(&sentral->client->dev, "setting PS_AUTOK to : %s\n", ENSTR(enable));

	return sentral_parameter_write(sentral, SPP_HTC, SP_HTC_PS_AUTOK_ENABLE,
			(void *)&value, sizeof(value));
}

static int sentral_config_load_ps(struct sentral_device *sentral)
{
	int rc;

	// ps thd add
	rc = sentral_config_ps_thd_add_set(sentral,
			sentral->platform_data.ps_thd_add);

	// ps thd fixed
	rc = sentral_config_ps_thd_fixed_set(sentral,
			sentral->platform_data.ps_thd_fixed);

	return 0;
}

static int sentral_config_load(struct sentral_device *sentral)
{
	int rc;

	// set ALS config
	rc = sentral_config_load_als(sentral);
	if (rc) {
		LOGE(&sentral->client->dev, "error (%d) loading ALS config\n", rc);
		return rc;
	}

	// set PS config
	rc = sentral_config_load_ps(sentral);
	if (rc) {
		LOGE(&sentral->client->dev, "error (%d) loading PS config\n", rc);
		return rc;
	}

	return 0;
}

static void sentral_state_restore(struct sentral_device *sentral)
{
	LOGI(&sentral->client->dev, "restoring shub configuration state\n");

	(void)sentral_config_load(sentral);
	(void)sentral_cal_info_load(sentral);
	(void)sentral_smart_cover_ps_thrd_set(sentral,
			sentral->smart_cover_ps_thrd);
	(void)sentral_fifo_watermark_autoset(sentral);
	(void)sentral_sensor_config_restore(sentral);
}

// calibration

static int sentral_calibration_accel_enable(struct sentral_device *sentral)
{
	struct sentral_param_sensor_config config = { 0 };
	int rc;

	LOGI(&sentral->client->dev, "set accel calibration to enable\n");

	// clear previous cal data
	memset(&sentral->cal_info.accel, 0, sizeof(sentral->cal_info.accel));
	clear_bit(HTC_CAL_SENSOR_ACCEL, &sentral->cal_status_mask);

	// set cal sensor
	sentral->cal_sensor = HTC_CAL_SENSOR_ACCEL;

	// set rate
	config.sample_rate = SENTRAL_CAL_ACCEL_RATE;
	rc = sentral_sensor_config_write(sentral, SST_ACCEL_OFFSET, &config);

	return rc;
}

static int sentral_calibration_accel_disable(struct sentral_device *sentral)
{
	struct sentral_param_sensor_config config = { 0 };

	LOGI(&sentral->client->dev, "set accel calibration to disable\n");

	sentral->cal_sensor = HTC_CAL_SENSOR_DISABLE;

	// disable accel cal sensor
	return sentral_sensor_config_write(sentral, SST_ACCEL_OFFSET, &config);
}

static int sentral_calibration_light_enable(struct sentral_device *sentral)
{
	struct sentral_param_sensor_config config = { 0 };
	int rc;

	LOGI(&sentral->client->dev, "set light calibration to enable\n");

	// clear previous cal data
	sentral->cal_info.als.kadc = 0;
	sentral->cal_als_kadc = 0;
	sentral->cal_als_count = 0;

	// save current als sensor config
	rc = sentral_sensor_config_read(sentral, SST_LIGHT,
			&sentral->config_light_prev);
	if (rc)
		return rc;

	// switch to fast polling mode
	rc = sentral_config_als_mode_set(sentral, true,
			sentral->platform_data.als_level_count, true);

	if (rc)
		return rc;

	// set cal sensor
	sentral->cal_sensor = HTC_CAL_SENSOR_LIGHT;

	// set rate
	config.sample_rate = SENTRAL_CAL_LIGHT_RATE;
	rc = sentral_sensor_config_write(sentral, SST_LIGHT, &config);

	return rc;
}

static int sentral_calibration_light_disable(struct sentral_device *sentral)
{
	int rc;

	LOGI(&sentral->client->dev, "set light calibration to disable\n");

	sentral->cal_sensor = HTC_CAL_SENSOR_DISABLE;

	// switch to previous mode
	rc = sentral_config_als_mode_set(sentral,
			!!sentral->platform_data.als_polling,
			sentral->platform_data.als_level_count, false);
	if (rc)
		return rc;

	// restore previous als state
	return sentral_sensor_config_write(sentral, SST_LIGHT,
			&sentral->config_light_prev);
}

static int sentral_calibration_disable(struct sentral_device *sentral)
{
	int rc = 0;

	switch (sentral->cal_sensor) {
	case HTC_CAL_SENSOR_ACCEL:
		rc = sentral_calibration_accel_disable(sentral);
		break;

	case HTC_CAL_SENSOR_LIGHT:
		rc = sentral_calibration_light_disable(sentral);
		break;

	}

	sentral->cal_sensor = HTC_CAL_SENSOR_DISABLE;

	return rc;
}

static int sentral_calibration_enable(struct sentral_device *sentral, u8 id)
{
	if ((id > 0) && sentral->cal_sensor) {
		LOGE(&sentral->client->dev,
				"calibration is already enabled for sensor %d\n",
				sentral->cal_sensor);
		return -EINVAL;
	}

	switch (id) {
	case HTC_CAL_SENSOR_DISABLE:
		return sentral_calibration_disable(sentral);

	case HTC_CAL_SENSOR_ACCEL:
		return sentral_calibration_accel_enable(sentral);

	case HTC_CAL_SENSOR_LIGHT:
		return sentral_calibration_light_enable(sentral);

	case HTC_CAL_SENSOR_PROX:
	case HTC_CAL_SENSOR_MAG:
	case HTC_CAL_SENSOR_GYRO:
	case HTC_CAL_SENSOR_ACCEL_X_L:
	case HTC_CAL_SENSOR_ACCEL_X_R:
	case HTC_CAL_SENSOR_ACCEL_Z:
	case HTC_CAL_SENSOR_STEP_RESET:
	default:
		return -EINVAL;
	}
}

static void sentral_crash_reset(struct sentral_device *sentral)
{
	int rc;

	LOGI(&sentral->client->dev, "[CRASH] Probable crash %u detected, restarting device ...\n",
			++sentral->crash_count);

	rc = sentral_touch_handshake_get(sentral, &sentral->touch_handshake);
	if (rc) {
		LOGE(&sentral->client->dev,
				"[CRASH] error (%d) reading touch handshake value\n", rc);
	}

	// queue reset
	queue_work(sentral->sentral_wq, &sentral->work_startup);
}

static int sentral_crc_get(struct sentral_device *sentral, u32 *crc)
{
	int rc = sentral_read_block(sentral, SR_CRC_HOST, (void *)crc, sizeof(*crc));
	return rc < 0 ? rc : 0;
}

static int sentral_debug_ps_data(struct sentral_device *sentral)
{
	struct sentral_htc_ps_data ps_data = { 0 };
	int rc;

	rc = sentral_parameter_read(sentral, SPP_HTC, SP_HTC_PS_DATA,
			(void *)&ps_data, sizeof(ps_data));
	if (rc)
		return rc;

	LOGI(&sentral->client->dev,
			"[SNS_DBG] PS { flag_far: %u, adc: %u, min_adc: %u, autok_thd: %u, flag_pocket: %u }\n",
			ps_data.flag_far, ps_data.adc, ps_data.min_adc, ps_data.autok_thd,
			ps_data.flag_pocket);

	return 0;
}

// IIO Buffer

static int sentral_iio_buffer_push(struct sentral_device *sentral,
		int sensor_id, unsigned char *data)
{
#ifdef SEN_DBG_IIO
	char dstr[sizeof(struct sentral_sensors_event) * 5 + 1];
	size_t dstr_len = 0;
	int i;
#endif /* SEN_DBG_IIO */

	// not enabled
	if (!test_bit(sensor_id, &sentral->enabled_mask)
			&& sentral_sensor_id_is_continuous_sensor(sensor_id)) {
		LOGD(&sentral->client->dev,
				"[IIO] dropping sample from disabled sensor: %d\n", sensor_id);
		return 0;
	}

	// startup filtering
	if ((sensor_id < SST_MAX) && (sentral->sensor_startup_filter[sensor_id])) {
		LOGD(&sentral->client->dev,
				"[IIO] dropping startup sample from sensor: %d\n", sensor_id);
		sentral->sensor_startup_filter[sensor_id]--;
		return 0;
	}

#ifdef SEN_DBG_IIO
	// debug
	for (i = 0, dstr_len = 0; i < sizeof(struct sentral_sensors_event); i++) {
		dstr_len += scnprintf(dstr + dstr_len, PAGE_SIZE - dstr_len, " 0x%02X",
				data[i]);
	}
	LOGD(&sentral->client->dev, "[IIO] buffer bytes: %s\n", dstr);
#endif /* SEN_DBG_IIO */


	if((sentral->indio_dev == NULL)||(data == NULL))
		{
			LOGE(&sentral->client->dev, "((sentral->indio_dev == NULL)||(data == NULL))\n");
			return 0;
		}

	return iio_push_to_buffers(sentral->indio_dev, data);
}

static size_t sentral_iio_buffer_push_rv(struct sentral_device *sentral,
		int sensor_id, void *buffer)
{
	int rc;
	struct sentral_sensors_event cw_data;
	struct sentral_sensor_data_rv *data =
			(struct sentral_sensor_data_rv *)buffer;

	int cw_id = sentral_sensor_id_sentral_to_cw(sensor_id);
	if (cw_id < 0) {
		LOGE(&sentral->client->dev, "error locating cw id for sensor id: %d\n", sensor_id);
		goto exit;
	}

	cw_data.sensor_id = cw_id;
	cw_data.data.values.x = data->x;
	cw_data.data.values.y = data->y;
	cw_data.data.values.z = data->z;
	cw_data.data.values.bias_x = data->w;
	cw_data.ts_ms = sentral->ts_sensor_millis;

	rc = sentral_iio_buffer_push(sentral, sensor_id, (unsigned char *)&cw_data);
	if (rc)
		LOGE(&sentral->client->dev, "error (%d) pushing to IIO buffers", rc);

exit:
	return sizeof(struct sentral_sensor_data_rv);
}

static size_t sentral_iio_buffer_push_amg(struct sentral_device *sentral,
		int sensor_id, void *buffer)
{
	int rc;
	struct sentral_sensors_event cw_data = { 0 };
	struct sentral_sensor_data_amg *data =
			(struct sentral_sensor_data_amg *)buffer;

	int cw_id = sentral_sensor_id_sentral_to_cw(sensor_id);
	if (cw_id < 0) {
		LOGE(&sentral->client->dev, "error locating cw id for sensor id: %d\n",
			sensor_id);
		goto exit;
	}

	cw_data.sensor_id = cw_id;
	cw_data.data.values.x = data->x;
	cw_data.data.values.y = data->y;
	cw_data.data.values.z = data->z;
	cw_data.data.values.bias_x = data->accuracy;
	cw_data.ts_ms = sentral->ts_sensor_millis;

	rc = sentral_iio_buffer_push(sentral, sensor_id, (unsigned char *)&cw_data);
	if (rc)
		LOGE(&sentral->client->dev, "error (%d) pushing to IIO buffers", rc);

exit:
	return sizeof(struct sentral_sensor_data_amg);
}

static size_t sentral_iio_buffer_push_uncal(struct sentral_device *sentral,
		int sensor_id, void *buffer)
{
	int rc;
	struct sentral_sensors_event cw_data = { 0 };
	struct sentral_sensor_data_uncal *data =
			(struct sentral_sensor_data_uncal *)buffer;

	int cw_id = sentral_sensor_id_sentral_to_cw(sensor_id);
	if (cw_id < 0) {
		LOGE(&sentral->client->dev, "error locating cw id for sensor id: %d\n", sensor_id);
		goto exit;
	}

	cw_data.sensor_id = cw_id;
	cw_data.data.values.x = data->x;
	cw_data.data.values.y = data->y;
	cw_data.data.values.z = data->z;
	cw_data.data.values.bias_x = data->bias_x;
	cw_data.data.values.bias_y = data->bias_y;
	cw_data.data.values.bias_z = data->bias_z;
	cw_data.ts_ms = sentral->ts_sensor_millis;

	rc = sentral_iio_buffer_push(sentral, sensor_id, (unsigned char *)&cw_data);
	if (rc)
		LOGE(&sentral->client->dev, "error (%d) pushing to IIO buffers", rc);

exit:
	return sizeof(struct sentral_sensor_data_uncal);
}

static size_t sentral_iio_buffer_push_u24(struct sentral_device *sentral,
		int sensor_id, void *buffer)
{
	int rc;
	struct sentral_sensors_event cw_data = { 0 };
	struct sentral_sensor_data_u24 *data =
			(struct sentral_sensor_data_u24 *)buffer;

	u32 value;

	int cw_id = sentral_sensor_id_sentral_to_cw(sensor_id);
	if (cw_id < 0) {
		LOGE(&sentral->client->dev, "error locating cw id for sensor id: %d\n", sensor_id);
		goto exit;
	}

	value = (data->bytes[0] << 16) | (data->bytes[1] << 8) | data->bytes[2];

	cw_data.sensor_id = cw_id;
	memcpy(&cw_data.data.bytes, &value, sizeof(value));
	cw_data.ts_ms = sentral->ts_sensor_millis;

	rc = sentral_iio_buffer_push(sentral, sensor_id, (unsigned char *)&cw_data);
	if (rc)
		LOGE(&sentral->client->dev, "error (%d) pushing to IIO buffers", rc);

exit:
	return sizeof(struct sentral_sensor_data_u24);
}

static size_t sentral_iio_buffer_push_u16(struct sentral_device *sentral,
		int sensor_id, void *buffer)
{
	int rc;
	struct sentral_sensors_event cw_data = { 0 };
	struct sentral_sensor_data_u16 *data =
			(struct sentral_sensor_data_u16 *)buffer;

	int cw_id = sentral_sensor_id_sentral_to_cw(sensor_id);
	if (cw_id < 0) {
		LOGE(&sentral->client->dev, "error locating cw id for sensor id: %d\n", sensor_id);
		goto exit;
	}

	cw_data.sensor_id = cw_id;
	cw_data.data.values.x = data->value;
	cw_data.ts_ms = sentral->ts_sensor_millis;

	rc = sentral_iio_buffer_push(sentral, sensor_id, (unsigned char *)&cw_data);
	if (rc)
		LOGE(&sentral->client->dev, "error (%d) pushing to IIO buffers", rc);

exit:
	return sizeof(struct sentral_sensor_data_u16);
}

static size_t sentral_iio_buffer_push_s16(struct sentral_device *sentral,
		int sensor_id, void *buffer)
{
	int rc;
	struct sentral_sensors_event cw_data = { 0 };
	struct sentral_sensor_data_s16 *data =
			(struct sentral_sensor_data_s16 *)buffer;

	int cw_id = sentral_sensor_id_sentral_to_cw(sensor_id);
	if (cw_id < 0) {
		LOGE(&sentral->client->dev, "error locating cw id for sensor id: %d\n", sensor_id);
		goto exit;
	}

	cw_data.sensor_id = cw_id;
	cw_data.data.values.x = data->value;
	cw_data.ts_ms = sentral->ts_sensor_millis;

	rc = sentral_iio_buffer_push(sentral, sensor_id, (unsigned char *)&cw_data);
	if (rc)
		LOGE(&sentral->client->dev, "error (%d) pushing to IIO buffers", rc);

exit:
	return sizeof(struct sentral_sensor_data_s16);
}

static size_t sentral_iio_buffer_push_none(struct sentral_device *sentral,
		int sensor_id)
{
	int rc;
	struct sentral_sensors_event cw_data = { 0 };

	int cw_id = sentral_sensor_id_sentral_to_cw(sensor_id);
	if (cw_id < 0) {
		LOGE(&sentral->client->dev, "error locating cw id for sensor id: %d\n", sensor_id);
		goto exit;
	}

	cw_data.sensor_id = cw_id;
	cw_data.data.values.x = 1;
	cw_data.ts_ms = sentral->ts_sensor_millis;

	rc = sentral_iio_buffer_push(sentral, sensor_id, (unsigned char *)&cw_data);
	if (rc)
		LOGE(&sentral->client->dev, "error (%d) pushing to IIO buffers", rc);

exit:
	return 0;
}

static size_t sentral_iio_buffer_push_meta_flush(struct sentral_device *sentral,
		int sensor_id, u8* sensor_data)
{
	int rc;
	struct sentral_sensors_event cw_data = { 0 };
	struct sentral_sensor_data_meta *data = 
        (struct sentral_sensor_data_meta *)sensor_data;

	int cw_id = sentral_sensor_id_sentral_to_cw(data->type);
	if (cw_id < 0) {
		LOGE(&sentral->client->dev, "error locating cw id for sensor id: %d\n", sensor_id);
		goto exit;
	}

	cw_data.sensor_id = CW_META_DATA;
	cw_data.data.values.x = cw_id;
	cw_data.ts_ms = sentral->ts_sensor_millis;

	rc = sentral_iio_buffer_push(sentral, sensor_id, (unsigned char *)&cw_data);
	if (rc)
		LOGE(&sentral->client->dev, "error (%d) pushing to IIO buffers", rc);

exit:
	return 0;
}

static size_t sentral_iio_buffer_push_step_count(struct sentral_device *sentral,
		int sensor_id, void *buffer)
{
	int rc;
	struct sentral_sensors_event cw_data = { 0 };
	struct sentral_sensor_data_u16 *data =
			(struct sentral_sensor_data_u16 *)buffer;

	u64 step_count = (sentral->step_count / USHRT_MAX);

	int cw_id = sentral_sensor_id_sentral_to_cw(sensor_id);
	if (cw_id < 0) {
		LOGE(&sentral->client->dev, "error locating cw id for sensor id: %d\n", sensor_id);
		goto exit;
	}

	if ((sentral->step_count != 0)
			&& (sentral->step_count_data_prev >= data->value)) {
		step_count++;
	}

	sentral->step_count = (step_count * USHRT_MAX) + data->value;
	sentral->step_count_data_prev = data->value;

	I("[SNS] Step Count: %llu,\n", sentral->step_count);

	cw_data.sensor_id = cw_id;
	cw_data.data.step_count.lsw = sentral->step_count & 0xFFFFFFFF;
	cw_data.data.step_count.msw = (sentral->step_count >> 32) & 0xFFFFFFFF;
	cw_data.ts_ms = sentral->ts_sensor_millis;

	rc = sentral_iio_buffer_push(sentral, sensor_id, (unsigned char *)&cw_data);
	if (rc)
		LOGE(&sentral->client->dev, "error (%d) pushing to IIO buffers", rc);

exit:
	return sizeof(struct sentral_sensor_data_u16);
}

static size_t sentral_iio_buffer_push_gesture(struct sentral_device *sentral,
		int sensor_id, void *buffer)
{
	int rc;
	struct sentral_sensors_event cw_data = { 0 };
	struct sentral_sensor_data_u16 *data =
			(struct sentral_sensor_data_u16 *)buffer;

	int gesture_id = sentral_gesture_to_cw_gesture(sentral, sensor_id, data->value);
	if (gesture_id < 0) {
		LOGE(&sentral->client->dev,
				"error locating cw gesture id for gesture id: %d\n",
				data->value);

		goto exit;
	}

	cw_data.sensor_id = HTC_GESTURE_MOTION;
	cw_data.data.values.x = gesture_id;
	cw_data.ts_ms = sentral->ts_sensor_millis;

	rc = sentral_iio_buffer_push(sentral, sensor_id, (unsigned char *)&cw_data);
	if (rc)
		LOGE(&sentral->client->dev, "error (%d) pushing to IIO buffers", rc);

exit:
	return sizeof(struct sentral_sensor_data_u16);
}

static size_t sentral_iio_buffer_push_volume(struct sentral_device *sentral)
{
	int rc;
	struct sentral_sensors_event cw_data = { 0 };

	cw_data.sensor_id = HTC_GESTURE_MOTION;
	cw_data.data.values.x = HTC_GESTURE_MOTION_TYPE_LAUNCH_CAMERA;
	cw_data.ts_ms = sentral->ts_sensor_millis;

	rc = sentral_iio_buffer_push(sentral, SST_VOLUME_DOWN,
			(unsigned char *)&cw_data);
	if (rc)
		LOGE(&sentral->client->dev, "error (%d) pushing to IIO buffers", rc);

	return 0;
}

static size_t sentral_iio_buffer_push_als(struct sentral_device *sentral,
		int sensor_id, void *buffer)
{
	int rc;
	struct sentral_sensors_event cw_data = { 0 };
	struct sentral_sensor_data_als *data =
			(struct sentral_sensor_data_als *)buffer;

	int cw_id = sentral_sensor_id_sentral_to_cw(sensor_id);
	if (cw_id < 0) {
		LOGE(&sentral->client->dev, "error locating cw id for sensor id: %d\n", sensor_id);
		goto exit;
	}

	I("[SNS] LS: { adc: %u, level: %u, raw_adc: %u, kadc: %u }\n",
			data->value, data->level, data->raw_adc, data->kadc);

	// one-shot data
	if (test_bit(sensor_id, &sentral->wait_one_shot_mask)) {
		memcpy(&sentral->data_light_last, data, sizeof(*data));
		set_bit(sensor_id, &sentral->data_ready_mask);
		wake_up_interruptible(&sentral->wq_one_shot);
		return sizeof(*data);
	}

	// in calibration mode
	if (sentral->cal_sensor == HTC_CAL_SENSOR_LIGHT) {
		sentral->cal_als_count++;
		sentral->cal_als_kadc += data->value;
		I("[SNS] LS Calibration: { sample: %u, kadc: %u }\n",
				sentral->cal_als_count,
				(sentral->cal_als_kadc / sentral->cal_als_count));

		if (sentral->cal_als_count == SENTRAL_CAL_LIGHT_KADC_COUNT) {
			sentral->cal_als_kadc /= SENTRAL_CAL_LIGHT_KADC_COUNT;
			(void)sentral_cal_info_als_set(sentral, 
					(u16)sentral->platform_data.als_goldh << 8
					| (u16)sentral->platform_data.als_goldl,
					sentral->cal_als_kadc);

			I("[SNS] LS Calibration Complete: { kadc: %u }\n",
					sentral->cal_als_kadc);

			(void)sentral_calibration_light_disable(sentral);
		}
	}

	cw_data.sensor_id = cw_id;
	// polling mode, convert to Lux
	if (sentral->als_setting.bits.polling) {
		cw_data.data.values.x = (((data->value / 100) * 8) * 10);
	} else {
		cw_data.data.values.x = data->value;
	}
	cw_data.data.values.y = data->level;
	cw_data.ts_ms = sentral->ts_sensor_millis;

	rc = sentral_iio_buffer_push(sentral, sensor_id, (unsigned char *)&cw_data);
	if (rc)
		LOGE(&sentral->client->dev, "error (%d) pushing to IIO buffers", rc);

exit:
	return sizeof(*data);
}

static size_t sentral_iio_buffer_push_ps(struct sentral_device *sentral,
		int sensor_id, void *buffer)
{
	int rc;
	struct sentral_sensors_event cw_data = { 0 };
	struct sentral_sensor_data_ps *data =
			(struct sentral_sensor_data_ps *)buffer;

	int cw_id = sentral_sensor_id_sentral_to_cw(sensor_id);
	if (cw_id < 0) {
		LOGE(&sentral->client->dev, "error locating cw id for sensor id: %d\n", sensor_id);
		goto exit;
	}

	I("[SNS] PS: { value: %u, int: %u, far: %u, thdl: %u, thdh: %u, min_ps: %u }\n",
			data->data.value, data->data.flag_int, data->data.flag_far,
			data->ps_thdl, data->ps_thdh, data->min_ps);

	// one-shot data
	if (test_bit(sensor_id, &sentral->wait_one_shot_mask) && !data->data.flag_int) {
		memcpy(&sentral->data_prox_last, data, sizeof(*data));
		set_bit(sensor_id, &sentral->data_ready_mask);
		wake_up_interruptible(&sentral->wq_one_shot);
		return sizeof(*data);
	}

	// filter out polling data
	if (!data->data.flag_int)
		return sizeof(*data);

	cw_data.sensor_id = cw_id;
	cw_data.data.values.y = data->data.value;
	cw_data.data.values.x = data->data.flag_far;
	cw_data.ts_ms = sentral->ts_sensor_millis;

	rc = sentral_iio_buffer_push(sentral, sensor_id, (unsigned char *)&cw_data);
	if (rc)
		LOGE(&sentral->client->dev, "error (%d) pushing to IIO buffers", rc);

exit:
	return sizeof(*data);
}

static int sentral_request_reset(struct sentral_device *sentral)
{
	int rc;

	LOGI(&sentral->client->dev, "reset request\n");
	rc = sentral_write_byte(sentral, SR_RESET_REQ, 1);
	return rc;
}

static int sentral_log_meta_event(struct sentral_device *sentral,
		struct sentral_data_meta *data)
{
	LOGI(&sentral->client->dev, "Meta Event: %s { 0x%02X, 0x%02X }\n",
			(data->event_id < SEN_META_MAX
					? sentral_meta_event_strings[data->event_id]
					: "Unknown"), data->byte_1, data->byte_2);

	return 0;
}

static int sentral_log_debug_data(struct sentral_device *sentral,
		struct sentral_data_debug *data)
{
	char buf[(SEN_DATA_DBG_MAXLEN * 5) + 1];
	size_t i;
	size_t count = 0;

	if (data->attr.length > SEN_DATA_DBG_MAXLEN)
		return -EINVAL;

	switch (data->attr.type) {
	case SEN_DEBUG_STRING:
		for (i = 0; (i < data->attr.length) && (i < SEN_DATA_DBG_MAXLEN); i++)
			count += scnprintf(buf + count, PAGE_SIZE - count, "%c",
					data->value[i]);
		break;

	case SEN_DEBUG_BINARY:
		for (i = 0; (i < data->attr.length) && (i < SEN_DATA_DBG_MAXLEN); i++)
			count += scnprintf(buf + count, PAGE_SIZE - count, "0x%02X ",
					data->value[i]);
		break;

	default:
		break;
	}
	dev_info(&sentral->client->dev, "Debug Event: %s\n", buf);

	return 0;
}

static int sentral_handle_meta_event(struct sentral_device *sentral, void *data)
{
	struct sentral_data_meta *meta_data = (struct sentral_data_meta *)data;

	if (sentral_log_meta_event(sentral, meta_data))
		LOGE(&sentral->client->dev, "error parsing meta event\n");

	// push flush complete events
	if (meta_data->event_id == SEN_META_FLUSH_COMPLETE) {
		//(void)sentral_iio_buffer_push_meta_flush(sentral, meta_data->byte_1, (u8*)meta_data);
		(void)sentral_iio_buffer_push_meta_flush(sentral, SST_META_EVENT, (u8*)meta_data);
	}

	// restore sensors on init
	if (meta_data->event_id == SEN_META_INITIALIZED) {
		sentral_state_restore(sentral);
		sentral_warmstart_restore(sentral);
		sentral->init_complete = true;
	}

	return sizeof(*meta_data);
}

static int sentral_handle_debug_data(struct sentral_device *sentral, void *data)
{
	struct sentral_data_debug *debug_data = (struct sentral_data_debug *)data;

	if (sentral_log_debug_data(sentral, debug_data))
		LOGE(&sentral->client->dev, "error parsing debug data\n");

	return sizeof(*debug_data);
}

static int sentral_handle_ts_msw(struct sentral_device *sentral, void *buffer)
{
	u16 ts = *(u16 *)buffer;

	mutex_lock(&sentral->lock_ts);

	sentral->ts_sensor_stime = ts << 16;

	mutex_unlock(&sentral->lock_ts);

	D("[TS] MSW: { ts: %5u 0x%04X }\n", ts, ts);

	return sizeof(ts);
}

static int sentral_handle_ts_lsw(struct sentral_device *sentral, void *buffer)
{
	u16 ts = *(u16 *)buffer;
	s64 dt_stime = 0;
	s64 dt_nanos = 0;

	mutex_lock(&sentral->lock_ts);

	sentral->ts_sensor_stime &= 0xFFFF0000;
	sentral->ts_sensor_stime |= ts;

	dt_stime = (s64)(sentral->ts_sensor_stime) - (s64)(sentral->ts_ref.hub_stime);

	dt_nanos = dt_stime * SENTRAL_SENSOR_TIMESTAMP_SCALE_NS;
	sentral->ts_sensor_nanos = sentral->ts_ref.system_nanos + dt_nanos;
	sentral->ts_sensor_millis = sentral->ts_sensor_nanos / 1000000LL;

	D("[TS] { ts: %u, ref_stime: %u, ref_nanos: %lld, dt_stime: %lld, dt_nanos: %lld, ts_nanos: %lld, ts_millis: %lld }\n",
			ts, sentral->ts_ref.hub_stime, sentral->ts_ref.system_nanos,
			dt_stime, dt_nanos, sentral->ts_sensor_nanos,
			sentral->ts_sensor_millis);

	mutex_unlock(&sentral->lock_ts);

	D("[TS] LSW: { ts: %5u 0x%04X }\n", ts, ts);

	return sizeof(ts);
}

// fifo

static int sentral_fifo_flush(struct sentral_device *sentral, u8 sensor_id)
{
	int rc;

	LOGI(&sentral->client->dev, "FIFO flush sensor ID: 0x%02X\n", sensor_id);
	rc = sentral_write_byte(sentral, SR_FIFO_FLUSH, sensor_id);
	return rc;
}

static int sentral_fifo_get_bytes_remaining(struct sentral_device *sentral,
	u16 *value)
{
	int rc;

	LOGD(&sentral->client->dev, "[FIFO BR]\n");

	rc = sentral_read_block(sentral, SR_FIFO_BYTES, (void *)value,
			sizeof(*value));

	LOGD(&sentral->client->dev, "FIFO bytes remaining: %u\n", *value);

	if (rc < 0) {
		LOGE(&sentral->client->dev, "error (%d) reading FIFO bytes remaining\n",
				rc);

		return rc;
	}

	return 0;
}

static int sentral_fifo_parse(struct sentral_device *sentral, u8 *buffer,
		size_t bytes)
{
	u8 sensor_id;
	size_t data_size;

	LOGD(&sentral->client->dev, "[FIFO Parse]\n");

	while (bytes) {
		// get sensor id
		sensor_id = *buffer++;

		LOGD(&sentral->client->dev, "[FIFO Parse] bytes: %zu, sensor_id: %u\n",
				bytes, sensor_id);

		bytes--;
		data_size = 0;

		switch (sensor_id) {

		case SST_TIMESTAMP_MSW:
			data_size = sentral_handle_ts_msw(sentral, buffer);
			break;

		case SST_TIMESTAMP_LSW:
			data_size = sentral_handle_ts_lsw(sentral, buffer);
			break;

		case SST_NOP:
			continue;

		case SST_SIGNIFICANT_MOTION:
		case SST_STEP_DETECTOR:
		case SST_TILT_DETECTOR:
		case SST_PICK_UP_GESTURE:
			data_size = sentral_iio_buffer_push_none(sentral, sensor_id);
			break;

		case SST_SHUB_DEBUG:
			data_size = sentral_handle_shub_log(sentral, buffer);
			break;

		case SST_LIGHT:
			data_size = sentral_iio_buffer_push_als(sentral, sensor_id, buffer);
			break;

		case SST_PROXIMITY:
			data_size = sentral_iio_buffer_push_ps(sentral, sensor_id, buffer);
			break;

		case SST_RELATIVE_HUMIDITY:
			data_size = sentral_iio_buffer_push_u16(sentral, sensor_id, buffer);
			break;

		case SST_VOLUME_UP:
		case SST_VOLUME_DOWN:
		{
#ifdef CONFIG_MTK
			if (vib_trigger)
				vib_trigger_event(vib_trigger, sentral->vibrate_ms);
#endif
			data_size = sentral_iio_buffer_push_volume(sentral);
		}
			break;

		case SST_TOUCH_GESTURE:
		{
#ifdef CONFIG_MTK
			if (vib_trigger)
				vib_trigger_event(vib_trigger, sentral->vibrate_ms);
#endif
			data_size = sentral_iio_buffer_push_gesture(sentral, sensor_id,
					buffer);
		}
			break;

		case SST_STEP_COUNTER:
			data_size = sentral_iio_buffer_push_step_count(sentral, sensor_id,
					buffer);
			break;

		case SST_TEMPERATURE:
		case SST_AMBIENT_TEMPERATURE:
			data_size = sentral_iio_buffer_push_s16(sentral, sensor_id, buffer);
			break;

		case SST_PRESSURE:
			data_size = sentral_iio_buffer_push_u24(sentral, sensor_id, buffer);
			break;

		case SST_ACCELEROMETER:
		case SST_GEOMAGNETIC_FIELD:
		case SST_GYROSCOPE:
		case SST_ORIENTATION:
		case SST_GRAVITY:
		case SST_LINEAR_ACCELERATION:
			data_size = sentral_iio_buffer_push_amg(sentral, sensor_id, buffer);
			break;

		case SST_ROTATION_VECTOR:
		case SST_GAME_ROTATION_VECTOR:
		case SST_GEOMAGNETIC_ROTATION_VECTOR:
			data_size = sentral_iio_buffer_push_rv(sentral, sensor_id, buffer);
			break;

		case SST_MAGNETIC_FIELD_UNCALIBRATED:
		case SST_GYROSCOPE_UNCALIBRATED:
			data_size = sentral_iio_buffer_push_uncal(sentral, sensor_id,
					buffer);
			break;

		case SST_META_EVENT:
			data_size = sentral_handle_meta_event(sentral, buffer);
			break;

		case SST_DEBUG:
			data_size = sentral_handle_debug_data(sentral, buffer);
			break;

		case SST_ACCEL_OFFSET:
			{
				struct sentral_sensor_data_amg *amg_data = 
						(struct sentral_sensor_data_amg *)buffer;

				LOGI(&sentral->client->dev,
						"Accel Cal Data: { %d, %d, %d, 0x%02X }\n",
						amg_data->x, amg_data->y, amg_data->z,
						amg_data->accuracy);

				if (amg_data->accuracy & SEN_ACAL_FLAG_COMPLETE) {
					(void)sentral_calibration_accel_disable(sentral);

					if (amg_data->accuracy == SEN_ACAL_FLAG_COMPLETE) {
						(void)sentral_cal_info_accel_set(sentral, amg_data->x,
								amg_data->y, amg_data->z);
					}
				}
				data_size = sizeof(struct sentral_sensor_data_amg);

			}
			break;

		default:
			LOGE(&sentral->client->dev, "invalid sensor type: %u\n", sensor_id);
			// queue another read
			queue_work(sentral->sentral_wq, &sentral->work_fifo_read);
			return -EINVAL;
		}

		buffer += data_size;
		bytes -= data_size;

	}
	return 0;
}

static int sentral_fifo_read_block(struct sentral_device *sentral, u8 *buffer,
		size_t bytes)
{
#ifdef SHUB_MTK_DMA
	size_t bytes_to_read = 0;
	size_t bytes_read = 0;
	int rc;

	while (bytes) {
		LOGD(&sentral->client->dev,
				"[FIFO Block] bytes: %zu, to_read: %zu, read: %zu\n",
				bytes, bytes_to_read, bytes_read);

		bytes_to_read = MIN(bytes, I2C_BLOCK_SIZE_MAX);
		rc = sentral_read_block(sentral, SR_FIFO_START, (void *)(buffer + bytes_read), bytes_to_read);
		if (rc < 0) {
			LOGE(&sentral->client->dev, "Read FIFO block ERROR!\n");
			return rc;
		}

		bytes -= rc;
		bytes_read += rc;
	}
	LOGD(&sentral->client->dev, "[FIFO Block] bytes_read: %zu\n", bytes_read);
	return bytes_read;
#else
	LOGD(&sentral->client->dev, "[FIFO Block] bytes: %zu\n", bytes);
	return sentral_read_block(sentral, SR_FIFO_START, (void *)buffer, bytes);
#endif /* SHUB_MTK_DMA */
}

static int sentral_fifo_read(struct sentral_device *sentral, u8 *buffer,
	size_t *bytes_read)
{
	u16 bytes_remaining = 0;
	int rc;

	LOGD(&sentral->client->dev, "[FIFO Read]\n");

	// sync ref timestamp
	if (sentral->ts_ref_reset && sentral->init_complete) {
		rc = sentral_ts_ref_get(sentral, &sentral->ts_ref);
		if (rc)
			LOGE(&sentral->client->dev, "error (%d) syncing ref time\n", rc);
		sentral->ts_ref_reset = 0;
	}

	// get bytes remaining
	rc = sentral_fifo_get_bytes_remaining(sentral, &bytes_remaining);
	if (rc) {
		LOGE(&sentral->client->dev,
				"error (%d) reading FIFO bytes remaining\n", rc);
		return rc;
	}
	*bytes_read = bytes_remaining;

	if (!bytes_remaining)
		return 0;

	// check buffer overflow
	if (bytes_remaining > DATA_BUFFER_SIZE) {
		LOGE(&sentral->client->dev, "FIFO read buffer overflow (%u > %u)\n",
				bytes_remaining, DATA_BUFFER_SIZE);
		return -EINVAL;
	}

	// read FIFO
	rc = sentral_fifo_read_block(sentral, buffer, bytes_remaining);
	if (rc < 0) {
		LOGE(&sentral->client->dev, "error (%d) reading FIFO\n", rc);
		return rc;
	}

	// parse buffer
	rc = sentral_fifo_parse(sentral, buffer, bytes_remaining);
	if (rc < 0) {
		LOGE(&sentral->client->dev, "error (%d) parsing FIFO\n", rc);
		return rc;
	}

	return 0;
}

static void sentral_do_work_fifo_read(struct work_struct *work)
{
	struct sentral_device *sentral = container_of(work, struct sentral_device,
			work_fifo_read);
	size_t bytes_read = 0;
	int rc;

	LOGD(&sentral->client->dev, "[FIFO Work]\n");

	rc = sentral_fifo_read(sentral, (void *)sentral->data_buffer, &bytes_read);
	if (rc) {
		LOGE(&sentral->client->dev, "error (%d) reading FIFO\n", rc);
//		return;
	}

	if (sentral->enabled_mask) {
		queue_delayed_work(sentral->sentral_wq, &sentral->work_watchdog,
				msecs_to_jiffies(SENTRAL_WATCHDOG_WORK_MSECS));
	}
}

static int sentral_set_register_flag(struct sentral_device *sentral, u8 reg,
		u8 flag, bool enable)
{
	int rc;
	u8 value;

	LOGD(&sentral->client->dev, "setting register 0x%02X flag 0x%02X to %u\n",
			reg, flag, enable);

	rc = sentral_read_byte(sentral, reg);
	if (rc < 0) {
		LOGE(&sentral->client->dev, "error (%d) reading register 0x%02X\n", rc,
				reg);

		return rc;
	}

	value = rc & ~(flag);
	if (enable)
		value |= flag;

	if (value == rc)
		return 0;

	rc = sentral_write_byte(sentral, reg, value);
	if (rc) {
		LOGE(&sentral->client->dev,
				"error (%d) setting register 0x%02X to 0x%02X\n", rc, reg,
				value);

		return rc;
	}

	return 0;
}

// chip control

static int sentral_set_chip_control_flag(struct sentral_device *sentral,
		u8 flag, bool enable)
{
	return sentral_set_register_flag(sentral, SR_CHIP_CONTROL, flag, enable);
}

static int sentral_set_cpu_run_enable(struct sentral_device *sentral,
		bool enable)
{
	LOGI(&sentral->client->dev, "setting CPU run to %s\n", ENSTR(enable));
	return sentral_set_chip_control_flag(sentral, SEN_CHIP_CTRL_CPU_RUN,
			enable);
}

static int sentral_set_host_upload_enable(struct sentral_device *sentral,
		bool enable)
{
	LOGI(&sentral->client->dev, "setting upload enable to %s\n", ENSTR(enable));
	return sentral_set_chip_control_flag(sentral, SEN_CHIP_CTRL_UPLOAD_ENABLE,
			enable);
}

// host iface control

static int sentral_set_host_iface_control_flag(struct sentral_device *sentral,
		u8 flag, bool enable)
{
	return sentral_set_register_flag(sentral, SR_HOST_CONTROL, flag, enable);
}

// algo standby

static int sentral_set_host_algo_standby_enable(struct sentral_device *sentral,
		bool enable)
{
	LOGI(&sentral->client->dev, "setting algorithm standby to %s\n",
			ENSTR(enable));
	return sentral_set_host_iface_control_flag(sentral,
			SEN_HOST_CTRL_ALGO_STANDBY, enable);
}

// ap suspend

static int sentral_set_host_ap_suspend_enable(struct sentral_device *sentral,
		bool enable)
{
	LOGI(&sentral->client->dev, "setting AP suspend to %s\n", ENSTR(enable));

	return sentral_set_host_iface_control_flag(sentral,
			SEN_HOST_CTRL_AP_SUSPENDED, enable);
}

// self-test

static int sentral_set_host_self_test_request(struct sentral_device *sentral)
{
	LOGI(&sentral->client->dev, "requesting sensor self-test\n");
	return sentral_set_host_iface_control_flag(sentral,
			SEN_HOST_CTRL_REQ_SELF_TEST, true);
}

// pass-through config

static int sentral_set_pt_config_flag(struct sentral_device *sentral, u8 flag, bool enable)
{
	return sentral_set_register_flag(sentral, SR_PT_CONFIG, flag, enable);
}

// pass-through mode

static int sentral_set_pt_enable(struct sentral_device *sentral, bool enable)
{
	int rc;
	int i;

	dev_info(&sentral->client->dev, "set pass-through mode to %s\n",
			ENSTR(enable));

	rc = sentral_set_pt_config_flag(sentral, SEN_PT_CONFIG_ENABLED, enable);
	if (rc)
		goto exit;

	for (i = 0; i < SENTRAL_PT_MAX_RETRY; i++) {
		usleep_range(8000, 10000);
		rc = sentral_read_byte(sentral, SR_PT_READY);
		if (rc < 0)
			goto exit;

		if ((rc & SEN_PT_CONFIG_ENABLED) == !!enable)
			return 0;
	}
	dev_err(&sentral->client->dev, "pass-through retries (%d) exhausted\n", SENTRAL_PT_MAX_RETRY);
	rc = -EINVAL;

exit:
	return rc;
}

static int sentral_firmware_load(struct sentral_device *sentral,
		const char *firmware_name)
{
	const struct firmware *fw;
	struct sentral_fw_header *fw_header;
	struct sentral_fw_cds *fw_cds;
	u32 *fw_data;
	size_t fw_data_size;
	int rc = 0;

	LOGI(&sentral->client->dev, "loading firmware: %s\n", firmware_name);

	// load fw from system
	rc = request_firmware(&fw, firmware_name, &sentral->client->dev);
	if (rc) {
		LOGE(&sentral->client->dev, "error (%d) loading firmware: %s\n", rc,
				firmware_name);

		goto exit;
	}

	// check fw size too small
	if (fw->size < sizeof(*fw_header)) {
		LOGE(&sentral->client->dev, "invalid firmware image size\n");
		goto exit;
	}

	// check fw signature
	fw_header = (struct sentral_fw_header *)fw->data;
	if (fw_header->signature != FW_IMAGE_SIGNATURE) {
		LOGE(&sentral->client->dev, "invalid firmware signature\n");
		goto exit;
	}

	// check fw size too big
	if ((sizeof(*fw_header) + fw_header->text_length) > fw->size) {
		LOGE(&sentral->client->dev, "invalid firmware image size\n");
		goto exit;
	}

	fw_cds = (struct sentral_fw_cds *)(sizeof(*fw_header) + fw->data
			+ fw_header->text_length - sizeof(struct sentral_fw_cds));

	// send reset request
	rc = sentral_request_reset(sentral);
	if (rc) {
		LOGE(&sentral->client->dev, "error (%d) requesting reset\n", rc);
		goto exit_release;
	}

	// enable host upload
	rc = sentral_set_host_upload_enable(sentral, true);
	if (rc) {
		LOGE(&sentral->client->dev, "error (%d) enabling host upload\n", rc);
		goto exit_release;
	}

	fw_data = (u32 *)(((u8 *)fw->data) + sizeof(*fw_header));
	fw_data_size = fw->size - sizeof(*fw_header);

	while (fw_data_size) {
		u32 buf[MIN(RAM_BUF_LEN, I2C_BLOCK_SIZE_MAX) / sizeof(u32)];
		size_t ul_size = MIN(fw_data_size, sizeof(buf));
		int i;

		for (i = 0; i < ul_size / 4; i++)
			buf[i] = swab32(*fw_data++);

		rc = sentral_write_block(sentral, SR_UPLOAD_DATA, (void *)buf, ul_size);
		if (rc < 0) {
			LOGE(&sentral->client->dev, "error (%d) uploading data\n", rc);
			goto exit_release;
		}

		fw_data_size -= ul_size;
	}

	// disable host upload
	rc = sentral_set_host_upload_enable(sentral, false);
	if (rc) {
		LOGE(&sentral->client->dev, "error (%d) disabling host upload\n", rc);
		goto exit_release;
	}

	// check CRC
	rc = sentral_crc_get(sentral, &sentral->fw_crc);
	if (rc < 0) {
		LOGE(&sentral->client->dev, "error (%d) reading host CRC\n", rc);
		goto exit_release;
	}

	LOGI(&sentral->client->dev, "host CRC: 0x%08X, fw CRC: 0x%08X\n",
			sentral->fw_crc, fw_header->text_crc);

	if (sentral->fw_crc != fw_header->text_crc) {
		LOGE(&sentral->client->dev,
				"invalid firmware CRC, expected 0x%08X got 0x%08X\n",
				sentral->fw_crc, fw_header->text_crc);

		goto exit_release;
	}

	LOGI(&sentral->client->dev, "firmware CRC OK\n");


	rc = 0;
exit_release:
	release_firmware(fw);
exit:
	return rc;
}

static int sentral_sensor_info_read(struct sentral_device *sentral, u8 id,
		struct sentral_param_sensor_info *info)
{
	int rc;

	rc = sentral_parameter_read(sentral, SPP_SENSORS, id, (void *)info,
			sizeof(struct sentral_param_sensor_info));

	if (rc) {
		LOGE(&sentral->client->dev,
				"error (%d) reading sensor parameter for sensor id: %u\n",
				rc, id);

		return rc;
	}

	return 0;
}

static int sentral_sensor_id_is_valid(u8 id)
{
	return (((id >= SST_FIRST) && (id < SST_MAX)) || (id == SST_ALL));
}

static int sentral_sensor_batch_set(struct sentral_device *sentral, u8 id,
		u16 rate, u16 timeout_ms)
{
	int rc;
	struct sentral_param_sensor_config config = { 0 };

	LOGI(&sentral->client->dev, "batch set id: %u, rate: %u, timeout: %u\n",
			id, rate, timeout_ms);

	if (!sentral_sensor_id_is_valid(id))
		return -EINVAL;

	rc = sentral_sensor_config_read(sentral, id, &config);
	if (rc) {
		LOGE(&sentral->client->dev,
				"error (%d) reading sensor config for sensor id: %u\n",
				rc, id);

		return rc;
	}

	config.sample_rate = rate;
	config.max_report_latency = timeout_ms;

	// update config
	rc = sentral_sensor_config_write(sentral, id, &config);
	if (rc) {
		LOGE(&sentral->client->dev,
				"error (%d) writing sensor config for sensor id: %u\n",
				rc, id);
		return rc;
	}

	return 0;
}

static int sentral_sensor_enable_set(struct sentral_device *sentral, u8 id,
		bool enable)
{
	unsigned long current_enabled_mask = sentral->enabled_mask;

	LOGI(&sentral->client->dev, "enable set id: %u, enable: %u\n", id, !!enable);

	if (!enable) {
		clear_bit(id, &sentral->enabled_mask);

		if (!sentral->enabled_mask && current_enabled_mask)
			cancel_delayed_work_sync(&sentral->work_watchdog);

		return sentral_sensor_batch_set(sentral, id, 0, 0);
	}

	switch (id) {
	case SST_ORIENTATION:
		sentral->sensor_startup_filter[SST_ORIENTATION] = SEN_SNS_STARTUP_FILT_ORIENT;
		break;
	case SST_ROTATION_VECTOR:
		sentral->sensor_startup_filter[SST_ROTATION_VECTOR] = SEN_SNS_STARTUP_FILT_RV;
		break;
	case SST_GEOMAGNETIC_ROTATION_VECTOR:
		sentral->sensor_startup_filter[SST_GAME_ROTATION_VECTOR] = SEN_SNS_STARTUP_FILT_GMRV;
		break;
	}

	set_bit(id, &sentral->enabled_mask);
	return 0;
}

static int sentral_easyaccess_enable(struct sentral_device *sentral,
		bool enable)
{
	int rc;
	int i;
	int ea_sensors[] = {
		SST_SHUB_DEBUG, SST_TOUCH_GESTURE, SST_VOLUME_DOWN, SST_VOLUME_UP
	};

	// PM sensor first to disable
	if (!enable) {
		rc = sentral_pm_enable(sentral, enable);
		if (rc) {
			LOGE(&sentral->client->dev, "error (%d) disabling pm sensors\n", rc);
			return rc;
		}
	}

	for (i = 0; i < ARRAY_SIZE(ea_sensors); i++) {
		int id = ea_sensors[i];
		rc = sentral_sensor_enable_set(sentral, id, enable);
		if (rc)
			return rc;

		if (enable)
			rc = sentral_sensor_batch_set(sentral, id, SENTRAL_SNS_EA_RATE, 0);

		if (rc) {
			LOGE(&sentral->client->dev, "error (%d) setting %s to %s\n", rc,
					sentral_sensor_type_strings[id], ENSTR(enable));
			return rc;
		}
	}

	// PM sensor last to enable
	if (enable) {
		rc = sentral_pm_enable(sentral, enable);
		if (rc) {
			LOGE(&sentral->client->dev, "error (%d) enabling pm sensors\n", rc);
			return rc;
		}
	}

	// debug PS data on disable
	if (!enable && test_bit(SST_PROXIMITY, &sentral->enabled_mask))  {
		rc = sentral_debug_ps_data(sentral);
		if (rc)
			LOGE(&sentral->client->dev, "error (%d) dumping PS params\n", rc);
	}

	// schedule a timestamp sync
	if (!enable)
		sentral->ts_ref_reset = 1;

	return 0;
}

static int sentral_crash_test(struct sentral_device *sentral, u8 test_id)
{
	return sentral_write_byte(sentral, SR_CRASH_TEST, test_id); 
}

#ifdef CONFIG_AK8789_HALLSENSOR
static int hallsensor_status_handler_func(struct notifier_block *this,
	unsigned long status, void *unused)
{
	struct sentral_device *sentral = priv_sentral;

	int pole_value = status & 0x01;
	int pole = (status & 0x02) >> HALL_POLE_BIT;
	int rc;
	LOGI(&sentral->client->dev, "%s\n", __FUNCTION__);
	if (pole == HALL_POLE_S) {
		u8 bool = (pole_value != HALL_FAR);
		if (bool != sentral->smart_cover_ps_en) {
			rc = sentral_smart_cover_covered_set(sentral, bool);
			sentral->smart_cover_ps_en = bool;
		}
	}
	return NOTIFY_OK;
}

static struct notifier_block hallsensor_status_handler = {
	.notifier_call = hallsensor_status_handler_func,
};
#endif

static ssize_t sentral_shub_log_data_to_string(struct sentral_device *sentral,
		struct sentral_data_shub_log *data, u64 ts, char *buf, size_t length)
{
	size_t count = 0;
	struct tm tod = { 0 };

	if ((data->code >= SEN_DEBUG_CODE_MAX) || (data->code < SEN_DEBUG_CODE_FIRST)) {
		LOGE(&sentral->client->dev, "invalid debug code: %u\n", data->code);
		return -EINVAL;
	}

	// current system time
	sentral_get_timeofday(&tod);
	count += scnprintf(buf + count, length - count,
			"%ld-%02d-%02d %02d:%02d:%02d GMT\n",
			(tod.tm_year + 1900L), (tod.tm_mon + 1), tod.tm_mday,
			tod.tm_hour, tod.tm_min, tod.tm_sec);

	count += scnprintf(buf + count, length - count, "[%5llu:%06llu] ",
			ts / NSEC_PER_SEC, (ts % NSEC_PER_SEC) / 1000LL);

	count += scnprintf(buf + count, length - count,
			"(0x%02X) { 0x%04X, 0x0000 } %s\n", data->code, data->tc,
			sentral_debug_event_id_strings[data->code]);

	count += scnprintf(buf + count, length - count, "[%5llu:%06llu] ",
			ts / NSEC_PER_SEC, (ts % NSEC_PER_SEC) / 1000LL);

	count += scnprintf(buf + count, length - count,
			"Accel Avg: { 0x%04X, 0x%04X, 0x%04X }\n", data->accel[0],
			data->accel[1], data->accel[2]);

	count += scnprintf(buf + count, length - count, "[%5llu:%06llu] ",
			ts / NSEC_PER_SEC, (ts % NSEC_PER_SEC) / 1000LL);

	count += scnprintf(buf + count, length - count,
			"PS counts: %u, far: %u\n", data->ps, data->far);

	return count;
}

static int shub_log_parse(struct sentral_device *sentral)
{
	struct sentral_shub_log *shub_log = &sentral->shub_log;
	struct sentral_data_shub_log *event_data;
	ssize_t rc = 0;

	D("%s:\n", __func__);

	while (sentral->shub_log_pos != shub_log->index) {
		D("%s: pos: %zu, size: %zu\n", __func__, sentral->shub_log_pos, sentral->shub_log_size);

		event_data = &shub_log->buffer[sentral->shub_log_pos].data;

		rc = sentral_shub_log_data_to_string(sentral, event_data,
				shub_log->buffer[sentral->shub_log_pos].ts,
				sentral->shub_log_buf + sentral->shub_log_size,
				(SENTRAL_SHUB_LOG_BUF_SIZE - sentral->shub_log_size));

		D("%s: rc: %zd\n", __func__, rc);

		if (rc < 0) {
			sentral->shub_log_pos = shub_log->index;
			return rc;
		}

		sentral->shub_log_size += rc;

		sentral->shub_log_pos++;
		sentral->shub_log_pos %= shub_log->size;

	}

	return 0;
}

static int shub_log_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int shub_log_release(struct inode *inode, struct file *file)
{
	D("%s:\n", __func__);

	file->private_data = NULL;
	return 0;
}

static ssize_t shub_log_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct sentral_device *sentral = container_of(file->private_data,
			struct sentral_device, misc_shub_log);

	int rc;

	size_t bytes = (sentral->shub_log_size - *ppos);
	if (bytes > count)
		bytes = count;

	D("%s: count: %zu, ppos: %lld, size: %zu, bytes: %zu\n", __func__, count, *ppos, sentral->shub_log_size, bytes);

	rc = copy_to_user(buf, sentral->shub_log_buf + *ppos, bytes);

	if (count != bytes)
		*ppos += bytes;

	return bytes;
}

static ssize_t shub_log_write(struct file *file, const char __user * buf, size_t size, loff_t *pos)
{
	return -ENODEV;
}

static long shub_log_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct sentral_device *sentral = container_of(file->private_data,
			struct sentral_device, misc_shub_log);

	int rc = 0;

	switch (cmd) {
	case SHUB_LOG_IOCTL_WAIT_FOR_NOTIFY:
		{
			unsigned int mcu_log_type = 0;
			wait_for_completion_interruptible(&sentral->complete_shub_log);
			put_user(mcu_log_type, (unsigned int __user *)arg);
			INIT_COMPLETION(sentral->complete_shub_log);
		}
		break;

	case SHUB_LOG_IOCTL_GET_LOG_SIZE:
		put_user(sentral->shub_log_size, (unsigned int __user *)arg);
		break;

	case SHUB_LOG_IOCTL_GET_LOG_START:
		if (!file)
			return -ENODEV;

		sentral->shub_log_size = 0;
		sentral->shub_log_buf = devm_kzalloc(&sentral->client->dev,
				SENTRAL_SHUB_LOG_BUF_SIZE, GFP_KERNEL);

		if (!sentral->shub_log_buf)
			return -ENOMEM;

		return shub_log_parse(sentral);

		break;

	case SHUB_LOG_IOCTL_GET_LOG_DONE:
		sentral->shub_log_size = 0;
		devm_kfree(&sentral->client->dev, sentral->shub_log_buf);
		break;

	case SHUB_LOG_IOCTL_SET_LOGMASK:
	case SHUB_LOG_IOCTL_GET_LOGMASK:
	case SHUB_LOG_IOCTL_SET_LOGLEVEL:
	case SHUB_LOG_IOCTL_GET_LOGLEVEL:
	default:
		E("%s(%d): INVALID param:0x%x\n", __func__, __LINE__, cmd);
		return -EINVAL;
	}

	return rc;
}

static const struct file_operations shub_log_fops = {
	.owner = THIS_MODULE,
	.read  = shub_log_read,
	.write = shub_log_write,
	.unlocked_ioctl = shub_log_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = shub_log_ioctl,
#endif
	.open  = shub_log_open,
	.release = shub_log_release,
};

static int sentral_miscdev_create(struct sentral_device *sentral)
{
	int rc;

	// shub_log
	sentral->misc_shub_log.minor = MISC_DYNAMIC_MINOR;
	sentral->misc_shub_log.name = SHUB_LOG_DEVICE_NAME;
	sentral->misc_shub_log.fops = &shub_log_fops;
	sentral->misc_shub_log.parent = &sentral->client->dev;
	rc = misc_register(&sentral->misc_shub_log);

	return 0;
}

static int sentral_miscdev_destroy(struct sentral_device *sentral)
{
	if (sentral->misc_shub_log.minor)
		misc_deregister(&sentral->misc_shub_log);

	return 0;
}

// SYSFS

// chip control

static ssize_t sentral_sysfs_chip_control_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	ssize_t count = 0;
	int rc;
	struct sentral_chip_control chip_control;

	rc = sentral_read_byte(sentral, SR_CHIP_CONTROL);
	if (rc < 0)
		return rc;

	chip_control.byte = rc;

	count += scnprintf(buf + count, PAGE_SIZE - count, "%-16s: %s\n",
			"CPU Run", (chip_control.bits.cpu_run ? "true" : "false"));

	count += scnprintf(buf + count, PAGE_SIZE - count, "%-16s: %s\n",
			"Upload Enable",
			(chip_control.bits.upload_enable ? "true" : "false"));

	return count;
}

static DEVICE_ATTR(chip_control, S_IRUGO, sentral_sysfs_chip_control_show,
		NULL);

// host status

static ssize_t sentral_sysfs_host_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	ssize_t count = 0;
	int rc;
	struct sentral_host_status host_status;

	rc = sentral_read_byte(sentral, SR_HOST_STATUS);
	if (rc < 0)
		return rc;

	host_status.byte = rc;

	count += scnprintf(buf + count, PAGE_SIZE - count, "%-16s: %s\n",
			"CPU Reset", (host_status.bits.cpu_reset ? "true" : "false"));

	count += scnprintf(buf + count, PAGE_SIZE - count, "%-16s: %s\n",
			"Algo Standby", (host_status.bits.algo_standby ? "true" : "false"));

	count += scnprintf(buf + count, PAGE_SIZE - count, "%-16s: %u\n",
			"Host Iface ID", (host_status.bits.host_iface_id >> 2) & 0x07);

	count += scnprintf(buf + count, PAGE_SIZE - count, "%-16s: %u\n",
			"Algo ID", (host_status.bits.algo_id >> 5) & 0x07);

	return count;
}

static DEVICE_ATTR(host_status, S_IRUGO, sentral_sysfs_host_status_show, NULL);

// chip status

static ssize_t sentral_sysfs_chip_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	ssize_t count = 0;
	int rc;
	const char bit_strings[][20] = { "EEPROM", "EEUploadDone", "EEUploadError",
			"Idle", "NoEEPROM" };

	int i;

	rc = sentral_read_byte(sentral, SR_CHIP_STATUS);
	if (rc < 0)
		return rc;

	LOGD(dev, "read chip_status: %d\n", rc);
	for (i = 0; i < sizeof(bit_strings) / 20; i++) {
		count += scnprintf(buf + count, PAGE_SIZE - count, "%-16s: %s\n",
				bit_strings[i], (rc & (1 << i) ? "true" : "false"));
	}

	return count;
}

static DEVICE_ATTR(chip_status, S_IRUGO, sentral_sysfs_chip_status_show, NULL);

// registers

static ssize_t sentral_sysfs_registers_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	ssize_t count = SR_MAX - SR_FIRST + 1;
	ssize_t used = 0;
	u8 regs[count];
	int rc;
	int i;

	mutex_lock(&sentral->lock);
	rc = sentral_read_block(sentral, SR_FIRST, (void *)&regs, count);
	if (rc < 0)
		goto exit;

	for (i = 0; i < count; i++)
		used += scnprintf(buf + used, PAGE_SIZE - used, "0x%02X: 0x%02X\n",
				SR_FIRST + i, regs[i]);

	rc = used;

exit:
	mutex_unlock(&sentral->lock);
	return rc;
}

static ssize_t sentral_sysfs_registers_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	u32 addr;
	u32 value;
	int rc;
	if (2 != sscanf(buf, "%u,%u", &addr, &value))
		return -EINVAL;

	LOGD(&sentral->client->dev, "sysfs registers: { addr: %u, value: %u }\n",
			addr, value);

	if ((addr > SR_MAX) || (addr < SR_FIRST))
		return -EINVAL;

	if (value > 0xFF)
		return -EINVAL;

	rc = sentral_write_byte(sentral, (u8)addr, (u8)value);
	if (rc)
		return rc;

	return count;
}

static DEVICE_ATTR(registers, S_IRUGO | S_IWUGO, sentral_sysfs_registers_show, sentral_sysfs_registers_store);

// sensor info

static ssize_t sentral_sysfs_sensor_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	ssize_t count = 0;
	int rc;
	int i;
	struct sentral_param_sensor_info sensor_info = { 0 };

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"%-33s,%4s,%6s,%6s,%4s,%8s,%6s,%6s\n", "SensorType",
			"Ver", "Power", "Range", "Res", "MaxRate", "FRes", "FMax");

	for (i = SST_FIRST; i < SST_MAX; i++) {
		rc = sentral_sensor_info_read(sentral, i, &sensor_info);
		if (rc) {
			LOGE(&sentral->client->dev,
					"error (%d) reading sensor info for sensor id: %u\n",
					rc, i);

			return rc;
		}

		if (!sensor_info.driver_id)
			continue;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				"%-28s (%2d),%4u,%6u,%6u,%4u,%8u,%6u,%6u\n",
				sentral_sensor_type_strings[i],
				i,
				sensor_info.driver_version,
				sensor_info.power,
				sensor_info.max_range,
				sensor_info.resolution,
				sensor_info.max_rate,
				sensor_info.fifo_reserved,
				sensor_info.fifo_max);
	}

	return count;
}

static DEVICE_ATTR(sensor_info, S_IRUGO, sentral_sysfs_sensor_info_show, NULL);

// sensor config

static ssize_t sentral_sysfs_sensor_config_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	ssize_t count = 0;
	int rc;
	int i;
	struct sentral_param_sensor_config sensor_config = { 0 };

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"%-33s,%6s,%11s,%12s,%13s\n", "SensorType", "Rate",
			"MaxLatency", "Sensitivity", "DynamicRange");

	for (i = SST_FIRST; i < SST_MAX; i++) {
		rc = sentral_sensor_config_read(sentral, i, &sensor_config);
		if (rc) {
			LOGE(&sentral->client->dev,
					"error (%d) reading sensor config for sensor id: %d\n",
					rc, i);

			return rc;
		}

		if (!sentral_sensor_type_strings[i])
			continue;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				"%-28s (%2d),%6u,%11u,%12u,%13u\n",
				sentral_sensor_type_strings[i],
				i,
				sensor_config.sample_rate,
				sensor_config.max_report_latency,
				sensor_config.change_sensitivity,
				sensor_config.dynamic_range);
	}

	return count;
}

static DEVICE_ATTR(sensor_config, S_IRUGO, sentral_sysfs_sensor_config_show,
		NULL);

static ssize_t sentral_sysfs_phys_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	struct sentral_param_phys_sensor_status_page phys;
	ssize_t count = 0;
	int rc;

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"%-15s, %5s, %5s, %5s, %10s\n", "Sensor", "Rate", "Range", "Int",
			"PowerMode");

	rc = sentral_parameter_read(sentral, SPP_SYS, SP_SYS_PHYS_SENSOR_STATUS,
			(void *)&phys, sizeof(phys));

	if (rc) {
		LOGE(&sentral->client->dev,
				"error (%d) reading physical sensor status\n", rc);
		return rc;
	}

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"%-15s, %5u, %5u, %5u, %10u\n", "Accelerometer",
			phys.accel.sample_rate, phys.accel.dynamic_range,
			phys.accel.flags.bits.int_enable, phys.accel.flags.bits.power_mode);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"%-15s, %5u, %5u, %5u, %10u\n", "Gyroscope",
			phys.gyro.sample_rate, phys.gyro.dynamic_range,
			phys.gyro.flags.bits.int_enable, phys.gyro.flags.bits.power_mode);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"%-15s, %5u, %5u, %5u, %10u\n", "Magnetometer",
			phys.mag.sample_rate, phys.mag.dynamic_range,
			phys.mag.flags.bits.int_enable, phys.mag.flags.bits.power_mode);

	return count;
}

static DEVICE_ATTR(phys_status, S_IRUGO, sentral_sysfs_phys_status_show, NULL);

// sensor status

static ssize_t sentral_sysfs_sensor_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	ssize_t count = 0;
	int rc;
	int i, j;
	struct sentral_param_sensor_status sensor_status[16];

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"%5s%10s%10s%10s%10s%10s%10s\n", "SID", "DataAvail", "I2CNACK",
			"DevIDErr", "TransErr", "DataLost", "PowerMode");

	for (i = 0; i < 2; i++) {
		rc = sentral_parameter_read(sentral, SPP_SYS,
				SP_SYS_SENSOR_STATUS_B0 + i, (void *)&sensor_status,
				sizeof(sensor_status));

		if (rc < 0) {
			LOGE(&sentral->client->dev,
					"error (%d) reading sensor status, bank: %d\n", rc, i);

			return rc;
		}
		for (j = 0; j < 16; j++) {
			count += scnprintf(buf + count, PAGE_SIZE - count, "%5d",
					i * 16 + j + 1);

			count += scnprintf(buf + count, PAGE_SIZE - count, "%10s",
					TFSTR(sensor_status[j].bits.data_available));

			count += scnprintf(buf + count, PAGE_SIZE - count, "%10s",
					TFSTR(sensor_status[j].bits.i2c_nack));

			count += scnprintf(buf + count, PAGE_SIZE - count, "%10s",
					TFSTR(sensor_status[j].bits.device_id_error));

			count += scnprintf(buf + count, PAGE_SIZE - count, "%10s",
					TFSTR(sensor_status[j].bits.transient_error));

			count += scnprintf(buf + count, PAGE_SIZE - count, "%10s",
					TFSTR(sensor_status[j].bits.data_lost));

			count += scnprintf(buf + count, PAGE_SIZE - count, "%10d",
					sensor_status[j].bits.power_mode);

			count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
		}
	}

	return count;
}

static DEVICE_ATTR(sensor_status, S_IRUGO, sentral_sysfs_sensor_status_show,
		NULL);

static ssize_t sentral_sysfs_touch_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	size_t count = 0;
	int rc;
	int i;
	struct sentral_htc_touch_status touch_status = { 0 };
	const char touch_status_strings[6][22] = {
		"Handshake Passed", "Handshake Failed", "Suspend Complete",
		"Resume Complete", "Touch Controller NACK", "Bsleepout",
	};

	rc = sentral_read_block(sentral, SR_TC_HS_CNT, (void *)&touch_status,
			sizeof(touch_status));

	if (rc < 0)
		return rc;

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"%-21s : %u\n", "Handshake Fail Count",
			touch_status.handshake_count);

	for (i = 0; i < ARRAY_SIZE(touch_status_strings); i++) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"%-21s : %s\n", touch_status_strings[i],
				TFSTR(touch_status.status & (1 << i)));
	}

	return count;
}

static DEVICE_ATTR(touch_status, S_IRUGO, sentral_sysfs_touch_status_show, NULL);

// algo standby

static ssize_t sentral_sysfs_algo_standby_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	u8 enable;
	int rc;

	rc = kstrtou8(buf, 10, &enable);
	if (rc) {
		LOGE(dev, "error (%d) parsing value\n", rc);
		return rc;
	}

	LOGD(&sentral->client->dev, "sysfs algo_standby: { enable: %u }\n", enable);

	return sentral_set_host_algo_standby_enable(sentral, !!enable);
}

static DEVICE_ATTR(algo_standby, S_IWUGO, NULL, sentral_sysfs_algo_standby_enable_store);

static ssize_t sentral_sysfs_ps_autok_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	int rc;
	u8 value;

	rc = sentral_config_ps_autok_get(sentral, &value);
	if (rc)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", value);
}

static ssize_t sentral_sysfs_ps_autok_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	int rc;
	u8 value;

	rc = kstrtou8(buf, 10, &value);
	if (rc) {
		LOGE(dev, "error (%d) parsing value\n", rc);
		return rc;
	}

	LOGD(&sentral->client->dev, "sysfs ps_autok: { value: %u }\n", value);

	return sentral_config_ps_autok_set(sentral, !!value);
}

static DEVICE_ATTR(ps_autok, S_IRUGO | S_IWUGO, sentral_sysfs_ps_autok_show,
		sentral_sysfs_ps_autok_store);

static ssize_t sentral_sysfs_ps_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	struct sentral_htc_ps_data ps_data = { 0 };
	int rc;

	rc = sentral_parameter_read(sentral, SPP_HTC, SP_HTC_PS_DATA,
			(void *)&ps_data, sizeof(ps_data));
	if (rc)
		return rc;

	return scnprintf(buf, PAGE_SIZE,
			"PS { flag_far: %u, adc: %u, min_adc: %u, autok_thd: %u, flag_pocket: %u }\n",
			ps_data.flag_far, ps_data.adc, ps_data.min_adc, ps_data.autok_thd,
			ps_data.flag_pocket);
}

static DEVICE_ATTR(ps_data, S_IRUGO, sentral_sysfs_ps_data_show, NULL);

// pm sensor enable

static ssize_t sentral_sysfs_pm_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	u8 enable;
	int rc;

	rc = kstrtou8(buf, 10, &enable);
	if (rc) {
		LOGE(dev, "error (%d) parsing value\n", rc);
		return rc;
	}

	mutex_lock(&sentral->lock);

	rc = sentral_pm_enable(sentral, !!enable);

	mutex_unlock(&sentral->lock);

	return count;
}

static DEVICE_ATTR(pm_enable, S_IWUGO, NULL, sentral_sysfs_pm_enable_store);

// reqest sensor self-test

static ssize_t sentral_sysfs_self_test_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	return sentral_set_host_self_test_request(sentral);
}

static DEVICE_ATTR(self_test, S_IWUGO, NULL, sentral_sysfs_self_test_store);

// pass-through mode enable

static ssize_t sentral_sysfs_pt_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	u8 enable;
	int rc;

	rc = kstrtou8(buf, 10, &enable);
	if (rc) {
		LOGE(dev, "error (%d) parsing value\n", rc);
		return rc;
	}

	return sentral_set_pt_enable(sentral, !!enable);
}

static DEVICE_ATTR(pt_enable, S_IWUGO, NULL, sentral_sysfs_pt_enable_store);

static ssize_t sentral_sysfs_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);

	queue_work(sentral->sentral_wq, &sentral->work_startup);

	return count;
}

static DEVICE_ATTR(reset, S_IWUGO, NULL, sentral_sysfs_reset_store);

// FIFO read

static ssize_t sentral_sysfs_fifo_read_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);

	queue_work(sentral->sentral_wq, &sentral->work_fifo_read);

	return count;
}

static DEVICE_ATTR(fifo_read, S_IWUGO, NULL, sentral_sysfs_fifo_read_store);

// debug registers

static ssize_t sentral_sysfs_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	u8 values[4];
	int rc;
	ssize_t count = 0;

	rc = sentral_read_block(sentral, SR_ERROR, (void *)&values, sizeof(values));
	if (rc < 0)
		return rc;

	// debug registers
	count += scnprintf(buf + count, PAGE_SIZE - count, "Debug Registers:\n");

	count += scnprintf(buf + count, PAGE_SIZE - count, "%-14s: (0x%02X) %s\n",
			"Error Register", values[0],
			sentral_debug_error_register_string[values[0]]);

	count += scnprintf(buf + count, PAGE_SIZE - count, "%-14s: (0x%02X) %s\n",
			"IRQ State", values[1],
			sentral_debug_irq_state_string[values[1]]);

	count += scnprintf(buf + count, PAGE_SIZE - count, "%-14s: (0x%02X) %s\n",
			"Debug State", values[3],
			sentral_debug_debug_state_string[values[3] & 0xF0]);

	count += scnprintf(buf + count, PAGE_SIZE - count, "%-14s: (0x%02X)\n",
			"Debug Value", values[2]);

	return count;
}

static DEVICE_ATTR(debug, S_IRUGO, sentral_sysfs_debug_show, NULL);

// debug htc_param

static ssize_t sentral_sysfs_debug_param_htc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	return sentral_parameter_page_dump(sentral, buf, PAGE_SIZE, SPP_HTC,
			SP_FL_HTC_FIRST, SP_FL_HTC_LAST);
}

static DEVICE_ATTR(debug_param_htc, S_IRUGO,
		sentral_sysfs_debug_param_htc_show, NULL);

// debug warm-start

static ssize_t sentral_sysfs_debug_param_warm_start_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	return sentral_parameter_page_dump(sentral, buf, PAGE_SIZE,
			SPP_ALGO_WARM_START, SP_FL_WARM_START_FIRST, SP_FL_WARM_START_LAST);
}

static DEVICE_ATTR(debug_param_warm_start, S_IRUGO,
		sentral_sysfs_debug_param_warm_start_show, NULL);

// debug algo

static ssize_t sentral_sysfs_debug_param_algo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	return sentral_parameter_page_dump(sentral, buf, PAGE_SIZE, SPP_ALGO_KNOBS,
			SP_FL_ALGO_FIRST, SP_FL_ALGO_LAST);
}

static DEVICE_ATTR(debug_param_algo, S_IRUGO,
		sentral_sysfs_debug_param_algo_show, NULL);

// easy access

static ssize_t sentral_sysfs_easyaccess_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	ssize_t count = 0;
	int i;
	int rc;
	int value = 0;

	mutex_lock(&sentral->lock);

	count += scnprintf(buf, PAGE_SIZE, "%2s, %6s\n", "#", "value");

	for (i = SP_HTC_EASY_ACC_FIRST; i <= SP_HTC_EASY_ACC_LAST; i++) {
		rc = sentral_htc_easyaccess_get(sentral, i, &value);
		if (rc < 0)
			continue;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				"%2d, %6d\n", i, value);
	}

	mutex_unlock(&sentral->lock);

	return count;
}

static ssize_t sentral_sysfs_easyaccess_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	unsigned int param;
	int value;
	int rc;

	if (2 != sscanf(buf, "%u %d", &param, &value))
		return -EINVAL;

	LOGD(&sentral->client->dev, "sysfs easyaccess: { param: %d, value: %u }\n",
			param, value);

	mutex_lock(&sentral->lock);

	rc = sentral_htc_easyaccess_set(sentral, param, value);
	if (rc)
		count = rc;

	mutex_unlock(&sentral->lock);

	return count;
}

static DEVICE_ATTR(easyaccess, S_IWUGO | S_IRUGO, sentral_sysfs_easyaccess_show,
		sentral_sysfs_easyaccess_store);

// smart cover

static ssize_t sentral_sysfs_smart_cover_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	int rc;
	u16 thrd;
	u8 covered;

	rc = sentral_smart_cover_covered_get(sentral, &covered);
	if (rc)
		return rc;

	rc = sentral_smart_cover_ps_thrd_get(sentral, &thrd);
	if (rc)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%04X %x\n", thrd, !!covered);
}

static ssize_t sentral_sysfs_smart_cover_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	u16 thrd;
	int rc;

	rc = kstrtou16(buf, 10, &thrd);
	if (rc) {
		LOGE(dev, "error (%d) parsing value\n", rc);
		return rc;
	}

	LOGD(&sentral->client->dev, "sysfs smart_cover: { thrd: %u }\n", thrd);

	mutex_lock(&sentral->lock);

	rc = sentral_smart_cover_ps_thrd_set(sentral, thrd);

	mutex_unlock(&sentral->lock);

	if (rc)
		return rc;

	return count;
}

static DEVICE_ATTR(smart_cover, S_IRUGO | S_IWUGO,
		sentral_sysfs_smart_cover_show, sentral_sysfs_smart_cover_store);

static ssize_t sentral_sysfs_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	struct sentral_hw_id hw_id = { 0 };
	char fw_version[SENTRAL_FW_VERSION_LEN] = { 0 };
	u32 fw_crc = 0;
	ssize_t count = 0;
	int rc;

	rc = sentral_hw_id_get(sentral, &hw_id);
	if (rc)
		return rc;

	count += scnprintf(buf + count, PAGE_SIZE - count, "HW ID: 0x%02X\n",
			hw_id.product_id);

	count += scnprintf(buf + count, PAGE_SIZE - count, "HW Version: 0x%02X\n",
			hw_id.revision_id);

	rc = sentral_fw_version_get(sentral, fw_version, sizeof(fw_version));
	if (rc)
		return rc;

	count += scnprintf(buf + count, PAGE_SIZE - count, "FW Version: %s\n",
			fw_version);

	rc = sentral_fw_crc_get(sentral, &fw_crc);
	if (rc)
		return rc;

	count += scnprintf(buf + count, PAGE_SIZE - count, "FW CRC: 0x%08X\n",
			fw_crc);

	count += scnprintf(buf + count, PAGE_SIZE - count,
		"Driver Version: %s.%s.%s.%s\n", SEN_DRV_PROJECT_ID,
		SEN_DRV_SUBPROJECT_ID, SEN_DRV_VERSION, SEN_DRV_BUILD);

	count += scnprintf(buf + count, PAGE_SIZE - count,
		"Driver Date: %s\n", SEN_DRV_DATE);

	return count;
}

static DEVICE_ATTR(version, S_IRUGO, sentral_sysfs_version_show, NULL);

// ANDROID sensor_poll_device_t method support

// activate

static ssize_t sentral_sysfs_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	ssize_t count = 0;
	unsigned long mcu_enabled_mask = 0;
	struct sentral_param_sensor_config sensor_config;
	int i;
	int rc;

	mutex_lock(&sentral->lock);

	for (i = SST_FIRST; i < SST_MAX; i++) {
		rc = sentral_sensor_config_read(sentral, i, &sensor_config);
		if (rc) {
			LOGE(&sentral->client->dev,
					"error (%d) reading sensor config for sensor id: %d\n",
					rc, i);

			goto exit;
		}

		if (sensor_config.sample_rate)
			set_bit(i, &mcu_enabled_mask);
	}

	count += scnprintf(buf, PAGE_SIZE, "0x%lX, mcu=0x%lX\n",
			sentral->enabled_mask, mcu_enabled_mask);

exit:
	mutex_unlock(&sentral->lock);
	return count;
}

#ifdef CONFIG_SYNC_TOUCH_STATUS

int touch_status(u8 reg, u8 status)
{
	int rc = -1;
	if (NULL == priv_sentral) {
		printk(KERN_ERR"%s, sentral is not register successfully\n", __FUNCTION__);
		return rc;
	}

	rc = sentral_write_byte(priv_sentral, reg, status);
       rc = sentral_read_byte(priv_sentral, reg);
       LOGI(&priv_sentral->client->dev, "[TP] %s: register is 0x%2x, status = 0x%2x, rc is 0x%2x\n", __func__, reg, status, rc);

       return 0;
}
int touch_get_status(u8 reg)
{
       int rc = -1;
	   if (NULL == priv_sentral) {
		   printk(KERN_ERR"%s, sentral is not register successfully\n", __FUNCTION__);
		   return rc;
	   }

       rc = sentral_read_byte(priv_sentral, reg);
       return rc;
}

#endif


static ssize_t sentral_sysfs_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	int rc;

	u32 id;
	u32 enable;
	u32 cw_id;
	bool flush_sensor = false;

	if (2 != sscanf(buf, "%u %u", &cw_id, &enable))
		return -EINVAL;

	LOGD(&sentral->client->dev, "sysfs enable: { id: %u, enable: %u }\n", cw_id,
			enable);

	id = sentral_sensor_id_cw_to_sentral(cw_id);
	if (!sentral_sensor_id_is_valid(id))
		return -EINVAL;

	switch (id) {
	case SST_TOUCH_GESTURE:
		rc = sentral_easyaccess_enable(sentral, !!enable);
		if (rc)
			goto exit_error;

		break;

	case SST_PROXIMITY:
//turn off psensor autok for MFG ROM, default value would be 1
#ifdef CONFIG_HUB_SENTRAL_MFG
		rc = sentral_config_ps_autok_set(sentral, 0);
		if (rc)
			goto exit_error;
#endif
		rc = sentral_sensor_enable_set(sentral, id, enable);
		if (rc)
			goto exit_error;

		rc = sentral_ps_polling_enable(sentral, false);
		if (rc)
			goto exit_error;

		break;

	default:
		flush_sensor = (test_bit(id, &sentral->enabled_mask) && !enable);
		rc = sentral_sensor_enable_set(sentral, id, enable);
		if (rc)
			goto exit_error;

		if (flush_sensor) {
			rc = sentral_fifo_flush(sentral, id);
			if (rc)
				goto exit_error;
		}
	}

	return count;

exit_error:
	if (enable && (rc == -EIO))
		sentral_crash_reset(sentral);

	return rc;
}

static DEVICE_ATTR(enable, S_IWUGO, sentral_sysfs_enable_show,
		sentral_sysfs_enable_store);

// set_delay

static ssize_t sentral_sysfs_delay_ms_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	int rc;

	u32 id;
	u32 delay_ms;
	u16 sample_rate = 0;
	u32 cw_id;

	if (2 != sscanf(buf, "%u %u", &cw_id, &delay_ms))
		return -EINVAL;

	LOGD(&sentral->client->dev, "sysfs delay_ms: { id: %u, delay_ms: %u}\n",
		cw_id, delay_ms);

	id = sentral_sensor_id_cw_to_sentral(cw_id);
	if (!sentral_sensor_id_is_valid(id))
		return -EINVAL;

	if (id == SST_TOUCH_GESTURE)
		return count;

	// convert millis to Hz
	if (delay_ms > 0) {
		delay_ms = MIN(1000, delay_ms);
		sample_rate = 1000 / delay_ms;
	}

	rc = sentral_sensor_batch_set(sentral, id, sample_rate, 0);
	if (rc)
		return rc;

	return count;
}

static DEVICE_ATTR(delay_ms, S_IWUGO, NULL, sentral_sysfs_delay_ms_store);

// batch

static ssize_t sentral_sysfs_batch_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%lld", sentral_get_boottime_ns() / 1000LL);
}

static ssize_t sentral_sysfs_batch_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	int rc;

	u32 id;
	u32 flags;
	u32 delay_ms;
	u32 timeout_ms;
	u32 cw_id;
	u16 sample_rate = 0;

	if (4 != sscanf(buf, "%u %u %u %u", &cw_id, &flags, &delay_ms, &timeout_ms))
		return -EINVAL;

	LOGD(&sentral->client->dev,
			"sysfs batch: { id: %u, flags: %u, delay_ms: %u, timeout_ms: %u }\n",
			cw_id, flags, delay_ms, timeout_ms);

	id = sentral_sensor_id_cw_to_sentral(cw_id);
	if (!sentral_sensor_id_is_valid(id))
		return -EINVAL;

	if (id == SST_TOUCH_GESTURE)
		return count;

	// convert millis to Hz
	if (delay_ms > 0) {
		delay_ms = MIN(1000, delay_ms);
		sample_rate = 1000 / delay_ms;
	}

	rc = sentral_sensor_batch_set(sentral, id, sample_rate, timeout_ms);
	if (rc)
		return rc;

	return count;
}

static DEVICE_ATTR(batch_enable, S_IWUGO, sentral_sysfs_batch_show, sentral_sysfs_batch_store);

// flush

static ssize_t sentral_sysfs_flush_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	// not supported
	return scnprintf(buf, PAGE_SIZE, "Queue counter = %d\n", 0);
}

static ssize_t sentral_sysfs_flush_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	u8 cw_id;
	u8 id;
	int rc;

	rc = kstrtou8(buf, 10, &cw_id);
	if (rc) {
		LOGE(dev, "error (%d) parsing value\n", rc);
		return rc;
	}

	LOGD(&sentral->client->dev, "sysfs flush: { id: %u }\n", cw_id);

	rc = sentral_sensor_id_cw_to_sentral(cw_id);
	if (rc < 0)
		return rc;

	id = rc;
	if (!sentral_sensor_id_is_valid(id))
		return -EINVAL;

	rc = sentral_fifo_flush(sentral, id);
	if (rc)
		return rc;

	return count;
}

static DEVICE_ATTR(flush, S_IWUGO, sentral_sysfs_flush_show, sentral_sysfs_flush_store);

static ssize_t set_g_sensor_user_offset(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	char *token;
	char *str_buf;
	char *running;
	long input_val[3] = {0};
	int rc, i;
	short temp_kvalue[3] = {0};
	sentral->user_calibration_flag = false;
	int user_cali_offset_buf[3] = {0};

	str_buf = kstrndup(buf, count, GFP_KERNEL);
	if (str_buf == NULL) {
		LOGE(&sentral->client->dev, "%s: cannot allocate buffer\n", __func__);
		return -1;
	}
	running = str_buf;

	for (i = 0; i < 3; i++) {
		token = strsep(&running, " ");
		if (token == NULL) {
			LOGE(&sentral->client->dev, "%s: token = NULL, i = %d\n", __func__, i);
			break;
		}

		rc = kstrtol(token, 10, &input_val[i]);
		if (rc) {
			LOGE(&sentral->client->dev, "%s: kstrtol fails, rc = %d, i = %d\n",
			  __func__, rc, i);
			kfree(str_buf);
			return rc;
		}
	}
	kfree(str_buf);

	LOGI(&sentral->client->dev, "%s: User Calibration(x, y, z) = (%ld, %ld, %ld)\n", __func__,
	  input_val[0], input_val[1], input_val[2]);

	temp_kvalue[0] = (short)input_val[0];
	temp_kvalue[1] = (short)input_val[1];
	temp_kvalue[2] = (short)input_val[2];

	LOGI(&sentral->client->dev, "%s: temp_kvalue(x, y, z) = (0x%x, 0x%x, 0x%x)\n", __func__,
	  temp_kvalue[0] & 0xFFFF, temp_kvalue[1] & 0xFFFF, temp_kvalue[2] & 0xFFFF);

	for (i = 0; i < 3; i++) {
		user_cali_offset_buf[i]=temp_kvalue[i]*16384/BMA253_USER_CALI_GRAVITY_EARTH;
		LOGI(&sentral->client->dev, "%s: user_cali_offset_buf[%d] = %d\n", __func__, i, user_cali_offset_buf[i]);
	}

	sentral->user_offset_buf[0] = user_cali_offset_buf[0];
	sentral->user_offset_buf[1] = user_cali_offset_buf[1];
	sentral->user_offset_buf[2] = user_cali_offset_buf[2];

	mutex_lock(&sentral->lock);
	rc = sentral_cal_info_accel_set(sentral,(s16)sentral->user_offset_buf[0], (s16)sentral->user_offset_buf[1],
			(s16)sentral->user_offset_buf[2]);
	mutex_unlock(&sentral->lock);

	sentral->user_calibration_flag = true;

	sentral->gs_kvalue = (0x67 << 24) | ((user_cali_offset_buf[2] >> 5) << 16) | ((user_cali_offset_buf[1] >> 5) << 8) | (user_cali_offset_buf[0] >> 5);

	return count;
}

static ssize_t g_sensor_user_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);

	if (sentral == NULL) {
		LOGE(&sentral->client->dev, "%s: sentral == NULL\n", __func__);
		return 0;
	}
	return sprintf(buf, "bma253->user_calibration_flag = %s, bma253->user_offset_buf[0] = %d, bma253->user_offset_buf[1] = %d, bma253->user_offset_buf[2] = %d\n",
			    (sentral->user_calibration_flag == true ? "true" : "false"), sentral->user_offset_buf[0], sentral->user_offset_buf[1], sentral->user_offset_buf[2]);
}

static DEVICE_ATTR(g_sensor_user_offset, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		g_sensor_user_offset_show, set_g_sensor_user_offset);


static DEVICE_ATTR(data_warm, S_IRUGO | S_IWUGO, sentral_sysfs_warm_data_show, sentral_sysfs_warm_data_store);
static DEVICE_ATTR(data_warm_enable, S_IRUGO | S_IWUGO, sentral_sysfs_warm_data_enable_show, sentral_sysfs_warm_data_enable_store);

static struct attribute *sentral_attributes[] = {
	&dev_attr_chip_control.attr,
	&dev_attr_host_status.attr,
	&dev_attr_chip_status.attr,
	&dev_attr_registers.attr,
	&dev_attr_algo_standby.attr,
	&dev_attr_self_test.attr,
	&dev_attr_pm_enable.attr,
	&dev_attr_pt_enable.attr,
	&dev_attr_reset.attr,
	&dev_attr_sensor_info.attr,
	&dev_attr_sensor_config.attr,
	&dev_attr_sensor_status.attr,
	&dev_attr_phys_status.attr,
	&dev_attr_enable.attr,
	&dev_attr_delay_ms.attr,
	&dev_attr_batch_enable.attr,
	&dev_attr_flush.attr,
	&dev_attr_fifo_read.attr,
	&dev_attr_easyaccess.attr,
	&dev_attr_smart_cover.attr,
	&dev_attr_debug.attr,
	&dev_attr_debug_param_htc.attr,
	&dev_attr_debug_param_warm_start.attr,
	&dev_attr_debug_param_algo.attr,
	&dev_attr_touch_status.attr,
	&dev_attr_ps_autok.attr,
	&dev_attr_ps_data.attr,
	&dev_attr_version.attr,
	&dev_attr_data_warm.attr,
    &dev_attr_data_warm_enable.attr,
	NULL
};

static const struct attribute_group sentral_attribute_group = {
	.attrs = sentral_attributes
};

// CW ATTRS

static ssize_t sentral_sysfs_cw_cal_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	int rc;
	u8 id;

	rc = kstrtou8(buf, 10, &id);
	if (rc) {
		LOGE(&sentral->client->dev, "error (%d) parsing value\n", rc);
		return rc;
	}

	LOGD(&sentral->client->dev, "sysfs calibrator_en: { id: %u }\n", id);

	rc = sentral_calibration_enable(sentral, id);
	if (rc)
		return rc;

	return count;
}

static ssize_t sentral_sysfs_cw_cal_status_accel_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "0x%x\n", test_bit(HTC_CAL_SENSOR_ACCEL,
			&sentral->cal_status_mask));
}

static ssize_t sentral_sysfs_cw_cal_status_mag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0x%x\n", 0);
}

static ssize_t sentral_sysfs_cw_cal_status_gyro_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0x%x\n", 0);
}

static ssize_t sentral_sysfs_cw_cal_data_accel_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);

//store 8bit data in NVRAM for g-sensor offset, so remore low 5bit
	return scnprintf(buf, PAGE_SIZE, "%d %d %d \n",
			sentral->cal_info.accel.offset_x >> 5,
			sentral->cal_info.accel.offset_y >> 5,
			sentral->cal_info.accel.offset_z >> 5);
}

static ssize_t sentral_sysfs_cw_cal_data_accel_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	int rc;
	int input[3];

	if (3 != sscanf(buf, "%d %d %d ", &input[0], &input[1], &input[2]))
		return -EINVAL;

	LOGD(&sentral->client->dev,
			"sysfs cal_data_accel: { x: %d, y: %d, z: %d }\n",
			input[0], input[1], input[2]);

//restore from NVRAM by shift left 5 bit
	input[0] = input[0] << 5;
	input[1] = input[1] << 5;
	input[2] = input[2] << 5;

	LOGI(&sentral->client->dev, "cal data accel: %d %d %d\n", (s16)input[0],
			(s16)input[1], (s16)input[2]);

	mutex_lock(&sentral->lock);

	rc = sentral_cal_info_accel_set(sentral,(s16)input[0], (s16)input[1],
			(s16)input[2]);

	mutex_unlock(&sentral->lock);

	if (rc)
		return rc;

	return count;
}

static ssize_t sentral_sysfs_cw_cal_data_accel_rl_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	// N/A
	return scnprintf(buf, PAGE_SIZE,
			"%d %d %d %d %d %d %d %d %d %d %d %d \n",
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

static ssize_t sentral_sysfs_cw_cal_data_ap_accel_rl_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	// N/A
	return 0;
}

static ssize_t sentral_sysfs_cw_cal_data_mag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	// N/A
	int data_mag[SEN_CW_MAG_MAX];
	int i = 0;
	int rc;
	int count=0;
	char page[PAGE_SIZE];

	memset(data_mag, 0, SEN_CW_MAG_MAX*sizeof(int));

    rc = sentral_sysfs_warm_data_show(dev, attr, page);
    if(rc<0)
    {
    
        return rc;
    }


    // convert cw calibrated data format to sentral calibrated data format.
    for(i=0; i<SEN_CW_MAG_MAX; i++)
    {
        //data_mag[i] = *(int*)warm_start_page_items[0].param_data
        memcpy(&data_mag[i], warm_start_page_items[i].param_data, sizeof(int));
        count += scnprintf(buf+count, PAGE_SIZE-count, "%d ", data_mag[i]);
        
    }
    
    count += scnprintf(buf+count, PAGE_SIZE-count, "\n");
	
	return count;
}

static ssize_t sentral_sysfs_cw_cal_data_mag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
    int rc;
    int i;
    int n=0;
	int data_mag[SEN_CW_MAG_MAX];
	char page[PAGE_SIZE];

	memset(page, 0, PAGE_SIZE);
    
	rc = sscanf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n"
	    , &data_mag[0],  &data_mag[1],  &data_mag[2],  &data_mag[3],  &data_mag[4]
		, &data_mag[5],  &data_mag[6],  &data_mag[7],  &data_mag[8],  &data_mag[9]
		, &data_mag[10], &data_mag[11], &data_mag[12], &data_mag[13], &data_mag[14]
		, &data_mag[15], &data_mag[16], &data_mag[17], &data_mag[18], &data_mag[19]
		, &data_mag[20], &data_mag[21], &data_mag[22], &data_mag[23], &data_mag[24]
        , &data_mag[25]
		);
    if(rc!=SEN_CW_MAG_MAX)
    {
    
        return -EINVAL;
    }
    
    for(i=0; i<SEN_CW_MAG_MAX; i++)
    {
        //data_mag[i] = *(int*)warm_start_page_items[0].param_data
        memcpy(warm_start_page_items[i].param_data, &data_mag[i], sizeof(int));
        //count += scnprintf(buf, PAGE_SIZE-count, "%d ", data_mag[i]);
    }

    n = scnprintf(page+n, PAGE_SIZE-n, SENTRAL_PAGE_ITEM_IO_FORMAT
            , SENTRAL_PAGE_UPDATE_FLAG_PARAM_NUMBER
            , SENTRAL_PAGE_UPDATE_FLAG_PARAM_SIZE  
            , SENTRAL_PAGE_UPDATE_FLAG_PARAM_TEST  
            , SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_0
            , SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_1
            , SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_2
            , SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_3
            , SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_4
            , SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_5
            , SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_6
            , SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_7
    );
    
    rc = sentral_sysfs_warm_data_store(dev, attr, page, n);
    if (rc<0)
    {
        return rc;
    }
	
	return count;
}

static ssize_t sentral_sysfs_cw_cal_data_gyro_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	// N/A
	return scnprintf(buf, PAGE_SIZE, "%d %d %d \n", 0, 0, 0);
}

static ssize_t sentral_sysfs_cw_cal_data_gyro_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	// N/A
	return count;
}

static ssize_t sentral_sysfs_cw_cal_data_light_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%x %x %x %x\n",
			SENTRAL_CAL_LIGHT_HEADER_MSB, SENTRAL_CAL_LIGHT_HEADER_LSB,
			sentral->cal_info.als.kadc & 0xFF,
			(sentral->cal_info.als.kadc >> 8) & 0xFF);
}

static ssize_t sentral_sysfs_cw_cal_data_light_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	int rc;
	u16 gadc;
	u16 kadc;

	rc = kstrtou16(buf, 16, &kadc);
	if (rc) {
		LOGE(&sentral->client->dev, "error (%d) parsing value\n", rc);
		return rc;
	}

	LOGD(&sentral->client->dev, "sysfs cal_data_light: { kadc: %u }\n", kadc);

	mutex_lock(&sentral->lock);

	gadc = (u16)sentral->platform_data.als_goldh << 8
			| (u16)sentral->platform_data.als_goldl;
	LOGI(&sentral->client->dev, "cal data light: gadc 0x%x, kadc 0x%x\n", gadc, kadc);

	rc = sentral_cal_info_als_set(sentral, gadc, kadc);

	mutex_unlock(&sentral->lock);

	if (rc)
		return rc;

	return count;
}

static ssize_t sentral_sysfs_cw_cal_data_prox_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	u16 value;
	int rc;

	rc = sentral_parameter_read(sentral, SPP_HTC, SP_HTC_PS_CAL_OUT,
			(void *)&value, sizeof(value));
	if (rc)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%x %x\n", (u8)(value & 0xFF),
			(u8)((value >> 8) & 0xFF));
}

static ssize_t sentral_sysfs_cw_cal_data_prox_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	int rc;
	u32 ps_thrd_l;
	u32 ps_canc;

	if (2 != sscanf(buf, "%x %x\n", &ps_canc, &ps_thrd_l))
		return -EINVAL;

	LOGD(&sentral->client->dev,
			"sysfs cal_data_prox: { ps_canc: %u, ps_thrd_l: %u }\n",
			ps_canc, ps_thrd_l);

	LOGI(&sentral->client->dev, "cal data proximity: threshold 0x%x, canc 0x%x\n", ps_thrd_l, ps_canc);

	mutex_lock(&sentral->lock);
	rc = sentral_cal_info_ps_set(sentral, 3, ps_thrd_l, ps_canc);
	mutex_unlock(&sentral->lock);

	if (rc)
		return rc;

	return count;
}

static ssize_t sentral_sysfs_cw_cal_data_baro_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	// N/A
	return scnprintf(buf, PAGE_SIZE, "%d %d %d %d \n", 0, 0, 0, 0);
}

static ssize_t sentral_sysfs_cw_cal_data_baro_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	// N/A
	return count;
}

static ssize_t sentral_sysfs_cw_gesture_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	u8 value;
	int rc;

	rc = sentral_parameter_read(sentral, SPP_HTC, SP_HTC_GESTURE_MASK,
			(void *)&value, sizeof(value));

	if (rc)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "0x%08X\n", value);
}

static ssize_t sentral_sysfs_cw_gesture_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	int rc;
	u16 value;
	u8 mask = 0;

	rc = kstrtou16(buf, 16, &value);
	if (rc) {
		LOGE(&sentral->client->dev, "error (%d) parsing value\n", rc);
		return rc;
	}
	LOGE(&sentral->client->dev, "%s, (0x%4x)\n", __FUNCTION__, value);

	if (value & (1 << HTC_GESTURE_MOTION_TYPE_SWIPE_UP))
		mask |= HTC_GESTURE_FLAG_SWIPE_UP;

	if (value & (1 << HTC_GESTURE_MOTION_TYPE_SWIPE_DOWN))
		mask |= HTC_GESTURE_FLAG_SWIPE_DOWN;

	if (value & (1 << HTC_GESTURE_MOTION_TYPE_SWIPE_LEFT))
		mask |= HTC_GESTURE_FLAG_SWIPE_LEFT;

	if (value & (1 << HTC_GESTURE_MOTION_TYPE_SWIPE_RIGHT))
		mask |= HTC_GESTURE_FLAG_SWIPE_RIGHT;

	if (value & (1 << HTC_GESTURE_MOTION_TYPE_LAUNCH_CAMERA))
		mask |= HTC_GESTURE_FLAG_VOLUME;

	if (value & (1 << HTC_GESTURE_MOTION_TYPE_DOUBLE_TAP))
		mask |= HTC_GESTURE_FLAG_DTAP;

	mutex_lock(&sentral->lock);

	rc = sentral_parameter_write(sentral, SPP_HTC, SP_HTC_GESTURE_MASK,
			(void *)&mask, sizeof(mask));

	if (rc < 0)
		count = rc;

	mutex_unlock(&sentral->lock);

	return count;
}

static ssize_t sentral_sysfs_cw_data_baro_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	// N/A
	return scnprintf(buf, PAGE_SIZE, "%x %x %x %x\n", 0, 0, 0, 0);
}

static ssize_t sentral_sysfs_cw_data_prox_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	int rc = 0;

	// zero last sample
	memset(&sentral->data_prox_last, 0, sizeof(sentral->data_prox_last));

	// enable PS polling
	rc = sentral_ps_polling_enable(sentral, true);
	if (rc)
		return rc;

	// set rate to 200 ms
	rc = sentral_sensor_batch_set(sentral, SST_PROXIMITY, 5, 0);

	set_bit(SST_PROXIMITY, &sentral->wait_one_shot_mask);
	wait_event_interruptible(sentral->wq_one_shot, test_bit(SST_PROXIMITY, &sentral->data_ready_mask));
	clear_bit(SST_PROXIMITY, &sentral->wait_one_shot_mask);
	clear_bit(SST_PROXIMITY, &sentral->data_ready_mask);

	rc = sentral_ps_polling_enable(sentral, false);
	if (rc)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%x %x \n", sentral->data_prox_last.data.flag_far,
			sentral->data_prox_last.data.value);
}

static ssize_t sentral_sysfs_cw_data_prox_polling_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	int rc = 0;

	// zero last sample
	memset(&sentral->data_prox_last, 0, sizeof(sentral->data_prox_last));

	// enable PS polling
	rc = sentral_ps_polling_enable(sentral, true);
	if (rc)
		return rc;

	// set rate to 100 ms
	rc = sentral_sensor_batch_set(sentral, SST_PROXIMITY, 10, 0);

	set_bit(SST_PROXIMITY, &sentral->wait_one_shot_mask);
	wait_event_interruptible(sentral->wq_one_shot, test_bit(SST_PROXIMITY, &sentral->data_ready_mask));
	clear_bit(SST_PROXIMITY, &sentral->wait_one_shot_mask);
	clear_bit(SST_PROXIMITY, &sentral->data_ready_mask);

	rc = sentral_ps_polling_enable(sentral, false);
	if (rc)
		return rc;

	return snprintf(buf, PAGE_SIZE, "ADC[0x%02X] status is %d\n",
		 sentral->data_prox_last.data.value, sentral->data_prox_last.data.flag_far);
}

static ssize_t sentral_sysfs_cw_data_light_polling_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	struct sentral_htc_als_setting als_setting = {{ 0 }};
	int rc = 0;

	// LS not enabled
	if (!test_bit(SST_LIGHT, &sentral->enabled_mask))
		return -EINVAL;

	// LS rate not set
	if (!sentral->sensor_config[SST_LIGHT].sample_rate)
		return -EINVAL;

	// zero last sample
	memset(&sentral->data_light_last, 0, sizeof(sentral->data_light_last));

	// save current als mode
	memcpy(&als_setting, &sentral->als_setting, sizeof(als_setting));

	// enable fast polling mode
	rc = sentral_config_als_mode_set(sentral, true,
			sentral->platform_data.als_level_count, true);
	if (rc)
		return rc;

	set_bit(SST_LIGHT, &sentral->wait_one_shot_mask);
	wait_event_interruptible(sentral->wq_one_shot, test_bit(SST_LIGHT, &sentral->data_ready_mask));
	clear_bit(SST_LIGHT, &sentral->wait_one_shot_mask);
	clear_bit(SST_LIGHT, &sentral->data_ready_mask);

	// restore previous mode
	rc = sentral_config_als_mode_set(sentral,
			!!sentral->platform_data.als_polling,
			sentral->platform_data.als_level_count, false);
	if (rc)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "ADC[0x%04X] => level %u\n",
			sentral->data_light_last.value, sentral->data_light_last.level);

}

static ssize_t sentral_sysfs_cw_als_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	struct sentral_htc_als_setting als_setting;
	int rc;

	rc = sentral_config_als_mode_get(sentral, &als_setting);
	if (rc)
		return rc;

	return snprintf(buf, PAGE_SIZE, "%u\n", als_setting.byte);
}

static ssize_t sentral_sysfs_cw_data_hub_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	u32 addr;
	u32 length;
	u8 regs[SR_MAX];
	int rc;
	int i;

	if (2 != sscanf(buf, "%u,%u", &addr, &length))
		return -EINVAL;

	LOGD(&sentral->client->dev,
			"sysfs sensor_hub_rdata: { addr: %u, length: %u }\n",
			addr, length);

	// invalid start register
	if ((addr > SR_MAX) || (addr < SR_FIRST))
		return -EINVAL;

	// invalid end register
	if ((addr + length) > SR_MAX)
		return -EINVAL;

	rc = sentral_read_block(sentral, (u8)addr, (void *)&regs, length);
	if (rc < 0)
		return rc;

	for (i = 0; i < length; i++)
		LOGD(&sentral->client->dev,
				"read mcu reg_addr = 0x%x, reg[%u] = 0x%x\n", addr + i, i,
				regs[i]);

	return count;
}

static ssize_t sentral_sysfs_cw_ps_canc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE,
			"P-sensor NOT calibrated,INTE_PS1_CANC = (0x%04X), INTE_PS2_CANC = (0x%04X)\n",
			sentral->cal_info.ps.ps_canc,
			sentral->cal_info.ps.ps_thrd_l);
}

static ssize_t sentral_sysfs_cw_ps_canc_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t sentral_sysfs_cw_als_kadc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "gadc = 0x%x, kadc = 0x%x",
			sentral->cal_info.als.gadc,
			sentral->cal_info.als.kadc);
}

static ssize_t sentral_sysfs_cw_firmware_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
		u8 firmware_version[6] = {0};
	
		firmware_version[0] = 3;		//255
		firmware_version[1] = 65;		//65
		firmware_version[2] = 30;		//29	
		firmware_version[3] = 31;		//27
		firmware_version[4] = 1;		// 1
		firmware_version[5] = 2;		// 3	
	
		return scnprintf(buf, PAGE_SIZE,
				 "Firmware Architecture version %u, "
				 "Sense version %u, Cywee lib version %u,"
				 " Water number %u"
				 ", Active Engine %u, Project Mapping %u\n",
				 firmware_version[0], firmware_version[1],
				 firmware_version[2], firmware_version[3],
				 firmware_version[4], firmware_version[5]);
}


static ssize_t sentral_sysfs_cw_firmware_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 firmware_version[6] = {0};
	u8 firmware_info[6] = {4, 2, 18, 14, 0, 1};

	firmware_version[0] = 3;
	firmware_version[1] = 65;
	firmware_version[2] = 30;
	firmware_version[3] = 31;
	firmware_version[4] = 1;
	firmware_version[5] = 2;


	return scnprintf(buf, PAGE_SIZE,
					 "%03u.%03u.%03u.%03u.%03u.%03u: "
					 "Jenkins build number %u, "
					 "Build time(hh:mm) %02u:%02u, "
					 "CW branch %u, CW mcu type %u\n",
					 firmware_version[0], firmware_version[1],
					 firmware_version[2], firmware_version[3],
					 firmware_version[4], firmware_version[5],
					 (firmware_info[0] << 8) | firmware_info[1],
					 firmware_info[2], firmware_info[3],
					 firmware_info[4], firmware_info[5]);
}

static ssize_t sentral_sysfs_cw_led_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t sentral_sysfs_cw_mcu_crash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	u8 test_id;
	int rc;

	rc = kstrtou8(buf, 10, &test_id);
	if (rc) {
		LOGE(dev, "error (%d) parsing value\n", rc);
		return rc;
	}

	LOGD(&sentral->client->dev, "sysfs trigger_crash: { test_id: %u }\n",
			test_id);

	if ((test_id < SEN_CRASH_TEST_FIRST) || (test_id >= SEN_CRASH_TEST_MAX))
		return -EINVAL;

	rc = sentral_crash_test(sentral, test_id);
	if (rc)
		return rc;

	return count;
}

static ssize_t sentral_sysfs_cw_mcu_wakeup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	u8 enable;
	int rc;

	rc = kstrtou8(buf, 10, &enable);
	if (rc) {
		LOGE(dev, "error (%d) parsing value\n", rc);
		return rc;
	}

	LOGD(&sentral->client->dev, "sysfs mcu_wakeup: { enable: %u }\n", enable);

	mutex_lock(&sentral->lock);

	rc = sentral_set_host_ap_suspend_enable(sentral, !!enable);

	mutex_unlock(&sentral->lock);

	if (rc < 0)
		return rc;

	return count;
}

static ssize_t sentral_sysfs_cw_mcu_log_mask_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t sentral_sysfs_cw_mcu_log_mask_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t sentral_sysfs_cw_mcu_log_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t sentral_sysfs_cw_mcu_log_level_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}



static bool is_htc_dbg_flag_set(void)
{
    //check if SSD ramdump flag is set
    if (get_radio_flag() & 0x8) {
        I("%s: true\n", __func__);
        return true;
    }
    else {
        I("%s: false\n", __func__);
        return false;
    }
}

static ssize_t sentral_sysfs_cw_debug_flag_show(struct device *dev,
                                 struct device_attribute *attr,
                                 char *buf)
{
    int dbg_flag_set;

    if (is_htc_dbg_flag_set())
        dbg_flag_set = 1;
    else
        dbg_flag_set = 0;

    return snprintf(buf, PAGE_SIZE, "%d\n", dbg_flag_set);
}

static ssize_t sentral_sysfs_cw_sensor_placement_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t sentral_sysfs_cw_vibrate_ms_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int error;
    u8 data;
    long data_temp = 0;
	struct sentral_device *sentral = dev_get_drvdata(dev);

    error = kstrtol(buf, 10, &data_temp);
    if (error) {
        LOGE(&sentral->client->dev,"%s: kstrtol fails, error = %d\n", __func__, error);
        return error;
    }

    data = data_temp;

    if (!priv_sentral) {
        LOGE(&sentral->client->dev,"%s: probe not completed\n", __func__);
        return -1;
    }

    LOGI(&sentral->client->dev,"%s: vibrate_ms%d\n", __func__, data);
    priv_sentral->vibrate_ms = data;

    return count;
}




static ssize_t sentral_sysfs_cw_mcu_crash_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%u\n", sentral->crash_count);
}

#ifdef CONFIG_MTK

#define I2C_FLAG_WRITE	0
#define I2C_FLAG_READ	1

static int mtk_i2c_master_operate(struct i2c_client *client, const char *buf, int count, int i2c_flag)
{
	int res = 0;

	switch(i2c_flag){
	case I2C_FLAG_WRITE:
	client->addr &=I2C_MASK_FLAG;
	client->timing = 400;
	res = i2c_master_send(client, buf, count);
	client->addr &=I2C_MASK_FLAG;
	break;

	case I2C_FLAG_READ:
	client->addr &=I2C_MASK_FLAG;
	client->addr|=I2C_WR_FLAG;
	client->addr|=I2C_RS_FLAG;
	client->timing = 400;
	res = i2c_master_send(client, buf, count);
	client->addr &=I2C_MASK_FLAG;
	break;
	default:
	D("[PS][cm36686] %s i2c_flag command not support!\n", __func__);
	break;
	}
	if(res < 0)
	{
		goto EXIT_ERR;
	}

	return 0;

	EXIT_ERR:
	D("[PS][cm36686 %s fail!\n", __func__);

	return res;
}

static int _cm36686_I2C_Read2(uint16_t slaveAddr,
		uint8_t cmd, uint8_t *pdata, int length)
{
	char buffer[3] = {0};
	int ret = 0, i;

	if (pdata == NULL)
		return -EFAULT;

	if (length > 2) {
		pr_err(
				"[PS_ERR][cm36686 error]%s: length %d> 2: \n", __func__, length);
		return ret;
	}
	buffer[0] = cmd;

	ret = mtk_i2c_master_operate(priv_sentral->client, buffer, 0x201, I2C_FLAG_READ);

	if (ret < 0) {
		pr_err(
				"[PS_ERR][cm36686 error]%s: I2C_RxData fail, slave addr: 0x%x\n", __func__, slaveAddr);
		return ret;
	}


	for (i = 0; i < length; i++) {
		*(pdata+i) = buffer[i];
	}
#if 0
	/* Debug use */
	printk(KERN_DEBUG "[cm36686] %s: I2C_RxData[0x%x] = 0x%x\n",
			__func__, slaveAddr, buffer);
#endif
	return ret;
}


static ssize_t sentral_sysfs_testing_ptmode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	char cmd[3];
	int ret = 0 ;
	mutex_lock(&sentral->lock);

	ret = sentral_set_pt_enable(sentral, 1);

	priv_sentral->client->addr = 0x60;
	mdelay(20);
	ret = _cm36686_I2C_Read2(0x60, 0x0c, cmd, 2);
	LOGI(&sentral->client->dev, "p_sensor id is 0x%2x, 0x%2x\n", cmd[0], cmd[1]);

	ret = _cm36686_I2C_Read2(0x60, 0x03, cmd, 2);
	LOGI(&sentral->client->dev, "p_sensor CONFIG is  low 0x%2x, high 0x%2x\n", cmd[0], cmd[1]);

	ret = _cm36686_I2C_Read2(0x60, 0x05, cmd, 2);
	LOGI(&sentral->client->dev, "p_sensor canc is 0x%2x, 0x%2x\n", cmd[0], cmd[1]);
	ret = _cm36686_I2C_Read2(0x60, 0x06, cmd, 2);
	LOGI(&sentral->client->dev, "p_sensor threshold low is 0x%2x, 0x%2x\n", cmd[0], cmd[1]);
	ret = _cm36686_I2C_Read2(0x60, 0x07, cmd, 2);
	LOGI(&sentral->client->dev, "p_sensor threshold high is 0x%2x, 0x%2x\n", cmd[0], cmd[1]);

	ret = _cm36686_I2C_Read2(0x60, 0x08, cmd, 2);
	LOGI(&sentral->client->dev, "p_sensor raw data is 0x%2x, 0x%2x\n", cmd[0], cmd[1]);

	ret = _cm36686_I2C_Read2(0x60, 0x0B, cmd, 2);
	LOGI(&sentral->client->dev, "p_sensor int flag is 0x%2x, 0x%2x\n", cmd[0], cmd[1]);


	priv_sentral->client->addr = 0x28;

	mdelay(20);
	ret = sentral_set_pt_enable(sentral, 0);

	mutex_unlock(&sentral->lock);

	return 0;
}

#endif /* CONFIG_MTK */


#ifdef NVRAM_DATA_ONESHOT


static int cwmcu_set_sensor_kvalue(struct sentral_device *sentral)
{
	/* Write single Byte because firmware can't write multi bytes now */

	u8 *gs_data = (u8 *)&sentral->gs_kvalue; /* gs_kvalue is u32 */
    u8 als_goldh = (sentral->platform_data.als_goldh == 0) ? 0x0A : (sentral->platform_data.als_goldh);
    u8 als_goldl = (sentral->platform_data.als_goldl == 0) ? 0x38 : (sentral->platform_data.als_goldl);
	u8 als_datal = 0, als_datah = 0;
	u8 ps_cancl = 0, ps_canch = 0, ps_thdl = 0, ps_thdh = 0;
	u16 gadc;
	u16 ps_thrd_l;
	u16 ps_canc;
	int rc;

	sentral->gs_calibrated = 0;
	sentral->gy_calibrated = 0;
	sentral->ls_calibrated = 0;
	if (gs_data[3] == 0x67) {
		__be32 be32_gs_data = cpu_to_be32(sentral->gs_kvalue);
		gs_data = (u8 *)&be32_gs_data;
		LOGI(&sentral->client->dev, "set gsensor kvalue\n");

		// g-sensor, restore from NVRAM by shift left 5 bit
		mutex_lock(&sentral->lock);
		rc = sentral_cal_info_accel_set(sentral,((s16)gs_data[0])<<5, ((s16)gs_data[1])<<5,
				((s16)gs_data[2])<<5);
		mutex_unlock(&sentral->lock);



		sentral->gs_calibrated = 1;
		LOGI(&sentral->client->dev, "Set g-sensor kvalue (x, y, z) = (0x%x, 0x%x, 0x%x)\n",
			gs_data[0], gs_data[1], gs_data[2]);
	}



	if ((sentral->als_kvalue & 0x6DA50000) == 0x6DA50000) {

		mutex_lock(&sentral->lock);	
		gadc = als_goldh << 8|als_goldl;
//		als_datal = (sentral->als_kvalue) & 0xFF;
//		als_datah = (sentral->als_kvalue >>  8) & 0xFF;
		rc = sentral_cal_info_als_set(sentral, gadc, sentral->als_kvalue);		
		mutex_unlock(&sentral->lock);

		sentral->ls_calibrated = 1;
		LOGI(&sentral->client->dev, "Set light-sensor kvalue is %x %x, gvalue is %x %x\n"
				, als_datah, als_datal, als_goldh, als_goldl);
	}
#if 0	
else {
		CWMCU_i2c_write_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,
				&als_goldl, 1);
		CWMCU_i2c_write_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,
				&als_goldh, 1);
		als_datal = als_goldl;
		CWMCU_i2c_write_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,
				&als_datal, 1);
		als_datah = als_goldh;
		CWMCU_i2c_write_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,
				&als_datah, 1);
		mcu_data->ls_calibrated = 0;
		I("Set light-sensor kvalue is %x %x, gvalue is %x %x\n"
				, als_datah, als_datal, als_goldh, als_goldl);
	}

	for (i = 0; i<10; i++) {
                als_levell = *(mcu_data->als_levels + i) & 0xFF;
                als_levelh = (*(mcu_data->als_levels + i) >> 8) & 0xFF;
                CWMCU_i2c_write_power(mcu_data,
                                CW_I2C_REG_SENSORS_SET_LEVEL_LIGHT,
                                &als_levell, 1);
                CWMCU_i2c_write_power(mcu_data,
                                CW_I2C_REG_SENSORS_SET_LEVEL_LIGHT,
                                &als_levelh, 1);
               I("Set light-sensor level is %d\n", ((als_levelh << 8) | als_levell));
	}
#endif

	if((sentral->ps_kheader & 0x50530000) == 0x50530000) {
		if (((sentral->ps_kvalue >> 16) & 0xFFFF) == 0xFFFF) {
			ps_cancl = (sentral->ps_kvalue) & 0xFF;
            ps_canch = 0x00;
			ps_canc = ps_canch << 8 | ps_cancl;

            if( (sentral->ps_kvalue & 0xFF00) != 0X0000) {
                ps_thdl = (sentral->ps_kvalue >> 8) & 0xFF;
                ps_thdh = 0x00;
            }
            else {
                ps_thdl = sentral->platform_data.ps_thd_fixed & 0xFF;
                ps_thdh = 0x00;
				
            }
			ps_thrd_l = ps_thdh << 8| ps_thdl;

						mutex_lock(&sentral->lock);
						rc = sentral_cal_info_ps_set(sentral, 3, ps_thrd_l, ps_canc);
						mutex_unlock(&sentral->lock);

						
                        LOGI(&sentral->client->dev, "set proximity-sensor kvalue is %x %x %x %x\n", ps_cancl, ps_canch, ps_thdl, ps_thdh);
                } else {
                        ps_cancl = sentral->ps_kvalue & 0xFF;
                        ps_canch = ((sentral->ps_kvalue) & 0xFF00) >> 8;
						ps_canc = ps_canch << 8 | ps_cancl;						
                       	if((sentral->ps_kvalue & 0xFFFF0000) != 0X00000000) {
                       	        ps_thdl = (sentral->ps_kvalue >> 16) & 0xFF;
                       	        ps_thdh = (sentral->ps_kvalue >> 24) & 0xFF;;
                       	}
                       	else {
                       	        ps_thdl = sentral->platform_data.ps_thd_fixed & 0xFF;
                       	        ps_thdh = (sentral->platform_data.ps_thd_fixed >> 8) & 0xFF;
                       	}
						ps_thrd_l = ps_thdh << 8| ps_thdl;

                        LOGI(&sentral->client->dev, "set proximity-sensor kvalue is %x %x %x %x\n",
                               ps_cancl, ps_canch, ps_thdl, ps_thdh);
                }
                sentral->ps_calibrated = 1;
        }
#if 0
	if (mcu_data->bs_kheader == 0x67) {
		__be32 be32_bs_data = cpu_to_be32(mcu_data->bs_kvalue);

		CWMCU_i2c_write(mcu_data,
			CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PRESSURE,
			&be32_bs_data, PRESSURE_CALIBRATOR_LEN, 1);
		mcu_data->bs_calibrated = 1;
		D(
		  "Set barometer kvalue (a, b, c, d) = "
		  "(0x%x, 0x%x, 0x%x, 0x%x)\n",
		  bs_data[3], bs_data[2], bs_data[1], bs_data[0]);
	}
#endif
	LOGI(&sentral->client->dev, "Sensor calibration matrix is (gs %u  ps %u ls %u)\n",
		sentral->gs_calibrated, sentral->ps_calibrated,
		sentral->ls_calibrated);

	return 0;
}

static void cwmcu_one_shot(struct work_struct *work)
{
	struct sentral_device *sentral = container_of(work, struct sentral_device,
			one_shot_work);
	cwmcu_set_sensor_kvalue(sentral);
}


static ssize_t get_nvram_cali_data(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "GS[0x%08X], GYRO[0x%08X], "
			 	"LS[0x%08X], PS[0x%08X 0x%08X]\n",
				sentral->gs_kvalue, sentral->gy_kvalue,
				sentral->als_kvalue, sentral->ps_kheader,
				sentral->ps_kvalue);
}

static ssize_t set_nvram_cali_data(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t count)
{
		struct sentral_device *sentral = dev_get_drvdata(dev);
       int sensors_id;
       u32 cali_data_h = 0;
       u32 cali_data_l = 0;
       sscanf(buf, "%d %x %x\n", &sensors_id, &cali_data_h, &cali_data_l);

       LOGI(&sentral->client->dev, "%s: Target sensor ID = %d\n", __func__, sensors_id);
       LOGI(&sentral->client->dev, "%s: cali_data(high, low) = (0x%08X, 0x%08X)\n", __func__,
                                               cali_data_h, cali_data_l);

       switch (sensors_id) {
       case CW_ACCELERATION:
		sentral->gs_kvalue = cali_data_h;
		break;
       case CW_GYRO:
		sentral->gy_kvalue = cali_data_h;
		break;
       case CW_LIGHT:
		sentral->als_kvalue = cali_data_h;
		break;
       case CW_PROXIMITY:
		sentral->ps_kheader = cali_data_h;
		sentral->ps_kvalue = cali_data_l;
		break;
       default:
		break;
       }

       return count;
}
#endif


static struct device_attribute sentral_cw_attrs[] = {
	__ATTR(calibrator_en, 0220, NULL, sentral_sysfs_cw_cal_enable_store),
	__ATTR(calibrator_status_acc, 0440, sentral_sysfs_cw_cal_status_accel_show,
			NULL),
	__ATTR(calibrator_status_mag, 0440, sentral_sysfs_cw_cal_status_mag_show,
			NULL),
	__ATTR(calibrator_status_gyro, 0440, sentral_sysfs_cw_cal_status_gyro_show,
			NULL),
	__ATTR(calibrator_data_acc, 0666, sentral_sysfs_cw_cal_data_accel_show,
			sentral_sysfs_cw_cal_data_accel_store),
	__ATTR(calibrator_data_acc_rl, 0440,
			sentral_sysfs_cw_cal_data_accel_rl_show, NULL),
	__ATTR(ap_calibrator_data_acc_rl, 0440,
			sentral_sysfs_cw_cal_data_ap_accel_rl_show, NULL),
	__ATTR(calibrator_data_mag, 0666, sentral_sysfs_cw_cal_data_mag_show,
			sentral_sysfs_cw_cal_data_mag_store),
	__ATTR(calibrator_data_gyro, 0666, sentral_sysfs_cw_cal_data_gyro_show,
			sentral_sysfs_cw_cal_data_gyro_store),
	__ATTR(calibrator_data_light, 0666, sentral_sysfs_cw_cal_data_light_show,
			sentral_sysfs_cw_cal_data_light_store),
	__ATTR(calibrator_data_proximity, 0666, sentral_sysfs_cw_cal_data_prox_show,
			sentral_sysfs_cw_cal_data_prox_store),
	__ATTR(calibrator_data_barometer, 0666, sentral_sysfs_cw_cal_data_baro_show,
			sentral_sysfs_cw_cal_data_baro_store),
	__ATTR(gesture_motion, 0660, sentral_sysfs_cw_gesture_show,
			sentral_sysfs_cw_gesture_store),
	__ATTR(data_barometer, 0440, sentral_sysfs_cw_data_baro_show, NULL),
	__ATTR(data_proximity, 0440, sentral_sysfs_cw_data_prox_show, NULL),
	__ATTR(data_proximity_polling, 0440,
			sentral_sysfs_cw_data_prox_polling_show, NULL),
	__ATTR(data_light_polling, 0440, sentral_sysfs_cw_data_light_polling_show,
			NULL),
	__ATTR(ls_mechanism, 0444, sentral_sysfs_cw_als_mode_show, NULL),
	__ATTR(sensor_hub_rdata, 0220, NULL, sentral_sysfs_cw_data_hub_store),
	__ATTR(ps_canc, 0666, sentral_sysfs_cw_ps_canc_show,
			sentral_sysfs_cw_ps_canc_store),
	__ATTR(data_light_kadc, 0440, sentral_sysfs_cw_als_kadc_show, NULL),
	__ATTR(firmware_version, 0440, sentral_sysfs_cw_firmware_version_show,
			NULL),
	__ATTR(firmware_info, 0440, sentral_sysfs_cw_firmware_info_show, NULL),
	__ATTR(led_en, 0220, NULL, sentral_sysfs_cw_led_enable_store),
	__ATTR(trigger_crash, 0220, NULL, sentral_sysfs_cw_mcu_crash_store),
	__ATTR(mcu_wakeup, 0220, NULL, sentral_sysfs_cw_mcu_wakeup_store),
	__ATTR(mcu_log_mask, 0660, sentral_sysfs_cw_mcu_log_mask_show,
			sentral_sysfs_cw_mcu_log_mask_store),
	__ATTR(mcu_log_level, 0660, sentral_sysfs_cw_mcu_log_level_show,
			sentral_sysfs_cw_mcu_log_level_store),
	__ATTR(dbg_flag, 0440, sentral_sysfs_cw_debug_flag_show, NULL),
	__ATTR(sensor_placement, 0220, NULL,
			sentral_sysfs_cw_sensor_placement_store),
	__ATTR(vibrate_ms, 0220, NULL, sentral_sysfs_cw_vibrate_ms_store),
	__ATTR(crash_count, 0440, sentral_sysfs_cw_mcu_crash_show, NULL),
#ifdef NVRAM_DATA_ONESHOT
	__ATTR(nvram_cali_data, 0666, get_nvram_cali_data, set_nvram_cali_data),
#endif
#ifdef CONFIG_MTK
	__ATTR(testing_ptmode, 0666, sentral_sysfs_testing_ptmode_show, NULL),
#endif /* CONFIG_MTK */

};

static int sentral_class_create(struct sentral_device *sentral)
{
	int rc = 0;

	// custom sensor hub class
	sentral->hub_class = class_create(THIS_MODULE, SENTRAL_HUB_CLASS_NAME);
	if (IS_ERR(sentral->hub_class)) {
		rc = PTR_ERR(sentral->hub_class);
		LOGE(&sentral->client->dev, "error creating hub class: %d\n", rc);
		goto exit;
	}

	// custom sensor hub device
	sentral->hub_device = device_create(sentral->hub_class, NULL, 0,
			"%s", SENTRAL_HUB_DEVICE_NAME);
	if (IS_ERR(sentral->hub_device)) {
		rc = PTR_ERR(sentral->hub_device);
		LOGE(&sentral->client->dev, "error creating hub device: %d\n", rc);
		goto exit_class_created;
	}

	// set device data
	rc = dev_set_drvdata(sentral->hub_device, sentral);
	if (rc) {
		LOGE(&sentral->client->dev, "error setting device data: %d\n", rc);
		goto exit_device_created;
	}

	return 0;

exit_device_created:
	device_unregister(sentral->hub_device);
exit_class_created:
	class_destroy(sentral->hub_class);
exit:
	return rc;
}

static void sentral_class_destroy(struct sentral_device *sentral)
{
	device_unregister(sentral->hub_device);
	class_destroy(sentral->hub_class);
}

// SYSFS

static int sentral_sysfs_create(struct sentral_device *sentral)
{
	int rc = 0;
	int i;

	sentral->bma253_class = class_create(THIS_MODULE, "htc_g_sensor");
	if (IS_ERR(sentral->bma253_class)) {
		rc = PTR_ERR(sentral->bma253_class);
		sentral->bma253_class = NULL;
		LOGE(&sentral->client->dev, "%s: could not allocate bma253_user_cali_class, res = %d\n",
		  __func__, rc);
		goto error_user_cali_class;
	}

	sentral->bma253_dev = device_create(sentral->bma253_class,
					    NULL, 0, "%s", "g_sensor");
	if (IS_ERR(sentral->bma253_dev)) {
		rc = PTR_ERR(sentral->bma253_dev);
		goto err_bma253_device_create;
	}

	rc = dev_set_drvdata(sentral->bma253_dev, sentral);
	if (rc)
		goto err_bma253_set_drvdata;

	rc = device_create_file(sentral->bma253_dev,
				 &dev_attr_g_sensor_user_offset);
	if (rc) {
		LOGE(&sentral->client->dev, "%s, create bma253_device_create_file fail!\n", __func__);
		goto err_create_bma253_device_file;
	}


	// link iio device
	rc = sysfs_create_link(&sentral->hub_device->kobj,
			&sentral->indio_dev->dev.kobj, "iio");
	if (rc < 0) {
		LOGE(&sentral->client->dev, "error (%d) creating device iio link\n",
				rc);

		goto error;
	}

	// create root nodes
	rc = sysfs_create_group(&sentral->hub_device->kobj,
			&sentral_attribute_group);

	if (rc) {
		LOGE(&sentral->client->dev, "error (%d) creating device root nodes\n",
				rc);

		goto error;
	}

	// create cw nodes
	for (i = 0; i < ARRAY_SIZE(sentral_cw_attrs); i++) {
		rc = device_create_file(sentral->hub_device, sentral_cw_attrs + i);
		if (rc) {
			LOGE(&sentral->client->dev, "error (%d) creating device node: %d\n",
					rc, i);
			goto error;
		}
	}

	return 0;

error:
err_create_bma253_device_file:
err_bma253_set_drvdata:
	put_device(sentral->bma253_dev);
	device_unregister(sentral->bma253_dev);
err_bma253_device_create:
	class_destroy(sentral->bma253_class);
error_user_cali_class:
	LOGE(&sentral->client->dev, "%s:Unable to create interface\n",
		__func__);

	return rc;
}

static void sentral_sysfs_destroy(struct sentral_device *sentral)
{

	// remove iio device link
	sysfs_remove_link(&sentral->hub_device->kobj, "iio");

	// remove group
	sysfs_remove_group(&sentral->hub_device->kobj, &sentral_attribute_group);

	device_remove_file(sentral->bma253_dev, &dev_attr_g_sensor_user_offset);
	device_unregister(sentral->bma253_dev);
	class_destroy(sentral->bma253_class);
}

static irqreturn_t sentral_irq_handler(int irq, void *dev_id)
{
	struct sentral_device *sentral = dev_id;

	LOGD(&sentral->client->dev, "IRQ received\n");

	// cancel any delayed watchdog work
	if (cancel_delayed_work(&sentral->work_watchdog) == 0)
		flush_workqueue(sentral->sentral_wq);

	queue_work(sentral->sentral_wq, &sentral->work_fifo_read);

	return IRQ_HANDLED;
}

static void sentral_do_work_watchdog(struct work_struct *work)
{
	struct sentral_device *sentral = container_of((struct delayed_work *)work,
			struct sentral_device, work_watchdog);
	u8 err = 0;
	u32 crc = 0;
	size_t bytes_read = 0;
	int i = 0;
	int rc;

	LOGI(&sentral->client->dev,
			"[WD] Watchdog bite, enable mask: 0x%08lX, batch mask: 0x%08lX\n",
			sentral->enabled_mask, sentral->sensors_batched_mask);

	// no sensors enabled
	if (!sentral->enabled_mask)
		return;

	// check error register
	err = sentral_error_get(sentral);
	LOGE(&sentral->client->dev, "[WD] Error register: 0x%02X\n", err);

	switch (err) {
	case SEN_DBG_ERROR_REG_CODE_MW_MATH:
	case SEN_DBG_ERROR_REG_CODE_MEM_ERROR:
		LOGE(&sentral->client->dev, "[WD] Fatal error: %u\n", err);
		sentral_crash_reset(sentral);
		return;
	}

	// check CRC for memory corruption
	rc = sentral_crc_get(sentral, &crc);
	if (rc) {
		LOGE(&sentral->client->dev, "[WD] error (%d) reading CRC\n", rc);
		return;
	}

	if (crc && (crc != sentral->fw_crc)) {
		LOGE(&sentral->client->dev, "[WD] CRC error { fw: %u, read: %u }\n",
				sentral->fw_crc, crc);
		sentral_crash_reset(sentral);
		return;
	}

	// check for missed interrupt
	rc = sentral_fifo_read(sentral, (void *)sentral->data_buffer, &bytes_read);
	if (rc) {
		LOGE(&sentral->client->dev, "[WD] error (%d) reading FIFO\n", rc);
		return;
	}

	// caught a missed interrupt, return
	if (bytes_read)
		return;

	// check for continuous sensor enabled
	for (i = SST_FIRST; i < SST_MAX; i++) {
		if (!test_bit(i, &sentral->enabled_mask))
			continue;

		if (!sentral_sensor_id_is_continuous_sensor(i))
			continue;

		if (test_bit(i, &sentral->sensors_batched_mask)
				&& (sentral->sensors_batched_value[i] > (SENTRAL_WATCHDOG_WORK_MSECS - 500))) {
			continue;
		}

		LOGE(&sentral->client->dev,
				"[WD] continuous sensor (%u) report failed\n", i);

		sentral_crash_reset(sentral);
		return;
	}

}

static void sentral_do_work_startup(struct work_struct *work)
{
	struct sentral_device *sentral = container_of(work,
			struct sentral_device, work_startup);

	int rc = 0;

	sentral->init_complete = false;

	// cancel jobs
	cancel_work_sync(&sentral->work_fifo_read);
	cancel_delayed_work_sync(&sentral->work_watchdog);

	// load firmware
	rc = sentral_firmware_load(sentral, sentral->platform_data.firmware);
	if (rc) {
		LOGE(&sentral->client->dev, "error (%d) loading firmware\n", rc);
		return;
	}
	mdelay(100);

	// enable host run
	rc = sentral_set_cpu_run_enable(sentral, true);
	if (rc) {
		LOGE(&sentral->client->dev, "error (%d) enabling cpu run\n", rc);
		return;
	}
}

// IIO
static int sentral_buffer_preenable(struct iio_dev *indio_dev)
{
    struct iio_buffer *buffer = indio_dev->buffer;
    struct sentral_device *sentral = iio_priv(indio_dev);
    
    buffer->access->set_bytes_per_datum(buffer, sizeof(struct sentral_sensors_event));
    LOGI(&sentral->client->dev, "Set data size to %d Bytes\n", (int)sizeof(struct sentral_sensors_event));

    return 0;
}
static const struct iio_buffer_setup_ops sentral_iio_buffer_setup_ops = {
	//.preenable = &iio_sw_buffer_preenable,
	.preenable = &sentral_buffer_preenable,
};

static int sentral_iio_buffer_create(struct iio_dev *indio_dev)
{
	struct sentral_device *sentral = iio_priv(indio_dev);
	int rc = 0;

	indio_dev->buffer = iio_kfifo_allocate(indio_dev);
	if (!indio_dev->buffer) {
		LOGE(&sentral->client->dev, "error allocating IIO kfifo buffer\n");
		return -ENOMEM;
	}

	indio_dev->buffer->scan_timestamp = true;
	indio_dev->setup_ops = &sentral_iio_buffer_setup_ops;

	rc = iio_buffer_register(indio_dev, indio_dev->channels,
			indio_dev->num_channels);

	if (rc) {
		LOGE(&sentral->client->dev, "error (%d) registering IIO buffer", rc);
		goto exit_free;
	}

	iio_scan_mask_set(indio_dev, indio_dev->buffer, SEN_SCAN_U32_1);
	iio_scan_mask_set(indio_dev, indio_dev->buffer, SEN_SCAN_U32_2);
	iio_scan_mask_set(indio_dev, indio_dev->buffer, SEN_SCAN_U32_3);
	iio_scan_mask_set(indio_dev, indio_dev->buffer, SEN_SCAN_U32_4);

	return rc;

exit_free:
	iio_kfifo_free(indio_dev->buffer);
	return rc;
}

static void sentral_iio_buffer_destroy(struct iio_dev *indio_dev)
{
	iio_buffer_unregister(indio_dev);
	iio_kfifo_free(indio_dev->buffer);
}

static int sentral_read_raw(struct iio_dev *indio_dev,
		       struct iio_chan_spec const *chan,
		       int *val,
		       int *val2,
		       long mask) {
	struct sentral_device *sentral = iio_priv(indio_dev);
	int ret = -EINVAL;

	if (chan->type != IIO_ACCEL)
		return ret;

	mutex_lock(&sentral->lock);
	switch (mask) {
	case 0:
		*val = sentral->iio_data[chan->channel2 - IIO_MOD_X];
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SCALE:
		/* Gain : counts / uT = 1000 [nT] */
		/* Scaling factor : 1000000 / Gain = 1000 */
		*val = 0;
		*val2 = 1000;
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;
	}
	mutex_unlock(&sentral->lock);

	return ret;
}


#define SENTRAL_IIO_CHANNEL(i) \
{\
	.type = IIO_ACCEL,\
	.indexed = 1,\
	.channel = i,\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),\
	.scan_index = i,\
	.scan_type = IIO_ST('u', 32, 32, 0),\
}

static const struct iio_chan_spec sentral_iio_channels[] = {
	SENTRAL_IIO_CHANNEL(SEN_SCAN_U32_1),
	SENTRAL_IIO_CHANNEL(SEN_SCAN_U32_2),
	SENTRAL_IIO_CHANNEL(SEN_SCAN_U32_3),
	SENTRAL_IIO_CHANNEL(SEN_SCAN_U32_4),
	IIO_CHAN_SOFT_TIMESTAMP(SEN_SCAN_TS),
};

static const struct iio_info sentral_iio_info = {
	.read_raw = &sentral_read_raw,
	.driver_module = THIS_MODULE,
};

static int sentral_suspend_notifier(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct sentral_device *sentral = container_of(nb, struct sentral_device,
			nb);
	int rc;

	LOGD(&sentral->client->dev, "suspend nb: %lu\n", event);

	switch (event) {

	case PM_SUSPEND_PREPARE:
		LOGI(&sentral->client->dev, "preparing to suspend ...\n");

		// notify sentral of suspend
		rc = sentral_set_host_ap_suspend_enable(sentral, true);
		if (rc) {
			LOGE(&sentral->client->dev,
					"error (%d) setting AP suspend to true\n", rc);
		}
		break;

	case PM_POST_SUSPEND:
		LOGI(&sentral->client->dev, "post suspend ...\n");

		// notify sentral of wakeup
		rc = sentral_set_host_ap_suspend_enable(sentral, false);
		if (rc) {
			LOGE(&sentral->client->dev,
					"error (%d) setting AP suspend to false\n", rc);
		}
		break;
	}

	return NOTIFY_DONE;
}

static int sentral_dt_parse(struct device *dev,
		struct sentral_platform_data *platform_data)
{
	struct property *prop;
	int rc = 0;
	u32 buf = 0;
	struct device_node *node = NULL;
	struct sentral_device *sentral = container_of(platform_data,
			struct sentral_device, platform_data);

#ifdef CONFIG_MTK
	// host interrupt GPIO
	rc = of_property_read_u32(dev->of_node, SENTRAL_OF_PNI_INT_GPIO, &platform_data->host_int_gpio);
	if (rc) {
		LOGE(dev, "error (%d) locating host interrupt GPIO platform node\n", rc);
		return rc;
	}
	LOGI(dev, "host_int_gpio: %u\n", platform_data->host_int_gpio);

	// host interrupt IRQ
	node = of_find_compatible_node(NULL, NULL, SENTRAL_OF_MTK_EINT);
	if (node == NULL) {
		LOGE(dev, "error locating host interrupt EINT platform node\n");
		return -EINVAL;
	}
	platform_data->host_int_irq = irq_of_parse_and_map(node, 0);
	LOGI(dev, "host_int_irq: %u\n", platform_data->host_int_irq);
#else
	// IRQ
	rc = of_get_named_gpio_flags(dev->of_node, SENTRAL_OF_GPIO_IRQ, 0, NULL);
	if (rc < 0) {
		LOGE(dev, "error (%d) reading platform node: %s\n", rc,
				SENTRAL_OF_GPIO_IRQ);
		return rc;
	}
	platform_data->gpio_irq = rc;
#endif /* CONFIG_MTK */

	// FW name
	rc = of_property_read_string(dev->of_node, SENTRAL_OF_PNI_FW,
			&platform_data->firmware);
	if (rc) {
		LOGE(dev, "error (%d) reading platform node: %s\n", rc,
				SENTRAL_OF_PNI_FW);
		return rc;
	}

	// ALS level count
	rc = of_property_read_u32(dev->of_node, SENTRAL_OF_ALS_LEVEL_COUNT,
			&platform_data->als_level_count);
	if (rc) {
		LOGE(dev, "error (%d) reading platform node: %s\n", rc,
				SENTRAL_OF_ALS_LEVEL_COUNT);
		return rc;
	}

	// check ALS level count
	if (platform_data->als_level_count > SENTRAL_ALS_LEVELS_MAX) {
		LOGE(dev, "invalid ALS level count: %u\n",
				platform_data->als_level_count);
		return -EINVAL;
	}

	// ALS level array
	rc = of_property_read_u32_array(dev->of_node, SENTRAL_OF_ALS_LEVELS,
			platform_data->als_levels, platform_data->als_level_count);
	if (rc) {
		LOGE(dev, "error (%d) reading platform node: %s\n", rc,
				SENTRAL_OF_ALS_LEVEL_COUNT);
		return rc;
	}

	// ALS gold L
	rc = of_property_read_u32(dev->of_node, SENTRAL_OF_ALS_GOLDL,
			&platform_data->als_goldl);
	if (rc) {
		LOGE(dev, "error (%d) reading platform node: %s\n", rc,
				SENTRAL_OF_ALS_GOLDL);
		return rc;
	}

	// ALS gold H
	rc = of_property_read_u32(dev->of_node, SENTRAL_OF_ALS_GOLDH,
			&platform_data->als_goldh);
	if (rc) {
		LOGE(dev, "error (%d) reading platform node: %s\n", rc,
				SENTRAL_OF_ALS_GOLDH);
		return rc;
	}

	// ALS polling mode
	rc = of_property_read_u32(dev->of_node, SENTRAL_OF_ALS_POLLING,
			&platform_data->als_polling);
	if (rc) {
		LOGE(dev, "error (%d) reading platform node: %s\n", rc,
				SENTRAL_OF_ALS_POLLING);
		return rc;
	}

	// ALS Lux ratio numerator
	rc = of_property_read_u32(dev->of_node, SENTRAL_OF_ALS_LUX_RATIO_N,
			&platform_data->als_lux_ratio_n);
	if (rc) {
		LOGE(dev, "error (%d) reading platform node: %s\n", rc,
				SENTRAL_OF_ALS_LUX_RATIO_N);
		return rc;
	}

	// ALS Lux ratio denominator
	rc = of_property_read_u32(dev->of_node, SENTRAL_OF_ALS_LUX_RATIO_D,
			&platform_data->als_lux_ratio_d);
	if (rc) {
		LOGE(dev, "error (%d) reading platform node: %s\n", rc,
				SENTRAL_OF_ALS_LUX_RATIO_D);
		return rc;
	}

	// PS thd add
	rc = of_property_read_u32(dev->of_node, SENTRAL_OF_PS_THD_ADD,
			&platform_data->ps_thd_add);
	if (rc) {
		LOGE(dev, "error (%d) reading platform node: %s\n", rc,
				SENTRAL_OF_PS_THD_ADD);
		return rc;
	}

	// PS thd fixed
	rc = of_property_read_u32(dev->of_node, SENTRAL_OF_PS_THD_FIXED,
			&platform_data->ps_thd_fixed);
	if (rc) {
		LOGE(dev, "error (%d) reading platform node: %s\n", rc,
				SENTRAL_OF_PS_THD_FIXED);
		return rc;
	}

	prop = of_find_property(dev->of_node, "vibrate_ms", NULL);
	if (prop) {
		of_property_read_u32(dev->of_node, "vibrate_ms", &buf);
		sentral->vibrate_ms = buf;
		LOGI(dev, "%s: vibrate_ms = %d\n", __func__, sentral->vibrate_ms);
	} else
		LOGE(dev, "%s: vibrate_ms not found", __func__);


	return 0;
}

static int sentral_probe_id(struct i2c_client *client)
{
	int rc;
	struct sentral_hw_id hw_id;

	rc = i2c_smbus_read_i2c_block_data(client, SR_PRODUCT_ID, sizeof(hw_id),
			(u8 *)&hw_id);

	if (rc < 0)
		return rc;

	LOGI(&client->dev, "Product ID: 0x%02X, Revision ID: 0x%02X\n",
			hw_id.product_id, hw_id.revision_id);

	return 0;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void sentral_pm_early_suspend(struct early_suspend *handler)
{
	struct sentral_device *sentral = container_of(handler,
			struct sentral_device, sentral_early_suspend);

	LOGI(&sentral->client->dev, "entering early suspend\n");

	// cancel crash detect
	if (cancel_delayed_work(&sentral->work_watchdog) == 0)
		flush_workqueue(sentral->sentral_wq);
}

static void sentral_pm_late_resume(struct early_suspend *handler)
{
	struct sentral_device *sentral = container_of(handler,
			struct sentral_device, sentral_early_suspend);

	LOGI(&sentral->client->dev, "entering late resume\n");

	// queue crash detect
	queue_delayed_work(sentral->sentral_wq, &sentral->work_watchdog,
			msecs_to_jiffies(SENTRAL_WATCHDOG_WORK_MSECS));
}

static const struct early_suspend sentral_early_suspend_ops = {
	.suspend = sentral_pm_early_suspend,
	.resume = sentral_pm_late_resume,
};
#endif /* CONFIG_HAS_EARLY_SUSPEND */

static int sentral_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct sentral_device *sentral;
	struct device *dev = &client->dev;
	struct iio_dev *indio_dev;
	int rc;
	char gsensor_id[2] = {2};

	// check i2c capabilities
	rc = i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_I2C_BLOCK);

	if (!rc) {
		LOGE(dev, "i2c_check_functionality error\n");
		return -ENODEV;
	}

#ifdef CONFIG_MTK
	//change timing of i2c to 400Khz
	client->timing = 400;
#endif /* CONFIG_MTK */

	// probe for product id
	rc = sentral_probe_id(client);
	if (rc) {
		LOGE(dev, "error (%d) reading hardware id\n", rc);
		return -ENODEV;
	}

#ifdef SHUB_MTK_DMA
	sentral_i2c_dma_va = (u8 *)dma_alloc_coherent(&client->dev, 4096,
			&sentral_i2c_dma_pa, GFP_KERNEL);

	if (NULL == sentral_i2c_dma_va) {
		LOGE(dev, "couldn't allocate I2C DMA buffer\n");
		return -ENOMEM;
	}
#endif /* SHUB_MTK_DMA */

	// allocate iio device
	indio_dev = iio_device_alloc(sizeof(*sentral));
	if (!indio_dev) {
		LOGE(dev, "couldn't allocate IIO device\n");
		return -ENOMEM;
	}

	// set sentral data
	sentral = iio_priv(indio_dev);
	sentral->client = client;
	sentral->device_id = id;
	sentral->indio_dev = indio_dev;

	// alloc fifo data buffer
	sentral->data_buffer = devm_kzalloc(&client->dev, DATA_BUFFER_SIZE,
			GFP_KERNEL);

	if (!sentral->data_buffer) {
		LOGE(&client->dev, "error allocating data buffer\n");
		rc = -ENOMEM;
		goto error_free;
	}

	// init shub log
	sentral->shub_log.size = SENTRAL_SHUB_LOG_COUNT;
	sentral->shub_log.index = 0;
	sentral->shub_log.count = 0;
	sentral->shub_log.buffer = devm_kzalloc(&client->dev,
			sizeof(struct sentral_shub_log) * sentral->shub_log.size, GFP_KERNEL);

	if (!sentral->shub_log.buffer) {
		LOGE(&client->dev, "error allocating event log buffer\n");
		rc = -ENOMEM;
		goto error_free;
	}

	// check platform data
	if (!client->dev.of_node) {
		LOGE(&client->dev, "error loading platform data\n");
		rc = -ENODEV;
		goto error_free;
	}

	// parse device tree
	rc = sentral_dt_parse(&client->dev, &sentral->platform_data);
	if (rc) {
		LOGE(&client->dev, "error parsing device tree\n");
		rc = -ENODEV;
		goto error_free;
	}

	// request GPIO
#ifdef CONFIG_MTK
	sentral->irq = sentral->platform_data.host_int_irq;
	if (gpio_is_valid(sentral->platform_data.host_int_gpio)) {
		mt_set_gpio_dir(sentral->platform_data.host_int_gpio, GPIO_DIR_IN);
		mt_set_gpio_mode(sentral->platform_data.host_int_gpio, GPIO_MODE_04);
	}
#else
	if (gpio_is_valid(sentral->platform_data.gpio_irq)) {
		rc = gpio_request_one(sentral->platform_data.gpio_irq, GPIOF_DIR_IN,
				"sentral-gpio-irq");

		if (rc) {
			LOGE(&client->dev, "error requesting GPIO\n");
			rc = -ENODEV;
			goto error_free;
		}
	}

	sentral->irq = client->irq = gpio_to_irq(sentral->platform_data.gpio_irq);
#endif /* CONFIG_MTK */

	// set i2c client data
	i2c_set_clientdata(client, indio_dev);

	// set iio data
	indio_dev->dev.parent = &client->dev;
	indio_dev->name = SENTRAL_NAME;
	indio_dev->info = &sentral_iio_info;
	indio_dev->modes = INDIO_BUFFER_HARDWARE;
	indio_dev->channels = sentral_iio_channels;
	indio_dev->num_channels = ARRAY_SIZE(sentral_iio_channels);

	// iio buffer
	if (sentral_iio_buffer_create(indio_dev)) {
		LOGE(&client->dev, "IIO buffer create failed\n");
		goto error_iio_buffer;
	}

	// iio device
	if (iio_device_register(indio_dev)) {
		LOGE(&client->dev, "IIO device register failed\n");
		goto error_iio_device;
	}

	// init work callbacks
	sentral->sentral_wq = create_singlethread_workqueue(SENTRAL_WORKQUEUE_NAME);

	INIT_WORK(&sentral->work_startup, sentral_do_work_startup);
	INIT_WORK(&sentral->work_fifo_read, sentral_do_work_fifo_read);
	INIT_DELAYED_WORK(&sentral->work_watchdog, &sentral_do_work_watchdog);
	INIT_WORK(&sentral->one_shot_work, cwmcu_one_shot);

	// init completion
	init_completion(&sentral->complete_shub_log);

	// init mutex, wakelock
	mutex_init(&sentral->lock);
	mutex_init(&sentral->lock_i2c);
	mutex_init(&sentral->lock_pio);
	mutex_init(&sentral->lock_ts);
	wake_lock_init(&sentral->w_lock, WAKE_LOCK_SUSPEND, dev_name(dev));

	// init waitqueue
	init_waitqueue_head(&sentral->wq_one_shot);
	sentral->wait_one_shot_mask = 0;

	sentral->ts_ref_reset = 1;
    sentral->warm_start_ready = 0;
	sentral->sensors_batched_mask = 0;
	memset(sentral->sensors_batched_value, 0, sizeof(sentral->sensors_batched_value));
	sentral->touch_handshake = 0;

#ifdef CONFIG_MTK
#else
	// setup irq handler
	LOGI(&sentral->client->dev, "requesting IRQ: %d, GPIO: %u\n", sentral->irq,
			sentral->platform_data.gpio_irq);

#endif
	rc = devm_request_threaded_irq(&sentral->client->dev, sentral->irq, NULL,
			sentral_irq_handler, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			dev_name(&sentral->client->dev), sentral);
	if (rc) {
		LOGE(&sentral->client->dev, "error (%d) requesting irq handler\n", rc);
		return rc;
	}

	// init pm
	device_init_wakeup(dev, 1);
	if (device_may_wakeup(dev)) {
		enable_irq_wake(sentral->irq);
	}
	sentral->nb.notifier_call = sentral_suspend_notifier;
	register_pm_notifier(&sentral->nb);

	// create custom class
	rc = sentral_class_create(sentral);
	if (rc) {
		LOGE(&sentral->client->dev, "error (%d) creating sensorhub class\n", rc);
		goto error_class;
	}

	// create misc device
	rc = sentral_miscdev_create(sentral);
	if (rc) {
		LOGE(&sentral->client->dev, "error (%d) creating misc device(s)\n", rc);
		goto error_miscdev;
	}

	// create sysfs nodes
	rc = sentral_sysfs_create(sentral);
	if (rc) {
		LOGE(&sentral->client->dev, "error (%d) creating sysfs objects\n", rc);
		goto error_sysfs;
	}


#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&sentral->sentral_early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */


#ifdef CONFIG_MTK
	vib_trigger_register_simple("vibrator", &vib_trigger);
#endif /* CONFIG_MTK */

#ifdef NVRAM_DATA_ONESHOT
//	schedule_work(&sentral->one_shot_work);
#endif

	priv_sentral = sentral;
#ifdef CONFIG_AK8789_HALLSENSOR
		hallsensor_register_notifier(&hallsensor_status_handler);
#endif

	rc = sentral_set_pt_enable(sentral, 1);
	client->addr = 0x10;		//g_sensor
	mdelay(20);
	rc = sentral_read_block(sentral, 0x00, gsensor_id, 2);
	LOGI(&sentral->client->dev, "g_sensor id is %d\n", gsensor_id[0]);

	client->addr = 0x28;
	mdelay(20);
	rc = sentral_set_pt_enable(sentral, 0);
	
	// startup
	schedule_work(&sentral->work_startup);

	return 0;

error_sysfs:
	sentral_miscdev_destroy(sentral);
error_miscdev:
	sentral_class_destroy(sentral);
error_class:
	iio_device_unregister(indio_dev);
error_iio_device:
	sentral_iio_buffer_destroy(indio_dev);
error_iio_buffer:
error_free:
	if (sentral->data_buffer)
		devm_kfree(&client->dev, sentral->data_buffer);

	if (sentral->shub_log.buffer)
		devm_kfree(&client->dev, sentral->shub_log.buffer);

	iio_device_free(indio_dev);
#ifdef SHUB_MTK_DMA
	if (sentral_i2c_dma_va) {
		dma_free_coherent(&client->dev, 4096, sentral_i2c_dma_va,
				sentral_i2c_dma_pa);
		sentral_i2c_dma_va = NULL;
		sentral_i2c_dma_pa = NULL;
	}
#endif /* SHUB_MTK_DMA */
	return -EIO;
}

static int sentral_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct sentral_device *sentral = iio_priv(indio_dev);

#ifdef CONFIG_AK8789_HALLSENSOR
	hallsensor_unregister_notifier(&hallsensor_status_handler);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&sentral->sentral_early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */

	disable_irq_wake(sentral->irq);
#ifdef CONFIG_MTK
	if (gpio_is_valid(sentral->platform_data.host_int_gpio))
		gpio_free(sentral->platform_data.host_int_gpio);
#else
	if (gpio_is_valid(sentral->platform_data.gpio_irq))
		gpio_free(sentral->platform_data.gpio_irq);
#endif /* CONFIG_MTK */

	wake_lock_destroy(&sentral->w_lock);

	complete_and_exit(&sentral->complete_shub_log, 0);

	cancel_work_sync(&sentral->work_fifo_read);
	cancel_delayed_work_sync(&sentral->work_watchdog);

	destroy_workqueue(sentral->sentral_wq);

	sentral_sysfs_destroy(sentral);
	sentral_miscdev_destroy(sentral);
	sentral_class_destroy(sentral);

	iio_device_unregister(indio_dev);
	sentral_iio_buffer_destroy(indio_dev);

	if (sentral->data_buffer)
		devm_kfree(&client->dev, sentral->data_buffer);

	if (sentral->shub_log.buffer)
		devm_kfree(&client->dev, sentral->shub_log.buffer);

	iio_device_free(indio_dev);

#ifdef SHUB_MTK_DMA
	if (sentral_i2c_dma_va) {
		dma_free_coherent(&client->dev, 4096, sentral_i2c_dma_va,
				sentral_i2c_dma_pa);
		sentral_i2c_dma_va = NULL;
		sentral_i2c_dma_pa = NULL;
	}
#endif /* SHUB_MTK_DMA */
	return 0;
}

static int sentral_suspend(struct device *dev)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	int rc = 0;

	LOGI(dev, "entered suspend\n");

	if (device_may_wakeup(dev))
		enable_irq_wake(sentral->irq);

	return rc;
}

static int sentral_resume(struct device *dev)
{
	struct sentral_device *sentral = dev_get_drvdata(dev);
	int rc = 0;

	LOGI(dev, "entered resume\n");

	if (device_may_wakeup(dev))
		disable_irq_wake(sentral->irq);

	return rc;
}

static const struct dev_pm_ops sentral_pm_ops = {
	.suspend = sentral_suspend,
	.resume = sentral_resume,
};

static const struct i2c_device_id sentral_i2c_id_table[] = {
	{"sentral", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, sentral_i2c_id_table);

static const struct of_device_id sentral_of_id_table[] = {
	{.compatible = "sentral",},
	{},
};
MODULE_DEVICE_TABLE(of, sentral_of_id_table);

static struct i2c_driver sentral_driver = {
	.probe = sentral_probe,
	.remove = sentral_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "sentral-iio",
		.of_match_table = sentral_of_id_table,
		.pm = &sentral_pm_ops,
	},
	.id_table = sentral_i2c_id_table,
};
module_i2c_driver(sentral_driver);

MODULE_AUTHOR("Jeremiah Mattison <jmattison@pnicorp.com>");
MODULE_DESCRIPTION("SENtral Sensor Hub Driver");
MODULE_LICENSE("GPL");
