#
# Makefile for miscellaneous I2C chip drivers.
#
# Device drivers are better grouped according to the functionality they
# implement rather than to the bus they are connected to.
#

obj-y += htc_devices_dtb.o

obj-$(CONFIG_INPUT_CAPELLA_CM36686)     += cm36686.o
obj-$(CONFIG_BOSCH_BMA253)  += bma253_bosch.o
#obj-$(CONFIG_SENSORS_AK09916)   += akm09916.o
#obj-$(CONFIG_SENSORS_NFC_PN548) += pn548.o
#obj-$(CONFIG_SENSORS_NFC_PN548) += pn548_htc.o
#obj-$(CONFIG_INPUT_CWSTM32_V2)	+= CwMcuSensor_v2.o
#obj-$(CONFIG_INPUT_CWSTM32)	+= CwMcuSensor.o
#obj-$(CONFIG_INPUT_LSM6DS3) += lsm6ds3_core.o
#obj-$(CONFIG_INPUT_LSM6DS3_I2C) += lsm6ds3_i2c.o
EXTRA_CFLAGS += -fno-pic
obj-$(CONFIG_SENTRAL_IIO) += sentral-iio.o
obj-$(CONFIG_AK8789_HALLSENSOR) += hall_sensor.o hall_sensor_notify.o

sentral-iio-objs += sentral-core.o
sentral-iio-objs += sentral-warmstart.o


#ifeq ($(CONFIG_ARCH_MTK_PROJECT),"htc_alpine")
#EXTRA_CFLAGS += -DNFC_POWER_OFF_SEQUENCE=1
#EXTRA_CFLAGS += -DNFC_OFF_MODE_CHARGING_ENABLE=1
#EXTRA_CFLAGS += -DNFC_GET_BOOTMODE=1
#EXTRA_CFLAGS += -DNFC_IOEXT_PIN_CTRL_CONFIG=1
#EXTRA_CFLAGS += -DNFC_READ_RFSKUID_MTK6795=1
#endif

MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -Werror
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
subdir-ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)


