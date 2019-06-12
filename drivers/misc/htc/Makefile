#include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/
obj-$(CONFIG_HTC_REBOOT_REASON_INFORMATION) += htc_reboot_info.o
obj-$(CONFIG_HTC_DEBUG_BOOTLOADER_LOG) += htc_bootloader_log.o
obj-y += htc_devices_dtb.o
obj-$(CONFIG_HTC_VIRTUAL_DEV) += htc_vdev.o
obj-$(CONFIG_HTC_FEATURES_RIL_PCN0004_HTC_GARBAGE_FILTER) += htc_port_list.o
