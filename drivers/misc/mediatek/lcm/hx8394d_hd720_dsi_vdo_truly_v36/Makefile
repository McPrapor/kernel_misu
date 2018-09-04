#
# Makefile for misc devices that really don't fit anywhere else.
#
#obj-y += hx8394c_wxga_dsi_vdo.o lcm_drv_hx8394c_wxga_dsi_vdo.o



#include $(srctree)/drivers/misc/mediatek/Makefile.custom
## to use arch/arm/mach-mt2601/flora01v1/lcm/inc/lcm_drv.h instead of drivers/misc/mediatek/lcm/inc/lcm_drv.h
#ccflags-$(CONFIG_ARCH_MT2601) += -I$(MTK_PROJECT_PATH_ROOT)/lcm/inc
obj-y += hx8394d_hd720_dsi_vdo_truly_v36.o
ccflags-$(CONFIG_MTK_LCM) += -I$(srctree)/drivers/misc/mediatek/lcm/inc

