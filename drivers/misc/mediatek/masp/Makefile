ifeq ($(CONFIG_MTK_SECURITY_SW_SUPPORT), y)
#######################################
# Specify source files
#######################################
#include $(MTK_ROOT_BUILD)/Makefile

MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
MASP_CORE_DRIVER_DIR := $(srctree)/drivers/misc/mediatek/masp

#Kernel 3.18 always use asfv2
asf_link := asfv2
############################################
#[core_obj] include all the files under asf/core
################################################
include $(MASP_CORE_DRIVER_DIR)/$(asf_link)/Makefile.core
include $(MASP_CORE_DRIVER_DIR)/$(asf_link)/Makefile.mach

# INCLUDE ASP LIBRARY
sec-y += $(core_obj) $(mach_obj)

# BUILD-IN
obj-y += sec.o
endif
