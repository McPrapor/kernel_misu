#!/bin/bash
LARCH=$(grep CONFIG_MTK_PLATFORM= .config | grep -o "\"[0-9a-zA-Z]*\"" | sed 's/"//g')
MARCH=$(grep -o "^CONFIG_ARCH_MT[0-9].*" .config | grep -o "MT[0-9a-zA-Z]*" | awk '{print tolower($0)}')
PROJ=$(grep -o "^CONFIG_ARCH_MTK_PROJECT=.*" .config | grep -o "\".*\"$" | sed 's/"//g')
echo ${LARCH} ${MARCH}
#XDIR=$(pwd)
[ -z "${LARCH}${MARCH}" ] && exit 1

mkdir -p tools/tools

rm -f drivers/misc/mediatek/include/disp_assert_layer.h
ln -s ../video/${LARCH}/disp_assert_layer.h drivers/misc/mediatek/include/

rm -f drivers/watchdog/mediatek/include/mt_wdt.h
ln -s ../wdt/${LARCH}/mt_wdt.h drivers/watchdog/mediatek/include/

rm -f drivers/usb/gadget/function/configfs.h
ln -s ../configfs.h drivers/usb/gadget/function/

rm -f drivers/usb/gadget/function/u_f.h
ln -s ../u_f.h drivers/usb/gadget/function/

rm -f drivers/usb/gadget/function/u_os_desc.h
ln -s ../u_os_desc.h drivers/usb/gadget/function/

rm -f drivers/misc/mediatek/include/mt-plat/pmic.h
ln -s ../../power/${LARCH}/pmic.h drivers/misc/mediatek/include/mt-plat/

rm -f drivers/misc/mediatek/include/mt6311.h
ln -s ../power/${LARCH}/mt6311.h drivers/misc/mediatek/include/

rm -f drivers/misc/mediatek/include/mt_i2c.h
ln -s ../i2c/${LARCH}/mt_i2c.h drivers/misc/mediatek/include/

rm -f drivers/misc/mediatek/include/mt-plat/mt_spi.h
ln -s ../../../../spi/mediatek/mt6735/mt_spi.h drivers/misc/mediatek/include/mt-plat/

#rm -f drivers/misc/mediatek/include/pinctrl-mtk-common.h
#ln -s ../../../pinctrl/mediatek/pinctrl-mtk-common.h drivers/misc/mediatek/include/

rm -f include/pinctrl-mtk-common.h
ln -s ../drivers/pinctrl/mediatek/pinctrl-mtk-common.h include/

rm -f drivers/misc/mediatek/include/gpio_cfg.h
ln -s ../gpio/${LARCH}/gpio_cfg.h drivers/misc/mediatek/include/

rm -f drivers/misc/mediatek/include/mt_gpio_base.h
ln -s ../gpio/${LARCH}/mt_gpio_base.h drivers/misc/mediatek/include/

rm -f drivers/misc/mediatek/include/mt_gpio_ext.h
ln -s ../gpio/${LARCH}/mt_gpio_ext.h drivers/misc/mediatek/include/

rm -f drivers/misc/mediatek/include/6735_gpio.h
ln -s ../gpio/${LARCH}/6735_gpio.h drivers/misc/mediatek/include/

rm -f drivers/misc/mediatek/include/mt-plat/${LARCH}/include/mt_spm.h
ln -s ../../../../base/power/${LARCH}/mt_spm.h drivers/misc/mediatek/include/mt-plat/${LARCH}/include/

rm -f drivers/misc/mediatek/include/mt-plat/${LARCH}/include/mt_cpufreq.h
ln -s ../../../../base/power/${LARCH}/mt_cpufreq.h drivers/misc/mediatek/include/mt-plat/${LARCH}/include/

rm -f drivers/misc/mediatek/include/mt-plat/${LARCH}/include/mt_gpufreq.h
ln -s ../../../../base/power/${LARCH}/mt_gpufreq.h drivers/misc/mediatek/include/mt-plat/${LARCH}/include/

rm -f drivers/misc/mediatek/include/mt-plat/${LARCH}/include/mt_clkbuf_ctl.h
ln -s ../../../../base/power/${LARCH}/mt_clkbuf_ctl.h drivers/misc/mediatek/include/mt-plat/${LARCH}/include/

rm -f drivers/misc/mediatek/include/mt-plat/${LARCH}/include/mach/mt_gpufreq.h
ln -s ../../../../../base/power/${LARCH}/mt_gpufreq.h drivers/misc/mediatek/include/mt-plat/${LARCH}/include/mach/

rm -f drivers/misc/mediatek/btcvsd/inc/${LARCH}/AudioBTCVSDDef.h
ln -s ../../AudioBTCVSDDef.h drivers/misc/mediatek/btcvsd/inc/${LARCH}/

rm -f drivers/misc/mediatek/cmdq/v2/cmdq_engine.h
ln -s ${LARCH}/cmdq_engine.h drivers/misc/mediatek/cmdq/v2/

rm -f drivers/misc/mediatek/ext_disp/external_display.h
ln -s ${LARCH}/external_display.h drivers/misc/mediatek/ext_disp/

rm -f drivers/misc/mediatek/ext_disp/extd_platform.h
ln -s ${LARCH}/extd_platform.h drivers/misc/mediatek/ext_disp/

rm -f drivers/misc/mediatek/m4u/${LARCH}/m4u_reg.h
ln -s ${MARCH}/m4u_reg.h drivers/misc/mediatek/m4u/mt6735/

rm -f drivers/misc/mediatek/m4u/${LARCH}/m4u_platform.h
ln -s ${MARCH}/m4u_platform.h drivers/misc/mediatek/m4u/mt6735/

rm -f drivers/misc/mediatek/uart/include/platform_uart.h
ln -s ../${LARCH}/platform_uart.h drivers/misc/mediatek/uart/include/

rm -f drivers/misc/mediatek/video/${LARCH}/${MARCH}/ddp_hal.h
ln -s ../ddp_hal.h  drivers/misc/mediatek/video/${LARCH}/${MARCH}/

rm -f drivers/misc/mediatek/video/${LARCH}/${MARCH}/ddp_info.h
ln -s ../ddp_info.h  drivers/misc/mediatek/video/${LARCH}/${MARCH}/

rm -f drivers/misc/mediatek/video/include/disp_event.h
ln -s ../${LARCH}/disp_event.h drivers/misc/mediatek/video/include/

rm -f drivers/misc/mediatek/video/include/disp_debug.h
ln -s ../${LARCH}/disp_debug.h drivers/misc/mediatek/video/include/

rm -f drivers/misc/mediatek/video/include/ddp_mmp.h
ln -s ../${LARCH}/ddp_mmp.h drivers/misc/mediatek/video/include/

rm -f drivers/misc/mediatek/video/include/ddp_dump.h
ln -s ../${LARCH}/ddp_dump.h drivers/misc/mediatek/video/include/

rm -f drivers/misc/mediatek/video/include/ddp_path.h
ln -s ../${LARCH}/ddp_path.h drivers/misc/mediatek/video/include/

rm -f drivers/misc/mediatek/video/${LARCH}/${MARCH}/disp_assert_layer.h
ln -s ../disp_assert_layer.h drivers/misc/mediatek/video/${LARCH}/${MARCH}/

rm -f drivers/misc/mediatek/video/common/color20/ddp_drv.h
ln -s ../../${LARCH}/ddp_drv.h drivers/misc/mediatek/video/common/color20/

rm -f include/mt_sd.h
ln -s ../drivers/mmc/host/mediatek/${LARCH}/mt_sd.h include/

#rm -rf drivers/misc/mediatek/flashlight/src/mt6735/v36bml/
#cp -a drivers/misc/mediatek/flashlight/src/mt6735/nd393c/ drivers/misc/mediatek/flashlight/src/mt6735/${PROJ}

export ARCH=arm64 
export CROSS_COMPILE=/usr/local/bin/aarch64-linux-android-4.9/bin/aarch64-linux-android-
#export CUSTOM_KERNEL_DCT=htc_v36bml_dugl
export TARGET_ARCH=arm64
#export TARGET_BUILD_VARIANT=user TARGET_PRODUCT=v36bml_dugl MTK_ROOT_CUSTOM=../mediatek/custom/
#make v36bml_dugl_defconfig
#make k53v1_64_bsp_defconfig
#make mrproper
#make k53v1_64_debug_defconfig
#./build.sh
#make
