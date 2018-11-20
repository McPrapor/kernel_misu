/*
 * accdet_htc.h
 *
 *  Created on: Dec 16, 2014
 *      Author: jay
 */

#ifndef ACCDET_HTC_H_
#define ACCDET_HTC_H_

#define DEVICE_HEADSET_ATTR(_name, _mode, _show, _store) \
	struct device_attribute dev_attr_headset_##_name = \
	__ATTR(_name, _mode, _show, _store)

#define BIT_HEADSET		(1 << 0)
#define BIT_HEADSET_NO_MIC	(1 << 1)
#define BIT_TTY_FULL		(1 << 2)
#define BIT_FM_HEADSET		(1 << 3)
#define BIT_FM_SPEAKER		(1 << 4)
#define BIT_TTY_VCO		(1 << 5)
#define BIT_TTY_HCO		(1 << 6)
#define BIT_35MM_HEADSET	(1 << 7)
#define BIT_TV_OUT		(1 << 8)
#define BIT_USB_CRADLE		(1 << 9)
#define BIT_TV_OUT_AUDIO	(1 << 10)
#define BIT_HDMI_CABLE		(1 << 11)
#define BIT_HDMI_AUDIO		(1 << 12)
#define BIT_USB_AUDIO_OUT	(1 << 13)
#define BIT_UNDEFINED		(1 << 14)

#define MASK_HEADSET		(BIT_HEADSET | BIT_HEADSET_NO_MIC)
#define MASK_35MM_HEADSET	(BIT_HEADSET | BIT_HEADSET_NO_MIC | \
				BIT_35MM_HEADSET | BIT_TV_OUT)
#define MASK_FM_ATTRIBUTE	(BIT_FM_HEADSET | BIT_FM_SPEAKER)
#define MASK_USB_HEADSET	(BIT_USB_AUDIO_OUT)

struct htc_headset_info {
	struct class *htc_accessory_class;
	struct device *headset_dev;
	struct device *tty_dev;
	struct device *fm_dev;
	struct device *debug_dev;
	struct mutex mutex_lock;

	struct switch_dev sdev_h2w;
	/* The variables were used by 35mm headset*/
	int hs_35mm_type;
	bool hs_mfg_mode;
};

enum {
	HTC_HEADSET_UNPLUG		= 0,
	HTC_HEADSET_MIC			= 1,
	HTC_HEADSET_NO_MIC		= 2,
	HTC_HEADSET_METRICO		= 3,
	HTC_HEADSET_UNKNOWN_MIC		= 4,
	HTC_HEADSET_UNSTABLE		= 5,
	HTC_HEADSET_TV_OUT		= 6,
	HTC_HEADSET_INDICATOR		= 7,
	HTC_HEADSET_ONEWIRE		= 8,
	HTC_HEADSET_UART		= 10,
};

#endif /* ACCDET_HTC_H_ */
