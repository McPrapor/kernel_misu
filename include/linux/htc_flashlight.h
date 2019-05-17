/*
 * include/linux/htc_flashlight.h - The flashlight header
 *
 * Copyright (C) 2014 HTC Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __HTC_FLASHLIGHT_H
#define __HTC_FLASHLIGHT_H

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#define FLASHLIGHT_NAME "flashlight"

#define FLASHLIGHT_OFF   0
#define FLASHLIGHT_TORCH 1
#define FLASHLIGHT_FLASH 2
#define FLASHLIGHT_NUM   3


enum flashlight_mode_flags {
	FL_MODE_OFF = 0,
	FL_MODE_TORCH,
	FL_MODE_FLASH,
	FL_MODE_PRE_FLASH,
	FL_MODE_TORCH_LED_A,
	FL_MODE_TORCH_LED_B,
	FL_MODE_TORCH_LEVEL_1,
	FL_MODE_TORCH_LEVEL_2,
	FL_MODE_CAMERA_EFFECT_FLASH,
	FL_MODE_CAMERA_EFFECT_PRE_FLASH,
	FL_MODE_FLASH_LEVEL1,
	FL_MODE_FLASH_LEVEL2,
	FL_MODE_FLASH_LEVEL3,
	FL_MODE_FLASH_LEVEL4,
	FL_MODE_FLASH_LEVEL5,
	FL_MODE_FLASH_LEVEL6,
	FL_MODE_FLASH_LEVEL7,
	FL_MODE_VIDEO_TORCH = 30,
	FL_MODE_VIDEO_TORCH_1,
	FL_MODE_VIDEO_TORCH_2,
	FL_MODE_VIDEO_TORCH_3,
	FL_MODE_VIDEO_TORCH_4,
};

/*
 * flashlight_brightness_attribute_definition: the definition of flashlight "brightness" attribute
 */

enum flashlight_brightness_attribute_definition
{ /* range: [0, 255] */
    FBAD_OFF        = 0,
    FBAD_TORCH1     = 125, /* torch 25mA */
    FBAD_TORCH2     = 126, /* torch 75mA */
    FBAD_TORCH      = 127, /* torch 125mA */
    FBAD_PREFLASH   = 128, /* torch 100mA */
    FBAD_FLASH1     = 130, /* flash 100mA */
    FBAD_FLASH2     = 131, /* flash 200mA */
    FBAD_FLASH3     = 132, /* flash 300mA */
    FBAD_FLASH4     = 133, /* flash 400mA */
    FBAD_FLASH5     = 134, /* flash 500mA */
    FBAD_FLASH6     = 135, /* flash 600mA */
    FBAD_FLASH7     = 136, /* flash 700mA */
    FBAD_FULL       = 255, /* flash 750mA */
};

#if defined(CONFIG_FLASHLIGHT_TPS61310) || defined(CONFIG_FLASHLIGHT_TPS61310_FRONT)
struct TPS61310_flashlight_platform_data {
	void (*gpio_init) (void);
	uint32_t flash_duration_ms;
	uint32_t led_count; /* 0: 1 LED, 1: 2 LED */
	uint32_t tps61310_strb0;
	uint32_t tps61310_strb1;
	uint32_t tps61310_reset;
	uint8_t mode_pin_suspend_state_low;
	uint32_t enable_FLT_1500mA;
	uint32_t disable_tx_mask;
	uint32_t power_save; //disable modem while FLASH 1.5A flash
	uint32_t power_save_2;
};
#endif
#if defined(CONFIG_FLASHLIGHT_TPS61310)
int tps61310_flashlight_control(int mode);
int tps61310_flashlight_mode(int mode);
int tps61310_flashlight_mode2(int mode2, int mode13);
#endif

#if defined(CONFIG_HTC_FLASHLIGHT_COMMON)
extern int (*htc_flash_main)(int ,int);
extern int (*htc_torch_main)(int ,int);
extern int (*htc_flash_front)(int ,int);
extern int (*htc_torch_front)(int ,int);
#endif

#undef __HTC_FLASHLIGHT_H
#endif

