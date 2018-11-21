/* Copyright (c) 2014, HTC Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/of.h>
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#endif

#define AUD_VDEV_NAME		"/htc_vdev"

static int rec_spk_sel_gpio = -1;

void setRecSpkSel(bool enable) {
	unsigned long local_select_gpio = 0;

	if (rec_spk_sel_gpio < 0) {
		pr_err("Error, cannot find the rec_spk_sel_gpio: %ld.\n ", local_select_gpio);
		return;
	}

	printk("[AUD] %s(): rec_spk_sel_gpio = %d.\n", __func__, rec_spk_sel_gpio);

	local_select_gpio = (rec_spk_sel_gpio | 0x80000000);
	mt_set_gpio_mode(local_select_gpio, GPIO_MODE_00);
	mt_set_gpio_dir(local_select_gpio, GPIO_DIR_OUT);
	if (enable)
		mt_set_gpio_out(local_select_gpio, GPIO_OUT_ONE);
	else
		mt_set_gpio_out(local_select_gpio, GPIO_OUT_ZERO);

}

int htc_vdev_init(void) {
	struct device_node *vdev_node;
	struct property *prop;

	printk("[AUD] %s(): release\n", __func__);
	vdev_node = of_find_node_by_path(AUD_VDEV_NAME);
	prop = of_find_property(vdev_node, "htc_aud, rec-spk-sel-gpio", NULL);
	if (prop) {
		of_property_read_u32(vdev_node, "htc_aud, rec-spk-sel-gpio", (u32*)&rec_spk_sel_gpio);
		printk("%s: rec_spk_sel_gpio = %d\n", __func__, rec_spk_sel_gpio);
	} else {
		printk("%s: rec_spk_sel_gpio not found", __func__);
		rec_spk_sel_gpio = -1;
	}

	return 0;
}

void htc_vdev_release(void) {
	printk("[AUD] %s(): release\n", __func__);
	rec_spk_sel_gpio = -1;
}
