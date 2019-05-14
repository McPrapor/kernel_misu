/*
 * drivers/leds/htc_flashlight_common.c - The flashlight common interface 
 * Copyright (C) 2014  HTC Corporation
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

int (*htc_flash_main)(int led1, int led2);
int (*htc_torch_main)(int led1, int led2);
int (*htc_flash_front)(int led1, int led2);
int (*htc_torch_front)(int led1, int led2);

