/*
 * include/linux/wpc_pwrbutton.h - platform data structure for WPC pwr btn.
 *
 * Copyright (C) 2013 WorldPiCOM, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_WPC_PWRBUTTON_H
#define _LINUX_WPC_PWRBUTTON_H

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>

#define WPC_PWRBUTTON_NAME "wpc_pwrbutton"
#define WPC_PWRBUTTON_PHYS "wpc_pwrbutton/input0"

struct wpc_pwrbutton_platform_data {
	int gpio_sys_req;
	int gpio_soft_poweroff;
};

#endif /* _LINUX_WPC_PWRBUTTON_H */
