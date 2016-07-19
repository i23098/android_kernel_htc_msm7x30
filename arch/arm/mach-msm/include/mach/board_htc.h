/* arch/arm/mach-msm/include/mach/BOARD_HTC.h
 * Copyright (C) 2007-2009 HTC Corporation.
 * Author: Thomas Tsai <thomas_tsai@htc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __ASM_ARCH_MSM_BOARD_HTC_H
#define __ASM_ARCH_MSM_BOARD_HTC_H

#include <linux/types.h>
#include <linux/list.h>
#include <asm/setup.h>
#include <mach/board.h>

/* common init routines for use by arch/arm/mach-msm/board-*.c */

void __init msm_init_pmic_vibrator(int);

struct mmc_platform_data;

#if defined(CONFIG_USB_FUNCTION_MSM_HSUSB) || defined(CONFIG_USB_MSM_72K) || defined(CONFIG_USB_MSM_72K_MODULE)    || defined(CONFIG_USB_CI13XXX_MSM)
int usb_get_connect_type(void);
void msm_otg_set_vbus_state(int online);
enum usb_connect_type {
	CONNECT_TYPE_CLEAR = -2,
	CONNECT_TYPE_UNKNOWN = -1,
	CONNECT_TYPE_NONE = 0,
	CONNECT_TYPE_USB,
	CONNECT_TYPE_AC,
	CONNECT_TYPE_9V_AC,
	CONNECT_TYPE_WIRELESS,
	CONNECT_TYPE_INTERNAL,
	CONNECT_TYPE_UNSUPPORTED,
#ifdef CONFIG_MACH_VERDI_LTE
	/* Y cable with USB and 9V charger */
	CONNECT_TYPE_USB_9V_AC,
#endif
};

#else
static inline void msm_otg_set_vbus_state(int online) {}
#endif

/* START: add USB connected notify function */
struct t_usb_status_notifier {
	struct list_head notifier_link;
	const char *name;
	void (*func)(int cable_type);
};
int msm_usb_register_notifier(struct t_usb_status_notifier *notifer);
static LIST_HEAD(g_lh_usb_notifier_list);
/* END: add USB connected notify function */

/***********************************
Direction: cable detect drvier -> battery driver or other
***********************************/
struct t_cable_status_notifier {
	struct list_head cable_notifier_link;
	const char *name;
	void (*func)(int cable_type);
};
int cable_detect_register_notifier(struct t_cable_status_notifier *);
static LIST_HEAD(g_lh_calbe_detect_notifier_list);

void board_get_sku_color_tag(char **);
void board_get_keycaps_tag(char **);
void board_get_cid_tag(char **);
void board_get_carrier_tag(char **);
int board_emmc_boot(void);

char *board_serialno(void);
unsigned long get_kernel_flag(void);
unsigned int get_radio_flag(void);
char *get_model_id(void);

extern void (*msm_hw_reset_hook)(void);
#endif
