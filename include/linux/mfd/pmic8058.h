/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*
 * Qualcomm PMIC8058 driver header file
 *
 */

#ifndef __MFD_PMIC8058_H__
#define __MFD_PMIC8058_H__

#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/mfd/pm8xxx/irq.h>
#include <linux/mfd/pm8xxx/gpio.h>
#include <linux/mfd/pm8xxx/mpp.h>
#include <linux/mfd/pm8xxx/vibrator.h>
#include <linux/mfd/pm8xxx/nfc.h>
#include <linux/mfd/pm8xxx/upl.h>
#include <linux/mfd/pm8xxx/misc.h>
#include <linux/mfd/pm8xxx/batt-alarm.h>
#include <linux/leds-pmic8058.h>
#include <linux/mfd/pm8xxx/tm.h>
#include <linux/pmic8058-xoadc.h>
#include <linux/regulator/pmic8058-regulator.h>
#include <linux/regulator/pm8058-xo.h>
#include <linux/pwm.h>
#include <linux/pmic8058-pwm.h>

#define PM8058_GPIOS		40
#define PM8058_MPPS		12

#define PM8058_GPIO_BLOCK_START	24
#define PM8058_MPP_BLOCK_START	16

#define PM8058_NR_IRQS		256

#define PM8058_IRQ_BLOCK_BIT(block, bit) ((block) * 8 + (bit))

/* MPPs and GPIOs [0,N) */
#define PM8058_MPP_IRQ(base, mpp)	((base) + \
					PM8058_IRQ_BLOCK_BIT(16, (mpp)))
#define PM8058_GPIO_IRQ(base, gpio)	((base) + \
					PM8058_IRQ_BLOCK_BIT(24, (gpio)))

/* PM8058 IRQ's */
#define PM8058_VCP_IRQ			PM8058_IRQ_BLOCK_BIT(1, 0)
#define PM8058_CHGILIM_IRQ		PM8058_IRQ_BLOCK_BIT(1, 3)
#define PM8058_VBATDET_LOW_IRQ		PM8058_IRQ_BLOCK_BIT(1, 4)
#define PM8058_BATT_REPLACE_IRQ		PM8058_IRQ_BLOCK_BIT(1, 5)
#define PM8058_CHGINVAL_IRQ		PM8058_IRQ_BLOCK_BIT(1, 6)
#define PM8058_CHGVAL_IRQ		PM8058_IRQ_BLOCK_BIT(1, 7)
#define PM8058_CHG_END_IRQ		PM8058_IRQ_BLOCK_BIT(2, 0)
#define PM8058_FASTCHG_IRQ		PM8058_IRQ_BLOCK_BIT(2, 1)
#define PM8058_CHGSTATE_IRQ		PM8058_IRQ_BLOCK_BIT(2, 3)
#define PM8058_AUTO_CHGFAIL_IRQ		PM8058_IRQ_BLOCK_BIT(2, 4)
#define PM8058_AUTO_CHGDONE_IRQ		PM8058_IRQ_BLOCK_BIT(2, 5)
#define PM8058_ATCFAIL_IRQ		PM8058_IRQ_BLOCK_BIT(2, 6)
#define PM8058_ATC_DONE_IRQ		PM8058_IRQ_BLOCK_BIT(2, 7)
#define PM8058_OVP_OK_IRQ		PM8058_IRQ_BLOCK_BIT(3, 0)
#define PM8058_COARSE_DET_OVP_IRQ	PM8058_IRQ_BLOCK_BIT(3, 1)
#define PM8058_VCPMAJOR_IRQ		PM8058_IRQ_BLOCK_BIT(3, 2)
#define PM8058_CHG_GONE_IRQ		PM8058_IRQ_BLOCK_BIT(3, 3)
#define PM8058_CHGTLIMIT_IRQ		PM8058_IRQ_BLOCK_BIT(3, 4)
#define PM8058_CHGHOT_IRQ		PM8058_IRQ_BLOCK_BIT(3, 5)
#define PM8058_BATTTEMP_IRQ		PM8058_IRQ_BLOCK_BIT(3, 6)
#define PM8058_BATTCONNECT_IRQ		PM8058_IRQ_BLOCK_BIT(3, 7)
#define PM8058_BATFET_IRQ		PM8058_IRQ_BLOCK_BIT(5, 4)
#define PM8058_VBATDET_IRQ		PM8058_IRQ_BLOCK_BIT(5, 5)
#define PM8058_VBAT_IRQ			PM8058_IRQ_BLOCK_BIT(5, 6)

#define PM8058_RTC_IRQ			PM8058_IRQ_BLOCK_BIT(6, 5)
#define PM8058_RTC_ALARM_IRQ		PM8058_IRQ_BLOCK_BIT(4, 7)
#define PM8058_PWRKEY_REL_IRQ		PM8058_IRQ_BLOCK_BIT(6, 2)
#define PM8058_PWRKEY_PRESS_IRQ		PM8058_IRQ_BLOCK_BIT(6, 3)
#define PM8058_KEYPAD_IRQ		PM8058_IRQ_BLOCK_BIT(9, 2)
#define PM8058_KEYSTUCK_IRQ		PM8058_IRQ_BLOCK_BIT(9, 3)
#define PM8058_BATT_ALARM_IRQ		PM8058_IRQ_BLOCK_BIT(5, 6)
#define PM8058_SW_0_IRQ			PM8058_IRQ_BLOCK_BIT(7, 1)
#define PM8058_IR_0_IRQ			PM8058_IRQ_BLOCK_BIT(7, 0)
#define PM8058_SW_1_IRQ			PM8058_IRQ_BLOCK_BIT(7, 3)
#define PM8058_IR_1_IRQ			PM8058_IRQ_BLOCK_BIT(7, 2)
#define PM8058_SW_2_IRQ			PM8058_IRQ_BLOCK_BIT(7, 5)
#define PM8058_IR_2_IRQ			PM8058_IRQ_BLOCK_BIT(7, 4)
#define PM8058_TEMPSTAT_IRQ		PM8058_IRQ_BLOCK_BIT(6, 7)
#define PM8058_OVERTEMP_IRQ		PM8058_IRQ_BLOCK_BIT(4, 2)
#define PM8058_ADC_IRQ			PM8058_IRQ_BLOCK_BIT(9, 4)
#define PM8058_OSCHALT_IRQ		PM8058_IRQ_BLOCK_BIT(4, 6)
#define PM8058_CBLPWR_IRQ		PM8058_IRQ_BLOCK_BIT(4, 3)
#define PM8058_RESOUT_IRQ		PM8058_IRQ_BLOCK_BIT(6, 4)

struct pmic8058_charger_data {
	unsigned int max_source_current;
	int charger_type;
	bool charger_data_valid;
};

enum pon_config{
	DISABLE_HARD_RESET = 0,
	SHUTDOWN_ON_HARD_RESET,
	RESTART_ON_HARD_RESET,
	MAX_PON_CONFIG,
};

enum pm8058_smpl_delay {
	PM8058_SMPL_DELAY_0p5,
	PM8058_SMPL_DELAY_1p0,
	PM8058_SMPL_DELAY_1p5,
	PM8058_SMPL_DELAY_2p0,
};

// from rtc
#define PM8XXX_RTC_DEV_NAME     "rtc-pm8xxx"
/**
 * struct pm8xxx_rtc_pdata - RTC driver platform data
 * @rtc_write_enable: variable stating RTC write capability
 */
struct pm8xxx_rtc_platform_data {
	bool rtc_write_enable;
};

//from pwrkey
#define PM8XXX_PWRKEY_DEV_NAME "pm8xxx-pwrkey"

/**
 * struct pm8xxx_pwrkey_platform_data - platform data for pwrkey driver
 * @pull up:  power on register control for pull up/down configuration
 * @kpd_trigger_delay_us: time delay for power key state change interrupt
 *                  trigger.
 * @wakeup: configure power key as wakeup source
 */
struct pm8xxx_pwrkey_platform_data  {
	bool pull_up;
	u32  kpd_trigger_delay_us;
	u32  wakeup;
};

//from keypad
#include <linux/input/matrix_keypad.h>

#define PM8XXX_KEYPAD_DEV_NAME     "pm8xxx-keypad"

/**
 * struct pm8xxx_keypad_platform_data - platform data for keypad
 * @keymap_data - matrix keymap data
 * @input_name - input device name
 * @input_phys_device - input device name
 * @num_cols - number of columns of keypad
 * @num_rows - number of row of keypad
 * @debounce_ms - debounce period in milliseconds
 * @scan_delay_ms - scan delay in milliseconds
 * @row_hold_ns - row hold period in nanoseconds
 * @wakeup - configure keypad as wakeup
 * @rep - enable or disable key repeat bit
 */
struct pm8xxx_keypad_platform_data {
	const struct matrix_keymap_data *keymap_data;

	const char *input_name;
	const char *input_phys_device;

	unsigned int num_cols;
	unsigned int num_rows;
	unsigned int rows_gpio_start;
	unsigned int cols_gpio_start;

	unsigned int debounce_ms;
	unsigned int scan_delay_ms;
	unsigned int row_hold_ns;

	bool wakeup;
	bool rep;
};

struct pm8058_platform_data {
	struct pm8xxx_mpp_platform_data		*mpp_pdata;
	struct pm8xxx_keypad_platform_data      *keypad_pdata;
	struct pm8xxx_gpio_platform_data	*gpio_pdata;
	struct pm8xxx_irq_platform_data		*irq_pdata;
	struct pm8xxx_rtc_platform_data		*rtc_pdata;
	struct pm8xxx_pwrkey_platform_data	*pwrkey_pdata;
	struct pm8xxx_vibrator_platform_data	*vibrator_pdata;
	struct pm8xxx_misc_platform_data	*misc_pdata;
	struct pmic8058_leds_platform_data	*leds_pdata;
	struct xoadc_platform_data		*xoadc_pdata;
	struct pm8058_pwm_pdata			*pwm_pdata;
	struct pm8058_vreg_pdata		*regulator_pdatas;
	int					num_regulators;
	struct pm8058_xo_pdata			*xo_buffer_pdata;
	int					num_xo_buffers;
	struct pmic8058_charger_data		*charger_pdata;
};

#ifdef CONFIG_PMIC8058
int pm8058_reset_pwr_off(int reset);
#else
static inline int pm8058_reset_pwr_off(int reset) { return 0; }
#endif


int pm8058_hard_reset_config(enum pon_config config);

/**
 * pm8058_smpl_control - enables/disables SMPL detection
 * @enable: 0 = shutdown PMIC on power loss, 1 = reset PMIC on power loss
 *
 * This function enables or disables the Sudden Momentary Power Loss detection
 * module.  If SMPL detection is enabled, then when a sufficiently long power
 * loss event occurs, the PMIC will automatically reset itself.  If SMPL
 * detection is disabled, then the PMIC will shutdown when power loss occurs.
 *
 * RETURNS: an appropriate -ERRNO error value on error, or zero for success.
 */
int pm8058_smpl_control(int enable);

/**
 * pm8058_smpl_set_delay - sets the SMPL detection time delay
 * @delay: enum value corresponding to delay time
 *
 * This function sets the time delay of the SMPL detection module.  If power
 * is reapplied within this interval, then the PMIC reset automatically.  The
 * SMPL detection module must be enabled for this delay time to take effect.
 *
 * RETURNS: an appropriate -ERRNO error value on error, or zero for success.
 */
int pm8058_smpl_set_delay(enum pm8058_smpl_delay delay);

/**
 * pm8058_watchdog_reset_control - enables/disables watchdog reset detection
 * @enable: 0 = shutdown when PS_HOLD goes low, 1 = reset when PS_HOLD goes low
 *
 * This function enables or disables the PMIC watchdog reset detection feature.
 * If watchdog reset detection is enabled, then the PMIC will reset itself
 * when PS_HOLD goes low.  If it is not enabled, then the PMIC will shutdown
 * when PS_HOLD goes low.
 *
 * RETURNS: an appropriate -ERRNO error value on error, or zero for success.
 */
int pm8058_watchdog_reset_control(int enable);

/**
 * pm8058_stay_on - enables stay_on feature
 *
 * PMIC stay-on feature allows PMIC to ignore MSM PS_HOLD=low
 * signal so that some special functions like debugging could be
 * performed.
 *
 * This feature should not be used in any product release.
 *
 * RETURNS: an appropriate -ERRNO error value on error, or zero for success.
 */
int pm8058_stay_on(void);

#endif  /* __MFD_PMIC8058_H__ */
