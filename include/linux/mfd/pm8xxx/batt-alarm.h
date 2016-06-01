/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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
/*
 * Qualcomm PMIC PM8xxx Battery Alarm driver
 *
 */
#ifndef __MFD_PM8XXX_BATT_ALARM_H__
#define __MFD_PM8XXX_BATT_ALARM_H__

#include <linux/bitops.h>
#include <linux/errno.h>

#define PM8XXX_BATT_ALARM_DEV_NAME	"pm8xxx-batt-alarm"

/**
 * enum pm8xxx_batt_alarm_core_data - PMIC core specific core passed into the
 *	batter alarm driver as platform data
 * @irq_name:
 * @reg_addr_batt_alarm_threshold:	PMIC threshold register address
 * @reg_addr_batt_alarm_ctrl1:		PMIC control 1 register address
 * @reg_addr_batt_alarm_ctrl2:		PMIC control 2 register address
 * @reg_addr_batt_alarm_pwm_ctrl:	PMIC PWM control register address
 */
struct pm8xxx_batt_alarm_core_data {
	char	*irq_name;
	u16	reg_addr_threshold;
	u16	reg_addr_ctrl1;
	u16	reg_addr_ctrl2;
	u16	reg_addr_pwm_ctrl;
};

#endif /* __MFD_PM8XXX_BATT_ALARM_H__ */
