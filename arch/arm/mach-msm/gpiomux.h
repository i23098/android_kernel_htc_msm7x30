/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */
#ifndef __ARCH_ARM_MACH_MSM_GPIOMUX_H
#define __ARCH_ARM_MACH_MSM_GPIOMUX_H

#include <linux/bitops.h>
#include <linux/errno.h>
#include <mach/msm_gpiomux.h>
#include "gpiomux-v1.h"

/**
 * struct msm_gpiomux_config: gpiomux settings for one gpio line.
 *
 * A complete gpiomux config is the combination of a drive-strength,
 * function, pull, and (sometimes) direction.  For functions other than GPIO,
 * the input/output setting is hard-wired according to the function.
 *
 * @gpio: The index number of the gpio being described.
 * @settings: The settings to be installed, specifically:
 *           GPIOMUX_ACTIVE: The setting to be installed when the
 *           line is active, or its reference count is > 0.
 *           GPIOMUX_SUSPENDED: The setting to be installed when
 *           the line is suspended, or its reference count is 0.
 */
struct msm_gpiomux_config {
	unsigned gpio;
	struct gpiomux_setting *settings[GPIOMUX_NSETTINGS];
};

/**
 * @GPIOMUX_VALID:	If set, the config field contains 'good data'.
 *                      The absence of this bit will prevent the gpiomux
 *			system from applying the configuration under all
 *			circumstances.
 */
enum {
	GPIOMUX_VALID	 = BIT(sizeof(gpiomux_config_t) * BITS_PER_BYTE - 1),
	GPIOMUX_CTL_MASK = GPIOMUX_VALID,
};

#ifdef CONFIG_MSM_GPIOMUX

/* Before using gpiomux, initialize the subsystem by telling it how many
 * gpios are going to be managed.  Calling any other gpiomux functions before
 * msm_gpiomux_init is unsupported.
 */
int msm_gpiomux_init(size_t ngpio);

/* Install a block of gpiomux configurations in gpiomux.  This is functionally
 * identical to calling msm_gpiomux_write many times.
 */
void msm_gpiomux_install(struct msm_gpiomux_config *configs, unsigned nconfigs);

/* Install a new setting in a gpio.  To erase a slot, use NULL.
 * The old setting that was overwritten can be passed back to the caller
 * old_setting can be NULL if the caller is not interested in the previous
 * setting
 * If a previous setting was not available to return (NULL configuration)
 * - the function returns 1
 * else function returns 0
 */
int msm_gpiomux_write(unsigned gpio, enum msm_gpiomux_setting which,
	struct gpiomux_setting *setting, struct gpiomux_setting *old_setting);

/* Architecture-internal function for use by the framework only.
 * This function can assume the following:
 * - the gpio value has passed a bounds-check
 * - the gpiomux spinlock has been obtained
 *
 * This function is not for public consumption.  External users
 * should use msm_gpiomux_write.
 */
void __msm_gpiomux_write(unsigned gpio, struct gpiomux_setting val);
#else
static inline int msm_gpiomux_init(size_t ngpio)
{
	return -ENOSYS;
}

static inline void
msm_gpiomux_install(struct msm_gpiomux_config *configs, unsigned nconfigs) {}

static inline int msm_gpiomux_write(unsigned gpio,
	enum msm_gpiomux_setting which, struct gpiomux_setting *setting,
	struct gpiomux_setting *old_setting)
{
	return -ENOSYS;
}
#endif

#endif
