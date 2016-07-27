/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2012, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/module.h>

#include <mach/clk.h>
#include <linux/ctype.h>
#include <linux/stddef.h>
#include <mach/socinfo.h>

#include "proc_comm.h"
#include "clock.h"
#include "clock-pcom.h"

static inline struct pcom_clk *to_clk_pcom(struct clk *clk)
{
	return container_of(clk, struct pcom_clk, c);
}

/*
 * glue for the proc_comm interface
 */
static int pc_clk_enable(struct clk *hw)
{
	int rc;
	unsigned id = to_clk_pcom(hw)->id;

	/* Ignore clocks that are always on */
	if (id == P_EBI1_CLK || id == P_EBI1_FIXED_CLK)
		return 0;

	rc = msm_proc_comm(PCOM_CLKCTL_RPC_ENABLE, &id, NULL);
	if (rc < 0)
		return rc;
	else
		return (int)id < 0 ? -EINVAL : 0;
}

static void pc_clk_disable(struct clk *hw)
{
	unsigned id = to_clk_pcom(hw)->id;

	/* Ignore clocks that are always on */
	if (id == P_EBI1_CLK || id == P_EBI1_FIXED_CLK)
		return;

	msm_proc_comm(PCOM_CLKCTL_RPC_DISABLE, &id, NULL);
}

int pc_clk_reset(struct clk *hw, enum clk_reset_action action)
{
	int rc;
	unsigned id = to_clk_pcom(hw)->id;

	if (action == CLK_RESET_ASSERT)
		rc = msm_proc_comm(PCOM_CLKCTL_RPC_RESET_ASSERT, &id, NULL);
	else
		rc = msm_proc_comm(PCOM_CLKCTL_RPC_RESET_DEASSERT, &id, NULL);

	if (rc < 0)
		return rc;
	else
		return (int)id < 0 ? -EINVAL : 0;
}

static int _pc_clk_set_rate(struct clk *hw, unsigned long rate)
{
	/* The rate _might_ be rounded off to the nearest KHz value by the
	 * remote function. So a return value of 0 doesn't necessarily mean
	 * that the exact rate was set successfully.
	 */
	unsigned r = rate;
	unsigned id = to_clk_pcom(hw)->id;
	int rc = msm_proc_comm(PCOM_CLKCTL_RPC_SET_RATE, &id, &r);
	if (rc < 0)
		return rc;
	else
		return (int)id < 0 ? -EINVAL : 0;
}

static int _pc_clk_set_min_rate(struct clk *hw, unsigned long rate)
{
	int rc;
	unsigned id = to_clk_pcom(hw)->id;
	unsigned r = rate;
	rc = msm_proc_comm(PCOM_CLKCTL_RPC_MIN_RATE, &id, &r);
	if (rc < 0)
		return rc;
	else
		return (int)id < 0 ? -EINVAL : 0;
}

static int pc_clk_set_rate(struct clk *hw, unsigned long rate)
{
	if (hw->flags & CLKFLAG_MIN)
		return _pc_clk_set_min_rate(hw, rate);
	else
		return _pc_clk_set_rate(hw, rate);
}

static int pc_clk_set_max_rate(struct clk *hw, unsigned long rate)
{
	unsigned id = to_clk_pcom(hw)->id;
	unsigned r = rate;
	int rc = msm_proc_comm(PCOM_CLKCTL_RPC_MAX_RATE, &id, &r);
	if (rc < 0)
		return rc;
	else
		return (int)id < 0 ? -EINVAL : 0;
}

static int pc_clk_set_flags(struct clk *hw, unsigned flags)
{
	unsigned id = to_clk_pcom(hw)->id;
	int rc = msm_proc_comm(PCOM_CLKCTL_RPC_SET_FLAGS, &id, &flags);
	if (rc < 0)
		return rc;
	else
		return (int)id < 0 ? -EINVAL : 0;
}

static unsigned long pc_clk_get_rate(struct clk *hw)
{
	unsigned id = to_clk_pcom(hw)->id;
	if (msm_proc_comm(PCOM_CLKCTL_RPC_RATE, &id, NULL))
		return 0;
	else
		return id;
}

static int pc_clk_is_enabled(struct clk *hw)
{
	unsigned id = to_clk_pcom(hw)->id;
	if (msm_proc_comm(PCOM_CLKCTL_RPC_ENABLED, &id, NULL))
		return 0;
	else
		return id;
}

static long pc_clk_round_rate(struct clk *hw, unsigned long rate)
{
	/* Not really supported; pc_clk_set_rate() does rounding on it's own. */
	return rate;
}

struct clk_ops clk_ops_pcom = {
	.enable = pc_clk_enable,
	.disable = pc_clk_disable,
	.reset = pc_clk_reset,
	.set_rate = pc_clk_set_rate,
	.set_max_rate = pc_clk_set_max_rate,
	.set_flags = pc_clk_set_flags,
	.get_rate = pc_clk_get_rate,
	.is_enabled = pc_clk_is_enabled,
	.round_rate = pc_clk_round_rate,
};

static int msm_clock_pcom_probe(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver msm_clock_pcom_driver = {
	.probe		= msm_clock_pcom_probe,
	.driver		= {
		.name	= "msm-clock-pcom",
		.owner	= THIS_MODULE,
	},
};
module_platform_driver(msm_clock_pcom_driver);

MODULE_LICENSE("GPL v2");
