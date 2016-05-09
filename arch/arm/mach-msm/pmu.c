/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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

#include <linux/platform_device.h>
#include <asm/pmu.h>
#include <mach/irqs.h>

static struct resource cpu_pmu_resource = {
	.start = INT_ARM11_PM,
	.end = INT_ARM11_PM,
	.flags	= IORESOURCE_IRQ,
};

static struct platform_device cpu_pmu_device = {
	.name		= "arm-pmu",
	.id		= -1,
	.resource	= &cpu_pmu_resource,
	.num_resources	= 1,
};

static struct platform_device *pmu_devices[] = {
	&cpu_pmu_device,
};

static int __init msm_pmu_init(void)
{
	return platform_add_devices(pmu_devices, ARRAY_SIZE(pmu_devices));
}

arch_initcall(msm_pmu_init);
