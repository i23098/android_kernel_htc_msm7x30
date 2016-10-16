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


#include <asm/setup.h>
#include <asm/errno.h>
#include <asm/sizes.h>
#include <linux/mutex.h>
#include <mach/msm_memtypes.h>
#include "smd_private.h"

#if defined(CONFIG_ARCH_MSM8960)
#include "rpm_resources.h"
#endif

static struct mem_region_t {
	u64 start;
	u64 size;
	/* reserved for future use */
	u64 num_partitions;
	int state;
	int mask;
	struct mutex state_mutex;
} mem_regions[MAX_NR_REGIONS];

static unsigned int nr_mem_regions;

enum {
	STATE_POWER_DOWN = 0x0,
	STATE_ACTIVE = 0x2,
	STATE_DEFAULT = STATE_ACTIVE
};

static int default_mask = ~0x0;

unsigned int get_num_memory_banks(void)
{
	return nr_mem_regions;
}

unsigned int get_memory_bank_size(unsigned int id)
{
	BUG_ON(id >= nr_mem_regions);
	return mem_regions[id].size;
}

unsigned int get_memory_bank_start(unsigned int id)
{
	BUG_ON(id >= nr_mem_regions);
	return mem_regions[id].start;
}
