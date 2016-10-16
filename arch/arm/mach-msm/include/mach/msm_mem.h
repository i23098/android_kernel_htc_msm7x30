/* arch/arm/mach-msm/include/mach/memory.h
 *
 * Copyright (C) 2007 Google, Inc.
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

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H
#include <linux/types.h>

/* physical offset of RAM */
#define PLAT_PHYS_OFFSET UL(CONFIG_PHYS_OFFSET)

#define MAX_PHYSMEM_BITS 32
#define SECTION_SIZE_BITS 28

/* Maximum number of Memory Regions */
#define MAX_NR_REGIONS 4

/* Certain configurations of MSM7x30 have multiple memory banks.
*  One or more of these banks can contain holes in the memory map as well.
*  These macros define appropriate conversion routines between the physical
*  and virtual address domains for supporting these configurations using
*  SPARSEMEM and a 3G/1G VM split.
*/

#if defined(CONFIG_ARCH_MSM7X30)
#ifndef __ASSEMBLY__
extern unsigned int ebi0_size;

#define EBI0_PHYS_OFFSET PHYS_OFFSET
#define EBI0_PAGE_OFFSET PAGE_OFFSET

#define EBI1_PHYS_OFFSET 0x40000000
#define EBI1_PAGE_OFFSET (EBI0_PAGE_OFFSET + ebi0_size)

#endif
#endif

#ifndef __ASSEMBLY__
void *allocate_contiguous_ebi(unsigned long, unsigned long, int);
unsigned long allocate_contiguous_ebi_nomap(unsigned long, unsigned long);
void clean_and_invalidate_caches(unsigned long, unsigned long, unsigned long);
void clean_caches(unsigned long, unsigned long, unsigned long);
void invalidate_caches(unsigned long, unsigned long, unsigned long);
int msm_get_memory_type_from_name(const char *memtype_name);

#ifdef CONFIG_CACHE_L2X0
extern void l2x0_cache_sync(void);
#define finish_arch_switch(prev)     do { l2x0_cache_sync(); } while (0)
#endif

/*
 * Need a temporary unique variable that no one will ever see to
 * hold the compat string. Line number gives this easily.
 * Need another layer of indirection to get __LINE__ to expand
 * properly as opposed to appending and ending up with
 * __compat___LINE__
 */
#define __CONCAT(a, b)	___CONCAT(a, b)
#define ___CONCAT(a, b)	a ## b

#define EXPORT_COMPAT(com)	\
static char *__CONCAT(__compat_, __LINE__)  __used \
	__attribute((__section__(".exportcompat.init"))) = com

#endif
#endif
