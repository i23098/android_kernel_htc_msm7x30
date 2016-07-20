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

#ifndef _ARCH_ARM_MACH_MSM_SMSM_H_
#define _ARCH_ARM_MACH_MSM_SMSM_H_

void smsm_print_sleep_info(uint32_t sleep_delay, uint32_t sleep_limit,
	uint32_t irq_mask, uint32_t wakeup_reason, uint32_t pending_irqs);
int smsm_set_sleep_limit(uint32_t sleep_limit);

/* ========================================================================*/
#if defined(CONFIG_MSM_SMD_PKG4)
struct smsm_interrupt_info {
	uint32_t aArm_en_mask;
	uint32_t aArm_interrupts_pending;
	uint32_t aArm_wakeup_reason;
	uint32_t aArm_rpc_prog;
	uint32_t aArm_rpc_proc;
	char aArm_smd_port_name[20];
	uint32_t aArm_gpio_info;
};
#elif defined(CONFIG_MSM_SMD_PKG3)
struct smsm_interrupt_info {
  uint32_t aArm_en_mask;
  uint32_t aArm_interrupts_pending;
  uint32_t aArm_wakeup_reason;
};
#elif !defined(CONFIG_MSM_SMD)
void *smem_alloc(unsigned id, unsigned size)
{
	return NULL;
}
#else
#error No SMD Package Specified; aborting
#endif

/* Add by Andy for HTC pm.c */
/* ========================================================================*/
struct smem_ram_ptn {
	char name[16];
	unsigned start;
	unsigned size;

	/* RAM Partition attribute: READ_ONLY, READWRITE etc.  */
	unsigned attr;

	/* RAM Partition category: EBI0, EBI1, IRAM, IMEM */
	unsigned category;

	/* RAM Partition domain: APPS, MODEM, APPS & MODEM (SHARED) etc. */
	unsigned domain;

	/* RAM Partition type: system, bootloader, appsboot, apps etc. */
	unsigned type;

	/* reserved for future expansion without changing version number */
	unsigned reserved2, reserved3, reserved4, reserved5;
} __attribute__ ((__packed__));


struct smem_ram_ptable {
	#define _SMEM_RAM_PTABLE_MAGIC_1 0x9DA5E0A8
	#define _SMEM_RAM_PTABLE_MAGIC_2 0xAF9EC4E2
	unsigned magic[2];
	unsigned version;
	unsigned reserved1;
	unsigned len;
	struct smem_ram_ptn parts[32];
	unsigned buf;
} __attribute__ ((__packed__));

/* SMEM RAM Partition */
enum {
	DEFAULT_ATTRB = ~0x0,
	READ_ONLY = 0x0,
	READWRITE,
};

enum {
	DEFAULT_CATEGORY = ~0x0,
	SMI = 0x0,
	EBI1,
	EBI2,
	QDSP6,
	IRAM,
	IMEM,
	EBI0_CS0,
	EBI0_CS1,
	EBI1_CS0,
	EBI1_CS1,
	SDRAM = 0xE,
};

enum {
	DEFAULT_DOMAIN = 0x0,
	APPS_DOMAIN,
	MODEM_DOMAIN,
	SHARED_DOMAIN,
};

enum {
	SYS_MEMORY = 1,        /* system memory*/
	BOOT_REGION_MEMORY1,   /* boot loader memory 1*/
	BOOT_REGION_MEMORY2,   /* boot loader memory 2,reserved*/
	APPSBL_MEMORY,         /* apps boot loader memory*/
	APPS_MEMORY,           /* apps  usage memory*/
};

extern spinlock_t smem_lock;

void smd_diag(void);

#if defined(CONFIG_MSM_N_WAY_SMSM)
enum {
	SMSM_APPS_STATE,
	SMSM_MODEM_STATE,
	SMSM_Q6_STATE,
	SMSM_APPS_DEM,
	SMSM_WCNSS_STATE = SMSM_APPS_DEM,
	SMSM_MODEM_DEM,
	SMSM_DSPS_STATE = SMSM_MODEM_DEM,
	SMSM_Q6_DEM,
	SMSM_POWER_MASTER_DEM,
	SMSM_TIME_MASTER_DEM,
	SMSM_NUM_ENTRIES,
};
#else
enum {
	SMSM_APPS_STATE = 1,
	SMSM_MODEM_STATE = 3,
	SMSM_NUM_ENTRIES,
};
#endif

enum {
	SMSM_APPS,
	SMSM_MODEM,
	SMSM_Q6,
	SMSM_WCNSS,
	SMSM_DSPS,
	SMSM_NUM_HOSTS,
};

void *smem_alloc2(unsigned id, unsigned size_in);
int smsm_change_intr_mask(uint32_t smsm_entry,
			  uint32_t clear_mask, uint32_t set_mask);
int smsm_get_intr_mask(uint32_t smsm_entry, uint32_t *intr_mask);
int smsm_state_cb_register(uint32_t smsm_entry, uint32_t mask,
	void (*notify)(void *, uint32_t old_state, uint32_t new_state),
	void *data);
int smsm_state_cb_deregister(uint32_t smsm_entry, uint32_t mask,
	void (*notify)(void *, uint32_t, uint32_t), void *data);
void smsm_reset_modem(unsigned mode);
void smsm_reset_modem_cont(void);
void smd_sleep_exit(void);

#define SMEM_NUM_SMD_STREAM_CHANNELS        64
#define SMEM_NUM_SMD_BLOCK_CHANNELS         64

enum {
	SMEM_APPS_Q6_SMSM = 3,
	SMEM_Q6_APPS_SMSM = 5,
	SMSM_NUM_INTR_MUX = 8,
};

int smsm_check_for_modem_crash(void);

#endif
