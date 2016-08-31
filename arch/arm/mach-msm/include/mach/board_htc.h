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
#include <mach/msm_camera.h>
#include <asm/setup.h>
#include <linux/input.h>
#include <linux/usb.h>
#include <linux/clkdev.h>
#include <linux/of_platform.h>
#if defined(CONFIG_ARCH_MSM7X30)
#include <linux/msm_ssbi_7x30.h>
#else
#include <linux/msm_ssbi.h>
#endif
#include <mach/msm_bus.h>

struct snd_endpoint {
	int id;
	const char *name;
};

struct msm_snd_endpoints {
	struct snd_endpoint *endpoints;
	unsigned num;
};

#define MSM_MAX_DEC_CNT 14
/* 7k target ADSP information */
/* Bit 23:0, for codec identification like mp3, wav etc *
 * Bit 27:24, for mode identification like tunnel, non tunnel*
 * bit 31:28, for operation support like DM, DMA */
enum msm_adspdec_concurrency {
	MSM_ADSP_CODEC_WAV = 0,
	MSM_ADSP_CODEC_ADPCM = 1,
	MSM_ADSP_CODEC_MP3 = 2,
	MSM_ADSP_CODEC_REALAUDIO = 3,
	MSM_ADSP_CODEC_WMA = 4,
	MSM_ADSP_CODEC_AAC = 5,
	MSM_ADSP_CODEC_RESERVED = 6,
	MSM_ADSP_CODEC_MIDI = 7,
	MSM_ADSP_CODEC_YADPCM = 8,
	MSM_ADSP_CODEC_QCELP = 9,
	MSM_ADSP_CODEC_AMRNB = 10,
	MSM_ADSP_CODEC_AMRWB = 11,
	MSM_ADSP_CODEC_EVRC = 12,
	MSM_ADSP_CODEC_WMAPRO = 13,
	MSM_ADSP_MODE_TUNNEL = 24,
	MSM_ADSP_MODE_NONTUNNEL = 25,
	MSM_ADSP_MODE_LP = 26,
	MSM_ADSP_OP_DMA = 28,
	MSM_ADSP_OP_DM = 29,
};

struct msm_adspdec_info {
	const char *module_name;
	unsigned module_queueid;
	int module_decid; /* objid */
	unsigned nr_codec_support;
};

/* Carries information about number codec
 * supported if same codec or different codecs
 */
struct dec_instance_table {
	uint8_t max_instances_same_dec;
	uint8_t max_instances_diff_dec;
};

struct msm_adspdec_database {
	unsigned num_dec;
	unsigned num_concurrency_support;
	unsigned int *dec_concurrency_table; /* Bit masked entry to *
					      *	represents codec, mode etc */
	struct msm_adspdec_info  *dec_info_list;
	struct dec_instance_table *dec_instance_list;
};

enum msm_mdp_hw_revision {
	MDP_REV_20 = 1,
	MDP_REV_22,
	MDP_REV_30,
	MDP_REV_303,
	MDP_REV_31,
	MDP_REV_40,
	MDP_REV_41,
	MDP_REV_42,
	MDP_REV_43,
	MDP_REV_44,
};

struct msm_panel_common_pdata {
	uintptr_t hw_revision_addr;
	int gpio;
	bool bl_lock;
	spinlock_t bl_spinlock;
	int (*backlight_level)(int level, int max, int min);
	int (*pmic_backlight)(int level);
	int (*rotate_panel)(void);
	int (*backlight) (int level, int mode);
	int (*panel_num)(void);
	void (*panel_config_gpio)(int);
	int (*vga_switch)(int select_vga);
	int *gpio_num;
	u32 mdp_max_clk;
	u32 mdp_max_bw;
	u32 mdp_bw_ab_factor;
	u32 mdp_bw_ib_factor;
	int mdp_rev;
	u32 ov0_wb_size;  /* overlay0 writeback size */
	u32 ov1_wb_size;  /* overlay1 writeback size */
	u32 mem_hid;
	char cont_splash_enabled;
	u32 splash_screen_addr;
	u32 splash_screen_size;
	char mdp_iommu_split_domain;
};

struct lcdc_platform_data {
	int (*lcdc_gpio_config)(int on);
	int (*lcdc_power_save)(int);
	unsigned int (*lcdc_get_clk)(void);
};

struct tvenc_platform_data {
	int poll;
	int (*pm_vid_en)(int on);
};

struct mddi_platform_data {
	int (*mddi_power_save)(int on);
	int (*mddi_sel_clk)(u32 *clk_rate);
	int (*mddi_client_power)(u32 client_id);
};

struct mipi_dsi_platform_data {
	int vsync_gpio;
	int (*dsi_power_save)(int on);
	int (*esd_fixup)(uint32_t mfd_data);
	int (*dsi_client_reset)(void);
	int (*get_lane_config)(void);
	int target_type;
};

enum mipi_dsi_3d_ctrl {
	FPGA_EBI2_INTF,
	FPGA_SPI_INTF,
};

#ifndef CONFIG_ARCH_MSM8X60
/* DSI PHY configuration */
struct mipi_dsi_phy_ctrl {
	uint32_t regulator[5];
	uint32_t timing[12];
	uint32_t ctrl[4];
	uint32_t strength[4];
	uint32_t pll[21];
};
#endif

struct mipi_dsi_panel_platform_data {
	int fpga_ctrl_mode;
	int fpga_3d_config_addr;
	int *gpio;
	struct mipi_dsi_phy_ctrl *phy_ctrl_settings;
};

#define PANEL_NAME_MAX_LEN 50
struct msm_fb_platform_data {
	int (*detect_client)(const char *name);
	int mddi_prescan;
	unsigned char ext_resolution;
	int (*allow_set_offset)(void);
	char prim_panel_name[PANEL_NAME_MAX_LEN];
	char ext_panel_name[PANEL_NAME_MAX_LEN];
};

struct msm_hdmi_platform_data {
	int irq;
	int (*cable_detect)(int insert);
	int (*comm_power)(int on, int show);
	int (*enable_5v)(int on);
	int (*core_power)(int on, int show);
	int (*cec_power)(int on);
	int (*init_irq)(void);
	bool (*check_hdcp_hw_support)(void);
};

struct msm_i2c_platform_data {
	int clk_freq;
	uint32_t rmutex;
	const char *rsl_id;
	uint32_t pm_lat;
	int pri_clk;
	int pri_dat;
	int aux_clk;
	int aux_dat;
	const char *clk;
	const char *pclk;
	int src_clk_rate;
	int use_gsbi_shared_mode;
	void (*msm_i2c_config_gpio)(int iface, int config_type);
	int share_uart_flag;
};

struct msm_i2c_ssbi_platform_data {
	const char *rsl_id;
	enum msm_ssbi_controller_type controller_type;
};

struct msm_vidc_platform_data {
	int memtype;
	u32 enable_ion;
	int disable_dmx;
	int disable_fullhd;
	u32 cp_enabled;
	int disable_turbo;
	int cont_mode_dpb_count;
	int memtype_pmem;
};

#define SHIP_BUILD	0
#define MFG_BUILD	1
#define ENG_BUILD	2

struct msm_mmc_platform_data;
int  msm_add_sdcc(unsigned int controller,
		struct msm_mmc_platform_data *plat);

struct msm_usb_host_platform_data;
int  msm_add_host(unsigned int host,
		struct msm_usb_host_platform_data *plat);

#ifdef CONFIG_MSM_RMT_STORAGE_SERVER
struct shared_ramfs_entry {
	uint32_t client_id;   	/* Client id to uniquely identify a client */
	uint32_t base_addr;	/* Base address of shared RAMFS memory */
	uint32_t size;		/* Size of the shared RAMFS memory */
	uint32_t server_status;	/* This will be initialized to 1 when
				   remote storage RPC server is available */
};
struct shared_ramfs_table {
	uint32_t magic_id;  	/* Identify RAMFS details in SMEM */
	uint32_t version;	/* Version of shared_ramfs_table */
	uint32_t entries;	/* Total number of valid entries   */
	struct shared_ramfs_entry ramfs_entry[3];	/* List all entries */
};

int __init rmt_storage_add_ramfs(void);
#endif

int board_mfg_mode(void);
unsigned int board_get_skuid(void);
unsigned int board_get_engineerid(void);
unsigned int board_get_skuid(void);
unsigned int board_get_smi_sz(void);
unsigned int board_get_memsize(void);
int board_build_flag(void);
int __init parse_tag_ddr_id(const struct tag *tags);
int __init parse_tag_cam(const struct tag *tag);
void msm_snddev_init(void);
void msm_snddev_init_timpani(void);
void msm_snddev_poweramp_on(void);
void msm_snddev_poweramp_off(void);
void msm_snddev_hsed_voltage_on(void);
void msm_snddev_hsed_voltage_off(void);
void msm_snddev_tx_route_config(void);
void msm_snddev_tx_route_deconfig(void);

extern unsigned int msm_shared_ram_phys; /* defined in arch/arm/mach-msm/io.c */

int emmc_partition_open(struct inode *inode, struct file *file);

extern int dying_processors_open(struct inode *inode, struct file *file);

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
