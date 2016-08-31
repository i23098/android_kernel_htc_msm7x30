/* arch/arm/mach-msm/include/mach/msm_camera.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
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

#ifndef __ASM_ARCH_MSM_CAMERA_H
#define __ASM_ARCH_MSM_CAMERA_H

#include <linux/types.h>
#include <linux/leds-pmic8058.h>
#include <linux/spinlock.h>

struct msm_camera_io_ext {
	uint32_t mdcphy;
	uint32_t mdcsz;
	uint32_t appphy;
	uint32_t appsz;
	uint32_t camifpadphy;
	uint32_t camifpadsz;
	uint32_t csiphy;
	uint32_t csisz;
	uint32_t csiirq;
	uint32_t csiphyphy;
	uint32_t csiphysz;
	uint32_t csiphyirq;
	uint32_t ispifphy;
	uint32_t ispifsz;
	uint32_t ispifirq;
};

struct msm_camera_io_clk {
	uint32_t mclk_clk_rate;
	uint32_t vfe_clk_rate;
};

struct msm_cam_expander_info {
	struct i2c_board_info const *board_info;
	int bus_id;
};

struct msm_camera_device_platform_data {
	void (*camera_gpio_on) (void);
	void (*camera_gpio_off)(void);
	struct msm_camera_io_ext ioext;
	struct msm_camera_io_clk ioclk;
	uint8_t csid_core;
	struct msm_bus_scale_pdata *cam_bus_scale_table;
};
enum msm_camera_csi_data_format {
	CSI_8BIT,
	CSI_10BIT,
	CSI_12BIT,
};
struct msm_camera_csi_params {
	enum msm_camera_csi_data_format data_format;
	uint8_t lane_cnt;
	uint8_t lane_assign;
	uint8_t settle_cnt;
	uint8_t dpcm_scheme;
	uint8_t mipi_driving_strength;/*from 0-3*/
	uint8_t hs_impedence;
};

#define MSM_CAMERA_FLASH_NONE 0
#define MSM_CAMERA_FLASH_LED  1

struct msm_camera_sensor_flash_pmic {
	uint8_t num_of_src;
	uint32_t low_current;
	uint32_t high_current;
	enum pmic8058_leds led_src_1;
	enum pmic8058_leds led_src_2;
	int (*pmic_set_current)(enum pmic8058_leds id, unsigned mA);
};

struct msm_camera_sensor_flash_pwm {
	uint32_t freq;
	uint32_t max_load;
	uint32_t low_load;
	uint32_t high_load;
	uint32_t channel;
};

struct pmic8058_leds_platform_data;
struct msm_camera_sensor_flash_current_driver {
	uint32_t low_current;
	uint32_t high_current;
	const struct pmic8058_leds_platform_data *driver_channel;
};

struct msm_camera_sensor_flash_external {
	uint32_t led_en;
	uint32_t led_flash_en;
	struct msm_cam_expander_info *expander_info;
};

struct msm_camera_sensor_flash_src {
	int flash_sr_type;
	int (*camera_flash)(int level);

	union {
		struct msm_camera_sensor_flash_pmic pmic_src;
		struct msm_camera_sensor_flash_pwm pwm_src;
		struct msm_camera_sensor_flash_current_driver
			current_driver_src;
		struct msm_camera_sensor_flash_external
			ext_driver_src;
	} _fsrc;
};

struct msm_camera_sensor_flash_data {
	int flash_type;
	struct msm_camera_sensor_flash_src *flash_src;
};

/* HTC_START linear led 20111011 */
struct camera_led_info {
	uint16_t enable;
	uint16_t low_limit_led_state;
	uint16_t max_led_current_ma;
	uint16_t num_led_est_table;
};

struct camera_led_est {
	uint16_t enable;
	uint16_t led_state;
	uint16_t current_ma;
	uint16_t lumen_value;
	uint16_t min_step;
	uint16_t max_step;
};

struct camera_flash_info {
	struct camera_led_info *led_info;
	struct camera_led_est *led_est_table;
};
/* HTC_END */

struct camera_flash_cfg {
	int num_flash_levels;
	int (*camera_flash)(int level);
	uint16_t low_temp_limit;
	uint16_t low_cap_limit;
	uint8_t postpone_led_mode;
	struct camera_flash_info *flash_info;	/* HTC linear led 20111011 */
};

struct msm_camera_sensor_strobe_flash_data {
	uint8_t flash_trigger;
	uint8_t flash_charge; /* pin for charge */
	uint8_t flash_charge_done;
	uint32_t flash_recharge_duration;
	uint32_t irq;
	spinlock_t spin_lock;
	spinlock_t timer_lock;
	int state;
};

struct msm_camera_rawchip_info {
	int rawchip_reset;
	int rawchip_intr0;
	int rawchip_intr1;
	uint8_t rawchip_spi_freq;
	uint8_t rawchip_mclk_freq;
	int (*camera_rawchip_power_on)(void);
	int (*camera_rawchip_power_off)(void);
	int (*rawchip_gpio_on)(void);
	void (*rawchip_gpio_off)(void);
	int (*rawchip_use_ext_1v2)(void);
};

enum sensor_flip_mirror_info {
	CAMERA_SENSOR_NONE,
	CAMERA_SENSOR_MIRROR,
	CAMERA_SENSOR_FLIP,
	CAMERA_SENSOR_MIRROR_FLIP,
};

struct msm_camera_sensor_platform_info {
	int mount_angle;
	int sensor_reset_enable;
	int sensor_reset;
	int sensor_pwd;
	int vcm_pwd;
	int vcm_enable;
	int privacy_light;
	enum sensor_flip_mirror_info mirror_flip;
	void *privacy_light_info;
};

struct msm_camera_gpio_conf {
	void *cam_gpiomux_conf_tbl;
	uint8_t cam_gpiomux_conf_tbl_size;
	uint16_t *cam_gpio_tbl;
	uint8_t cam_gpio_tbl_size;
};

struct msm_actuator_info {
	struct i2c_board_info const *board_info;
	int bus_id;
	int vcm_pwd;
	int vcm_enable;
};

enum msm_camera_platform{
	MSM_CAMERA_PLTFORM_8X60	= 0,
	MSM_CAMERA_PLTFORM_7X30	= 1,
	MSM_CAMERA_PLTFORM_MAX	= 2,
};

struct msm_camera_sensor_info {
	const char *sensor_name;
	int sensor_reset_enable;
	int sensor_reset;
	int sensor_pwd;
	int vcm_pwd;
	int vcm_enable;
	int mclk;
	int flash_type;
	int need_suspend;
	struct msm_camera_sensor_platform_info *sensor_platform_info;
	struct msm_camera_device_platform_data *pdata;
	struct resource *resource;
	uint8_t num_resources;
	struct msm_camera_sensor_flash_data *flash_data;
	int csi_if;
	struct msm_camera_csi_params csi_params;
	struct msm_camera_sensor_strobe_flash_data *strobe_flash_data;
	char *eeprom_data;
	struct msm_camera_gpio_conf *gpio_conf;
	struct msm_actuator_info *actuator_info;
	int (*camera_power_on)(void);
	int (*camera_power_off)(void);
	int use_rawchip;
	int (*sensor_version)(void);
	int (*camera_main_get_probe)(void);
	void (*camera_main_set_probe)(int);
#if 1 /* HTC to be removed */
	/* HTC++ */
	void(*camera_clk_switch)(void);
	int power_down_disable; /* if close power */
	int full_size_preview; /* if use full-size preview */
	int cam_select_pin; /* for two sensors */
	int mirror_mode; /* for sensor upside down */
	int zero_shutter_mode; /* for doing zero shutter lag on MIPI */
	int sensor_lc_disable; /* for sensor lens correction support */
	int(*camera_pm8058_power)(int); /* for express */
	struct camera_flash_cfg* flash_cfg;
	int gpio_set_value_force; /*true: force to set gpio  */
	int dev_node;
	int camera_platform;
	uint8_t led_high_enabled;
	uint32_t kpi_sensor_start;
	uint32_t kpi_sensor_end;
	uint8_t (*preview_skip_frame)(void);
#endif
};

#endif
