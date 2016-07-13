/* arch/arm/mach-msm/htc_wifi_nvs.c
 *
 * Code to extract WiFi calibration information from ATAG set up
 * by the bootloader.
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Dmitry Shmidt <dimitrysh@google.com>
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/export.h>

#include <asm/setup.h>

/* configuration tags specific to msm */
#define NVS_MAX_SIZE	0x800U
#define NVS_LEN_OFFSET	0x0C
#define NVS_DATA_OFFSET	0x40

static unsigned char wifi_nvs_ram[NVS_MAX_SIZE];

unsigned char *get_wifi_nvs_ram( void )
{
	return wifi_nvs_ram;
}
EXPORT_SYMBOL(get_wifi_nvs_ram);

unsigned char *wlan_random_mac(unsigned char *set_mac_addr)
{
    static unsigned char mac_addr[6] = {0, 0, 0, 0, 0, 0};
    if (set_mac_addr != NULL) {
		mac_addr[0] = set_mac_addr[0];
		mac_addr[1] = set_mac_addr[1];
		mac_addr[2] = set_mac_addr[2];
		mac_addr[3] = set_mac_addr[3];
		mac_addr[4] = set_mac_addr[4];
		mac_addr[5] = set_mac_addr[5];
    }
    return mac_addr;
}
EXPORT_SYMBOL(wlan_random_mac);

static int __init parse_tag_msm_wifi(const struct tag *tag)
{
	unsigned char *dptr = (unsigned char *)(&tag->u);
	unsigned size;

	size = min((tag->hdr.size - 2) * sizeof(__u32), NVS_MAX_SIZE);
	memcpy(wifi_nvs_ram, dptr, size);

	pr_info("[atag]WiFi Data size = %d\n", tag->hdr.size);
	return 0;
}

__tagtable(ATAG_MSM_WIFI, parse_tag_msm_wifi);

void __init early_init_dt_setup_msm_wifi_data(char * data, size_t len) {
	unsigned size;

	size = min(len, NVS_MAX_SIZE);
	memcpy(wifi_nvs_ram, data, size);

	pr_info("[dt]WiFi Data size = %d\n", len / 4);
}

static unsigned wifi_get_nvs_size(void)
{
	unsigned char *ptr;
	unsigned len;

	ptr = get_wifi_nvs_ram();
	/* Size in format LE assumed */
	memcpy(&len, ptr + NVS_LEN_OFFSET, sizeof(len));
	len = min(len, (NVS_MAX_SIZE - NVS_DATA_OFFSET));
	return len;
}

static int wifi_calibration_read_proc(struct file *file, char __user *buf,
			size_t len, loff_t *offset)
{
	unsigned char *ptr = get_wifi_nvs_ram();
	return simple_read_from_buffer(buf, len, offset,
				ptr + NVS_DATA_OFFSET, wifi_get_nvs_size());
}

static int wifi_data_read_proc(struct file *file, char __user *buf,
			size_t len, loff_t *offset)
{
	unsigned char *ptr = get_wifi_nvs_ram();
	return simple_read_from_buffer(buf, len, offset,
				ptr, NVS_DATA_OFFSET);
}

static struct file_operations wifi_calibration_fops = {
	.read = wifi_calibration_read_proc,
	.llseek = default_llseek,
};

static struct file_operations wifi_data_fops = {
	.read = wifi_data_read_proc,
	.llseek = default_llseek,
};

static int __init wifi_nvs_init(void)
{
	struct proc_dir_entry *entry;

	entry = proc_create("calibration", 0444, NULL,
				&wifi_calibration_fops);
	if (!entry) {
		pr_err("%s: could not create proc entry for wifi calibration\n",
				__func__);
		return 0;
	}

	entry = proc_create("wifi_data", 0444, NULL,
				&wifi_data_fops);
	if (!entry) {
		pr_err("%s: could not create proc entry for wifi data\n",
				__func__);
		return 0;
	}
	return 0;
}

late_initcall(wifi_nvs_init);
