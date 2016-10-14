/* arch/arm/mach-msm/htc_awb_cal.c */
/* Code to extract Camera AWB calibration information from ATAG
set up by the bootloader.

Copyright (C) 2008 Google, Inc.
Author: Dmitry Shmidt <dimitrysh@google.com>

This software is licensed under the terms of the GNU General Public
License version 2, as published by the Free Software Foundation, and
may be copied, distributed, and modified under those terms.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <asm/setup.h>
#include <linux/export.h>

/* for outputing file to filesystem : /data/awb_calibration_data_hboot.txt */
#include <linux/fs.h>
#include <linux/syscalls.h>

/* configuration tags specific to msm */
#define AWB_CAL_MAX_SIZE	0x1000U     /* 0x1000 = 4096 bytes */

struct qct_lsc_struct{
	unsigned long int	lsc_verify;
	unsigned long int	lsc_fuseid[4];
	float 			pCalcParam[17*13*4];
	unsigned long int	lsc_checksum;
};

struct qct_awb_lsc_struct{
	unsigned long int caBuff[8];/* AWB Calibartion */
	struct qct_lsc_struct qct_lsc_data;/* LSC Calibration */
	unsigned long int flashcaBuff[8];  //flash_camera
};

static unsigned char cam_awb_ram[AWB_CAL_MAX_SIZE];

unsigned char *get_cam_awb_cal(void)
{
	return cam_awb_ram;
}

EXPORT_SYMBOL(get_cam_awb_cal);

static int __init parse_tag_cam_awb_cal(const struct tag *tag)
{
	unsigned char *dptr = (unsigned char *)(&tag->u);
	size_t size;

	size = min((tag->hdr.size - 2) * sizeof(__u32), AWB_CAL_MAX_SIZE);
	memcpy(cam_awb_ram, dptr, size); /* HTC */

	pr_info("[atag]CamAwb data size = %d\n", tag->hdr.size);
	return 0;
}

__tagtable(ATAG_MSM_AWB_CAL, parse_tag_cam_awb_cal);

void __init early_init_dt_setup_awb_cal(char * data, size_t len) {
	unsigned size;

	size = min(len, AWB_CAL_MAX_SIZE);
	memcpy(cam_awb_ram, data, size); /* HTC */

	pr_info("[dt]CamAwb data size = %d\n", len / 4);
}

static ssize_t awb_calibration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	unsigned char *ptr;

	ptr = get_cam_awb_cal();
	/* fixed : workaround because of defined 8 parameters now */

	ret = sizeof(struct qct_awb_lsc_struct);/* 8*4; */
	memcpy(buf, ptr, ret);

	return ret;
}

static DEVICE_ATTR(awb_cal, 0444, awb_calibration_show, NULL);

static struct kobject *cam_awb_cal;

static int cam_get_awb_cal(void)
{
	int ret ;

	/* Create /sys/android_camera_awb_cal/awb_cal */
	cam_awb_cal = kobject_create_and_add("android_camera_awb_cal", NULL);
	if (cam_awb_cal == NULL) {
		pr_info("cam_get_awb_cal: subsystem_register failed\n");
		ret = -ENOMEM;
		return ret ;
	}

   /* dev_attr_[register_name]<== DEVICE_ATTR(awb_cal, 0444,
   awb_calibration_show, NULL); */
	ret = sysfs_create_file(cam_awb_cal, &dev_attr_awb_cal.attr);
	if (ret) {
		pr_info("cam_get_awb_cal:: sysfs_create_file failed\n");
		kobject_del(cam_awb_cal);
	}
	return 0 ;
}

late_initcall(cam_get_awb_cal);
