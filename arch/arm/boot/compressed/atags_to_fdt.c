#include <asm/setup.h>
#include <libfdt.h>

static int node_offset(void *fdt, const char *node_path)
{
	int offset = fdt_path_offset(fdt, node_path);
	if (offset == -FDT_ERR_NOTFOUND)
		offset = fdt_add_subnode(fdt, 0, node_path);
	return offset;
}

static int setprop(void *fdt, const char *node_path, const char *property,
		   uint32_t *val_array, int size)
{
	int offset = node_offset(fdt, node_path);
	if (offset < 0)
		return offset;
	return fdt_setprop(fdt, offset, property, val_array, size);
}

static int setprop_string(void *fdt, const char *node_path,
			  const char *property, const char *string)
{
	int offset = node_offset(fdt, node_path);
	if (offset < 0)
		return offset;
	return fdt_setprop_string(fdt, offset, property, string);
}

static int setprop_values(void *fdt, const char *node_path,
			  const char *property, const void *val, int len)
{
	int offset = node_offset(fdt, node_path);
	if (offset < 0)
		return offset;
	return fdt_setprop(fdt, offset, property, val, len);
}

static int setprop_cell(void *fdt, const char *node_path,
			const char *property, uint32_t val)
{
	int offset = node_offset(fdt, node_path);
	if (offset < 0)
		return offset;
	return fdt_setprop_cell(fdt, offset, property, val);
}

/*
 * Convert and fold provided ATAGs into the provided FDT.
 *
 * REturn values:
 *    = 0 -> pretend success
 *    = 1 -> bad ATAG (may retry with another possible ATAG pointer)
 *    < 0 -> error from libfdt
 */
int atags_to_fdt(void *atag_list, void *fdt, int total_space)
{
	struct tag *atag = atag_list;
	uint32_t mem_reg_property[2 * NR_BANKS];
	int memcount = 0;
	int ret;

	/* make sure we've got an aligned pointer */
	if ((u32)atag_list & 0x3)
		return 1;

	/* if we get a DTB here we're done already */
	if (*(u32 *)atag_list == fdt32_to_cpu(FDT_MAGIC))
	       return 0;

	/* validate the ATAG */
	if (atag->hdr.tag != ATAG_CORE ||
	    (atag->hdr.size != tag_size(tag_core) &&
	     atag->hdr.size != 2))
		return 1;

	/* let's give it all the room it could need */
	ret = fdt_open_into(fdt, fdt, total_space);
	if (ret < 0)
		return ret;

	for_each_tag(atag, atag_list) {
		if (atag->hdr.tag == ATAG_CMDLINE) {
			setprop_string(fdt, "/chosen", "bootargs",
					atag->u.cmdline.cmdline);
		} else if (atag->hdr.tag == ATAG_MEM) {
			if (memcount >= sizeof(mem_reg_property)/4)
				continue;
			if (!atag->u.mem.size)
				continue;
			mem_reg_property[memcount++] = cpu_to_fdt32(atag->u.mem.start);
			mem_reg_property[memcount++] = cpu_to_fdt32(atag->u.mem.size);
		} else if (atag->hdr.tag == ATAG_INITRD2) {
			uint32_t initrd_start, initrd_size;
			initrd_start = atag->u.initrd.start;
			initrd_size = atag->u.initrd.size;
			setprop_cell(fdt, "/chosen", "linux,initrd-start",
					initrd_start);
			setprop_cell(fdt, "/chosen", "linux,initrd-end",
					initrd_start + initrd_size);
		} else if (atag->hdr.tag == ATAG_BLUETOOTH) {
			setprop_values(fdt, "/chosen", "linux,bt_mac",
				(unsigned char *)(&atag->u), (atag->hdr.size-2)*sizeof(__u32));
		} else if (atag->hdr.tag == ATAG_MSM_WIFI) {
			setprop_values(fdt, "/chosen", "linux,wifi",
				(unsigned char *)(&atag->u), (atag->hdr.size-2)*sizeof(__u32));
		} else if (atag->hdr.tag == ATAG_MSM_AWB_CAL) {
			setprop_values(fdt, "/chosen", "linux,awb_cal",
				(unsigned char *)(&atag->u), (atag->hdr.size-2)*sizeof(__u32));
		} else if (atag->hdr.tag == ATAG_MFG_GPIO_TABLE) {
			setprop_values(fdt, "/chosen", "linux,gpio_table",
				(unsigned char *)(&atag->u), (atag->hdr.size-2)*sizeof(__u32));
		} else if (atag->hdr.tag == ATAG_MSM_PARTITION) {
			setprop_values(fdt, "/chosen", "linux,msm_partitions",
				(unsigned char *)(&atag->u), (atag->hdr.size-2)*sizeof(__u32));
		} else if (atag->hdr.tag == ATAG_MEMSIZE) {
			setprop_cell(fdt, "/chosen", "linux,memsize",
					atag->u.revision.rev);
		} else if (atag->hdr.tag == ATAG_ALS) {
			setprop_cell(fdt, "/chosen", "linux,als_calibration",
					atag->u.als_kadc.kadc);
		} else if (atag->hdr.tag == ATAG_ENGINEERID) {
			setprop_cell(fdt, "/chosen", "linux,engineerid",
					atag->u.revision.rev);
		} else if (atag->hdr.tag == ATAG_SMI) {
			setprop_cell(fdt, "/chosen", "linux,smi",
					atag->u.mem.size);
		} else if (atag->hdr.tag == ATAG_HWID) {
			setprop_cell(fdt, "/chosen", "linux,hwid",
					atag->u.revision.rev);
		} else if (atag->hdr.tag == ATAG_SKUID) {
			setprop_cell(fdt, "/chosen", "linux,skuid",
					atag->u.revision.rev);
		} else if (atag->hdr.tag == ATAG_HERO_PANEL_TYPE) {
			setprop_cell(fdt, "/chosen", "linux,panel_type",
					atag->u.revision.rev);
		} else if (atag->hdr.tag == ATAG_GS) {
			setprop_cell(fdt, "/chosen", "linux,gs_calibration",
					atag->u.revision.rev);
		} else if (atag->hdr.tag == ATAG_REVISION) {
			__u32 revision[2];
			revision[0] = cpu_to_fdt32(atag->u.revision.rev);
			revision[1] = cpu_to_fdt32(atag->u.revision.rev);
			if (atag->hdr.size > 3) {
				revision[1] = cpu_to_fdt32(atag->u.revision.rev2);
			}
			setprop_values(fdt, "/chosen", "linux,revision",
					revision, sizeof(revision));
		} else if (atag->hdr.tag == ATAG_PS) {
			__u32 ps_settings[2];
			ps_settings[0] = cpu_to_fdt32(atag->u.serialnr.low);
			ps_settings[1] = cpu_to_fdt32(atag->u.serialnr.high);
			setprop_values(fdt, "/chosen", "linux,ps_calibration",
					ps_settings, sizeof(ps_settings));
		}
	}

	if (memcount)
		setprop(fdt, "/memory", "reg", mem_reg_property, 4*memcount);

	return fdt_pack(fdt);
}
