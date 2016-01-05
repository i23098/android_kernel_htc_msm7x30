###########
#sound - v3.3
rm -rf sound/ include/sound/
git checkout HEAD sound/ include/sound/
git add sound/ include/sound/

###########
#usb 3.3
rm -rf drivers/usb include/linux/usb include/linux/usbdevice_fs.h include/linux/usb.h include/linux/usb_usual.h
git checkout HEAD drivers/usb include/linux/usb include/linux/usbdevice_fs.h include/linux/usb.h include/linux/usb_usual.h

###########
#of 3.4
rm -rf scripts/dtc drivers/of include/linux/of.h include/linux/of_address.h include/linux/of_fdt.h include/linux/of_gpio.h include/linux/of_irq.h include/linux/of_net.h include/linux/of_pci.h include/linux/of_platform.h
git checkout HEAD scripts/dtc drivers/of include/linux/of.h include/linux/of_address.h include/linux/of_fdt.h include/linux/of_gpio.h include/linux/of_irq.h include/linux/of_net.h include/linux/of_pci.h include/linux/of_platform.h

###########
#of 3.4
rm -rf arch/arm/boot/compressed/ scripts/kconfig/
git checkout HEAD arch/arm/boot/compressed/ scripts/kconfig/


###########
#dts 3.4
rm -rf arch/arm/boot/dts/
git checkout HEAD arch/arm/boot/dts/

###########
# pm 3.2
rm -rf drivers/base/power/generic_ops.c drivers/base/power/sysfs.c drivers/base/power/trace.c include/linux/pm.h
git checkout HEAD drivers/base/power/generic_ops.c drivers/base/power/sysfs.c drivers/base/power/trace.c include/linux/pm.h

###########
# fs/ext4 3.2
rm -rf include/trace/events/ext4.h fs/ext4/ fs/btrfs fs/fat fs/xfs fs/cifs fs/exofs fs/gfs2
git checkout HEAD include/trace/events/ext4.h fs/ext4/ fs/btrfs fs/fat fs/xfs fs/cifs fs/exofs fs/gfs2

##########
# /fs/*.c 3.1
rm -rf fs/namei.c fs/splice.c fs/stat.c
git checkout HEAD fs/namei.c fs/splice.c fs/stat.c

##########
# drivers/base/regmap 3.1
rm -rf drivers/base/regmap include/linux/regmap.h
git checkout HEAD drivers/base/regmap include/linux/regmap.h

##########
# drivers/bluetooth 3.3
rm -rf drivers/bluetooth
git checkout HEAD drivers/bluetooth

##########
# drivers/staging/zram/ 3.8
rm -rf drivers/staging/zram/
git checkout HEAD drivers/staging/zram/

###########
# fs/nfs 3.2
rm -rf fs/nfs fs/nfsd include/linux/sunrpc include/linux/nfs4.h include/linux/nfs_fs.h include/linux/nfs_page.h include/linux/nfs_xdr.h net/sunrpc
git checkout HEAD fs/nfs fs/nfsd include/linux/sunrpc include/linux/nfs4.h include/linux/nfs_fs.h include/linux/nfs_page.h include/linux/nfs_xdr.h net/sunrpc

###########
# kernel/power/ 3.2
rm -rf kernel/power/ drivers/base/power/
git checkout HEAD kernel/power/ drivers/base/power/

##########
# /net 3.2
rm -rf include/linux/ieee80211.h include/net/ net/ include/linux/netfilter
git checkout HEAD include/linux/ieee80211.h include/net/ net/ include/linux/netfilter
