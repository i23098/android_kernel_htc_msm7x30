#
# Makefile for the Linux network device drivers.
#

#
# Networking Core Drivers
#
obj-$(CONFIG_BONDING) += bonding/
obj-$(CONFIG_DUMMY) += dummy.o
obj-$(CONFIG_EQUALIZER) += eql.o
obj-$(CONFIG_IFB) += ifb.o
obj-$(CONFIG_MACVLAN) += macvlan.o
obj-$(CONFIG_MACVTAP) += macvtap.o
obj-$(CONFIG_MII) += mii.o
obj-$(CONFIG_MDIO) += mdio.o
obj-$(CONFIG_NET) += Space.o loopback.o
obj-$(CONFIG_NETCONSOLE) += netconsole.o
obj-$(CONFIG_PHYLIB) += phy/
obj-$(CONFIG_RIONET) += rionet.o
obj-$(CONFIG_TUN) += tun.o
obj-$(CONFIG_VETH) += veth.o
obj-$(CONFIG_VIRTIO_NET) += virtio_net.o
obj-$(CONFIG_KINETO_GAN) += kineto_gan.o

#
# Networking Drivers
#
obj-$(CONFIG_ARCNET) += arcnet/
obj-$(CONFIG_DEV_APPLETALK) += appletalk/
obj-$(CONFIG_CAIF) += caif/
obj-$(CONFIG_CAN) += can/
obj-$(CONFIG_ETRAX_ETHERNET) += cris/
obj-$(CONFIG_ETHERNET) += ethernet/
obj-$(CONFIG_FDDI) += fddi/
obj-$(CONFIG_HIPPI) += hippi/
obj-$(CONFIG_HAMRADIO) += hamradio/
obj-$(CONFIG_IRDA) += irda/
obj-$(CONFIG_PLIP) += plip/
obj-$(CONFIG_PPP) += ppp/
obj-$(CONFIG_PPP_ASYNC) += ppp/
obj-$(CONFIG_PPP_BSDCOMP) += ppp/
obj-$(CONFIG_PPP_DEFLATE) += ppp/
obj-$(CONFIG_PPP_MPPE) += ppp/
obj-$(CONFIG_PPP_SYNC_TTY) += ppp/
obj-$(CONFIG_PPPOE) += ppp/
obj-$(CONFIG_PPPOL2TP) += ppp/
obj-$(CONFIG_PPTP) += ppp/
obj-$(CONFIG_PPPOLAC) += ppp/
obj-$(CONFIG_PPPOPNS) += ppp/
obj-$(CONFIG_SLIP) += slip/
obj-$(CONFIG_SLHC) += slip/
obj-$(CONFIG_NET_SB1000) += sb1000.o
obj-$(CONFIG_SUNGEM_PHY) += sungem_phy.o
obj-$(CONFIG_TR) += tokenring/
obj-$(CONFIG_WAN) += wan/
obj-$(CONFIG_WLAN) += wireless/
obj-$(CONFIG_WIMAX) += wimax/

obj-$(CONFIG_VMXNET3) += vmxnet3/
obj-$(CONFIG_XEN_NETDEV_FRONTEND) += xen-netfront.o
obj-$(CONFIG_XEN_NETDEV_BACKEND) += xen-netback/

obj-$(CONFIG_USB_CATC)          += usb/
obj-$(CONFIG_USB_KAWETH)        += usb/
obj-$(CONFIG_USB_PEGASUS)       += usb/
obj-$(CONFIG_USB_RTL8150)       += usb/
obj-$(CONFIG_USB_HSO)		+= usb/
obj-$(CONFIG_USB_USBNET)        += usb/
obj-$(CONFIG_USB_ZD1201)        += usb/
obj-$(CONFIG_USB_IPHETH)        += usb/
obj-$(CONFIG_USB_CDC_PHONET)   += usb/

#
# RMNET
#
ifdef CONFIG_ARCH_MSM7X30_SMD
obj-$(CONFIG_ARCH_MSM7X30_SMD) += 7x30-smd/
else # Else CONFIG_ARCH_MSM7X30_SMD
obj-$(CONFIG_MSM_RMNET) += msm_rmnet.o
ifdef CONFIG_ARCH_MSM8X60
obj-$(CONFIG_ARCH_MSM8X60_LTE) += 8x60-lte/
else
obj-$(CONFIG_MSM_RMNET_SDIO) += msm_rmnet_sdio.o
endif
endif # End CONFIG_ARCH_MSM7X30_LTE
obj-$(CONFIG_MSM_RMNET_BAM) += msm_rmnet_bam.o
