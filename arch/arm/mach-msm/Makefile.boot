# MSM7x30
   zreladdr-$(CONFIG_MACH_SPADE)	:= 0x04008000
params_phys-$(CONFIG_MACH_SPADE)	:= 0x04000100
initrd_phys-$(CONFIG_MACH_SPADE)	:= 0x05000000
   zreladdr-$(CONFIG_MACH_SAGA)	:= 0x04408000
params_phys-$(CONFIG_MACH_SAGA)	:= 0x04400100
initrd_phys-$(CONFIG_MACH_SAGA)	:= 0x05400000


dtb-$(CONFIG_ARCH_MSM8X60) += msm8660-surf.dtb
dtb-$(CONFIG_ARCH_MSM8960) += msm8960-cdp.dtb
