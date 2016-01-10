###########
# sound - v3.3
rm -rf sound/ include/sound/
git checkout HEAD sound/ include/sound/
git add sound/ include/sound/

###########
# usb 3.3
rm -rf drivers/usb include/linux/usb include/linux/usbdevice_fs.h include/linux/usb.h include/linux/usb_usual.h
git checkout HEAD drivers/usb include/linux/usb include/linux/usbdevice_fs.h include/linux/usb.h include/linux/usb_usual.h

###########
# of 3.4
rm -rf scripts/dtc drivers/of include/linux/of.h include/linux/of_address.h include/linux/of_fdt.h include/linux/of_gpio.h include/linux/of_irq.h include/linux/of_net.h include/linux/of_pci.h include/linux/of_platform.h
git checkout HEAD scripts/dtc drivers/of include/linux/of.h include/linux/of_address.h include/linux/of_fdt.h include/linux/of_gpio.h include/linux/of_irq.h include/linux/of_net.h include/linux/of_pci.h include/linux/of_platform.h

###########
# of 3.4
rm -rf arch/arm/boot/compressed/ scripts/kconfig/
git checkout HEAD arch/arm/boot/compressed/ scripts/kconfig/

###########
# dts 3.4
rm -rf arch/arm/boot/dts/
git checkout HEAD arch/arm/boot/dts/

###########
# pm 3.2
rm -rf drivers/base/power/generic_ops.c drivers/base/power/sysfs.c drivers/base/power/trace.c include/linux/pm.h
git checkout HEAD drivers/base/power/generic_ops.c drivers/base/power/sysfs.c drivers/base/power/trace.c include/linux/pm.h

###########
# fs/ext4 3.2
rm -rf include/trace/events/ext4.h fs/ext4/ fs/btrfs fs/fat fs/xfs fs/cifs fs/exofs fs/gfs2 fs/compat.c
git checkout HEAD include/trace/events/ext4.h fs/ext4/ fs/btrfs fs/fat fs/xfs fs/cifs fs/exofs fs/gfs2 fs/compat.c

##########
# /fs/*.c 3.1
rm -rf fs/namei.c fs/splice.c fs/stat.c
git checkout HEAD fs/namei.c fs/splice.c fs/stat.c

##########
# drivers/base/regmap 3.2
rm -rf drivers/base/regmap include/linux/regmap.h include/trace/events/regmap.h
git checkout HEAD drivers/base/regmap include/linux/regmap.h include/trace/events/regmap.h

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

##########
# drivers/net/ 3.2
rm -rf drivers/net/
git checkout HEAD drivers/net/

##########
# kernel 3.2
rm -rf kernel/debug kernel/events kernel/events kernel/gcov include/linux/timex.h include/linux/clockchips.h include/linux/ring_buffer.h include/linux/stacktrace.h include/linux/trace_clock.h kernel/rcutiny.c kernel/stacktrace.c kernel/irq kernel/time kernel/trace
rm -rf include/linux/module.h include/linux/moduleparam.h include/linux/tracepoint.h include/linux/utsname.h kernel/capability.c kernel/configs.c kernel/dma.c kernel/kexec.c kernel/params.c kernel/sched_autogroup.c kernel/sched_autogroup.h kernel/sched_cpupri.c kernel/sched_cpupri.h kernel/sched_stats.h  kernel/tracepoint.c
rm -rf kernel/async.c kernel/audit.c kernel/audit_tree.c kernel/compat.c kernel/crash_dump.c kernel/cred.c kernel/delayacct.c kernel/futex_compat.c kernel/groups.c kernel/hung_task.c kernel/irq_work.c kernel/kfifo.c kernel/kprobes.c kernel/ksysfs.c kernel/latencytop.c kernel/lockdep_proc.c kernel/mutex-debug.c
rm -rf kernel/notifier.c kernel/padata.c kernel/pid.c kernel/posix-cpu-timers.c kernel/posix-timers.c kernel/profile.c kernel/range.c kernel/relay.c kernel/rtmutex-debug.c kernel/rtmutex-tester.c  kernel/rtmutex.c kernel/rwsem.c kernel/semaphore.c kernel/smp.c kernel/spinlock.c kernel/stop_machine.c kernel/sysctl_check.c kernel/taskstats.c kernel/time.c kernel/utsname_sysctl.c kernel/watchdog.c
rm -rf kernel/auditsc.c arch/arm/kernel/ptrace.c include/linux/audit.h kernel/sysctl_binary.c kernel/sysctl.c arch/arm/include/asm/exception.h kernel/cpu_pm.c kernel/kmod.c kernel/lockdep.c kernel/module.c kernel/mutex.c kernel/mutex.h kernel/softirq.c kernel/srcu.c
git checkout HEAD kernel/debug  kernel/events kernel/events kernel/gcov include/linux/timex.h include/linux/clockchips.h include/linux/ring_buffer.h include/linux/stacktrace.h include/linux/trace_clock.h kernel/rcutiny.c kernel/stacktrace.c kernel/irq kernel/time kernel/trace
git checkout HEAD include/linux/module.h include/linux/moduleparam.h include/linux/tracepoint.h include/linux/utsname.h kernel/capability.c kernel/configs.c kernel/dma.c kernel/kexec.c kernel/params.c kernel/sched_autogroup.c kernel/sched_autogroup.h kernel/sched_cpupri.c kernel/sched_cpupri.h kernel/sched_stats.h  kernel/tracepoint.c
git checkout HEAD kernel/async.c kernel/audit.c kernel/audit_tree.c kernel/compat.c kernel/crash_dump.c kernel/cred.c kernel/delayacct.c kernel/futex_compat.c kernel/groups.c kernel/hung_task.c kernel/irq_work.c kernel/kfifo.c kernel/kprobes.c kernel/ksysfs.c kernel/latencytop.c kernel/lockdep_proc.c kernel/mutex-debug.c
git checkout HEAD kernel/notifier.c kernel/padata.c kernel/pid.c kernel/posix-cpu-timers.c kernel/posix-timers.c kernel/profile.c kernel/range.c kernel/relay.c kernel/rtmutex-debug.c kernel/rtmutex-tester.c  kernel/rtmutex.c kernel/rwsem.c kernel/semaphore.c kernel/smp.c kernel/spinlock.c kernel/stop_machine.c kernel/sysctl_check.c kernel/taskstats.c kernel/time.c kernel/utsname_sysctl.c kernel/watchdog.c
git checkout HEAD kernel/auditsc.c arch/arm/kernel/ptrace.c include/linux/audit.h kernel/sysctl_binary.c kernel/sysctl.c arch/arm/include/asm/exception.h kernel/cpu_pm.c kernel/kmod.c kernel/lockdep.c kernel/module.c kernel/mutex.c kernel/mutex.h kernel/softirq.c kernel/srcu.c

##########
# kernel/rcu* v3.4
rm -rf kernel/rcu.h kernel/rcupdate.c kernel/rcutiny.c kernel/rcutiny_plugin.h kernel/rcutorture.c kernel/rcutree.c kernel/rcutree_plugin.h  kernel/rcutree_trace.c
git checkout HEAD kernel/rcu.h kernel/rcupdate.c kernel/rcutiny.c kernel/rcutiny_plugin.h kernel/rcutorture.c kernel/rcutree.c kernel/rcutree_plugin.h  kernel/rcutree_trace.c

##########
# block v3.4
rm -rf block/blk-cgroup.h block/bsg-lib.c block/ioctl.c block/partition-generic.c block/scsi_ioctl.c
git checkout HEAD block/blk-cgroup.h block/bsg-lib.c block/ioctl.c block/partition-generic.c block/scsi_ioctl.c
# rm -rf block/blk-cgroup.c block/blk-core.c block/blk-exec.c block/blk-flush.c block/blk-ioc.c block/blk-map.c block/blk-merge.c block/blk-settings.c block/blk-softirq.c block/blk-sysfs.c block/blk-tag.c block/blk-throttle.c block/blk.h block/bsg.c block/cfq-iosched.c block/compat_ioctl.c block/deadline-iosched.c block/elevator.c block/genhd.c block/noop-iosched.c
# git checkout HEAD block/blk-cgroup.c block/blk-core.c block/blk-exec.c block/blk-flush.c block/blk-ioc.c block/blk-map.c block/blk-merge.c block/blk-settings.c block/blk-softirq.c block/blk-sysfs.c block/blk-tag.c block/blk-throttle.c block/blk.h block/bsg.c block/cfq-iosched.c block/compat_ioctl.c block/deadline-iosched.c block/elevator.c block/genhd.c block/noop-iosched.c 
