###########
# sound - v3.3
rm -rf sound/ include/sound/
git checkout HEAD -- sound/ include/sound/
git add sound/ include/sound/

###########
# usb 3.3
rm -rf drivers/usb include/linux/usb include/linux/usbdevice_fs.h include/linux/usb.h include/linux/usb_usual.h
git checkout HEAD -- drivers/usb include/linux/usb include/linux/usbdevice_fs.h include/linux/usb.h include/linux/usb_usual.h

###########
# of 3.4
rm -rf scripts/dtc drivers/of include/linux/of.h include/linux/of_address.h include/linux/of_fdt.h include/linux/of_gpio.h include/linux/of_irq.h include/linux/of_net.h include/linux/of_pci.h include/linux/of_platform.h
git checkout HEAD -- scripts/dtc drivers/of include/linux/of.h include/linux/of_address.h include/linux/of_fdt.h include/linux/of_gpio.h include/linux/of_irq.h include/linux/of_net.h include/linux/of_pci.h include/linux/of_platform.h

###########
# of 3.4
rm -rf arch/arm/boot/compressed/ scripts/kconfig/
git checkout HEAD -- arch/arm/boot/compressed/ scripts/kconfig/

###########
# dts 3.4
rm -rf arch/arm/boot/dts/
git checkout HEAD -- arch/arm/boot/dts/

##########
# drivers/bluetooth 3.3
rm -rf drivers/bluetooth
git checkout HEAD -- drivers/bluetooth

##########
# drivers/staging/zram/ 3.8
rm -rf drivers/staging/zram/
git checkout HEAD -- drivers/staging/zram/

###########
# fs/nfs 3.3
rm -rf fs/nfs fs/nfsd include/linux/sunrpc include/linux/nfs4.h include/linux/nfs_fs.h include/linux/nfs_page.h include/linux/nfs_xdr.h net/sunrpc
git checkout HEAD -- fs/nfs fs/nfsd include/linux/sunrpc include/linux/nfs4.h include/linux/nfs_fs.h include/linux/nfs_page.h include/linux/nfs_xdr.h net/sunrpc

###########
# kernel/power/ 3.3
rm -rf kernel/power/ drivers/base/power/
git checkout HEAD -- kernel/power/ drivers/base/power/

##########
# /net 3.3
rm -rf include/linux/ieee80211.h include/net/ net/ include/linux/netfilter
git checkout HEAD -- include/linux/ieee80211.h include/net/ net/ include/linux/netfilter

##########
# drivers/net/ 3.3
rm -rf drivers/net/
git checkout HEAD -- drivers/net/

##########
# kernel 3.3
rm -rf kernel/debug kernel/events arch/arm/include/asm/socket.h include/asm-generic/socket.h include/linux/dynamic_queue_limits.h include/linux/errqueue.h include/linux/ethtool.h include/linux/freezer.h include/linux/if_link.h include/linux/inet_diag.h include/linux/neighbour.h include/linux/netdev_features.h include/linux/netdevice.h include/linux/nfs_fs.h include/linux/nfs_fs_sb.h include/linux/nfs_idmap.h include/linux/nfs_xdr.h include/linux/res_counter.h include/linux/skbuff.h include/linux/sock_diag.h include/linux/tcp.h kernel/acct.c kernel/cgroup_freezer.c kernel/cpuset.c kernel/freezer.c kernel/hung_task.c kernel/jump_label.c kernel/kprobes.c kernel/lockdep.c kernel/module.c kernel/pid_namespace.c
git checkout HEAD -- kernel/debug kernel/events arch/arm/include/asm/socket.h include/asm-generic/socket.h include/linux/dynamic_queue_limits.h include/linux/errqueue.h include/linux/ethtool.h include/linux/freezer.h include/linux/if_link.h include/linux/inet_diag.h include/linux/neighbour.h include/linux/netdev_features.h include/linux/netdevice.h include/linux/nfs_fs.h include/linux/nfs_fs_sb.h include/linux/nfs_idmap.h include/linux/nfs_xdr.h include/linux/res_counter.h include/linux/skbuff.h include/linux/sock_diag.h include/linux/tcp.h kernel/acct.c kernel/cgroup_freezer.c kernel/cpuset.c kernel/freezer.c kernel/hung_task.c kernel/jump_label.c kernel/kprobes.c kernel/lockdep.c kernel/module.c kernel/pid_namespace.c

##########
# kernel/rcu* v3.4
rm -rf kernel/rcu.h kernel/rcupdate.c kernel/rcutiny.c kernel/rcutiny_plugin.h kernel/rcutorture.c kernel/rcutree.c kernel/rcutree_plugin.h  kernel/rcutree_trace.c kernel/nsproxy.c include/linux/nsproxy.h
git checkout HEAD -- kernel/rcu.h kernel/rcupdate.c kernel/rcutiny.c kernel/rcutiny_plugin.h kernel/rcutorture.c kernel/rcutree.c kernel/rcutree_plugin.h  kernel/rcutree_trace.c kernel/nsproxy.c include/linux/nsproxy.h include/trace/events/rcu.h include/linux/rcupdate.h

##########
# block v3.4
rm -rf block/blk-cgroup.h block/bsg-lib.c block/ioctl.c block/partition-generic.c block/scsi_ioctl.c fs/partitions block/partitions block/blk-integrity.c block/blk-lib.c block/blk-sysfs.c block/bsg.c block/compat_ioctl.c include/linux/fd.h
git checkout HEAD -- block/blk-cgroup.h block/bsg-lib.c block/ioctl.c block/partition-generic.c block/scsi_ioctl.c block/partitions block/blk-integrity.c block/blk-lib.c block/blk-sysfs.c block/bsg.c block/compat_ioctl.c include/linux/fd.h
# rm -rf block/blk-cgroup.c block/blk-core.c block/blk-exec.c block/blk-flush.c block/blk-ioc.c block/blk-map.c block/blk-merge.c block/blk-settings.c block/blk-softirq.c block/blk-tag.c block/blk-throttle.c block/blk.h block/cfq-iosched.c block/deadline-iosched.c block/elevator.c block/genhd.c block/noop-iosched.c
# git checkout HEAD -- block/blk-cgroup.c block/blk-core.c block/blk-exec.c block/blk-flush.c block/blk-ioc.c block/blk-map.c block/blk-merge.c block/blk-settings.c block/blk-softirq.c block/blk-tag.c block/blk-throttle.c block/blk.h block/cfq-iosched.c block/deadline-iosched.c block/elevator.c block/genhd.c block/noop-iosched.c

##########
# mm/kernel v3.3
rm -rf include/linux/workqueue.h kernel/workqueue.c mm/debug-pagealloc.c mm/failslab.c mm/huge_memory.c mm/hugetlb.c mm/init-mm.c mm/kmemleak.c mm/ksm.c mm/memory-failure.c mm/memory_hotplug.c mm/mempolicy.c mm/mmu_notifier.c mm/nobootmem.c mm/nommu.c mm/percpu-vm.c mm/slab.c mm/slob.c mm/sparse.c
git checkout HEAD -- include/linux/workqueue.h kernel/workqueue.c mm/debug-pagealloc.c mm/failslab.c mm/huge_memory.c mm/hugetlb.c mm/init-mm.c mm/kmemleak.c mm/ksm.c mm/memory-failure.c mm/memory_hotplug.c mm/mempolicy.c mm/mmu_notifier.c mm/nobootmem.c mm/nommu.c mm/percpu-vm.c mm/slab.c mm/slob.c mm/sparse.c

##########
# drivers/regulator drivers/block 3.4
rm -rf include/linux/errno.h include/linux/regulator drivers/regulator drivers/block
git checkout HEAD -- include/linux/errno.h include/linux/regulator drivers/regulator drivers/block

##########
# lib 3.3
rm -rf lib/raid6 lib/mpi lib/pci_iomap.c lib/zlib_deflate lib/xz lib/swiotlb.c lib/string.c lib/smp_processor_id.c lib/sha1.c lib/rwsem-spinlock.c lib/reciprocal_div.c lib/ratelimit.c lib/lcm.c lib/kstrtox.c lib/klist.c lib/iomap.c lib/hexdump.c lib/fault-inject.c lib/dynamic_queue_limits.c lib/dynamic_debug.c lib/dma-debug.c lib/digsig.c lib/devres.c lib/decompress_unlzo.c lib/dec_and_lock.c lib/debugobjects.c lib/crc8.c lib/cpumask.c lib/cordic.c lib/clz_tab.c lib/checksum.c lib/bug.c lib/btree.c lib/atomic64_test.c lib/atomic64.c include/linux/proportions.h
git checkout HEAD -- lib/raid6 lib/mpi lib/pci_iomap.c lib/zlib_deflate lib/xz lib/swiotlb.c lib/string.c lib/smp_processor_id.c lib/sha1.c lib/rwsem-spinlock.c lib/reciprocal_div.c lib/ratelimit.c lib/lcm.c lib/kstrtox.c lib/klist.c lib/iomap.c lib/hexdump.c lib/fault-inject.c lib/dynamic_queue_limits.c lib/dynamic_debug.c lib/dma-debug.c lib/digsig.c lib/devres.c lib/decompress_unlzo.c lib/dec_and_lock.c lib/debugobjects.c lib/crc8.c lib/cpumask.c lib/cordic.c lib/clz_tab.c lib/checksum.c lib/bug.c lib/btree.c lib/atomic64_test.c lib/atomic64.c include/linux/proportions.h

##########
# mm 3.4
rm -rf include/linux/shrinker.h include/linux/smp.h include/linux/vm_event_item.h include/trace/events/vmscan.h mm/backing-dev.c mm/process_vm_access.c mm/slab.c mm/slub.c mm/sparse.c mm/swap.c mm/swap_state.c mm/thrash.c mm/truncate.c mm/util.c
rm -rf arch/arm/include/asm/exec.h include/asm-generic/mman-common.h mm/bounce.c mm/compaction.c mm/dmapool.c mm/fadvise.c mm/huge_memory.c mm/hugetlb.c mm/hwpoison-inject.c mm/ksm.c mm/memory-failure.c mm/mempolicy.c mm/mempool.c mm/mincore.c mm/mlock.c mm/nobootmem.c mm/nommu.c mm/page_cgroup.c
git checkout HEAD -- include/linux/shrinker.h include/linux/smp.h include/linux/vm_event_item.h include/trace/events/vmscan.h mm/backing-dev.c mm/process_vm_access.c mm/slab.c mm/slub.c mm/sparse.c mm/swap.c mm/swap_state.c mm/thrash.c mm/truncate.c mm/util.c
git checkout HEAD -- arch/arm/include/asm/exec.h include/asm-generic/mman-common.h mm/bounce.c mm/compaction.c mm/dmapool.c mm/fadvise.c mm/huge_memory.c mm/hugetlb.c mm/hwpoison-inject.c mm/ksm.c mm/memory-failure.c mm/mempolicy.c mm/mempool.c mm/mincore.c mm/mlock.c mm/nobootmem.c mm/nommu.c mm/page_cgroup.c

####v3.5
git checkout HEAD arch/arm/include/asm/dma-mapping.h arch/arm/mm/dma-mapping.c arch/arm/common/dmabounce.c

####v3.4
git checkout HEAD -- block/blk-core.c block/cfq-iosched.c mm/shmem.c include/linux/filter.h include/linux/mmzone.h include/linux/swap.h lib/crc32.c include/linux/kconfig.h
git checkout HEAD -- drivers/of/gpio.c drivers/of/gpio.c drivers/regulator/dummy.c drivers/regulator/tps65910-regulator.c
git checkout HEAD -- drivers/regulator/twl-regulator.c include/linux/regulator/driver.h include/linux/genalloc.h lib/genalloc.c fs/binfmt_elf.c lib/crc32.c

####v3.3
git checkout HEAD -- include/linux/iocontext.h block/blk-ioc.c lib/kstrtox.c mm/backing-dev.c include/trace/events/writeback.h
git checkout HEAD -- mm/memcontrol.c mm/vmscan.c include/linux/memcontrol.h arch/sh/kernel/idle.c block/bsg-lib.c fs/direct-io.c
git checkout HEAD -- block/blk.h block/blk-tag.c block/blk-throttle.c block/genhd.c block/elevator.c mm/page-writeback.c mm/vmalloc.c
git checkout HEAD -- drivers/base/regmap/regcache.c drivers/base/regmap/regmap.c drivers/staging/iio/adc/ad799x_core.c
