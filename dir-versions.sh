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
# drivers/staging/zram/ 3.8
rm -rf drivers/staging/zram/
git checkout HEAD -- drivers/staging/zram/

##########
# drivers/staging/zsmalloc 3.9
rm -rf drivers/staging/zsmalloc
git checkout HEAD -- drivers/staging/zsmalloc

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
# drivers/regulator drivers/block 3.4
rm -rf include/linux/errno.h include/linux/regulator drivers/regulator drivers/block
git checkout HEAD -- include/linux/errno.h include/linux/regulator drivers/regulator drivers/block

##########
# mm 3.4
rm -rf include/linux/shrinker.h include/linux/smp.h include/linux/vm_event_item.h include/trace/events/vmscan.h mm/backing-dev.c mm/process_vm_access.c mm/slab.c mm/slub.c mm/sparse.c mm/swap.c mm/swap_state.c mm/thrash.c mm/truncate.c mm/util.c
rm -rf arch/arm/include/asm/exec.h include/asm-generic/mman-common.h mm/bounce.c mm/compaction.c mm/dmapool.c mm/fadvise.c mm/huge_memory.c mm/hugetlb.c mm/hwpoison-inject.c mm/ksm.c mm/memory-failure.c mm/mempolicy.c mm/mempool.c mm/mincore.c mm/mlock.c mm/nobootmem.c mm/nommu.c mm/page_cgroup.c
git checkout HEAD -- include/linux/shrinker.h include/linux/smp.h include/linux/vm_event_item.h include/trace/events/vmscan.h mm/backing-dev.c mm/process_vm_access.c mm/slab.c mm/slub.c mm/sparse.c mm/swap.c mm/swap_state.c mm/thrash.c mm/truncate.c mm/util.c
git checkout HEAD -- arch/arm/include/asm/exec.h include/asm-generic/mman-common.h mm/bounce.c mm/compaction.c mm/dmapool.c mm/fadvise.c mm/huge_memory.c mm/hugetlb.c mm/hwpoison-inject.c mm/ksm.c mm/memory-failure.c mm/mempolicy.c mm/mempool.c mm/mincore.c mm/mlock.c mm/nobootmem.c mm/nommu.c mm/page_cgroup.c

####v3.17
git checkout HEAD -- crypto/sha512_generic.c

####v3.6
git checkout HEAD -- drivers/char/random.c

####v3.5
git checkout HEAD -- arch/arm/include/asm/dma-mapping.h arch/arm/mm/dma-mapping.c arch/arm/common/dmabounce.c

####v3.4
git checkout HEAD -- block/blk-core.c block/cfq-iosched.c mm/shmem.c include/linux/filter.h include/linux/mmzone.h include/linux/swap.h lib/crc32.c include/linux/kconfig.h
git checkout HEAD -- drivers/of/gpio.c drivers/of/gpio.c drivers/regulator/dummy.c drivers/regulator/tps65910-regulator.c mm/oom_kill.c
git checkout HEAD -- drivers/regulator/twl-regulator.c include/linux/regulator/driver.h include/linux/genalloc.h lib/genalloc.c fs/binfmt_elf.c lib/crc32.c
git checkout HEAD -- fs/ioprio.c
