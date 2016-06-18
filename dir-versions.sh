##########
# drivers/staging/zsmalloc 3.9
rm -rf drivers/staging/zsmalloc
git checkout HEAD -- drivers/staging/zsmalloc kernel/irq/spurious.c

##########
# fs/f2fs 3.11
rm -rf fs/f2fs
git checkout HEAD -- fs/f2fs

####v3.17
git checkout HEAD -- crypto/sha512_generic.c arch/arm/crypto/aes-armv4.S  arch/arm/crypto/aes_glue.c  arch/arm/crypto/sha1-armv4-large.S arch/arm/crypto/sha1_glue.c | wc -l

####v3.18
git checkout HEAD -- lib/lzo/lzo1x_decompress_safe.c

# cleanup
find * | grep "\.h" | awk '{print "chmod 664 " $1}' | sh -
find * | grep "\.c" | awk '{print "chmod 664 " $1}' | sh -
find * | grep "Makefile" | awk '{print "chmod 664 " $1}' | sh -
find * | grep "Kconfig" | awk '{print "chmod 664 " $1}' | sh -

# for recheck: lib/genalloc.c include/linux/genalloc.h
