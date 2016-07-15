####v3.17
git checkout HEAD -- crypto/sha512_generic.c arch/arm/crypto/aes-armv4.S  arch/arm/crypto/aes_glue.c  arch/arm/crypto/sha1-armv4-large.S arch/arm/crypto/sha1_glue.c

####v3.18
git checkout HEAD -- lib/lzo/lzo1x_decompress_safe.c

# cleanup
find * | grep "\.h" | awk '{print "chmod 664 " $1}' | sh -
find * | grep "\.c" | awk '{print "chmod 664 " $1}' | sh -
find * | grep "Makefile" | awk '{print "chmod 664 " $1}' | sh -
find * | grep "Kconfig" | awk '{print "chmod 664 " $1}' | sh -

# for recheck: lib/genalloc.c include/linux/genalloc.h
