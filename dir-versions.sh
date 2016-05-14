##########
# drivers/staging/zram/ 3.8
rm -rf drivers/staging/zram/
git checkout HEAD -- drivers/staging/zram/

##########
# drivers/staging/zsmalloc 3.9
rm -rf drivers/staging/zsmalloc
git checkout HEAD -- drivers/staging/zsmalloc

####v3.17
git checkout HEAD -- crypto/sha512_generic.c arch/arm/crypto/aes-armv4.S  arch/arm/crypto/aes_glue.c  arch/arm/crypto/sha1-armv4-large.S arch/arm/crypto/sha1_glue.c | wc -l

# cleanup
find * | grep "\.h" | awk '{print "chmod 664 " $1}' | sh -
find * | grep "\.c" | awk '{print "chmod 664 " $1}' | sh -
find * | grep "Makefile" | awk '{print "chmod 664 " $1}' | sh -
find * | grep "Kconfig" | awk '{print "chmod 664 " $1}' | sh -

