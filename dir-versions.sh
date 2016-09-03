####v3.17
git checkout HEAD -- crypto/sha512_generic.c arch/arm/crypto/aes-armv4.S  arch/arm/crypto/aes_glue.c  arch/arm/crypto/sha1-armv4-large.S arch/arm/crypto/sha1_glue.c

####v3.18
git checkout HEAD -- lib/lzo/lzo1x_decompress_safe.c

# for recheck: lib/genalloc.c include/linux/genalloc.h
# use code from 78be3176c4335b8ff3d9625ed3fc571e7d2ba8f4 for reboot chip msm7x30_reset
# rewrite arch/arm/mach-msm/clock-7x30.c:msm_clocks_7x30 to official way
