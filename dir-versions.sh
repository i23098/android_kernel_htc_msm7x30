##########
# drivers/staging/zram/ 3.8
rm -rf drivers/staging/zram/
git checkout HEAD -- drivers/staging/zram/

##########
# drivers/staging/zsmalloc 3.9
rm -rf drivers/staging/zsmalloc
git checkout HEAD -- drivers/staging/zsmalloc

####v3.17
git checkout HEAD -- crypto/sha512_generic.c

####v3.6
git checkout HEAD -- drivers/char/random.c

####v3.5
git checkout HEAD -- arch/arm/include/asm/dma-mapping.h arch/arm/mm/dma-mapping.c arch/arm/common/dmabounce.c
