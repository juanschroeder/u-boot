#ifndef __CONFIG_H
#define __CONFIG_H

#include <linux/sizes.h>

//#define CONFIG_SYS_SDRAM_BASE		0x80000000
#define CFG_SYS_SDRAM_BASE		0x80000000
#define CONFIG_SYS_INIT_SP_ADDR		(CFG_SYS_SDRAM_BASE + SZ_2M)


#define CFG_EXTRA_ENV_SETTINGS \
	"fdt_addr=0x87000000\0" \
	"kernel_addr=0x80400000\0" \
	"image=boot/Image\0" \
	"mmcdev=0\0" \
	"mmcpart=4\0"

#endif
