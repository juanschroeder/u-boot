#ifndef __OPENHWGROUP_CVWSOC_GENESYS2_CONFIG_H
#define __OPENHWGROUP_CVWSOC_GENESYS2_CONFIG_H

#include <linux/sizes.h>
#include "openhwgroup_cvwsoc_common.h"


#define CVWSOC_VGA_ADDR             0x100B0000
#define CVWSOC_VGA_CLK_DIV          0x7
#define CVWSOC_FB_ADDR              0xBFF00000
#define CVWSOC_FB_HEIGHT            240
#define CVWSOC_FB_WIDTH             320
#define CVWSOC_FB_SIZE			  (CVWSOC_FB_WIDTH * CVWSOC_FB_HEIGHT * 2)


#endif
