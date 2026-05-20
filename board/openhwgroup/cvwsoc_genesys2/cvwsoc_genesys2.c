// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026, Juan Schroeder <jcschroeder@gmail.com>
 */

#include <dm.h>
#include <dm/ofnode.h>
#include <env.h>
#include <fdtdec.h>
#include <image.h>
#include <log.h>
#include <spl.h>
#include <init.h>
#include <usb.h>
#include <virtio_types.h>
#include <virtio.h>
#include <configs/openhwgroup_cvwsoc_genesys2.h>
#include <../common/cvwsoc_common.h>

int board_init(void) {

    ofnode fb;
    fdt_addr_t fb_base;
    fdt_size_t fb_size;
    u32 ui_width, ui_height;
    int ret;

    fb = ofnode_by_compatible(ofnode_null(), "simple-framebuffer");
    if (ofnode_valid(fb) && ofnode_is_enabled(fb))
    {
        fb_base = ofnode_get_addr_size_index(fb, 0, &fb_size);
        if (fb_base == FDT_ADDR_T_NONE)
            return 0;

        ret = ofnode_read_u32(fb, "width", &ui_width);
        if (ret)
            return 0;

        ret = ofnode_read_u32(fb, "height", &ui_height);
        if (ret)
            return 0;

        vga_init(CVWSOC_VGA_ADDR, fb_base, ui_width, ui_height,
        CVWSOC_VGA_CLK_DIV);
        vga_fb_init(fb_base, ui_width, ui_height);

    }

    return 0;
}


int ft_board_setup(void *fdt, struct bd_info *bd) {
    return 0;
}
