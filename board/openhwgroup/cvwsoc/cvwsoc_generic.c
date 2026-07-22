// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026, Juan Schroeder <jcschroeder@gmail.com>
 */

#include <dm.h>
#include <dm/ofnode.h>
#include <env.h>
#include <fdtdec.h>
#include <log.h>
#include <init.h>
#include <usb.h>
#include <clk.h>
#include <configs/openhwgroup_cvwsoc.h>
#include <../common/cvwsoc_common.h>

struct vga_bus_mode {
    ulong bus_rate;
    const axi_vga_mode_t *mode;
};

static const struct vga_bus_mode vga_bus_modes[] = {
    {
        .bus_rate = 60000000UL,
        .mode = &vga_mode_320x240_in_640x480_60mhz,
    },
    {
        .bus_rate = 81250000UL,
        .mode = &vga_mode_320x240_in_640x480_81mhz,
    },
    {
        .bus_rate = 100000000UL,
        .mode = &vga_mode_320x240_in_640x480_100mhz,
    },
    {
        .bus_rate = 200000000UL,
        .mode = &vga_mode_320x240_in_640x480_200mhz,
    },
};

static int vga_get_bus_rate(ofnode fb, ulong *bus_rate)
{
    struct clk clk;
    ulong rate;
    int ret;

    ret = clk_get_by_index_nodev(fb, 0, &clk);
    if (ret)
        return ret;

    rate = clk_get_rate(&clk);

    clk_release_all(&clk, 1);

    if (IS_ERR_VALUE(rate))
        return (long)rate;

    if (!rate)
        return -EINVAL;

    *bus_rate = rate;

    return 0;
}

static const axi_vga_mode_t *vga_find_mode(ulong bus_rate)
{
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(vga_bus_modes); i++) {
        if (vga_bus_modes[i].bus_rate == bus_rate)
            return vga_bus_modes[i].mode;
    }

    return NULL;
}


int board_init(void) {

    ofnode fb;
    fdt_addr_t fb_base;
    fdt_size_t fb_size;
    u32 ui_width, ui_height;
    ulong bus_rate;
    int ret;
    ofnode busclk;
    const axi_vga_mode_t *vga_mode;

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

        ret = vga_get_bus_rate(fb, &bus_rate);
        if (ret) {
            printf("%s: Failed to obtain VGA bus clock: %d\n",
                __func__, ret);
            return 0;
        }

        vga_mode = vga_find_mode(bus_rate);
        if (!vga_mode) {
            printf("%s: Unsupported VGA bus frequency: %lu Hz\n",
                __func__, bus_rate);
            return 0;
        }

        printf("%s: Initializing VGA at 0x%x with framebuffer at 0x%lx\n",
             __func__, CVWSOC_VGA_ADDR, fb_base);
        vga_init(CVWSOC_VGA_ADDR, fb_base, vga_mode);

    }
    else
    {
        printf("%s: No framebuffer node found.\n", __func__);
    }

    return 0;
}


int ft_board_setup(void *fdt, struct bd_info *bd) {
    return 0;
}
