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

#include <configs/openhwgroup_cvwsoc_common.h>
#include "cvwsoc_common.h"


#include <stdint.h>
#include <stddef.h>

const axi_vga_mode_t vga_mode_320x240_in_640x480_60mhz = {

    .clkdiv = 2,

    .hvis  = 320,
    .hfp   = 16,
    .hsync = 96,
    .hbp   = 520,

    .vvis  = 240,
    .vfp   = 10,
    .vsync = 2,
    .vbp   = 273,

    .fb_width  = 320,
    .fb_height = 240,
    .burst_len = 0x4f,
    .ctrl      = 0x00000001,
};


const axi_vga_mode_t vga_mode_320x240_in_640x480_81mhz = {
    .clkdiv = 3,

    .hvis = 320,
    .hfp = 16,
    .hsync = 96,
    .hbp = 428,

    .vvis = 240,
    .vfp = 10,
    .vsync = 2,
    .vbp = 273,

    .fb_width = 320,
    .fb_height = 240,

    .burst_len = 0x4f,
    .ctrl = 0x00000001
};

const axi_vga_mode_t vga_mode_320x240_in_640x480_100mhz = {
    .clkdiv = 4,

    .hvis = 320,
    .hfp = 16,
    .hsync = 96,
    .hbp = 368,

    .vvis = 240,
    .vfp = 10,
    .vsync = 2,
    .vbp = 273,

    .fb_width = 320,
    .fb_height = 240,

    .burst_len = 0x4f,
    .ctrl = 0x00000001
};

const axi_vga_mode_t vga_mode_320x240_in_640x480_200mhz = {
    .clkdiv = 8,

    .hvis = 320,
    .hfp = 16,
    .hsync = 96,
    .hbp = 368,

    .vvis = 240,
    .vfp = 10,
    .vsync = 2,
    .vbp = 273,

    .fb_width = 320,
    .fb_height = 240,

    .burst_len = 0x4f,
    .ctrl = 0x00000001
};

int vga_fb_init(const uint64_t uiFBBaseAddr,
                const uint32_t uiFbWidth,
                const uint32_t uiFbHeight)
{
    const uint16_t bars[8] = {
        0xFFFF,
        0xFFE0,
        0x07FF,
        0x07E0,
        0xF81F,
        0xF800,
        0x001F,
        0x0000
    };

    volatile uint16_t *fb = (volatile uint16_t *)(uintptr_t)uiFBBaseAddr;
    uint32_t barw = uiFbWidth / 8;
    if (barw == 0) {
        barw = 1;
    }

    for (uint32_t y = 0; y < uiFbHeight; y++) {
        volatile uint16_t *row = fb + y * uiFbWidth;
        for (uint32_t x = 0; x < uiFbWidth; x++) {
            row[x] = bars[(x / barw) & 7];
        }
    }

    return 0;
}

int vga_init_mode(const uint32_t uiVGABaseAddr,
                  const uint64_t uiFBBaseAddr,
                  const axi_vga_mode_t *mode)
{
    volatile uint32_t *vga;

    if (mode == NULL) {
        return -1;
    }

    vga = (volatile uint32_t *)(void *)(uintptr_t)uiVGABaseAddr;

    vga[0] = 0x00000000;

    vga[1] = mode->clkdiv;

    vga[2] = mode->hvis;
    vga[3] = mode->hfp;
    vga[4] = mode->hsync;
    vga[5] = mode->hbp;

    vga[6] = mode->vvis;
    vga[7] = mode->vfp;
    vga[8] = mode->vsync;
    vga[9] = mode->vbp;

    vga[10] = (uint32_t)(uiFBBaseAddr & 0xffffffffu);
    vga[11] = (uint32_t)(uiFBBaseAddr >> 32);

    vga[12] = mode->fb_width * mode->fb_height * 2u;
    vga[13] = mode->burst_len;

    vga[0] = mode->ctrl;

    return 0;
}

int vga_init(const uint32_t uiVGABaseAddr,
             const uint64_t uiFBBaseAddr,
             const axi_vga_mode_t *mode)
{
    int ret;

    ret = vga_fb_init(uiFBBaseAddr, mode->fb_width, mode->fb_height);

    if (ret != 0) {
        return ret;
    }

    return vga_init_mode(uiVGABaseAddr, uiFBBaseAddr, mode);
}
