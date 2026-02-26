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


int vga_fb_init(const uint64_t uiFBBaseAddr, const uint32_t uiFbWidth, const uint32_t uiFBHeight)
{

    // RGB565 color bars
    const uint16_t bars[8] = { 0xFFFF, 0xFFE0, 0x07FF, 0x07E0, 0xF81F, 0xF800, 0x001F, 0x0000 };

    volatile uint16_t *fb = (volatile uint16_t *)(uintptr_t)CVWSOC_FB_ADDR;
    uint32_t barw = uiFbWidth / 8;
    if (barw == 0) 
        barw = 1;

    for (uint32_t y = 0; y < uiFBHeight; y++) {
        volatile uint16_t *row = fb + y * uiFbWidth;
        for (uint32_t x = 0; x < uiFbWidth; x++) {
            row[x] = bars[(x / barw) & 7];
        }
    }

    return 0;
}

int vga_init(const uint32_t uiVGABaseAddr, const uint64_t uiFBBaseAddr,
    const uint32_t uiFbWidth, const uint32_t uiFBHeight,
    const uint32_t uiClkDiv)
{
    
    volatile uint32_t *vga = (volatile uint32_t*)(void*)(uintptr_t) uiVGABaseAddr;

    // # disable
    vga[0] = 0x00000000;

    // # clkdiv (CHANGE depending on BUSCLK )
    vga[1] = uiClkDiv;

    // # 320x240
    vga[2] = 320;   // HVIS
    vga[3] = 16;    // HFP
    vga[4] = 96;    // HSYNC
    vga[5] = 368;   // HBP
    vga[6] = 240;   // VVIS
    vga[7] = 10;    // VFP
    vga[8] = 2;     // VSYNC
    vga[9] = 273;   // VBP

    // # framebuffer base
    vga[10] = uiFBBaseAddr;   // START_ADDR_LOW
    vga[11] = 0x00000000;         // START_ADDR_HIGH

    // # frame size + burst
    vga[12] = uiFbWidth * uiFBHeight * 2;   // FRAME_SIZE (320*240*2)
    //vga[13] = 0x0000000F;         // BURST_LEN (16 beats)
    vga[13] = 0x00000004F;         // BURST_LEN (80 beats): value 16 causes underflow and black stripes

    // # enable (polarity bits = 0)
    vga[0] = 0x00000001;

    return 0;
}
