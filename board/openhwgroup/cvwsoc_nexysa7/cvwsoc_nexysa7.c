// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018, Bin Meng <bmeng.cn@gmail.com>
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


#include <configs/openhwgroup_cvwsoc_nexysa7.h>


#include <init.h>


#define CVWSOC_VGA_ADDR             0x100B0000
#define CVWSOC_FB_ADDR              0x87500000
#define CVWSOC_FB_HEIGHT            240
#define CVWSOC_FB_WIDTH             320
#define CVWSOC_FB_SIZE			  (CVWSOC_FB_WIDTH * CVWSOC_FB_HEIGHT * 2)


int vga_fb_init(void)
{

    // RGB565 color bars
    const uint16_t bars[8] = { 0xFFFF, 0xFFE0, 0x07FF, 0x07E0, 0xF81F, 0xF800, 0x001F, 0x0000 };

    volatile uint16_t *fb = (volatile uint16_t *)(uintptr_t)CVWSOC_FB_ADDR;
    uint32_t barw = CVWSOC_FB_WIDTH / 8;
    if (barw == 0) 
        barw = 1;

    for (uint32_t y = 0; y < CVWSOC_FB_HEIGHT; y++) {
        volatile uint16_t *row = fb + y * CVWSOC_FB_WIDTH;
        for (uint32_t x = 0; x < CVWSOC_FB_WIDTH; x++) {
            row[x] = bars[(x / barw) & 7];
        }
    }

    return 0;
}

int vga_init(void)
{
    
    volatile uint32_t *vga = (volatile uint32_t*)(void*)(uintptr_t) CVWSOC_VGA_ADDR;

    // # disable
    vga[0] = 0x00000000;

    // # clkdiv (CHANGE if your input clk isn't 100 MHz)
    vga[1] = 0x00000004;

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
    vga[10] = CVWSOC_FB_ADDR;   // START_ADDR_LOW
    vga[11] = 0x00000000;         // START_ADDR_HIGH

    // # frame size + burst
    vga[12] = CVWSOC_FB_WIDTH * CVWSOC_FB_HEIGHT * 2;   // FRAME_SIZE (320*240*2)
    //vga[13] = 0x0000000F;         // BURST_LEN (16 beats)
    vga[13] = 0x00000004F;         // BURST_LEN (80 beats): value 16 causes underflow and black stripes

    // # enable (polarity bits = 0)
    vga[0] = 0x00000001;

    return 0;
}



int board_init(void) {

    vga_init();
    vga_fb_init();

    return 0;
}


int ft_board_setup(void *fdt, struct bd_info *bd) {
    return 0;
}
