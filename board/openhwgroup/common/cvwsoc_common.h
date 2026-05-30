#ifndef __CVWSOC_COMMON_CONFIG_H
#define __CVWSOC_COMMON_CONFIG_H


typedef struct {
    uint32_t clkdiv;

    uint32_t hvis;
    uint32_t hfp;
    uint32_t hsync;
    uint32_t hbp;

    uint32_t vvis;
    uint32_t vfp;
    uint32_t vsync;
    uint32_t vbp;

    uint32_t fb_width;
    uint32_t fb_height;

    uint32_t burst_len;
    uint32_t ctrl;
} axi_vga_mode_t;


extern const axi_vga_mode_t vga_mode_320x240_in_640x480_100mhz;
extern const axi_vga_mode_t vga_mode_320x240_in_640x480_200mhz;

int vga_fb_init(const uint64_t uiFBBaseAddr, const uint32_t uiFbWidth, const uint32_t uiFBHeight);
int vga_init(const uint32_t uiVGABaseAddr,
             const uint64_t uiFBBaseAddr,
             const axi_vga_mode_t *mode);


#endif
