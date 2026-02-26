#ifndef __CVWSOC_COMMON_CONFIG_H
#define __CVWSOC_COMMON_CONFIG_H

int vga_fb_init(const uint64_t uiFBBaseAddr, const uint32_t uiFbWidth, const uint32_t uiFBHeight);
int vga_init(const uint32_t uiVGABaseAddr, const uint64_t uiFBBaseAddr,
    const uint32_t uiFbWidth, const uint32_t uiFBHeight,
    const uint32_t uiClkDiv);


#endif