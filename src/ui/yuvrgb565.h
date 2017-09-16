#ifndef UI_YUVRGB565_H_
#define UI_YUVRGB565_H_

// Blit YUV image to RGB565 buffer
// (assumed to be 320x240)
#include <stdint.h>

extern void BlitYUVtoRGB565x2(const uint8_t *yuv, int srcw, int srch,
    int dstx, int dsty, uint16_t *rgb);

#endif  // UI_YUVRGB565_H_
