#ifndef UI_YUVRGB565_H_
#define UI_YUVRGB565_H_

// Blit YUV image to RGB565 buffer
// (assumed to be 320x240)
#include <stdint.h>

static uint16_t YUVtoRGB565(int32_t y, int32_t u, int32_t v) {
  u -= 128;
  v -= 128;
  int32_t r = (7968 * y + 9082 * v) >> 16;
  r = r < 0 ? 0 : r > 31 ? 31 : r;
  int32_t g = (16192 * y - 6388 * u - 9399 * v) >> 16;
  g = g < 0 ? 0 : g > 63 ? 63 : g;
  int32_t b = (7968 * y + 16190 * u) >> 16;
  b = b < 0 ? 0 : b > 31 ? 31 : b;
  return (r << 11) + (g << 5) + b;
}

extern void BlitYUVtoRGB565x2(const uint8_t *yuv, int srcw, int srch,
    int dstx, int dsty, uint16_t *rgb);

#endif  // UI_YUVRGB565_H_
