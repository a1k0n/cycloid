#include "ui/yuvrgb565.h"

// YUV -> RGB565 transformation matrix
//    [7968,      0,  9082]
//    [16192, -6388, -9399]  >> 16
//    [7968,  16190,     0]

// warning: no bounds checking
void BlitYUVtoRGB565x2(const uint8_t *yuv, int srcw, int srch,
    int dstx, int dsty, uint16_t *rgb) {
  rgb += dstx + (dsty * 320);
  for (int j = 0; j < srch; j++) {
    for (int i = 0; i < srcw; i++) {
      int32_t y = (*yuv++);
      int32_t u = (*yuv++) - 128;
      int32_t v = (*yuv++) - 128;
      int32_t r = (7968 * y + 9082 * v) >> 16;
      r = r < 0 ? 0 : r > 31 ? 31 : r;
      int32_t g = (16192 * y - 6388 * u - 9399 * v) >> 16;
      g = g < 0 ? 0 : g > 63 ? 63 : g;
      int32_t b = (7968 * y + 16190 * u) >> 16;
      b = b < 0 ? 0 : b > 31 ? 31 : b;
      uint16_t rgb565 = (r << 11) + (g << 5) + b;
      rgb[0] = rgb565;
      rgb[1] = rgb565;
      rgb[320] = rgb565;
      rgb[321] = rgb565;
      rgb += 2;
    }
    rgb += (320 - srcw*2) + 320;
  }
}
