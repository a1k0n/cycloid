#include "ui/drawtext.h"

static const uint8_t FONT[] = {
#include "ft2font.txt"
};

static const uint8_t FONTWIDTH[] = {
#include "ft2fontwidth.txt"
};

void DrawText(const char *str, int x, int y, uint16_t color, uint16_t *buf) {
  while (char c = *str++) {
    uint8_t w = FONTWIDTH[c];
    if (x + w - 1 >= 320) {  // off screen
      return;
    }
    for (int j = 0; j < 10; j++) {
      uint8_t b = FONT[c*10 + j];
      for (int i = 0; i < w - 1; i++) {
        if (b & 128) {
          buf[x + i + (y+j)*320] = color;
        }
        b <<= 1;
      }
    }
    x += w;
  }
}
