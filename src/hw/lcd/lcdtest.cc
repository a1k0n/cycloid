#include "hw/lcd/fbdev.h"

int main() {
  LCDScreen screen;

  if (!screen.Open()) {
    return -1;
  }

  uint16_t *buf = screen.GetBuffer();
  for (int j = 0; j < 240; j++) {
    for (int i = 0; i < 320; i++) {
      buf[j*320 + i] = (((i>>4) + (j>>4)) & 1) ? 65535 : 0;
    }
  }

  screen.Close();
}
