#include <string.h>
#include <math.h>

#include "ui/display.h"
#include "ui/yuvrgb565.h"

bool UIDisplay::Init() {
  if (!screen_.Open()) {
    return false;
  }
  // clear screen
  // TODO(asloane): awesome splash screen
  memset(screen_.GetBuffer(), 0, 320*240*2);
}

void UIDisplay::UpdateBirdseye(const uint8_t *yuv, int w, int h) {
  // show in upper left corner, i guess
  BlitYUVtoRGB565x2(yuv, w, h, 0, 0, screen_.GetBuffer());
  // also want to draw activations and state estimate on here somehow...?
}

void UIDisplay::UpdateEncoders(uint16_t *wheel_pos) {
  uint16_t *buf = screen_.GetBuffer();

  // 5-pixel radius for each wheel,
  //
  // x. .x
  // .. ..
  //
  // .. ..
  // x. .x

  static const uint16_t gray = (3<<11) + (7<<5) + (3);
  for (int i = 0; i < 40; i++) {
    memset(buf + i*320 + 320-40, 0, 40*2);
  }
  for (int i = 0; i < 4; i++) {
    int x = 319 - 5 - 11*(i & 1);
    int y = 5 + 11*(i & 2);
    x += 5 * sin((wheel_pos[i] & 31) * 2 * M_PI / 32.0);
    y += 5 * cos((wheel_pos[i] & 31) * 2 * M_PI / 32.0);
    buf[y*320 + x] = 65535;
  }
}
