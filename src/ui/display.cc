#include <stdio.h>
#include <string.h>
#include <math.h>

#include "ui/display.h"
#include "ui/drawtext.h"
#include "ui/yuvrgb565.h"

bool UIDisplay::Init() {
  if (!screen_.Open()) {
    return false;
  }
  // clear screen
  // TODO(asloane): awesome splash screen
  memset(screen_.GetBuffer(), 0, 320*240*2);
  DrawTextBig("cycloid running", 0, 220, 0x01ff, screen_.GetBuffer());

  return true;
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

void UIDisplay::UpdateConfig(const char *configmenu[], int nconfigs,
    int config_item, const int16_t *config_values) {
  // 112x56 * 2 -> 224x112 top left taken by birdseye
  // config will start at y = 112 .. 220 -> 5 lines of big text, 10 lines of
  // small text.
  // let's display 5 items, with the selected one on line 3 (the middle)
  // inverted

  uint16_t *buf = screen_.GetBuffer();

  memset(buf + 112*320, 0, 100*320*2);
  for (int i = 0; i < 5; i++) {
    int configoffset = (config_item - 2 + i) % nconfigs;
    if (configoffset < 0) configoffset += nconfigs;
    const char *configname = configmenu[configoffset];
    int16_t configvalue = config_values[configoffset];
    int16_t absvalue = configvalue < 0 ? -configvalue : configvalue;
    char numbuf[8];  // "-327.67\0"
    snprintf(numbuf, sizeof(numbuf)-1, "%c%d.%02d",
        configvalue < 0 ? '-' : ' ',
        absvalue / 100, absvalue % 100);
    int numwidth = TextWidthBig(numbuf);
    if (i == 2) {
      // invert middle line
      memset(buf + (112 + i*20)*320, 0xff, 20*2*320);
      DrawTextBig(configname, 0, 112 + 20*i, 0, buf);
      DrawTextBig(numbuf, 320-numwidth, 112 + 20*i, 0, buf);
    } else {
      DrawTextBig(configname, 0, 112 + 20*i, 0xffff, buf);
      DrawTextBig(numbuf, 320-numwidth, 112 + 20*i, 0xffff, buf);
    }
  }
}
