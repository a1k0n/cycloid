#include <stdio.h>
#include <string.h>
#include <math.h>

#include "coneslam/imgproc.h"
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
  UpdateStatus("cycloid started", 0x01ff);

  return true;
}

void UIDisplay::UpdateBirdseye(const uint8_t *yuv, int w, int h) {
  // show in upper left corner, i guess
  // also show it upside down with a series of horizontal blits
  for (int j = 0; j < h; j++) {
    BlitYUVtoRGB565x2(yuv + j*w*3, w, 1, 0, 2*(h-j-1), screen_.GetBuffer());
  }
  // also want to draw activations and state estimate on here somehow...?
}

void UIDisplay::UpdateEncoders(uint16_t *wheel_pos) {
  uint16_t *buf = screen_.GetBuffer();

  // 5-pixel radius for each wheel
  // for (int i = 0; i < 40; i++) {
  //   memset(buf + i*320 + 320-40, 0, 40*2);
  // }
  for (int i = 0; i < 4; i++) {
    int x = 317 - 5 - 11*(i & 1);
    int y = 12 + 5 + 11*(i & 2);
    x += 5 * sin((wheel_pos[i] & 31) * 2 * M_PI / 32.0);
    y += 5 * cos((wheel_pos[i] & 31) * 2 * M_PI / 32.0);
    buf[y*320 + x] = 65535;
    buf[y*320 + x+1] = 65535;
    buf[y*320 + x-1] = 65535;
    buf[y*320 + x-2] = 0;
    buf[y*320 + x+320] = 65535;
    buf[y*320 + x+319] = 0;
    buf[y*320 + x-320] = 65535;
    buf[y*320 + x-321] = 0;
  }
}

void UIDisplay::UpdateConeView(const uint8_t *yuv, int ncones, int *conesx) {
  uint16_t *buf = screen_.GetBuffer();

  for (int j = 0; j < 112; j++) {
    int y0 = coneslam::conedetect_vpy/2 - 92 + j;
    const uint8_t *y = yuv + y0*640*2;
    const uint8_t *u = yuv + 640*480 + y0*320;
    const uint8_t *v = yuv + 640*600 + y0*320;
    for (int i = 0; i < 320; i++) {
      buf[j*320 + i] = YUVtoRGB565(y[i*2], u[i], v[i]);
    }
  }

  // draw cyan lines where cones were detected
  for (int i = 0; i < ncones; i++) {
    int x = conesx[i] / 2;
    for (int j = 2; j < 10; j++) {
      buf[j*320 + 90*320 + x] = 0x07ff;
      if (x < 319) {
        buf[j*320 + 90*320 + x+1] = 0x07ff;
      }
      if (x > 0) {
        buf[j*320 + 90*320 + x-1] = 0x07ff;
      }
    }
  }
}

void UIDisplay::UpdateParticleView(const coneslam::Localizer *l) {
  // first determine our offsets and scale; what is the min/max landmark
  // location
  float minx = 0, miny = 0, maxx = 0, maxy = 0;
  for (int i = 0; i < l->NumLandmarks(); i++) {
    const coneslam::Landmark &lm = l->GetLandmarks()[i];
    bool first = i == 0;
    if (first || lm.x < minx) minx = lm.x;
    if (first || lm.y < miny) miny = lm.y;
    if (first || lm.x > maxx) maxx = lm.x;
    if (first || lm.y > maxy) maxy = lm.y;
  }
  // now determine our coordinate system to scale to a 320x112 display with
  // margin on the edges
  float scale = 80.0 / (maxy - miny);
  float y0 = 56 + 0.5 * (maxy + miny) * scale;
  float x0 = 160 - 0.5 * (maxx + minx) * scale;

  static const uint16_t orange = (31<<11) + (30<<5) + (0);
  uint16_t *buf = screen_.GetBuffer();
  for (int i = 0; i < l->NumLandmarks(); i++) {
    const coneslam::Landmark &lm = l->GetLandmarks()[i];
    // draw an orange dot
    int x = x0 + scale * lm.x;
    int y = y0 - scale * lm.y;
    if (x >= 2 && x < 318 && y >= 2 && y < 110) {
      buf[320*y + x - 1] = orange;
      buf[320*y + x] = orange;
      buf[320*y + x + 1] = orange;
      buf[320*y + x + 2] = orange;
      buf[320*y + x + 320] = orange;
      buf[320*y + x + 321] = orange;
      buf[320*y + x - 320] = orange;
      buf[320*y + x - 319] = orange;
    }
  }

  static const uint16_t yellow = (31<<11) + (63<<5) + (0);
  for (int i = 0; i < l->NumParticles(); i++) {
    const coneslam::Particle &p = l->GetParticles()[i];
    int x = x0 + scale * p.x;
    int y = y0 - scale * p.y;
    if (x >= 0 && x < 320 && y >= 0 && y < 112) {
      buf[320*y + x] = yellow;
    }
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

void UIDisplay::UpdateStatus(const char *status, uint16_t color) {
  uint16_t *buf = screen_.GetBuffer();

  memset(buf + 220*320, 0, 20*320*2);
  DrawTextBig(status, 0, 220, color, buf);
}

void UIDisplay::UpdateStateEstimate(float v, float delta, float y,
      float psi, float kappa) {
  // drop this just under the birdseye view
  uint16_t *buf = screen_.GetBuffer();
  memset(buf + 112*320, 0, 10*320*2);
  char strbuf[100];
  snprintf(strbuf, sizeof(strbuf), "v%+0.1f d%+0.1f y%+0.2f, o%+0.2f k%+0.2f",
      v, delta, y, psi, kappa);
  DrawText(strbuf, 0, 112, 0xffff, buf);
}

