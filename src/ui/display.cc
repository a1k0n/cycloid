#include "ui/display.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "lens/fisheye.h"
#include "ui/drawtext.h"
#include "ui/yuvrgb565.h"

bool UIDisplay::Init() {
  mode_ = TRACKMAP;

  if (!screen_.Open()) {
    return false;
  }
  // clear screen
  // TODO(asloane): awesome splash screen
  memset(screen_.GetBuffer(), 0, 320 * 240 * 2);
  UpdateStatus("cycloid started", 0x01ff);

  backgroundyuv_ = new uint8_t[76800];
  memset(backgroundyuv_, 0, 76800);
  FILE *fp = fopen("map.yuv420", "rb");
  if (!fp) {
    perror("no background map loaded; map.yuv420");
  } else {
    fread(backgroundyuv_, 1, 76800, fp);
    fclose(fp);
  }
  frontremap_ = NULL;
  return true;
}

void UIDisplay::InitCamera(const FisheyeLens &lens, float camtilt) {
  frontremap_ = new uint16_t[2 * 320 * 120];
  uint16_t *remapdata = frontremap_;
  float Rc = cos(camtilt - M_PI / 2);
  float Rs = sin(camtilt - M_PI / 2);
  for (int j = 0; j < 120; j++) {
    for (int i = 0; i < 320; i++) {
      // reverse-project i, j into world space through camera K
      // -120   0 160
      //    0 120  71
      //    0   0   1
      float x = (i - 160.0f) / -120.0f;
      float y = (j - 71.0f) / 120.0f;
      // un-rotate into camera frame by camera tilt about y axis
      float u = y * Rc - Rs;
      float v = x;
      float z = y * Rs + Rc;
      u /= fabsf(z);
      v /= fabsf(z);
      // swap u and v axes afterward and distort to image space
      lens.DistortPoint(u, v, z > 0 ? 1 : -1, &x, &y);
      // scale by 64 and write into table
      *remapdata++ = (uint16_t)64.0f * x;
      *remapdata++ = (uint16_t)64.0f * y;
    }
  }
}

#if 0
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
    int y0 = coneslam::GetVpy()/2 - 92 + j;
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
  float scale = 60.0 / (maxy - miny);
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
#endif

void UIDisplay::UpdateCameraView(
    const uint8_t *yuv, const std::vector<std::pair<float, float>> &gridpts) {
  switch (mode_) {
    case CAMERAVIEW: {
      uint16_t buf[320*240];
      uint16_t *scr = buf;
      for (int j = 0; j < 240; j++) {
        const uint8_t *y = yuv + j * 640 * 2;
        const uint8_t *u = yuv + 640 * 480 + j * 320;
        const uint8_t *v = yuv + 640 * 600 + j * 320;
        for (int i = 0; i < 320; i++) {
          *scr++ = YUVtoRGB565(y[i * 2], u[i], v[i]);
        }
      }
      uint16_t c = 0x001f;
      for (size_t i = 0; i < gridpts.size(); i++) {
        int x = gridpts[i].first * 0.5;
        int y = gridpts[i].second * 0.5;
        if (x < 1 || x >= 319 || y < 1 || y >= 239) {
          continue;
        }
        buf[x + y * 320 - 320] = c;
        buf[x + y * 320 - 1] = c;
        buf[x + y * 320] = c;
        buf[x + y * 320 + 1] = c;
        buf[x + y * 320 + 320] = c;
      }
      scr = screen_.GetBuffer();
      memcpy(scr, buf, 320*240*2);
      // no room to show config or status, but that's ok
      break;
    }
    case FRONTVIEW: {
      uint16_t *scr = screen_.GetBuffer();
      remapYUV(frontremap_, yuv, scr);
      memcpy(scr + 120 * 320, configbuf_, 100 * 320 * 2);       // config edit
      memcpy(scr + 220 * 320, statusbuf_, sizeof(statusbuf_));  // status bar
      break;
    }
    default:
      break;
  }
}

void UIDisplay::UpdateCeiltrackView(const float *xytheta, float xgrid,
                                    float ygrid, float sizx, float sizy,
                                    const int32_t *obs1, const int32_t *obs2,
                                    float wheel_v) {
  if (mode_ != TRACKMAP) {
    return;
  }

  uint16_t buf[120 * 320];
  static const uint16_t green = (6 << 11) + (63 << 5) + (6);
  {
    const uint8_t *yuv = backgroundyuv_;
    for (int j = 0; j < 112; j++) {
      const uint8_t *y = yuv + j * 320;
      const uint8_t *u = yuv + 320 * 160 + (j >> 1) * 160;
      const uint8_t *v = yuv + 320 * (160 + 40) + (j >> 1) * 160;
      for (int i = 0; i < 320; i++) {
        buf[j * 320 + i] = YUVtoRGB565(y[i], u[i >> 1], v[i >> 1]);
      }
    }
    // we have 8 extra rows in the middle here
    // 01010 101010 10101 light cyanish (pretty much #55aaaa)
    memset(buf + 112 * 320, 0x55, (120 - 112) * 320 * 2);
  }

  float scale = 320 / sizx;
#if 0
  // don't need this anymore:
  // ceiling light markers are now baked into the background image
  static const uint16_t yellow = (31<<11) + (63<<5) + (6);
  for (float x = 0; x < 320; x += xgrid * scale) {
    int xi = (int)x;
    for (float y = 0; y < 112; y += ygrid * scale) {
      // draw markers for ceiling light locations
      int idx = ((int)y) * 320 + xi;
      buf[idx] = yellow;
      if (idx > 320) buf[idx - 320] = yellow;
      if (idx > 320) buf[idx + 320] = yellow;
      if (xi > 0) buf[idx - 1] = yellow;
      if (xi < 319) buf[idx + 1] = yellow;
    }
  }
#endif

  float x0 = xytheta[0] * scale;
  float y0 = xytheta[1] * -scale;
  float C = cos(xytheta[2]);
  float S = sin(xytheta[2]);
  for (int i = 0; i < 10; i++) {
    int x = x0 + C * i;
    int y = y0 - S * i;
    if (x >= 0 && x < 320 && y >= 0 && y < 112) {
      buf[y * 320 + x] = green;
    }
  }

  static const uint16_t orange = (31 << 11) + (40 << 5) + (0);
  static const uint16_t blue = (0 << 11) + (0 << 5) + (31);
  for (int i = 0; i < 256; i++) {
    float relang = (i - 128) * M_PI / 256.0 + xytheta[2];
    float C = cos(relang), S = sin(relang);
    for (int j = 10; j < 20 && j < 10 + (obs1[i] >> 6); j++) {
      int x = x0 + C * j;
      int y = y0 - S * j;
      if (x >= 0 && x < 320 && y >= 0 && y < 112) {
        buf[y * 320 + x] = blue;
      }
    }
    for (int j = 10; j < 20 && j < 10 + (obs2[i] >> 6); j++) {
      int x = x0 + C * j;
      int y = y0 - S * j;
      if (x >= 0 && x < 320 && y >= 0 && y < 112) {
        buf[y * 320 + x] = orange;
      }
    }
  }
  char vbuf[6];
  snprintf(vbuf, sizeof(vbuf), "%0.1f", wheel_v);
  DrawText(vbuf, 320 - 30, 0, 0xffff, buf);

  // blit buffer to screen all at once
  uint16_t *scr = screen_.GetBuffer();
  memcpy(scr, buf, sizeof(buf));                            // map
  memcpy(scr + 120 * 320, configbuf_, 100 * 320 * 2);       // config edit
  memcpy(scr + 220 * 320, statusbuf_, sizeof(statusbuf_));  // status bar
}

void UIDisplay::UpdateConfig(const char *configmenu[], int nconfigs,
                             int config_item, const int16_t *config_values) {
  // 112x56 * 2 -> 224x112 top left taken by birdseye
  // config will start at y = 112 .. 220 -> 5 lines of big text, 10 lines of
  // small text.
  // let's display 5 items, with the selected one on line 3 (the middle)
  // inverted

  memset(configbuf_, 0, sizeof(configbuf_));

  uint16_t *buf = configbuf_;
  for (int i = 0; i < 5; i++) {
    int configoffset = (config_item - 2 + i) % nconfigs;
    if (configoffset < 0) configoffset += nconfigs;
    const char *configname = configmenu[configoffset];
    int16_t configvalue = config_values[configoffset];
    int16_t absvalue = configvalue < 0 ? -configvalue : configvalue;
    char numbuf[11];  // "-327.67\0"
    snprintf(numbuf, sizeof(numbuf) - 1, "%c%d.%02d",
             configvalue < 0 ? '-' : ' ', absvalue / 100, absvalue % 100);
    int numwidth = TextWidthBig(numbuf);
    if (i == 2) {
      // invert middle line
      memset(buf + i * 20 * 320, 0xff, 20 * 2 * 320);
      DrawTextBig(configname, 0, 20 * i, 0, buf);
      DrawTextBig(numbuf, 320 - numwidth, 20 * i, 0, buf);
    } else {
      DrawTextBig(configname, 0, 20 * i, 0xffff, buf);
      DrawTextBig(numbuf, 320 - numwidth, 20 * i, 0xffff, buf);
    }
  }

  // blit to screen
  uint16_t *scr = screen_.GetBuffer();
  memcpy(scr + 120 * 320, buf, 100 * 320 * 2);
}

void UIDisplay::UpdateStatus(const char *status, uint16_t color) {
  memset(statusbuf_, 0, sizeof(statusbuf_));
  DrawTextBig(status, 0, 0, color, statusbuf_);

  uint16_t *scr = screen_.GetBuffer();
  memcpy(scr + 220 * 320, statusbuf_, sizeof(statusbuf_));
}

void UIDisplay::UpdateDashboard(float v, float w, int32_t lon, int32_t lat,
                                int numSV, float gpsv, float mlon, float mlat,
                                float mag_north, float mag_east, float ye,
                                float psie, float autok, float autov) {
  char numbuf[32];
  uint16_t buf[120 * 320];
  memset(buf, 0, sizeof(buf));
  snprintf(numbuf, sizeof(numbuf) - 1, "v: %0.2f w %+0.3f", v, w);
  DrawTextBig(numbuf, 0, 0, 0xffff, buf);
  snprintf(numbuf, sizeof(numbuf) - 1, "%+011d %+0.1fm", lon, mlon);
  DrawTextBig(numbuf, 0, 20, 0xffff, buf);
  snprintf(numbuf, sizeof(numbuf) - 1, "%+011d %+0.1fm", lat, mlat);
  DrawTextBig(numbuf, 0, 40, 0xffff, buf);
  snprintf(numbuf, sizeof(numbuf) - 1, "numSV:%d gpsV %0.1f", numSV, gpsv);
  DrawTextBig(numbuf, 0, 60, 0xffff, buf);
  snprintf(numbuf, sizeof(numbuf) - 1, "y %0.1fm psi %+0.3f", ye, psie);
  DrawTextBig(numbuf, 0, 80, 0xffff, buf);
  snprintf(numbuf, sizeof(numbuf) - 1, "k %0.3f v %0.1f", autok, autov);
  DrawTextBig(numbuf, 0, 100, 0xffff, buf);

  for (int i = 0; i < 20; i++) {
    float x = 318 + i * mag_east;
    float y = 20 - i * mag_north;
    buf[(int)x + ((int)y) * 320] = 0xffe0;
    buf[(int)x + 1 + ((int)y) * 320] = 0xffe0;
    buf[(int)x + ((int)y + 1) * 320] = 0xffe0;
  }

  memcpy(screen_.GetBuffer(), buf, sizeof(buf));
}

void UIDisplay::NextMode() {
  mode_ = (DisplayMode)(((int)mode_) + 1);
  if (mode_ == NUM_MODES) {
    mode_ = (DisplayMode)0;
  }
}

void UIDisplay::remapYUV(const uint16_t *maptbl, const uint8_t *yuv,
                         uint16_t *buf) {
  // assumes 320x120 remap table and 320-wide destination buf
  for (int idx = 0; idx < 320 * 120; idx++) {
    int i = *maptbl++;
    int j = *maptbl++;
    if (j >= 480 * 64 || i >= 640 * 64) {
      *buf++ = 0;
      continue;
    }
    // FIXME: interpolate
    int y = yuv[(j >> 6) * 640 + (i >> 6)];
    int u = yuv[640 * 480 + (j >> 7) * 320 + (i >> 7)];
    int v = yuv[640 * 600 + (j >> 7) * 320 + (i >> 7)];
    *buf++ = YUVtoRGB565(y, u, v);
  }
}