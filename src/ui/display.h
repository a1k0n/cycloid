#ifndef UI_DISPLAY_H_
#define UI_DISPLAY_H_

#include "hw/lcd/fbdev.h"
#include "localization/coneslam/localize.h"

#include <vector>

class FisheyeLens;

class UIDisplay {
 public:
  enum DisplayMode { TRACKMAP = 0, CAMERAVIEW, FRONTVIEW, NUM_MODES };

  bool Init();
  void InitCamera(const FisheyeLens &lens, float camtilt);

#if 0
  void UpdateBirdseye(const uint8_t *yuv, int w, int h);

  void UpdateConeView(const uint8_t *yuv, int ncones, int *conesx);

  void UpdateParticleView(const coneslam::Localizer *l);
#endif

  void UpdateCameraView(const uint8_t *yuv,
                        const std::vector<std::pair<float, float>> &gridpts);

  void UpdateCeiltrackView(const float *xytheta, float xgrid, float ygrid,
                           float sixz, float sizy, const int32_t *obs1,
                           const int32_t *obs2, float wheel_v, float fps, int map);

  void UpdateConfig(const char *configmenu[], int nconfigs, int config_item,
                    const int16_t *config_values);

  void UpdateDashboard(float v, float w, int32_t lon, int32_t lat, int numSV,
                       float gpsv, float mlon, float mlat, float mag_north,
                       float mag_east, float ye, float psie, float autok,
                       float autov, float heading);

  void UpdateEncoders(uint16_t *wheel_pos);
  void UpdateStatus(const char *status, uint16_t color = 0xffff);

  void NextMode();

  uint16_t *GetScreenBuffer() { return screen_.GetBuffer(); }

 private:
  void remapYUV(const uint16_t *maptbl, const uint8_t *yuv, uint16_t *buf);

  LCDScreen screen_;
  uint8_t *backgroundyuv_;
  uint16_t *frontremap_;
  DisplayMode mode_;

  uint16_t configbuf_[100 * 320];
  uint16_t statusbuf_[20 * 320];
};

#endif  // UI_DISPLAY_H_
