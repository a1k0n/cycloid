#ifndef UI_DISPLAY_H_
#define UI_DISPLAY_H_

#include "hw/lcd/fbdev.h"
#include "localization/coneslam/localize.h"

class UIDisplay {
 public:
  bool Init();

  void UpdateBirdseye(const uint8_t *yuv, int w, int h);
  void UpdateStateEstimate(float v, float delta, float y,
      float psi, float kappa);

  void UpdateConeView(const uint8_t *yuv, int ncones, int *conesx);

  void UpdateParticleView(const coneslam::Localizer *l);

  void UpdateCeiltrackView(const float *xytheta, float xgrid, float ygrid,
                           float sixz, float sizy, const int32_t *obs1,
                           const int32_t *obs2);

  void UpdateConfig(const char *configmenu[], int nconfigs,
      int config_item, const int16_t *config_values);

  void UpdateEncoders(uint16_t *wheel_pos);
  void UpdateStatus(const char *status, uint16_t color = 0xffff);

  uint16_t *GetScreenBuffer() { return screen_.GetBuffer(); }

 private:
  LCDScreen screen_;
  uint8_t *backgroundyuv_;
};

#endif  // UI_DISPLAY_H_
