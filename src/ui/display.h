#ifndef UI_DISPLAY_H_
#define UI_DISPLAY_H_

#include <Eigen/Dense>
#include "hw/lcd/fbdev.h"

class UIDisplay {
 public:
  bool Init();

  void UpdateBirdseye(const uint8_t *yuv, int w, int h);
  void UpdateStateEstimate(float v, float delta, float y,
      float psi, float kappa);

  void UpdateConfig(const char *configmenu[], int nconfigs,
      int config_item, const int16_t *config_values);

  void UpdateEncoders(uint16_t *wheel_pos);
  void UpdateStatus(const char *status, uint16_t color = 0xffff);

  void UpdateLocalization(const Eigen::VectorXf &prob, float ye);

  uint16_t *GetScreenBuffer() { return screen_.GetBuffer(); }

 private:
  LCDScreen screen_;
};

#endif  // UI_DISPLAY_H_
