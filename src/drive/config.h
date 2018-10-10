#ifndef DRIVE_CONFIG_H_
#define DRIVE_CONFIG_H_

#include <cstdint>
#include <cstdio>

// circumference of tire (meters) / number of encoder ticks
const float V_SCALE = 0.02;  // 40cm circumference, 20 ticks

// Dynamic configuration variables
// can be changed via commandline or controller
class DriverConfig {
 public:
  int16_t cone_thresh;

  // all these int16_ts are 1/100th scale
  int16_t speed_limit;  // m/s, maximum allowed speed
  int16_t traction_limit;  // m/s^2 (lateral force, v*w product)

  int16_t steering_kpy;  // PID curve following proportional const
  int16_t steering_kvy;  // derivative const

  int16_t motor_bw;  // control bandwidths
  int16_t yaw_bw;

  int16_t lm_precision;  // landmark precision (1/sigma^2)
  int16_t lm_bogon_thresh;  // min angle-error (radians) for true cone detection

  int16_t srv_cal;  // servo fixed value for turn calibration

  DriverConfig() {
    // Default values
    cone_thresh = 300;

    speed_limit = 8.0 * 100;
    traction_limit = 8.0 * 100;

    steering_kpy = 0.2 * 100;
    steering_kvy = 0.6 * 100;

    motor_bw = 0.05 * 100;
    yaw_bw = 0.6 * 100;

    lm_precision = 100;
    lm_bogon_thresh = 0.31 * 100;

    srv_cal = 64;
  }

  bool Save() {
    FILE *fp = fopen("driverconfig.bin", "wb");
    if (!fp) {
      return false;
    }
    fwrite(this, sizeof(*this), 1, fp);
    fclose(fp);
    return true;
  }

  bool Load() {
    FILE *fp = fopen("driverconfig.bin", "rb");
    if (!fp) {
      return false;
    }
    fseek(fp, 0, SEEK_END);
    if (ftell(fp) != sizeof(*this)) {
      fprintf(stderr, "driverconfig is %ld bytes; "
          "config should be %lu; ignoring\n", ftell(fp), sizeof(this));
      fclose(fp);
      return true;
    }
    fseek(fp, 0, SEEK_SET);
    size_t n = fread(this, sizeof(*this), 1, fp);
    fclose(fp);
    return n == 1;
  }
};

#endif  // DRIVE_CONFIG_H_
