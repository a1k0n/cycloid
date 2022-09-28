#ifndef DRIVE_CONFIG_H_
#define DRIVE_CONFIG_H_

// AUTO-GENERATED! Edit conf.py instead.

#include <stdint.h>

class DriverConfig {
 public:
  int16_t speed_limit;
  int16_t throttle_cap;
  int16_t throttle_slew;
  int16_t throttle_bias;
  int16_t pi_thr_scale;
  int16_t pi_brake_scale;
  int16_t pi_steer_scale;
  int16_t pi_v_scale;
  int16_t orange_thresh;
  int16_t black_thresh;
  int16_t servo_offset;
  int16_t servo_min;
  int16_t servo_max;

  DriverConfig() {
    // Default values
    speed_limit          = 300;
    throttle_cap         = 100;
    throttle_slew        = 400;
    throttle_bias        = 0;
    pi_thr_scale         = 100;
    pi_brake_scale       = 100;
    pi_steer_scale       = 100;
    pi_v_scale           = 100;
    orange_thresh        = 150;
    black_thresh         = 40;
    servo_offset         = 0;
    servo_min            = -100;
    servo_max            = 100;
  }

  static const char *confignames[];
  static const int N_CONFIGITEMS;

  bool Load();
  bool Save();
  int SerializedSize() const;
  int Serialize(uint8_t *buf, int buflen);
};

#endif  // DRIVE_CONFIG_H_
