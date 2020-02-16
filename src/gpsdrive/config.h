#ifndef GPSDRIVE_CONFIG_H_
#define GPSDRIVE_CONFIG_H_

// AUTO-GENERATED! Edit conf.py instead.

#include <stdint.h>

class DriverConfig {
 public:
  int16_t speed_limit;
  int16_t Ax_limit;
  int16_t Ay_limit;
  int16_t servo_rate;
  int16_t servo_offset;
  int16_t servo_kI;
  int16_t servo_min;
  int16_t servo_max;

  DriverConfig() {
    // Default values
    speed_limit          = 300;
    Ax_limit             = 800;
    Ay_limit             = 1200;
    servo_rate           = 100;
    servo_offset         = 0;
    servo_kI             = 20;
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

#endif  // GPSDRIVE_CONFIG_H_
