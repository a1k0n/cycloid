#ifndef GPSDRIVE_CONFIG_H_
#define GPSDRIVE_CONFIG_H_

// AUTO-GENERATED! Edit conf.py instead.

#include <stdint.h>

class DriverConfig {
 public:
  int16_t speed_limit;
  int16_t Ax_limit;
  int16_t Ay_limit;
  int16_t motor_gain;
  int16_t motor_kI;
  int16_t servo_rate;
  int16_t servo_offset;
  int16_t servo_kP;
  int16_t servo_kI;
  int16_t servo_min;
  int16_t servo_max;
  int16_t steering_kpy;
  int16_t steering_kvy;
  int16_t lookahead;
  int16_t deadreckon_time;

  DriverConfig() {
    // Default values
    speed_limit          = 300;
    Ax_limit             = 800;
    Ay_limit             = 1200;
    motor_gain           = 13;
    motor_kI             = 200;
    servo_rate           = -84;
    servo_offset         = -44;
    servo_kP             = 100;
    servo_kI             = 1000;
    servo_min            = -100;
    servo_max            = 100;
    steering_kpy         = 4;
    steering_kvy         = 50;
    lookahead            = 0;
    deadreckon_time      = 6;
  }

  static const char *confignames[];
  static const int N_CONFIGITEMS;

  bool Load();
  bool Save();
  int SerializedSize() const;
  int Serialize(uint8_t *buf, int buflen);
};

#endif  // GPSDRIVE_CONFIG_H_
