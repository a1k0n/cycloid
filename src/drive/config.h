#ifndef DRIVE_CONFIG_H_
#define DRIVE_CONFIG_H_

// AUTO-GENERATED! Edit conf.py instead.

#include <stdint.h>

class DriverConfig {
 public:
  int16_t speed_limit;
  int16_t traction_limit;
  int16_t lookahead_dist;
  int16_t lookahead_time;
  int16_t lookahead_kmax;
  int16_t path_penalty;
  int16_t cone_penalty;
  int16_t car_penalty;
  int16_t motor_gain;
  int16_t motor_kI;
  int16_t servo_rate;
  int16_t servo_offset;
  int16_t servo_kI;
  int16_t servo_min;
  int16_t servo_max;
  int16_t brakezone1_x;
  int16_t brakezone1_y;
  int16_t brakezone1_v;
  int16_t brakezone2_x;
  int16_t brakezone2_y;
  int16_t brakezone2_v;

  DriverConfig() {
    // Default values
    speed_limit          = 300;
    traction_limit       = 900;
    lookahead_dist       = 175;
    lookahead_time       = 60;
    lookahead_kmax       = 100;
    path_penalty         = 9;
    cone_penalty         = 10;
    car_penalty          = 10;
    motor_gain           = 13;
    motor_kI             = 200;
    servo_rate           = 100;
    servo_offset         = 0;
    servo_kI             = 20;
    servo_min            = -100;
    servo_max            = 100;
    brakezone1_x         = 1250;
    brakezone1_y         = -300;
    brakezone1_v         = 500;
    brakezone2_x         = 650;
    brakezone2_y         = -520;
    brakezone2_v         = 300;
  }

  static const char *confignames[];
  static const int N_CONFIGITEMS;

  bool Load();
  bool Save();
  int SerializedSize() const;
  int Serialize(uint8_t *buf, int buflen);
};

#endif  // DRIVE_CONFIG_H_
