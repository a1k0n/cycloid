#ifndef DRIVE_CONFIG_H_
#define DRIVE_CONFIG_H_

#include <cmath>
#include <cstdint>
#include <cstdio>

// circumference of tire (meters) / number of encoder ticks
const float WHEEL_DIAMETER = 0.0666;
// const float WHEEL_DIAMETER = 0.08;
const float DRIVE_RATIO = 84./25. * 2.1;  // 84t spur, 25t pinion, 2.1 final drive
const float MOTOR_POLES = 3;  // brushless sensor counts per motor revolution
const float V_SCALE = WHEEL_DIAMETER*M_PI / DRIVE_RATIO / MOTOR_POLES;

// Dynamic configuration variables
// can be changed via commandline or controller
class DriverConfig {
 public:
  // all these int16_ts are 1/100th scale
  int16_t speed_limit;  // m/s, maximum allowed speed
  int16_t traction_limit;  // m/s^2 (lateral force, v*w product)

  int16_t lookahead_dist;   // value function lookahead distance
  int16_t lookahead_krate;  // value function lookahead turn rate
  int16_t path_penalty;     // cone / lane penalty magnitude

  // motor parameters
  int16_t motor_gain;  // control gain for motor
  int16_t motor_kI;    // integral term

  int16_t turnin_lift;  // amount to lift the throttle on turn-in

  // servo control parameters
  int16_t servo_rate;      // servo speed
  int16_t servo_offset;    // center offset
  int16_t servo_finetune;  // closed-loop adjust rate

  int16_t lm_precision;  // landmark precision (1/sigma^2)

  DriverConfig() {
    // Default values
    speed_limit = 3.0 * 100;
    traction_limit = 8.0 * 100;

    lookahead_dist = 2.0 * 100;
    lookahead_krate = 0.8 * 100;
    path_penalty = 3;

    motor_gain = 0.25 * 100;
    motor_kI = 3.0 * 100;

    turnin_lift = 0;

    servo_rate = -1.70 * 100;
    servo_offset = 0.30 * 100;
    servo_finetune = -2.00 * 100;

    lm_precision = 100;
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
          "config should be %u; ignoring\n", ftell(fp), sizeof(this));
      fclose(fp);
      return true;
    }
    fseek(fp, 0, SEEK_SET);
    size_t n = fread(this, sizeof(*this), 1, fp);
    fclose(fp);
    return n == 1;
  }

  int SerializedSize() const {
    return 8 + sizeof(*this);
  }

  int Serialize(uint8_t *buf, int buflen) {
    int siz = SerializedSize();
    memcpy(buf, "cfg1", 4);
    memcpy(buf+4, &siz, 4);
    memcpy(buf+8, this, sizeof(*this));
    return siz;
  }
};

#endif  // DRIVE_CONFIG_H_
