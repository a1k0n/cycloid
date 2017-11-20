#ifndef DRIVE_CONFIG_H_
#define DRIVE_CONFIG_H_

// Dynamic configuration variables
// can be changed via commandline or controller
class DriverConfig {
 public:
  int16_t y_scale;
  int16_t u_scale;
  int16_t v_scale;
  int16_t yellow_thresh;

  // all these int16_ts are 1/100th scale
  int16_t speed_limit;  // m/s, maximum allowed speed
  int16_t max_throttle;  // 0-1, maximum ESC input

  int16_t traction_limit;  // m/s^2 (lateral force, v*w product)

  int16_t steering_kpy;  // PID curve following proportional const
  int16_t steering_kvy;  // derivative const

  int16_t brake_kP;  // NOTUSED FIXME
  int16_t motor_kP;  // PID control of motor
  int16_t motor_kI;
  int16_t motor_kD;
  int16_t motor_offset;

  int16_t lane_offset;  // m, constant distance from centerline to aim for
  int16_t lane_offset_per_k;  // m/m, degree to which it steers into turns

  DriverConfig() {
    // Default values
    y_scale = 0;
    u_scale = -100;
    v_scale = 0;
    yellow_thresh = 30;

    speed_limit = 3.0 * 100;
    max_throttle = 0.6 * 100;
    traction_limit = 4.0 * 100;

    steering_kpy = 1.0 * 100;
    steering_kvy = 2.0 * 100;

    brake_kP = 7.50 * 100;
    motor_kP = 4.00 * 100;
    motor_kI = 0.0 * 100;  // NOTUSED FIXME
    motor_kD = 0.0 * 100;  // NOTUSED FIXME
    motor_offset = 13;

    lane_offset = 0.0 * 100;
    lane_offset_per_k = 0.15 * 100;
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
      fprintf(stderr, "driverconfig is %d bytes; "
          "config should be %d; ignoring\n", ftell(fp), sizeof(this));
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
