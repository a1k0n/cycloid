#ifndef IMU_IMU_H_
#define IMU_IMU_H_

#include <stdint.h>
#include <Eigen/Dense>
#include "gpio/i2c.h"

// TODO: rename to imu/dev.h for IMU device raw access
// then make an IMU class which is self-calibrating, has a kalman filter, etc.

struct IMURawState {
  int16_t gyro_x, gyro_y, gyro_z, gyro_temp;
  int16_t mag_x, mag_y, mag_z;
  int16_t accel_x, accel_y, accel_z;
};

struct IMUState {
  Eigen::Vector3f w;  // angular velocity, rads/sec (gyro)
  Eigen::Vector3f N;  // north pole direction (mag)
  Eigen::Vector3f g;  // acceleration + gravity (m/s^2)
};

class IMU {
 public:
  explicit IMU(const I2C &i2c) : i2c_(i2c), YTY_(10, 10) {}

  bool Init();

  // read raw sensor values
  // unsupported
  // bool ReadRaw(IMURawState *state);

  // Read adjusted sensor values after various calibrations (gyro
  // temperature-compensated offset, magnetometer offset, accelerometer offset)
  // and adjusting axes to the car's local frame (x right, y up, z forward)
  bool ReadCalibrated(IMUState *state);

  // convert raw readings to calibrated ones; ReadCalibrated just calls this
  // unsupported
  // void Calibrate(const IMURawState &rawstate, IMUState *state);

  // write magnetometer calibration out
  bool LoadMagCalibration();
  bool SaveMagCalibration();

  bool ReadMag(Eigen::Vector3f *mag);
  bool ReadIMU(Eigen::Vector3f *accel, Eigen::Vector3f *gyro, float *temp);

 private:
  bool CalibrateMag(const Eigen::Vector3f &mag, bool is_calibrated, Eigen::Vector3f *north);
  bool SolveMagCalibration();

  const I2C &i2c_;

  Eigen::Vector3f magadj_;
  Eigen::MatrixXd YTY_;
  Eigen::Matrix3f proj_;
  Eigen::Vector3f center_;

  bool mag_calibrated_;

  // std::vector<Eigen::Vector3f> mag_cal_points_;
  // Eigen::Matrix4f mag_XTX_;
  // Eigen::Vector4f mag_XTY_;
  // Eigen::Vector3f mag_bias_;
};

#endif  // IMU_IMU_H_
