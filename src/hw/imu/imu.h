#ifndef HW_IMU_IMU_H_
#define HW_IMU_IMU_H_

#include <string>
#include <Eigen/Dense>

#include "hw/gpio/i2c.h"

using Eigen::Vector3f;

class IMU {
 public:
  virtual bool Init() = 0;

  virtual bool ReadIMU(Eigen::Vector3f *accel, Eigen::Vector3f *gyro,
                       float *temp) = 0;

  static IMU *GetI2CIMU(const I2C &i2c, const std::string &type);
};

#endif  // HW_IMU_IMU_H_
