#ifndef HW_IMU_MPU9150_H_
#define HW_IMU_MPU9150_H_

#include "hw/gpio/i2c.h"
#include "hw/imu/imu.h"

class MPU9150 : public IMU {
 public:
  explicit MPU9150(const I2C &i2c) : i2c_(i2c) {}

  virtual ~MPU9150();

  virtual bool Init();

  virtual bool ReadIMU(Eigen::Vector3f *accel, Eigen::Vector3f *gyro,
                       float *temp);

 private:
  const I2C &i2c_;
};

#endif  // HW_IMU_MPU9150_H_
