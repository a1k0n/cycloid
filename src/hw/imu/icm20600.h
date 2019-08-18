#ifndef HW_IMU_ICM20600_H_
#define HW_IMU_ICM20600_H_

#include "hw/gpio/i2c.h"
#include "hw/imu/imu.h"

class ICM20600 : public IMU {
 public:
  explicit ICM20600(const I2C &i2c) : i2c_(i2c) {}

  virtual ~ICM20600();

  virtual bool Init();

  virtual bool ReadIMU(Eigen::Vector3f *accel, Eigen::Vector3f *gyro,
                       float *temp);

 private:
  const I2C &i2c_;
};

#endif  // HW_IMU_ICM20600_H_
