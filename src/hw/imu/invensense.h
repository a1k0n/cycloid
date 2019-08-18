#ifndef HW_IMU_INVENSENSE_H_
#define HW_IMU_INVENSENSE_H_

#include "hw/gpio/i2c.h"
#include "hw/imu/imu.h"

class InvensenseIMU : public IMU {
 public:
  explicit InvensenseIMU(const I2C &i2c, uint8_t addr, float zorient)
      : i2c_(i2c), i2caddr_(addr), zorient_(zorient) {}

  virtual ~InvensenseIMU();

  virtual bool Init();

  virtual bool ReadIMU(Eigen::Vector3f *accel, Eigen::Vector3f *gyro);

 private:
  const I2C &i2c_;
  const uint8_t i2caddr_;
  const float zorient_;
};

#endif  // HW_IMU_ICM20600_H_
