#include "hw/imu/mag.h"

class HMC5883L : public Magnetometer {
 public:
  static const uint8_t DefaultAddr = 0x1e;

  HMC5883L(I2C *i2c, uint8_t addr = DefaultAddr);

  bool Init();
  bool ReadMag(Eigen::Vector3f *mag);

 private:
  I2C *i2c_;
  uint8_t addr_;
};
