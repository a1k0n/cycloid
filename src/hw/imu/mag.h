#ifndef HW_IMU_MAG_H_
#define HW_IMU_MAG_H_

#include <Eigen/Dense>

#include "hw/gpio/i2c.h"
#include "inih/cpp/INIReader.h"

class Magnetometer {
 public:
  virtual bool Init() = 0;
  virtual bool ReadMag(Eigen::Vector3f *mag) = 0;

  static Magnetometer *GetI2CMag(I2C *i2c, const INIReader &ini);
};

#endif  // HW_IMU_MAG_H_
