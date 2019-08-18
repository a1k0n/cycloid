#include <stdio.h>
#include <string>

#include "hw/gpio/i2c.h"
#include "hw/imu/imu.h"
#include "hw/imu/mpu9150.h"
#include "hw/imu/icm20600.h"

IMU *IMU::GetI2CIMU(const I2C &i2c, const std::string &type) {
  if (type == "mpu9150" || type == "mpu9250" || type == "mpu6050" ||
      type == "mpu6500") {
    return new MPU9150(i2c);
  } else if (type == "icm20600" || type == "icm20602") {
    return new ICM20600(i2c);
  }
  fprintf(stderr, "unsupported imu type: %s\n", type.c_str());
  return NULL;
}
