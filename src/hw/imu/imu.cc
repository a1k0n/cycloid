#include <stdio.h>
#include <string>

#include "hw/gpio/i2c.h"
#include "hw/imu/imu.h"
#include "hw/imu/invensense.h"
#include "inih/cpp/INIReader.h"

IMU *IMU::GetI2CIMU(const I2C &i2c, const INIReader &ini) {
  std::string imutype = ini.GetString("imu", "device", "");
  if (imutype.empty()) {
    fprintf(stderr, "please specify [imu] device=xyz in config\n");
    return NULL;
  }
  float zorient = ini.GetReal("imu", "orientation", 1.0);
  if (imutype == "mpu9150" || imutype == "mpu9250" || imutype == "mpu6050" ||
      imutype == "mpu6500") {
    uint8_t addr = ini.GetInteger("imu", "addr", 0x68);
    return new InvensenseIMU(i2c, addr, zorient);
  } else if (imutype == "icm20600" || imutype == "icm20602") {
    uint8_t addr = ini.GetInteger("imu", "addr", 0x69);
    return new InvensenseIMU(i2c, addr, zorient);
  }
  fprintf(stderr, "unsupported imu type: %s\n", imutype.c_str());
  return NULL;
}
