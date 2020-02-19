#include "hw/gpio/i2c.h"
#include "hw/imu/mag.h"
#include "hw/imu/hmc5883l.h"

#include <string>


Magnetometer *Magnetometer::GetI2CMag(I2C *i2c, const INIReader &ini) {
  std::string magtype = ini.GetString("magnetomter", "device", "");
  if (magtype.empty()) {
    fprintf(stderr, "please specify [magnetometer] device=xyz in config\n");
    return NULL;
  }
  if (magtype == "hmc5883l") {
    uint8_t addr = ini.GetInteger("imu", "addr", HMC5883L::DefaultAddr);
    return new HMC5883L(i2c, addr);
  } else {
    return NULL;
  }
}
