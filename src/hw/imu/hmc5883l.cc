#include "hw/imu/hmc5883l.h"

#include <byteswap.h>

#include "hw/gpio/i2c.h"

HMC5883L::HMC5883L(I2C *i2c, uint8_t addr) : i2c_(i2c), addr_(addr) {}

bool HMC5883L::Init() {
  // config compass
  if (!i2c_->Write(addr_, 0x00, 0x38))  // CRA: 75Hz rate w/ 2 averages
    goto fail;
  if (!i2c_->Write(addr_, 0x01, 0x20))  // CRB: set gain 1090 LSB/Gauss
    goto fail;
  if (!i2c_->Write(addr_, 0x02, 0x00))  // continuous measurement
    goto fail;

  return true;
fail:
  fprintf(stderr, "unable to access HMC5883L on i2c address %02x\n", addr_);
  return false;
}

bool HMC5883L::ReadMag(Eigen::Vector3f *mag) {
  uint8_t axis_buf[6];
  if (!i2c_->Read(addr_, 0x03, 6, axis_buf)) return false;
  int16_t mx = bswap_16(*reinterpret_cast<uint16_t *>(axis_buf + 0));  // front?
  int16_t my = bswap_16(*reinterpret_cast<uint16_t *>(axis_buf + 2));  // up
  int16_t mz = bswap_16(*reinterpret_cast<uint16_t *>(axis_buf + 4));  // side?
  *mag = Eigen::Vector3f(mx, my, mz) * (1.0 / 1090.0);
  return true;
}
