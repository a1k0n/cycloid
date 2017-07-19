#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include "./i2c.h"

static const char I2C_DEVICE[] = "/dev/i2c-1";

bool I2C::Open() {
  fd_ = open(I2C_DEVICE, O_RDWR);
  if (fd_ == -1) {
    perror(I2C_DEVICE);
    return false;
  }
  return true;
}

void I2C::Close() {
  if (fd_ != -1) {
    close(fd_);
    fd_ = -1;
  }
}

bool I2C::Write(uint8_t addr, uint8_t reg, uint8_t value) const {
  uint8_t outbuf[2];
  struct i2c_rdwr_ioctl_data packets;
  struct i2c_msg messages[1];

  outbuf[0] = reg;
  outbuf[1] = value;

  messages[0].addr  = addr;
  messages[0].flags = 0;
  messages[0].len   = 2;
  messages[0].buf   = outbuf;

  packets.msgs  = messages;
  packets.nmsgs = 1;
  if (ioctl(fd_, I2C_RDWR, &packets) < 0) {
    perror("i2c_write");
    return false;
  }
  return true;
}

bool I2C::Write(uint8_t addr, uint8_t reg, int len, const uint8_t *buf) const {
  uint8_t outbuf[33];
  struct i2c_rdwr_ioctl_data packets;
  struct i2c_msg messages[1];

  outbuf[0] = reg;
  memcpy(outbuf+1, buf, len);

  messages[0].addr  = addr;
  messages[0].flags = 0;
  messages[0].len   = len + 1;
  messages[0].buf   = outbuf;

  packets.msgs  = messages;
  packets.nmsgs = 1;
  if (ioctl(fd_, I2C_RDWR, &packets) < 0) {
    perror("i2c_write");
    return false;
  }
  return true;
}

bool I2C::Read(uint8_t addr, uint8_t reg, int len, uint8_t *outbuf) const {
  struct i2c_rdwr_ioctl_data packets;
  struct i2c_msg messages[2];

  messages[0].addr  = addr;
  messages[0].flags = 0;
  messages[0].len   = 1;
  messages[0].buf   = &reg;

  messages[1].addr  = addr;
  messages[1].flags = I2C_M_RD;
  messages[1].len   = len;
  messages[1].buf   = outbuf;

  packets.msgs      = messages;
  packets.nmsgs     = 2;
  if (ioctl(fd_, I2C_RDWR, &packets) < 0) {
    perror("i2c_read");
    return false;
  }

  return true;
}
