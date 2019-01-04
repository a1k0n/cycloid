#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include "hw/car/stm32rs232.h"

const char *DEVICE = "/dev/serial0";

STM32HatSerial::STM32HatSerial() {
  fd_ = -1;
  sync_ = false;
}

bool STM32HatSerial::Init() {
  fd_ = open(DEVICE, O_RDWR);
  if (fd_ == -1) {
    perror(DEVICE);
    return false;
  }

  struct termios tios;
  if (tcgetattr(fd_, &tios)) {
    goto error;
  }

  // disable flow control and all that, and ignore break and parity errors
  tios.c_iflag = IGNBRK | IGNPAR;
  tios.c_oflag = 0;
  tios.c_lflag = 0;
  tios.c_cc[VTIME] = 0;  // no read timeout
  tios.c_cc[VMIN] = 1;  // reads must have at least 1 byte
  cfsetspeed(&tios, B115200);

  if (tcsetattr(fd_, TCSAFLUSH, &tios)) {
    goto error;
  }

  tcflush(fd_, TCIOFLUSH);

  // write junk to the port just to flush out the receive buffer on the remote
  // end
  {
    uint8_t buf[6] = {0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa};
    write(fd_, buf, 6);
  }

  return true;

error:
  perror(DEVICE);
  close(fd_);
  fd_ = -1;
  return false;
}

bool STM32HatSerial::SetControls(uint8_t led, int8_t esc, int8_t servo) {
  // write control packet w/ header and checksum
  uint8_t buf[5] = {0x55, led, (uint8_t)esc, (uint8_t)servo, 0};
  buf[4] = 0;
  for (int i = 0; i < 4; i++) {
    buf[4] += buf[i];
  }
  buf[4] = ~buf[4];
  return write(fd_, buf, 5) == 5;
}

static int read_fully(int fd, uint8_t *buf, int ntoread) {
  while (ntoread) {
    int n = read(fd, buf, ntoread);
    if (n == ntoread) {
      break;
    }
    if (n < 0) {
      return -1;
    }
    ntoread -= n;
    buf += n;
  }
  return 0;
}

bool STM32HatSerial::AwaitSync(uint16_t *encoder_pos, uint16_t *encoder_dt) {
  uint8_t buf[32];
  if (!sync_) {
    for (;;) {
      int n = read(fd_, buf, 1);
      if (n < 1) {
        if (n == -1) {
          perror(DEVICE);
        }
        if (n == 0) {
          continue;
        }
        return false;
      }
      if (buf[0] == 0xaa) {
        break;
      }
      if (buf[0] == 0xfe) {
        fprintf(stderr, "STM32HatSerial: remote checksum err\n");
      }
    }
    if (read_fully(fd_, buf+1, 5)) {
      return false;
    }
    sync_ = true;
  } else {
    if (read_fully(fd_, buf, 6)) {
      return false;
    }
    if (buf[0] == 0xfe) {
      if (buf[0] == 0xfe) {
        fprintf(stderr, "STM32HatSerial: remote checksum err\n");
        sync_ = false;
        return false;
      }
    }
  }

  uint8_t cksum = 0;
  for (int i = 0; i < 6; i++) {
    cksum += buf[i];
  }
  if (cksum != 0xff) {
    fprintf(stderr, "STM32HatSerial checksum fail: %02x\n", cksum);
    return false;
  }

  *encoder_pos = buf[1] + (buf[2] << 8);
  *encoder_dt  = buf[3] + (buf[4] << 8);

  return true;
}
