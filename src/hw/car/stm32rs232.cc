#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include "hw/car/stm32rs232.h"
#include "inih/cpp/INIReader.h"

STM32HatSerial::STM32HatSerial(const INIReader &ini) {
  fd_ = -1;
  sync_ = false;
  device_ = ini.GetString("car", "device", "/dev/serial0").c_str();
  if (!ini.HasValue("car", "meters_per_wheeltick")) {
    fprintf(stderr,
            "STM32HatSerial: please specify [car] meters_per_wheeltick\n"
            " -- cannot use wheel encoder without it\n");
  }
  meters_per_tick_ = ini.GetReal("car", "meters_per_wheeltick", 0);
  ds_ = v_ = 0;
}

bool STM32HatSerial::Init() {
  fd_ = open(device_, O_RDWR);
  if (fd_ == -1) {
    perror(device_);
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
  tios.c_cc[VMIN] = 1;   // reads must have at least 1 byte
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
  perror(device_);
  close(fd_);
  fd_ = -1;
  return false;
}

bool STM32HatSerial::SetControls(unsigned led, float throttle, float steering) {
  if (throttle < -1) throttle = -1;
  else if (throttle > 1) throttle = 1;
  if (steering < -1) steering = -1;
  else if (steering > 1) steering = 1;

  int8_t esc = 127.0*throttle;
  int8_t servo = 127.0*steering;
  // write control packet w/ header and checksum
  uint8_t buf[5] = {0x55, (uint8_t)led, (uint8_t)esc, (uint8_t)servo, 0};
  buf[4] = 0;
  for (int i = 0; i < 4; i++) {
    buf[4] += buf[i];
  }
  buf[4] = ~buf[4];
  return write(fd_, buf, 5) == 5;
}

bool STM32HatSerial::GetWheelMotion(float *ds, float *v) {
  if (meters_per_tick_ == 0) {
    return false;
  }
  *ds = ds_;
  *v = v_;
  return true;
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

void STM32HatSerial::RunMainLoop(ControlCallback *cb) {
  uint16_t last_wpos, wheeldt;
  timeval last_t;

  AwaitSync(&last_wpos, &wheeldt);
  gettimeofday(&last_t, NULL);
  for (;;) {
    uint16_t wpos;
    if (!AwaitSync(&wpos, &wheeldt)) {
      continue;
    }
    timeval t;
    gettimeofday(&t, NULL);

    uint16_t wheel_delta = last_wpos - wpos;
    last_wpos = wpos;
    ds_ = meters_per_tick_ * wheel_delta;
    if (wheel_delta == 0) {
      v_ = 0;
    } else if (wheeldt > meters_per_tick_ * 1e6 / 30.0) {
      // occasionally the firmware outputs a ridiculously small but nonzero
      // wheel period, so restrict to reasonable values (< 30m/s max)
      v_ = meters_per_tick_ * 1e6 / wheeldt;
    }
    float dt = t.tv_sec - last_t.tv_sec + (t.tv_usec - last_t.tv_usec) * 1e-6;

    if (!cb->OnControlFrame(this, dt)) {
      break;
    }
  }
}

bool STM32HatSerial::AwaitSync(uint16_t *encoder_pos, uint16_t *encoder_dt) {
  uint8_t buf[32];
  if (!sync_) {
    for (;;) {
      int n = read(fd_, buf, 1);
      if (n < 1) {
        if (n == -1) {
          perror(device_);
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
    if (read_fully(fd_, buf + 1, 5)) {
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
  *encoder_dt = buf[3] + (buf[4] << 8);

  return true;
}
