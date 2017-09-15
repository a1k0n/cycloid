#include <fcntl.h>
#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>

#include "hw/lcd/fbdev.h"

static const char *DEVICE = "/dev/fb1";

bool LCDScreen::Open() {
  fd_ = open(DEVICE, O_RDWR);
  if (fd_ == -1) {
    perror(DEVICE);
    return false;
  }

  framebuf_ = reinterpret_cast<uint16_t*>(mmap(
        0, 320*240*2, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0));

  return true;
}

void LCDScreen::Close() {
  if (fd_ != -1) {
    close(fd_);
    fd_ = -1;
  }
}
