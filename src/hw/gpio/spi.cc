#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "gpio/spi.h"

bool SPIDev::open(const char *devname) {
  fd = ::open(devname, O_RDWR);
  if (fd == -1) {
    perror(devname);
    return false;
  }
  // we use the default settings of these ioctls so they are omitted:
  // SPI_IOC_WR_MODE          (CPHA=0, CPOL=0, CS_HIGH=0, LSB_FIRST=0, etc)
  // SPI_IOC_WR_LSB_FIRST     (no, msb first)
  // SPI_IOC_WR_BITS_PER_WORD (8)
  // SPI_IOC_WR_MAX_SPEED_HZ  (500kHz)
  return true;
}

SPIDev::~SPIDev() {
  if (fd != -1)
    close(fd);
}

int SPIDev::xfer(const uint8_t* txbuf, uint8_t* rxbuf, int len) {
  int ret;
  struct spi_ioc_transfer tr;
  memset(&tr, 0, sizeof(tr));
  tr.tx_buf = (uint64_t)txbuf;
  tr.rx_buf = (uint64_t)rxbuf;
  tr.len = len;
  tr.delay_usecs = 0;
  tr.speed_hz = 500000;
  tr.bits_per_word = 8;
  tr.cs_change = 1;

  ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
  if (ret < 1) {
    perror("can't send spi message");
    return -1;
  }

  return ret;
}
