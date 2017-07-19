#ifndef HW_GPIO_SPI_H_
#define HW_GPIO_SPI_H_

#include <stdint.h>

class SPIDev {
 public:
  SPIDev() { fd = -1; }
  ~SPIDev();

  bool open(const char *devname);

  // txbuf or rxbuf can be NULL for a one-way transfer
  int xfer(const uint8_t* txbuf, uint8_t* rxbuf, int len);

 private:
  int fd;
};

#endif  // HW_GPIO_SPI_H_
