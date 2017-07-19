#ifndef HW_GPIO_I2C_H_
#define HW_GPIO_I2C_H_

#include <stdint.h>

class I2C {
 public:
  ~I2C() { Close(); }

  bool Open();
  void Close();

  bool Write(uint8_t addr, uint8_t reg, uint8_t value) const;
  bool Write(uint8_t addr, uint8_t reg, int len, const uint8_t *buf) const;
  bool Read(uint8_t addr, uint8_t reg, int len, uint8_t *outbuf) const;

 private:
  int fd_;
};

#endif  // HW_GPIO_I2C_H_
