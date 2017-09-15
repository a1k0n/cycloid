#ifndef HW_LCD_FBDEV_H_
#define HW_LCD_FBDEV_H_

// Framebuffer device for ILI9340/ILI9341 320x240 LCD on SPI
// Using Linux fbtft device:
// $ sudo modprobe fbtft_device name=adafruit22a rotate=90

// The framebuffer device automatically handles syncing to the LCD with a page
// fault on the mmap'd buffer.

#include <stdint.h>
#include <stdlib.h>

class LCDScreen {
 public:
  LCDScreen() { fd_ = -1; framebuf_ = NULL; }
  ~LCDScreen() { if (fd_ != -1) Close(); }

  bool Open();
  void Close();

  uint16_t *GetBuffer() { return framebuf_; }

 private:
  int fd_;
  uint16_t *framebuf_;
};

#endif  // HW_LCD_FBDEV_H_
