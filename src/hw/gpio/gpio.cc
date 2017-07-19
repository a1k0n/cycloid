#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "./gpio.h"

#define BLOCK_SIZE               (4*1024)
#define BCM2708_PERI_BASE        0x20000000L
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000)

volatile unsigned *gpio;

bool gpio_init() {
  /* open /dev/mem */
  int mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
  if (mem_fd < 0) {
    perror("can't open /dev/mem");
    return false;
  }

  void* gpio_map = mmap(
      NULL,                  // Any adddress in our space will do
      BLOCK_SIZE,            // Map length
      PROT_READ|PROT_WRITE,  // Enable reading & writting to mapped memory
      MAP_SHARED,            // Shared with other processes
      mem_fd,                // File to map
      GPIO_BASE);  // Offset to GPIO peripheral

  close(mem_fd);

  if (gpio_map == MAP_FAILED) {
    perror("mmap /dev/mem");
    return false;
  }

  gpio = (volatile unsigned *)gpio_map;

  return true;
}
