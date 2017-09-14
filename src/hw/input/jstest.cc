#include <stdio.h>
#include <unistd.h>

#include "input/js.h"

int main() {
  JoystickInput js;

  if (!js.Open()) {
    return 1;
  }

  for (;;) {
    int t = 0, s = 0;
    uint16_t b = 0;
    if (js.ReadInput(&t, &s, &b)) {
      printf("%6d %6d %4x\r", t, s, b);
      fflush(stdout);
    }
    usleep(10000);
  }
}
