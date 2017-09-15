#include <stdio.h>
#include <unistd.h>

#include "hw/input/js.h"

#include <stdio.h>
#include <unistd.h>

#include "hw/input/js.h"

class TestInputReceiver : public InputReceiver {
  void OnDPadPress(char direction) {
    printf("DPadPress   %c\n", direction);
  }

  void OnDPadRelease(char direction) {
    printf("DPadRelease %c\n", direction);
  }

  void OnButtonPress(char button) {
    printf("ButtonPress   %d\n", button);
  }
  void OnButtonRelease(char button) {
    printf("ButtonRelease %d\n", button);
  }

  void OnAxisMove(int axis, int16_t value) {
    printf("Axis %d %d\n", axis, value);
  }
};

int main() {
  JoystickInput js;

  if (!js.Open()) {
    return 1;
  }

  TestInputReceiver tir;

  for (;;) {
    int t = 0, s = 0;
    uint16_t b = 0;
    js.ReadInput(&tir);
    usleep(10000);
  }
}
