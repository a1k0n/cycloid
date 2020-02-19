#include <stdio.h>
#include <unistd.h>

#include "hw/input/js.h"
#include "inih/cpp/INIReader.h"

class TestInputReceiver : public JoystickListener {
  void OnDPadPress(char direction) {
    printf("DPadPress   %c\n", direction);
  }

  void OnDPadRelease(char direction) {
    printf("DPadRelease %c\n", direction);
  }

  void OnButtonPress(char button) {
    printf("ButtonPress   %c\n", button);
  }
  void OnButtonRelease(char button) {
    printf("ButtonRelease %c\n", button);
  }

  void OnAxisMove(int axis, int16_t value) {
    printf("Axis %d %d\n", axis, value);
  }
};

int main() {
  JoystickInput js;
  INIReader ini("cycloid.ini");

  if (!js.Open(ini)) {
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
