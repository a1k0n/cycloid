#include <string>

#include "hw/car/car.h"
#include "hw/car/stm32rs232.h"
#include "inih/cpp/INIReader.h"

CarHW *CarHW::GetCar(const INIReader &ini) {
  std::string carif = ini.GetString("car", "interface", "");
  if (carif == "stm32rs232") {
    return new STM32HatSerial(ini);
  } else {
    fprintf(stderr, "Please add [car] interface=<pigpio/stm32rs32/etc> to cycloid.ini\n");
     return NULL;
  }
}