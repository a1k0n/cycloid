#add_library(car teensy.cc)

#add_executable(servotest servotest.cc)
#target_link_libraries(servotest car gpio)

add_library(car car.h car.cc stm32i2c.cc stm32i2c.h stm32rs232.cc stm32rs232.h)
