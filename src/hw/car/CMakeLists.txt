#add_library(car teensy.cc)

#add_executable(servotest servotest.cc)
#target_link_libraries(servotest car gpio)

add_library(car stm32i2c.cc stm32rs232.cc)
