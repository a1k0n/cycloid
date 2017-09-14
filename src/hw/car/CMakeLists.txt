add_library(car teensy.cc)

add_executable(servotest servotest.cc)
target_link_libraries(servotest car gpio)
