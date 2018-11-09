add_library(imu mpu9150.cc)

add_executable(mpu9150 mpu9150_main.cc)
target_link_libraries(mpu9150 imu gpio)
