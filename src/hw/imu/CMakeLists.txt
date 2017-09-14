add_library(imu mpu9150.cc)

add_executable(imu_log imu_log.cc)
target_link_libraries(imu_log imu gpio)
