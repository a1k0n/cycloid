add_library(imu imu.cc invensense.cc imu.h invensense.h)

add_executable(imutest imu_main.cc)
target_link_libraries(imutest imu gpio inih)

install(TARGETS imutest DESTINATION bin)
