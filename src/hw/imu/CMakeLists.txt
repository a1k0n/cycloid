add_library(imu imu.cc invensense.cc imu.h invensense.h)
add_library(mag mag.cc mag.h hmc5883l.cc hmc5883l.h)

add_executable(imutest imu_main.cc)
target_link_libraries(imutest imu gpio inih)

install(TARGETS imutest DESTINATION bin)
