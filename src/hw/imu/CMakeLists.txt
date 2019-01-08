# newer board revisions use ICM20600 instead of MPU-9250
option(
  MPU9250
  "Use the MPU-9250 IMU instead of the ICM20600. The MPU-9250 is only used on Cycloid HAT boards =< 1.2"
  OFF)
# The MPU-9150 and MPU-9250 are virtually identical in this context!

# Implementation notes: The below is purposefully more verbose than necessary.
# However, by definining `imu` to be only an INTERFACE library, we guarantee
# that all targets get compiled by default.
add_library(mpu9150 mpu9150.cc)
add_library(icm20600 icm20600.cc)
add_library(imu INTERFACE)
if(${MPU9250})
  target_link_libraries(imu INTERFACE mpu9150)
else()
  target_link_libraries(imu INTERFACE icm20600)
endif()

add_executable(icm20600_main icm20600_main.cc)
target_link_libraries(icm20600_main icm20600 gpio)

add_executable(mpu9150_main mpu9150_main.cc)
target_link_libraries(mpu9150_main mpu9150 gpio)

install(TARGETS icm20600 DESTINATION bin)
install(TARGETS mpu9150 DESTINATION bin)
