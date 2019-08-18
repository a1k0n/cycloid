# Implementation notes: The below is purposefully more verbose than necessary.
# However, by definining `imu` to be only an INTERFACE library, we guarantee
# that all targets get compiled by default.
add_library(imu imu.cc icm20600.cc mpu9150.cc imu.h icm20600.h mpu9150.h)

add_executable(icm20600_main icm20600_main.cc icm20600.cc)
target_link_libraries(icm20600_main gpio)

add_executable(mpu9150_main mpu9150_main.cc mpu9150.cc)
target_link_libraries(mpu9150_main gpio)

add_executable(imutest icm20600_main.cc icm20600.cc)

install(TARGETS icm20600_main DESTINATION bin)
install(TARGETS mpu9150_main DESTINATION bin)
