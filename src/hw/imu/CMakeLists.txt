# newer board revisions use ICM20600 instead of MPU-9250
# FIXME(asloane): make this configurable somehow without modifying makefiles

add_library(imu icm20600.cc)

add_executable(icm20600 icm20600_main.cc)
target_link_libraries(icm20600 imu gpio)

add_executable(mpu9150 mpu9150_main.cc mpu9150.cc)
target_link_libraries(mpu9150 gpio)

install(TARGETS icm20600 DESTINATION bin)
install(TARGETS mpu9150 DESTINATION bin)
