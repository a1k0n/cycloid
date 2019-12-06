add_executable(cf cf.cc fit.h pid.h)
target_link_libraries(cf car input gpio imu inih pigpio pthread)

add_executable(motorlearn motorlearn.cc)
target_link_libraries(motorlearn car gpio imu inih pigpio pthread)

add_definitions(-DTESTDATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/testdata")
add_executable(motorcontrol_test motorcontrol_test.cc)
add_test(mc motorcontrol_test)
