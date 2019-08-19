add_executable(drive
    drive.cc controller.cc trajtrack.cc
    config.h controller.h trajtrack.h vflookup.h vflookup.cc)

target_link_libraries(drive car cam mmal input gpio imu ui lcd coneslam ceiltrack pigpio inih)
install(TARGETS drive DESTINATION bin)

# add_executable(localize_test localize_test.cc localize.cc)
add_executable(trajtrack_test trajtrack_test.cc trajtrack.cc)
install(TARGETS trajtrack_test DESTINATION bin)

add_executable(controller_test controller_test.cc controller.cc trajtrack.cc vflookup.cc vflookup.h)
target_link_libraries(controller_test coneslam)
install(TARGETS controller_test DESTINATION bin)