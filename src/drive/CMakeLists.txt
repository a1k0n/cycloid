add_executable(drive drive.cc controller.cc trajtrack.cc)
target_link_libraries(drive car cam mmal input gpio imu ui lcd coneslam)

# add_executable(localize_test localize_test.cc localize.cc)
add_executable(trajtrack_test trajtrack_test.cc trajtrack.cc)
