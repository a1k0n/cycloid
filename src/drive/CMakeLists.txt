add_executable(drive drive.cc imgproc.cc controller.cc ekf.cc)
# localize.cc
target_link_libraries(drive car cam mmal input gpio imu ui lcd)

# add_executable(localize_test localize_test.cc localize.cc)
