add_executable(drive drive.cc controller.cc trajtrack.cc)
target_link_libraries(drive car cam mmal input gpio imu ui lcd coneslam)
install(TARGETS drive DESTINATION bin)

# add_executable(localize_test localize_test.cc localize.cc)
add_executable(trajtrack_test trajtrack_test.cc trajtrack.cc)
install(TARGETS trajtrack_test DESTINATION bin)

add_executable(controller_test controller_test.cc controller.cc trajtrack.cc)
install(TARGETS controller_test DESTINATION bin)