add_library(cam cam.cc)
add_executable(camtest camtest.cc)

target_link_libraries(cam mmal)
target_link_libraries(camtest cam mmal)
