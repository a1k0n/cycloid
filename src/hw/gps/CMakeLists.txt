add_library(gps ubx.h ubx.cc)

add_executable(ubx_main ubx_main.cc)
target_link_libraries(ubx_main gps)
