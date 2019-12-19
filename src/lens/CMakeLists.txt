add_library(lens fisheye.cc fisheye.h)

add_definitions(-DTESTDATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/testdata")
add_executable(fisheye_test fisheye_test.cc)
target_link_libraries(fisheye_test lens)