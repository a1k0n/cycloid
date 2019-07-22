add_library(ceiltrack ceiltrack.h ceiltrack.cc)

add_definitions(-DTESTDATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/testdata")
add_executable(ceiltrack_test localize_test.cc)
target_link_libraries(ceiltrack_test ceiltrack z)
