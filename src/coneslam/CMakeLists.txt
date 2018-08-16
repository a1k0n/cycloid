add_library(coneslam localize.cc)

add_executable(localize_test localize_test.cc)
target_link_libraries(localize_test coneslam)
