add_library(coneslam localize.cc imgproc.cc)

add_executable(localize_test localize_test.cc)
target_link_libraries(localize_test coneslam)

add_executable(imgproc_test imgproc_test.cc)
target_link_libraries(imgproc_test coneslam)

#add_test(imgproc imgproc_test)
add_test(localize localize_test)