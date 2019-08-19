add_library(input input.h js.cc js.h)

add_executable(jstest jstest.cc)
target_link_libraries(jstest input inih)

install(TARGETS jstest DESTINATION bin)