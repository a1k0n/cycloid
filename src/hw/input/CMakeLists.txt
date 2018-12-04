add_library(input js.cc)

add_executable(jstest jstest.cc)
target_link_libraries(jstest input)

install(TARGETS jstest DESTINATION bin)