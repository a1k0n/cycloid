add_library(lcd fbdev.cc)

add_executable(lcdtest lcdtest.cc)
target_link_libraries(lcdtest lcd)

install(TARGETS lcdtest DESTINATION bin)
