add_executable(gpsdrive
  main.cc
  config.h
  config.cc
  gpsdrive.cc
  gpsdrive.h
)

target_link_libraries(gpsdrive car input gpio imu ui lcd inih pthread pigpio)
install(TARGETS gpsdrive DESTINATION bin)

