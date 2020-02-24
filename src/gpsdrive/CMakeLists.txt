add_executable(gpsdrive
  main.cc
  config.h
  config.cc
  gpsdrive.cc
  gpsdrive.h
  trajtrack.cc
  trajtrack.h
)

target_link_libraries(gpsdrive car input gpio gps imu mag ui lcd inih pthread pigpio)
install(TARGETS gpsdrive DESTINATION bin)

