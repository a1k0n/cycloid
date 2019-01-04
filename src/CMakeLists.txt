cmake_minimum_required(VERSION 3.1)
project(cycloid)

find_package(PkgConfig)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

add_subdirectory(userland)

pkg_check_modules(EIGEN REQUIRED eigen3)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(userland)
include_directories(userland/interface/vcos/pthreads)
include_directories(userland/interface/vmcs_host/linux)
include_directories(userland/host_applications/linux/libs/bcm_host/include)

add_subdirectory(hw/cam)
add_subdirectory(hw/car)
add_subdirectory(hw/gpio)
add_subdirectory(hw/imu)
add_subdirectory(hw/input)
add_subdirectory(hw/lcd)
add_subdirectory(ui)
add_subdirectory(coneslam)
add_subdirectory(drive)
add_subdirectory(controlloop)
