SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_C_COMPILER armv8-rpi3-linux-gnueabihf-gcc CACHE STRING "c compiler")
SET(CMAKE_CXX_COMPILER armv8-rpi3-linux-gnueabihf-g++ CACHE STRING "c++ compiler")
SET(CMAKE_ASM_COMPILER armv8-rpi3-linux-gnueabihf-gcc CACHE STRING "asm compiler")
SET(CMAKE_SYSTEM_PROCESSOR arm)

add_definitions("-mcpu=arm1176jzf-s -mfpu=vfp -mfloat-abi=hard -funsafe-math-optimizations -mfp16-format=ieee")

