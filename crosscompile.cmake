SET(CMAKE_SYSTEM_NAME Linux)

#SET(CMAKE_C_COMPILER armv7-rpi2-linux-gnueabihf-gcc)
#SET(CMAKE_CXX_COMPILER armv7-rpi2-linux-gnueabihf-g++)
#SET(CMAKE_ASM_COMPILER armv7-rpi2-linux-gnueabihf-gcc)

# for compiling tests, install zlib to target environment:
# cd zlib-1.2.11
# CHOST=armv8-rpi3-linux-gnueabihf ./configure
# make
# make install prefix=/<path>/armv8-rpi3-linux-gnueabihf-7.2.0/armv8-rpi3-linux-gnueabihf/sysroot/usr

# This compiler requires libstdc++ from gcc 7.2.0 which isn't available in raspbian jessie
SET(CMAKE_C_COMPILER armv8-rpi3-linux-gnueabihf-gcc CACHE STRING "c compiler")
SET(CMAKE_CXX_COMPILER armv8-rpi3-linux-gnueabihf-g++ CACHE STRING "c++ compiler")
SET(CMAKE_ASM_COMPILER armv8-rpi3-linux-gnueabihf-gcc CACHE STRING "asm compiler")
SET(CMAKE_SYSTEM_PROCESSOR arm)

#ADD_DEFINITIONS("-march=armv6")
#add_definitions("-mcpu=arm1176jzf-s -mfpu=vfp -mfloat-abi=hard")
add_definitions("-mcpu=cortex-a53 -mfpu=neon-fp-armv8 -mfloat-abi=hard -funsafe-math-optimizations -mfp16-format=ieee")

# rdynamic means the backtrace should work
IF (CMAKE_BUILD_TYPE MATCHES "Debug")
   add_definitions(-rdynamic)
ENDIF()

# avoids annoying and pointless warnings from gcc
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -U_FORTIFY_SOURCE")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -U_FORTIFY_SOURCE")
SET(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -c")
