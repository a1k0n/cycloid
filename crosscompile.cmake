SET(CMAKE_SYSTEM_NAME Linux)

SET(CMAKE_C_COMPILER armv7-rpi2-linux-gnueabihf-gcc)
SET(CMAKE_CXX_COMPILER armv7-rpi2-linux-gnueabihf-g++)
SET(CMAKE_ASM_COMPILER armv7-rpi2-linux-gnueabihf-gcc)

# This compiler requires libstdc++ from gcc 5.2.0 which isn't available in raspbian jessie
#SET(CMAKE_C_COMPILER armv8-rpi3-linux-gnueabihf-gcc)
#SET(CMAKE_CXX_COMPILER armv8-rpi3-linux-gnueabihf-g++)
#SET(CMAKE_ASM_COMPILER armv8-rpi3-linux-gnueabihf-gcc)
SET(CMAKE_SYSTEM_PROCESSOR arm)

#ADD_DEFINITIONS("-march=armv6")
#add_definitions("-mcpu=arm1176jzf-s -mfpu=vfp -mfloat-abi=hard")
add_definitions("-mcpu=cortex-a53 -mfpu=neon-fp-armv8 -mfloat-abi=hard -funsafe-math-optimizations")

# rdynamic means the backtrace should work
IF (CMAKE_BUILD_TYPE MATCHES "Debug")
   add_definitions(-rdynamic)
ENDIF()

# avoids annoying and pointless warnings from gcc
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -U_FORTIFY_SOURCE")
SET(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -c")
