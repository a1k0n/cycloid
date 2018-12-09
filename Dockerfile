#FROM resin/rpi-raspbian:stretch AS build
FROM mitchallen/pi-cross-compile AS build

# Make the number of compile jobs configurable.
ARG jobs=8

# Install build tools and remove apt-cache afterwards
RUN apt-get -q update
RUN apt-get install -yq --no-install-recommends \
  build-essential \
  cmake \
  libeigen3-dev \
  pkg-config

# Install debugging packages. TODO Remove once this Dockerfile is stable.
RUN apt-get install -yq --no-install-recommends \
  cmake-curses-gui \
  git \
  vim
RUN apt-get clean 

# Copy all the app source into docker context
COPY . /usr/cycloid
WORKDIR /usr/cycloid

RUN cat crosscompile.cmake.patch >> crosscompile.cmake

# Build our binary
WORKDIR /usr/build
ENV PATH "$PATH:/pitools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin"
# LIBRARY_TYPE is a custom way of 'userland' to switch between static/shared.
# There is also a 'vcos' library whose static behavior can be controlled through
# VCOS_PTHREADS_BUILD_SHARED; however setting this to FALSE does not work (obvious link error).
# In order not to modify 'userland', we just set the library install target correctly.
# VMCS_INSTALL_PREFIX.
RUN cmake /usr/cycloid/src \
  -DCMAKE_TOOLCHAIN_FILE=/usr/cycloid/crosscompile.cmake\
  -DBUILD_SHARED_LIBS=OFF -DLIBRARY_TYPE=STATIC \
  -DCMAKE_INSTALL_PREFIX=/usr/local -DVMCS_INSTALL_PREFIX=/usr/local
RUN cmake --build . -- --jobs=$jobs
RUN cmake --build . -- --jobs=$jobs install

# Travis fails on the resin image if 'RUN' is used with "exec format error".
# FROM resin/rpi-raspbian:stretch
# COPY --from=build /usr/local /usr/local
# 
# RUN ldconfig
# #switch on systemd init system in container
# ENV INITSYSTEM on
# 
# CMD drive
