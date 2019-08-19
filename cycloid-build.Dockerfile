FROM a1k0n/rpi3-gcc8:v0.1.0 AS build

# Make the number of compile jobs configurable.
ARG jobs=8

# Install build tools and remove apt-cache afterwards
RUN apt-get -q update
RUN apt-get install -yq --no-install-recommends \
  build-essential \
  cmake \
  libeigen3-dev \
  wget \
  pkg-config

# Build our binary
WORKDIR /build/src
ENV PATH "$PATH:/build/x-tools/armv8-rpi3-linux-gnueabihf/bin/"
RUN wget -O - http://www.zlib.net/zlib-1.2.11.tar.gz | tar zxvf -
WORKDIR /build/src/zlib-1.2.11
RUN CHOST=armv8-rpi3-linux-gnueabihf ./configure && make && make install prefix=/build/x-tools/armv8-rpi3-linux-gnueabihf/armv8-rpi3-linux-gnueabihf/sysroot/usr
