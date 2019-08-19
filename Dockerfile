FROM a1k0n/cycloid-build:v0.1.0 AS build

# Make the number of compile jobs configurable.
ARG jobs=8

# Copy all the app source into docker context
COPY . /build/cycloid

WORKDIR /build/o
# LIBRARY_TYPE is a custom way of 'userland' to switch between static/shared.
# There is also a 'vcos' library whose static behavior can be controlled through
# VCOS_PTHREADS_BUILD_SHARED; however setting this to FALSE does not work (obvious link error).
# In order not to modify 'userland', we just set the library install target correctly.
# VMCS_INSTALL_PREFIX.
RUN cmake /build/cycloid/src \
  -DCMAKE_TOOLCHAIN_FILE=/build/cycloid/crosscompile.travis \
  -DBUILD_SHARED_LIBS=OFF -DLIBRARY_TYPE=STATIC \
  -DCMAKE_INSTALL_PREFIX=/usr/local -DVMCS_INSTALL_PREFIX=/usr/local
RUN cmake --build . -- --jobs=$jobs

# Travis fails on the resin image if 'RUN' is used with "exec format error".
# FROM resin/rpi-raspbian:stretch
# COPY --from=build /usr/local /usr/local
# 
# RUN ldconfig
# #switch on systemd init system in container
# ENV INITSYSTEM on
