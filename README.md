Self-racing robot using traffic cones to localize. Very much a work in progress.

[![Build Status](https://travis-ci.org/a1k0n/cycloid.svg?branch=master)](https://travis-ci.org/a1k0n/cycloid)

[![Racing on a basketball court](https://img.youtube.com/vi/abS4v-PrAvE/0.jpg)](https://www.youtube.com/watch?v=abS4v-PrAvE)

This uses a pre-planned track, and doesn't have any tools for planning the
track yet other than a bunch of ipython notebooks.


## Hardware (updated!)

### Blaze R2 build
I've switched to a 1/10th scale touring car (for flat ground courses) as it
handles much better and also eliminates the very finicky wheel encoders used on
the Rustler.

Touring car:

 - HobbyKing Blaze R2 ($99)
 - Turnigy TrackStar 9.5T sensored brushless motor 4120KV ($34)
 - Turnigy TrackStar 80A brushless sensored ESC ($40)
 - TrackStar TS-411MG digital servo ($17)
 - Basher 6600mAh 2S2P 40C hardcase LiHV pack ($38) (smaller batteries are
   fine, but I like being able to run the computer all day on one charge)

Electronics:

 - Raspberry Pi 3 ($35)
 - SanDisk Ultra 32GB microSDHC UHS-I Card ($9) -- it's important to have the
   right kind of SD card or it won't keep up with recordings, and it's hard to
   tell ahead of time which ones will work.
 - Arducam OV5647 Raspberry Pi 3 camera with LS-40180 fisheye lens ($30)
 - ILI9340 SPI 240x320 LCD display, via the fbtft device driver (about $7)
 - Some XT60 plugs and an extra 6-wire brushless sensor cable (~10)
 - Nintendo Wii U Pro Wireless controller (I use a genuine one connected via
   bluetooth -- apparently knockoffs don't connect to the RPi very well)
 - Custom HAT board w/ STM32F030 and ICM-20600 IMU: https://easyeda.com/a1k0n/cycloid (PCB +
   parts is about $5 apiece in qty 10 from JLPCB / LCSC, plus you have to
   solder it all together)

Total hardware cost comes to under $350, plus the cost of a Wii U Pro
controller and your time building and soldering everything together.

### Traxxas Rustler build

The previous car was a Traxxas Rustler (not VXL), stock ESC, brushed Titan 12T
motor, aftermarket 2S lipo battery.

 - Traxxas Rustler R2R ($170)
 - Raspberry Pi 3 ($35)
 - Teensy 3.2 ($20)
 - MPU-9250 9-DoF IMU (about $3 on eBay)
 - LM2596S DC-DC buck converter (about $1 on eBay)
 - ILI9340 SPI 240x320 LCD display, via the fbtft device driver (about $7)
 - Arducam OV5647 Raspberry Pi 3 camera with LS-40180 fisheye lens ($30)
 - Custom wheel encoders: http://www.blargh.co/2013/12/rc-car-quadrature-wheel-encoders.html
 - Custom PCB connecting everything together -- see `hw/pcb/`
 - Wii U Pro controller (connected via bluetooth)

### 3D printed mounting parts

Besides the RC car, there are three 3D-printed parts to mount the camera to the
front bumper support, a "base plate" which carries all the PCBs and a "servo
plate" which holds the base plate to the servo bracket; see `hw/scad/`. I drill
and tap all the holes with a #4-40 tap set and use regular #4-40 screws (same
used in computer hardware to hold hard drives on, etc).

## Code

Can be compiled on a host PC with a cross compiler (e.g. on macOS you can
install this: https://www.jaredwolff.com/toolchains/) or on the Raspberry Pi
itself.

If you want to build the code, clone with `git clone --recursive
https://github.com/a1k0n/cycloid` in order to get the Raspberry Pi userland
submodule (otherwise just run `git submodule init` and `git submodule update`).

Here's how I build it:

Once you have a raspberry pi cross compiler (see two paragraphs above), edit
`crosscompile.cmake` to point to the correct compiler name / path, and do this:

```
$ mkdir build
$ cd build
$ cmake -DCMAKE_TOOLCHAIN_FILE=../crosscompile.cmake ../src -DCMAKE_BUILD_TYPE=RelWithDebInfo
$ make -j4
```

The main executable will be in `build/drive/drive`; scp that to your raspberry
pi on the car, pair a bluetooth joystick, and run it.

### Configurations
If you are running version 1.2 or below of the Cycloid HAT board, you have a
different IMU (MPU-9250/MPU-9150). You need to enable this in CMake. The option
is called `MPU9250` and can be turned to `ON` by passing `-D MPU9250=ON` in a
cmake call (like the inital configuration), or you can enable it directly
in `ccmake` or `cmake-gui`.

