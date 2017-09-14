Glorified line-following robot.


Car is a Traxxas Rustler (not VXL), stock ESC, brushed Titan 12T motor,
aftermarket 2S lipo battery.


[![Car driving in my backyard](https://img.youtube.com/vi/rkZvHl7T1OU/0.jpg)](https://www.youtube.com/watch?v=rkZvHl7T1OU)


## Hardware

 - Traxxas Rustler R2R
 - Raspberry Pi 3
 - Teensy 3.2
 - MPU-9250 9-DoF IMU (about $3 on eBay)
 - LM2596S DC-DC buck converter (about $1 on eBay)
 - Custom PCB connecting everything together -- see `hw/pcb/`
 - Moga 2 Bluetooth game controller

Besides the RC car, there are three 3D-printed parts to mount the camera to the
front bumper support, a "base plate" which carries all the PCBs and a "servo
plate" which holds the base plate to the servo bracket; see `hw/scad/`

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

