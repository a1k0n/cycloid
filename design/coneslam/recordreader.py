import numpy as np
import struct

imgsiz = 640 * 480 + 2 * 320 * 240
framesiz = 55 + imgsiz


def read_frame(f):
    buf = f.read(framesiz)
    if len(buf) < framesiz:
        return None, None
    header = struct.unpack("=IIIbbffffffBHHHHHHHH", buf[:55])
    tstamp = header[1] + header[2] / 1000000.
    throttle, steering = header[3:5]
    accel = np.float32(header[5:8])
    gyro = np.float32(header[8:11])
    servo = header[11]
    wheels = np.uint16(header[12:16])
    periods = np.uint16(header[16:20])
    frame = np.frombuffer(buf[55:], np.uint8).reshape(-1, 640)

    record = (tstamp, throttle, steering, accel, gyro, servo,
              wheels, periods, frame)

    return True, record
