import annotate
import numpy as np
import struct
from math import sqrt

imgsiz = (640, 480)

import argparse

def read_frame(f):
    buf = f.read(4)
    if len(buf) < 4:
        return False, None
    siz, = struct.unpack("=I", buf)
    buf = f.read(siz - 4)
    if len(buf) < siz - 4:
        print("Failed to unpack: ", len(buf), siz)
        return False, None
    header = struct.unpack("=IIbbffffffBHHHHHHHH", buf[:51])
    tstamp = header[0] + header[1] / 1000000.
    throttle, steering = header[2:4]
    accel = np.float32(header[4:7])
    gyro = np.float32(header[7:10])
    servo = header[10]
    wheels = np.uint16(header[11:15])
    periods = np.uint16(header[15:19])

    ptr = 51
    nparticles, = struct.unpack("=I", buf[ptr:ptr+4])

    # for particles w/ heading
    particles = np.frombuffer(buf[55:55+nparticles*16], np.float32).reshape((-1, 4))
    ptr += 4 + nparticles*16

    # previous version: particles without heading
    # particles = np.frombuffer(buf[55:55+nparticles*12], np.float32).reshape((-1, 3))
    # ptr += 4 + nparticles*12

    controldata = struct.unpack("=17f", buf[ptr:ptr+68])

    ptr += 68
    ncones, = struct.unpack("=I", buf[ptr:ptr+4])
    conesx = np.frombuffer(buf[4:4+ncones*4], np.int32)
    ptr += 4 + ncones*4

    frame = np.frombuffer(buf[-640*480 - 320*240*2:], np.uint8)

    carstate = (throttle, steering, accel, gyro, servo, wheels, periods)

    record = (tstamp, carstate, particles, controldata, conesx, frame)

    return True, record


def parse_args():
    p = argparse.ArgumentParser()
    
    p.add_argument('-i', '--interactive', dest='interactive', action='store_true',
            help='Pause after every frame (default true)')
    p.add_argument('-ni', '--no-interactive', dest='interactive', action='store_false')
    p.set_defaults(feature=True)

    p.add_argument('-o', '--output', type=str, default=None,
            help='Video file to save to')
    p.add_argument('recordfile', type=str, help='Recording file from car')
    return p.parse_args()

if __name__ == '__main__':
    import sys
    import cv2
    import params

    args = parse_args()

    f = open(args.recordfile)
    vidout = None
    if args.output: 
        vidout = cv2.VideoWriter(args.output, cv2.VideoWriter_fourcc(
            'X', '2', '6', '4'), 28.8, (640, 480), True)
    t0 = None
    n = 0
    variance = 0.0 
    max_dt = 0.0
    while True:
        ok, data = read_frame(f)
        if not ok:
            print("read_frame failed")
            break

        tstamp, carstate, particles, controldata, conesx, frame = data
        bgr = cv2.cvtColor(
            frame.reshape((-1, imgsiz[0])), cv2.COLOR_YUV2BGR_I420)
        if t0 is None:
            t0 = tstamp
            tstamp_last = tstamp 
            dt = 0
        else:
            n += 1
            dt = tstamp - tstamp_last
            tstamp_last = tstamp

        variance += dt**2
        max_dt= max(dt, max_dt)

        annotate.draw_steering(bgr, carstate[1], carstate[4])
        annotate.draw_speed(bgr, tstamp, carstate[5], carstate[6])
        annotate.draw_throttle(bgr, carstate[0])
        cv2.imshow("camera", bgr)
        if vidout is not None:
            vidout.write(bgr)
        print(carstate)

        # draw the particle filter state
        partview = np.zeros((240, 450), np.uint8)
        partxy = (25*particles[:, :2]).astype(np.int)
        inview = ((partxy[:, 0] >= 0) & (partxy[:, 0] < 450) &
                  (partxy[:, 1] <= 0) & (partxy[:, 1] > -240))
        partview[(-partxy[inview, 1], partxy[inview, 0])] = 255
        cv2.imshow("particles", partview)

        for x in conesx:
            cv2.line(bgr, (x, params.vpy), (x, params.vpy + params.bandheight),
                     (0, 255, 255), 2)

        (x, y, theta, vf, vr, w, ierr_v, ierr_w, delta,
         target_k, target_v, target_w, ye, psie, k, bw_w, bw_v) = controldata
        print 'xy', x, y, 'theta', theta, 'ye', ye, 'psie', psie, 'k', k
        print 'target_k', target_k, 'target_v', target_v, target_k*target_v
        print 'target_w', target_w, 'w', w

        if args.interactive:
            k = cv2.waitKey()
            if k == ord('q'):
                break
    avg_hz = float(n) / (tstamp - t0)
    avg = float(tstamp - t0) / n
    stddev = sqrt(variance - avg**2 ) / n 
    stddev_percent = 100. * stddev / avg
    print("Average: {:.2f} (fps) / {:.2f} (ms), Standard Deviation: {:.2f} (ms) / {:.2f} (%), Max dT: {:.2f} (ms), Total Frames: {}".format(
        avg_hz, avg * 1000, stddev * 1000, stddev_percent, max_dt * 1000,  n))

    f.close()
