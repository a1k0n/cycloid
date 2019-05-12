import annotate
import chunk
import numpy as np
import struct
from math import sqrt

imgsiz = (640, 480)

def read_frame(f):
    try:
        ck = chunk.Chunk(f, False, False, True)
    except EOFError:
        return False, None
    if ck.getname() != b'CYCF':
        print("Not a cycloid IFF log file (got ", ck.getname(), "?)")
        return False, None

    # read timestamp header
    ts = struct.unpack("=II", ck.read(8))

    framedata = {
        'tstamp': ts[0] + ts[1] / 1000000.
    }

    # read all embedded chunks
    while True:
        try:
            ick = chunk.Chunk(ck, False, False, True)
        except EOFError:
            break
        n = ick.getname()
        if n == b'CSta':  # car state
            data = struct.unpack("=bbffffffBHHHHHHHH", ick.read())
            throttle, steering = data[0:2]
            accel = np.float32(data[2:5])
            gyro = np.float32(data[5:8])
            servo = data[8]
            wheels = np.uint16(data[9:13])
            periods = np.uint16(data[13:17])
            framedata['carstate'] = (throttle, steering, accel, gyro, servo, wheels, periods)
        elif n == b'MCL4':  # monte carlo localization, 4-float state (particles w/ heading)
            framedata['particles'] = np.frombuffer(ick.read(), np.float32).reshape((-1, 4))
        elif n == b'aCDF':  # activation CDF, new thing
            framedata['activations'] = np.frombuffer(ick.read(), np.int32)
        elif n == b'LM01':  # expected landmark location
            numL, = struct.unpack('B', ick.read(1))
            c0c1 = np.frombuffer(ick.read(), np.uint16).reshape((-1, numL))
            nP = c0c1.shape[0] // 2
            framedata['c0'] = c0c1[:nP]
            framedata['c1'] = c0c1[nP:]
        elif n == b'CTLs':  # controller state
            framedata['controldata'] = struct.unpack("=17f", ick.read())
        elif n == b'CTL2':  # controller state
            framedata['controldata'] = struct.unpack("=26f", ick.read())
        elif n == b'Y420':  # YUV420 frame
            w, = struct.unpack('=H', ick.read(2))
            framedata['yuv420'] = np.frombuffer(ick.read(), np.uint8).reshape((-1, w))
        else:
            ick.skip()

    return True, framedata


class RecordIterator:
    def __init__(self, f):
        self.f = f

    def __iter__(self):
        return self

    def __next__(self):
        ok, data = read_frame(self.f)
        if not ok:
            raise StopIteration
        return data

    def next(self):
        return self.__next__()


def ang2pix(lat, lon):
    theta = np.pi/2 - lat
    theta = theta*(1 + dist_coeffs[0]*theta**2)
    x = (fx*np.outer(theta, np.sin(-lon)) + cx)
    y = (-fy*np.outer(theta, np.cos(-lon)) + cy)
    return np.stack([x, y]).transpose(1, 2, 0).astype(np.float32)


def parse_args():
    import argparse

    p = argparse.ArgumentParser()
    
    p.add_argument('-i', '--interactive', dest='interactive', action='store_true',
            help='Pause after every frame (default true)')
    p.add_argument('-ni', '--no-interactive', dest='interactive', action='store_false')

    p.add_argument('-o', '--output', type=str, default=None,
            help='Video file to save to')
    p.add_argument('-r', '--remap', dest='remap', action='store_true',
                   help='remap frame to equirectangular projection')
    p.add_argument('-e', '--exposure', type=float, default=1.0, help='exposure compensation')
    p.add_argument('recordfile', type=str, help='Recording file from car')
    return p.parse_args()

if __name__ == '__main__':
    import sys
    import cv2
    import params

    args = parse_args()

    remap = None

    if args.remap:
        camera_matrix = np.load("../../tools/camcal/camera_matrix.npy")
        dist_coeffs = np.load("../../tools/camcal/dist_coeffs.npy")
        # downconvert camera matrix from 2592x1944
        fx, fy, _ = np.diag(camera_matrix) / 4.05
        cx, cy = camera_matrix[:2, 2] / 4.05
        print('camera calibration: fx', fx, 'fy', fy, 'cx', cx, 'cy', cy, 'k1', dist_coeffs[0])
        f32map = ang2pix(np.linspace(np.pi/2, -np.pi/2, 1024),
                         np.linspace(-np.pi + np.pi/4, np.pi + np.pi/4, 2048))
        remap = cv2.convertMaps(f32map, None, cv2.CV_16SC2)


    f = open(args.recordfile, "rb")
    vidout = None
    if args.output: 
        siz = (640, 480)
        if remap is not None:
            siz = (remap[0].shape[1], remap[0].shape[0])
        vidout = cv2.VideoWriter(args.output, cv2.VideoWriter_fourcc(
#            'M', 'J', 'P', 'G'), 30, siz, True)
            'X', '2', '6', '4'), 30, siz, True)
        if vidout is None:
            sys.exit(1)

    t0 = None
    n = 0
    variance = 0.0 
    max_dt = 0.0
    ang = 0
    while True:
        ok, data = read_frame(f)
        if not ok:
            print("read_frame failed")
            break

        tstamp = data['tstamp']
        carstate = data['carstate']
        particles = data['particles']
        controldata = data['controldata']
        frame = data['yuv420']
        bgr = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_I420)
        bgr = np.clip(bgr * args.exposure, 0, 255).astype(np.uint8)
        anncenter = (320, 420)
        if remap is not None:
            print(ang)
            f32map = ang2pix(np.linspace(np.pi/2, -np.pi/2, 1024),
                             np.linspace(-np.pi + np.pi/4 + ang, np.pi + np.pi/4 + ang, 2048))
            remap = cv2.convertMaps(f32map, None, cv2.CV_16SC2)
            bgr = cv2.remap(bgr, remap[0], remap[1], cv2.INTER_LINEAR)
            anncenter = (bgr.shape[1]//2, 600)
        if t0 is None:
            t0 = tstamp
            tstamp_last = tstamp 
            dt = 0
        else:
            n += 1
            dt = tstamp - tstamp_last
            tstamp_last = tstamp

        ang += carstate[3][2]*dt
        variance += dt**2
        max_dt= max(dt, max_dt)

        annotate.draw_steering(bgr, carstate[1], carstate[4], anncenter)
        annotate.draw_speed(bgr, tstamp, carstate[5], carstate[6], (anncenter[0] - 100, anncenter[1]))
        annotate.draw_throttle(bgr, carstate[0], (anncenter[0], anncenter[1] + 50))
        if args.interactive:
            cv2.imshow("camera", bgr)
        if vidout is not None:
            vidout.write(bgr)
        print(carstate)

        if 'activations' in data:
            a = data['activations'][1:] - data['activations'][:-1]
            a += 128*14
            a = np.clip(a / 14, 0, 255).astype(np.uint8)
            N = a.shape[0]
            a = cv2.resize(a[None, :], (N, 30))
            if 'c0' in data:
                a[21:, data['c0'] % N] = 255
                a[11:20, data['c1'] % N] = 255
            cv2.imshow("acts", a)

        # draw the particle filter state
        partview = np.zeros((240, 450), np.uint8)
        partxy = (25*particles[:, :2]).astype(np.int)
        inview = ((partxy[:, 0] >= 0) & (partxy[:, 0] < 450) &
                  (partxy[:, 1] <= 0) & (partxy[:, 1] > -240))
        partview[(-partxy[inview, 1], partxy[inview, 0])] = 255
        if args.interactive:
            cv2.imshow("particles", partview)

        (x, y, vf, vr, w, prevsteer, prevthrottle, ierr_k, target_v, target_w, bw_w, bw_v) = controldata[:12]
        print('xy', x, y)
        #print('target_k', target_k, 'target_v', target_v, target_k*target_v)
        print('target_w', target_w, 'w', w)

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
