import cv2
import numpy as np
import struct

import annotate
from params import vpy, bandheight

imgsiz = 640 * 480 + 2 * 320 * 240
framesiz = 55 + imgsiz

np.set_printoptions(suppress=True)


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


def init_remap():
    camera_matrix = np.load("../../tools/camcal/camera_matrix.npy")
    dist_coeffs = np.load("../../tools/camcal/dist_coeffs.npy")
    Rdown = np.load("../../tools/camcal/Rdown.npy")
    camera_matrix[:2] /= 4.  # for 640x480

    camera_mm_scale = 153  # tunable parameter to scale 1mm/pixel
    pixel_scale_mm = 25  # but we use 25mm buckets to detect 50mm line position

    undistort_map = cv2.fisheye.undistortPoints(
        np.mgrid[:640, :480].T.astype(np.float32),
        camera_matrix, dist_coeffs)

    f = camera_mm_scale / pixel_scale_mm
    imgsiz = (128, 64)
    new_camera_matrix = np.array([
        [-f, 0, imgsiz[0] / 2],
        [0, f, imgsiz[1]],
        [0, 0, 1]], np.float32)

    udmap1, udmap2 = cv2.fisheye.initUndistortRectifyMap(
        camera_matrix, dist_coeffs, Rdown, new_camera_matrix,
        imgsiz, cv2.CV_16SC2)
    return udmap1, udmap2, undistort_map


def replay(fname, f):
    udmap1, udmap2, udmapsd = init_remap()

    while True:
        ok, record = read_frame(f)
        (tstamp, throttle, steering, accel, gyro, servo,
         wheels, periods, frame) = record
        if not ok:
            break

        bgr = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_I420)
        bgr[vpy:vpy+bandheight] = 255-bgr[vpy:vpy+bandheight]

        annotate.draw_throttle(bgr, throttle)
        annotate.draw_steering(bgr, steering, servo)
        annotate.draw_speed(bgr, tstamp, wheels, periods)

        cv2.imshow("frame", bgr)

        k = cv2.waitKey()
        if k == ord('q'):
            break


if __name__ == '__main__':
    import sys

    replay(sys.argv[1], open(sys.argv[1]))
