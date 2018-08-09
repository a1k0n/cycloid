import cv2
import numpy as np
import struct


imgsiz = 640 * 480 + 2 * 320 * 240
framesiz = 55 + imgsiz

vpy = 212  # vanishing point y coordinate ; could be determined from Rdown

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


def draw_steering(bgr, steering, servo):
    # make steering wheel, lower center
    servo = 128*(servo - 125)/70.0
    # sdeg = steering  # just 1:1 i guess?
    sdeg = servo  # just 1:1 i guess?
    srad = sdeg * np.pi / 180.0
    S, C = 16*30*np.sin(srad), 16*30*np.cos(srad)
    cv2.circle(bgr, (320, 420), 30, (255, 255, 255), 1, cv2.LINE_AA)
    center = (320*16, 420*16)
    cv2.line(bgr, (int(center[0] - C), int(center[1] + S)),
             (int(center[0] + C), int(center[1] - S)),
             (255, 255, 255), 1, cv2.LINE_AA, 4)
    cv2.ellipse(bgr, (320, 420), (30, 30), 0, -90, -90 - steering,
                (255, 180, 180), 5, cv2.LINE_AA)
    cv2.ellipse(bgr, (320, 420), (30, 30), 0, -90, -90 - servo,
                (0, 180, 255), 2, cv2.LINE_AA)


last_ts = None
last_wheels = None


def draw_speed(bgr, tstamp, wheels, periods):
    # draw a little spedometer in the lower left
    # just draw the needle for each period now
    global last_ts, last_wheels

    center = (40, 420)
    radius = 30

    METERS_PER_ENCODER_TICK = np.pi * 0.101 / 20
    v = 2 * METERS_PER_ENCODER_TICK * 1e6 / np.float32(periods)
    # cv2.putText(bgr, "%0.1f %0.1f %0.1f %0.1f m/s" % tuple(v), (10, 470),
    #             cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1,
    #             cv2.LINE_AA)

    if last_ts is None:
        last_ts = tstamp
        last_wheels = wheels
        return

    dw = wheels - last_wheels
    if np.all(dw == 0):
        last_ts = tstamp
        last_wheels = wheels
        return
    # vv = METERS_PER_ENCODER_TICK * np.float32(dw) / (tstamp - last_ts)
    # av = 0.5 * np.mean(v[dw != 0] + vv[dw != 0])
    av = np.mean(v[dw != 0])

    mph = 2.23694 * av

    # draw ticks
    for i in range(13):
        phi = (i - 6) * 0.4
        C, S = radius * np.cos(phi), radius * np.sin(phi)
        cv2.line(bgr, (int(center[0] + S), int(center[1] - C)),
                 (int(center[0] + 0.8*S), int(center[1] - 0.8*C)),
                 (255, 255, 255), 1, cv2.LINE_AA)

    phi = (mph - 6) * 0.4
    C, S = radius * np.cos(phi), radius * np.sin(phi)
    cv2.line(bgr, (int(center[0] + S), int(center[1] - C)),
             (int(center[0]), int(center[1])),
             (180, 255, 180), 2, cv2.LINE_AA)

    cv2.putText(bgr, "%0.1f mph" % (mph), (30, 460),
                cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1,
                cv2.LINE_AA)
    last_ts = tstamp
    last_wheels = wheels


def replay(fname, f):
    udmap1, udmap2, udmapsd = init_remap()

    while True:
        ok, record = read_frame(f)
        (tstamp, throttle, steering, accel, gyro, servo,
         wheels, periods, frame) = record
        if not ok:
            break

        bgr = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_I420)
        bgr[vpy:vpy+4] = 255-bgr[vpy:vpy+4]

        # show throttle / steering
        cv2.line(bgr, (320, 470), (320 + throttle, 470),
                 throttle > 0 and (0, 255, 0) or (0, 95, 255), 5)
        draw_steering(bgr, steering, servo)
        draw_speed(bgr, tstamp, wheels, periods)

        cv2.imshow("frame", bgr)

        k = cv2.waitKey()
        if k == ord('q'):
            break


if __name__ == '__main__':
    import sys

    replay(sys.argv[1], open(sys.argv[1]))
