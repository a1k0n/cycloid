import annotate
import numpy as np
import struct

imgsiz = (640, 480)


def read_frame(f):
    buf = f.read(4)
    if len(buf) < 4:
        return False, None
    siz, = struct.unpack("=I", buf)
    buf = f.read(siz - 4)
    if len(buf) < siz - 4:
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


if __name__ == '__main__':
    import sys
    import cv2
    import params

    f = open(sys.argv[1])
    vidout = None
    if len(sys.argv) > 2:
        vidout = cv2.VideoWriter("replay.h264", cv2.VideoWriter_fourcc(
            'X', '2', '6', '4'), 28.8, (640, 480), True)
    t0 = None
    n = 0
    while True:
        ok, data = read_frame(f)
        if not ok:
            break

        tstamp, carstate, particles, controldata, conesx, frame = data
        bgr = cv2.cvtColor(
            frame.reshape((-1, imgsiz[0])), cv2.COLOR_YUV2BGR_I420)
        if t0 is None:
            t0 = tstamp
        else:
            n += 1

        annotate.draw_steering(bgr, carstate[1], carstate[4])
        annotate.draw_speed(bgr, tstamp, carstate[5], carstate[6])
        annotate.draw_throttle(bgr, carstate[0])
        cv2.imshow("camera", bgr)
        if vidout is not None:
            vidout.write(bgr)

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

        k = cv2.waitKey()
        if k == ord('q'):
            break
    print 'avg fps', float(n) / (tstamp - t0)

    f.close()
