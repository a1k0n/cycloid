import cv2
import numpy as np

import ekf
import imgproc
import recordreader

np.set_printoptions(suppress=True)


def draw_measurement(timg, B, ps, color, thick):
    # curvature k = x'y'' - y'x'' / (x'^2 + y'^2)^(3/2)
    # y = t
    # y' = 1
    # y'' = 0
    # x = at^2 + bt + c
    # x' = 2at + b
    # x'' = 2a
    # curvature @ t=0
    # 2a / (b^2 + 1)^(3/2)
    # tan(psi_e) = x'/y'
    # x0 = c

    pixel_scale_m = 0.025
    po = ps / 2  # pixel offset

    sy = timg.shape[0] / ps
    sx = timg.shape[1] / ps

    if B is not None:
        pp = np.poly1d(B)
        for i in range(sy):
            y1 = sy - i
            x1 = sx / 2 + pp(i*pixel_scale_m) / pixel_scale_m
            y2 = sy - i - 1
            x2 = sx / 2 + pp((i + 1)*pixel_scale_m) / pixel_scale_m
            cv2.line(timg, (int(ps*x1 + po), int(ps*y1 + po)),
                     (int(ps*x2 + po), int(ps*y2 + po)), color, thick)


def draw_state(timg, x, ps, weight):
    v, delta, ye, psi_e, kappa = x[:5]

    pixel_scale_m = 0.025
    po = ps / 2  # pixel offset

    sy = timg.shape[0] / ps
    sx = timg.shape[1] / ps

    # draw a circle for ye/psie/kappa
    if abs(kappa) > 1e-4:
        rr = 1.0 / kappa - ye
        cx, cy = rr * np.cos(psi_e), -rr * np.sin(psi_e)
        cx, cy = cx / pixel_scale_m, cy / pixel_scale_m
        r = ps / (np.abs(kappa) * pixel_scale_m)
        cx, cy = ps*(sx / 2 + cx) + po, ps*(sy - cy) + po
        cv2.circle(timg, (int(cx), int(cy)), int(r), (255, 255, 0), weight)
        # cv2.circle(timg, (int(cx), int(cy)), 4, color, thick+1)
        #cv2.line(timg, (int((sx / 2 + -ye/pixel_scale_m)*ps + po),
        #                int(sy * ps)), (int(cx), int(cy)), (255, 255, 0), weight)
    
    def proj(x, y):
        x = ps * (sx / 2 + x / pixel_scale_m)
        y = ps * (sy - y / pixel_scale_m)
        return (int(x), int(y))

    # draw an arc for v/delta
    vx, vy, theta = 0, 0, 0
    dt = 1.0/30
    for i in range(10):
        tt = theta + dt * delta / 2.0
        theta += dt * delta
        ny = vy + v * dt * np.cos(tt)
        nx = vx + v * dt * np.sin(tt)
        cv2.line(timg, proj(vx, vy), proj(nx, ny), (0, 180, 255), weight)
        vx, vy = nx, ny
        

def replay(fname, f):
    vidout = None

    x, P = ekf.initial_state()
    t0 = None
    dt = 1.0 / 30
    # gyrozs = []
    wheels_last = None
    frameno = 0

    statelist = []
    encoderlist = []
    tlist = []
    last_throttle = 0
    last_steering = 0

    LL_center, LL_imu, LL_encoders = 0, 0, 0

    while True:
        ok, record = recordreader.read_frame(f)
        (tstamp, throttle, steering, accel, gyro, servo,
         wheels, periods, frame) = record
        if not ok:
            break

        bgr = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR)
        bgrbig = cv2.resize(bgr[::-1], None, fx=8, fy=8,
                            interpolation=cv2.INTER_NEAREST)
        frame = np.int32(frame)
        print 'frame.shape', frame.shape
        cv2.imshow("frame", bgrbig)
        # cv2.waitKey()

        frameno += 1
        print fname, 'frame', frameno

        if t0 is not None:
            dt = tstamp - t0

        x0, P0 = np.copy(x), np.copy(P)

        t = (0.7*last_throttle + 0.3*throttle)
        s = last_steering
        x, P = ekf.predict(x, P, dt, t / 127.0, s / 127.0)

        last_throttle, last_steering = throttle, steering
        #print 'x_predict\n', x
        xpred, Ppred = np.copy(x), np.copy(P)

        hv, th, B, yc, Rk = imgproc.detect_centerline(frame[:, :, 1])

        if B is not None:
            x, P, LL_center = ekf.update_centerline(x, P, B[0], B[1], B[2], yc, Rk)
            #print 'x_centerline\n', x

        #print 'accel', accel
        #print 'gyro', gyro[2]
        x, P, LL_imu = ekf.update_IMU(x, P, gyro[2])
        #print 'x_gyro\n', x

        #print 'wheels', wheels, 'periods', periods
        if wheels_last is not None:
            ds = np.sum(wheels - wheels_last) / 4.0
            if ds != 0:
                x, P, LL_encoders = ekf.update_encoders(x, P, ds/dt, float(servo))
                #print 'x_encoders\n', x
            else:
                x, P, LL_encoders = ekf.update_encoders(x, P, 0, float(servo))
                #print 'x_encoders\n', x

        # gyrozs.append(gyro[2])
        # print 'gyro', gyro[2], 'mean', np.mean(gyrozs), 'std', np.std(gyrozs)

        #print 'LL', LL_center, LL_imu, LL_encoders

        #print 'k', x[4], 1.0 / P[4, 4]

        timg = cv2.resize(
            th[::-1],
            (320, int(320 * th.shape[0] / th.shape[1])),
            interpolation=cv2.INTER_NEAREST)
        timg = np.uint8(timg * (255.0 / np.max(timg)))
        mimg = cv2.resize(
            bgr[::-1],
            (320, int(320 * bgr.shape[0] / bgr.shape[1])),
            interpolation=cv2.INTER_NEAREST)
        hvimg = cv2.resize(
            hv[::-1],
            (320, int(320 * hv.shape[0] / hv.shape[1])),
            interpolation=cv2.INTER_NEAREST)

        vidframe = np.zeros((timg.shape[0] + frame.shape[0], 640, 3), np.uint8)
        if vidout is None:
            vidout = cv2.VideoWriter("replay.h264", cv2.VideoWriter_fourcc(
                'X', '2', '6', '4'), 30, (640, vidframe.shape[0]), True)
        # vidframe[:frame.shape[0], :320, 0] = 255 - frame
        # vidframe[:frame.shape[0], :320, 1] = 255 - frame
        # vidframe[:frame.shape[0], :320, 2] = 255 - frame
        vidframe[:mimg.shape[0]:, 320:, :] = mimg
        vidframe[-hvimg.shape[0]:, :320, 0] = 128 - hvimg * 0.25
        vidframe[-hvimg.shape[0]:, :320, 1] = 128 - hvimg * 0.25
        vidframe[-hvimg.shape[0]:, :320, 2] = 128 - hvimg * 0.25
        vidframe[frame.shape[0]:, 320:, 0] = timg
        vidframe[frame.shape[0]:, 320:, 1] = timg
        vidframe[frame.shape[0]:, 320:, 2] = timg

        vidscale = timg.shape[1] / th.shape[1]

        cv2.line(vidframe, (160, 130), (160 + throttle, 130),
                 (0, 255, 0), 3)
        cv2.line(vidframe, (160, 135), (160 + steering, 135),
                 (255, 180, 180), 3)

        cv2.putText(vidframe, "YUV -U channel", (10, 12),
                    cv2.FONT_HERSHEY_PLAIN, 0.9, (200, 255, 255))

        cv2.putText(vidframe, "birdseye, 25mm/pixel", (330, 12),
                    cv2.FONT_HERSHEY_PLAIN, 0.9, (200, 255, 255))

        cv2.putText(vidframe, "horizontal convolution w/ kernel",
                    (10, 140 + 12),
                    cv2.FONT_HERSHEY_PLAIN, 0.9, (200, 255, 255))
        cv2.putText(vidframe, "[-1, -1, 2, 2, -1, -1]",
                    (10, 140 + 12 + 12),
                    cv2.FONT_HERSHEY_PLAIN, 0.9, (200, 255, 255))

        cv2.putText(vidframe, "ReLU after conv, quadratic fit",
                    (330, 140 + 12),
                    cv2.FONT_HERSHEY_PLAIN, 0.9, (200, 255, 255))

        if B is not None:
            # ye = -B[2]
            # psi_e = np.arctan(B[1])
            # k = 2*B[0] / (np.sqrt(B[1]**2 + 1)**3)
            draw_measurement(
                vidframe[frame.shape[0]:, 320:],
                B, vidscale, (0, 255, 0), 1)

        # draw_state(vidframe[frame.shape[0]:, 320:], x[:5], vidscale, 2)
        draw_state(vidframe[frame.shape[0]:, 320:], xpred[:5], vidscale, 2)
        try:
            # U = np.linalg.cholesky(P[:5, :5]).T
            U = np.linalg.cholesky(Ppred[:5, :5]).T
            # now we render x, and each x + U[i] / x - U[i]

            for i in range(5):
                draw_state(vidframe[frame.shape[0]:, 320:],
                           x[:5] + U[i], vidscale, 1)
                draw_state(vidframe[frame.shape[0]:, 320:],
                           x[:5] - U[i], vidscale, 1)
        except np.linalg.linalg.LinAlgError:
            pass

        #print 'x5   ', x[:5]
        #print 'Prec5', 1.0 / np.diag(P[:5, :5])

        vidout.write(vidframe)
        cv2.imshow('f', vidframe)
        k = cv2.waitKey()
        if k == ord('q'):
            break
        if k == ord(',') and len(statelist) > 0:  # previous frame
            x, P = statelist[-1]
            wheels_last = encoderlist[-1]
            #print 'wheels_last', wheels_last
            t0 = tlist[-1]
            statelist.pop()
            encoderlist.pop()
            tlist.pop()
            f.seek(-recordreader.framesiz*2, 1)
            frameno -= 2
        else:
            statelist.append((x0, P0))
            encoderlist.append(wheels_last)
            tlist.append(t0)
            wheels_last = wheels
            t0 = tstamp


if __name__ == '__main__':
    import sys

    replay(sys.argv[1], open(sys.argv[1]))
