import numpy as np
import struct
import imgproc
import ekf

np.set_printoptions(suppress=True)


def replay_LL(fname, f):
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

    LLsum = np.zeros(3)

    steertrim = open("steertrim.txt", "w")
    likelihood = open("likelihood.txt", "w")

    while True:
        imgsiz = imgproc.bucketcount.shape[0] * imgproc.bucketcount.shape[1] * 3
        framesiz = 55 + imgsiz
        buf = f.read(framesiz)
        if len(buf) < framesiz:
            break
        header = struct.unpack("=IIIbbffffffBHHHHHHHH", buf[:55])
        if header[0] != framesiz:
            print "recording frame size doesn't match this parser"
            raise Exception("image should be %d bytes (%dx%d), is %d bytes" % (
                (framesiz,) + imgproc.bucketcount.shape + (header[0],)))
        tstamp = header[1] + header[2] / 1000000.
        throttle, steering = header[3:5]
        accel = np.float32(header[5:8])
        gyro = np.float32(header[8:11])
        servo = header[11]
        wheels = np.uint16(header[12:16])
        periods = np.uint16(header[16:20])
        frame = np.frombuffer(buf[55:], np.uint8).reshape(
            (imgproc.bucketcount.shape[0], imgproc.bucketcount.shape[1], 3))
        frame = np.int32(frame)

        frameno += 1

        if t0 is not None:
            dt = tstamp - t0

        x0, P0 = np.copy(x), np.copy(P)

        t = (last_throttle + 2*throttle) / 3.0
        s = last_steering
        #x, P = ekf.predict(x, P, dt, throttle / 127.0, steering / 127.0)
        x, P = ekf.predict(x, P, dt, t / 127.0, s / 127.0)
        last_throttle, last_steering = throttle, steering
        # print 'x_predict\n', x
        xpred, Ppred = np.copy(x), np.copy(P)

        hv, th, B, yc, Rk = imgproc.detect_centerline(frame[:, :, 1])


        LL_center, LL_imu, LL_encoders = 0, 0, 0

        if B is not None:
            x, P, LL_center = ekf.update_centerline(x, P, B[0], B[1], B[2], yc, Rk)
            # print 'x_centerline\n', x

        # print 'accel', accel
        # print 'gyro', gyro[2]
        x, P, LL_imu = ekf.update_IMU(x, P, gyro[2])
        # print 'x_gyro\n', x

        # print 'wheels', wheels, 'periods', periods
        if wheels_last is not None:
            ds = np.sum(wheels - wheels_last) / 4.0
            if ds != 0:
                x, P, LL_encoders = ekf.update_encoders(x, P, ds/dt, float(servo))
                # print 'x_encoders\n', x
            else:
                x, P, LL_encoders = ekf.update_encoders(x, P, 0, float(servo))
                # print 'x_encoders\n', x
        wheels_last = wheels

        print >>steertrim, x[0], x[1], last_steering, gyro
        print >>likelihood, LL_center, LL_imu, LL_encoders

        # gyrozs.append(gyro[2])
        # print 'gyro', gyro[2], 'mean', np.mean(gyrozs), 'std', np.std(gyrozs)

        print 'LL', LL_center, LL_imu, LL_encoders
        LLsum += [LL_center, LL_imu, LL_encoders]

    print 'final x\n', x
    print 'final P\n', np.diag(P)
    return LLsum


if __name__ == '__main__':
    import sys

    print replay_LL(sys.argv[1], open(sys.argv[1]))
