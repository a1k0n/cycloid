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

    while True:
        framesiz = 320*140 + 55
        buf = f.read(framesiz)
        if len(buf) < framesiz:
            break
        header = struct.unpack("=IIIbbffffffBHHHHHHHH", buf[:55])
        tstamp = header[1] + header[2] / 1000000.
        throttle, steering = header[3:5]
        accel = np.float32(header[5:8])
        gyro = np.float32(header[8:11])
        servo = header[11]
        wheels = np.uint16(header[12:16])
        periods = np.uint16(header[16:20])
        frame = np.frombuffer(buf[55:], np.uint8).reshape((-1, 320))

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

        m, hv, th, B, yc, Rk = imgproc.detect_centerline(frame)


        LL_center, LL_imu, LL_encoders = 0, 0, 0

        if B is not None:
            # No Rk augmentation: [  1511.82892081  -1151.05519279 -13387.46449827]
            # Rk *= 2  # [   774.77745676  -1149.97915352 -13387.34062435]
            # Rk *= 0.5  # [  2157.30426716  -1146.89230878 -13384.45517675]
            # Rk *= 0.25  # [  2720.73178577  -1142.60890088 -13384.52882874]
            # Rk *= 0.125  # [  3102.00202631  -1131.43688061 -13383.10351163]
            # Rk -> divide by number of activations [  1683.43728218  -1118.14023456 -13379.9952535 ]
            # Rk[:3, :3] / 16.0  -> [  1628.07800111  -1111.44020082 -13378.9630254 ]
            # Rk[:3, :3] / 8.0  -> [  2008.41358695  -1133.03634863 -13382.78859215]
            # Rk /= N -> 

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

        # gyrozs.append(gyro[2])
        # print 'gyro', gyro[2], 'mean', np.mean(gyrozs), 'std', np.std(gyrozs)

        print 'LL', LL_center, LL_imu, LL_encoders
        LLsum += [LL_center, LL_imu, LL_encoders]
    return LLsum


if __name__ == '__main__':
    import sys

    print replay_LL(sys.argv[1], open(sys.argv[1]))
