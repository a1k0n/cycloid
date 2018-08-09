import numpy as np
import ekf
import pickle


np.set_printoptions(suppress=True)
data = pickle.load(open("20180804-194415.cones.pickle"))
prelabels = None
try:
    prelabels = pickle.load(open("labeled_cones.txt"))
except Exception:
    pass
# relative landmark locations, with unknown scale -- pixel measurements
# from a google maps screenshot
L0 = np.array([
    [0, 0],
    [0, 296],
    [0, 482],
    [0, 778],
    [716, 0],
    [716, 778],
    [425, 296],
    [425, 482],
])

L0[:, 1] -= 389
a = 0.875  # fudge factor for scale w.r.t. ticks
L = L0*a


def main():
    x, P = ekf.initial_state()
    P = np.diag([100, 100, 0.01])
    last_wheels = data[0][0][6]
    dt = 1.0 / 30

    i = 0
    totalL = 0
    while i < len(data):
        d = data[i]

        gyro = d[0][4][2]
        dw = d[0][6] - last_wheels
        last_wheels = d[0][6]
        ds = 0.25 * np.sum(dw)
        # print x, dt, ds, gyro
        x, P = ekf.predict(x, P, dt, ds, gyro)

        for n, z in enumerate(data[i][1]):
            zz = np.arctan(z[0])
            if prelabels and len(prelabels) > i:
                j = prelabels[i][n]
            else:
                break
            ll = L[j]

            R = np.eye(3) * 5
            R[0, 0] = 0.02  # 1*(d[2][n] - 1)
            x, P, LL = ekf.update_lm_bearing(x, P, -zz, ll[0], ll[1], R)
            totalL += LL

        i += 1

    return totalL


if __name__ == '__main__':
    print main()
