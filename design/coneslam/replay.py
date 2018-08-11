import numpy as np
import cv2
import ekf
import recordreader
import pickle


VIDEO = False

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
    f = open("home20180804/cycloid-20180804-194415.rec")
    bg = cv2.imread("satview.png")

    camera_matrix = np.load("../../tools/camcal/camera_matrix.npy")
    dist_coeffs = np.load("../../tools/camcal/dist_coeffs.npy")
    camera_matrix[:2] /= 4.  # for 640x480

    x, P = ekf.initial_state()
    P = np.diag([100, 100, 0.01])
    last_wheels = data[0][0][6]
    dt = 1.0 / 30

    done = False
    i = 0
    if VIDEO:
        vidout = cv2.VideoWriter("replay.h264", cv2.VideoWriter_fourcc(
            'X', '2', '6', '4'), 30, (bg.shape[1], bg.shape[0]), True)
    while not done and i < len(data):
        ok, frame = recordreader.read_frame(f)
        if not ok:
            break
        d = data[i]

        gyro = d[0][4][2]
        dw = d[0][6] - last_wheels
        last_wheels = d[0][6]
        ds = 0.25 * np.sum(dw)
        # print x, dt, ds, gyro
        x, P = ekf.predict(x, P, dt, ds, gyro)

        bgr = cv2.cvtColor(frame[-1], cv2.COLOR_YUV2BGR_I420)

        mapview = bg.copy()
        x0, y0 = x[:2] / a
        dx, dy = 20*np.cos(x[2]), 20*np.sin(x[2])
        U, V, _ = np.linalg.svd(P[:2, :2])
        axes = np.sqrt(V) * 0.5
        angle = np.arctan2(U[0, 1], U[0, 0]) * 180 / np.pi
        # print axes, angle, T[i, 2]
        cv2.ellipse(mapview, (123+int(x0), 475-int(y0)), (int(axes[0]), int(axes[1])),
                    angle, 0, 360, (0, 0, 220), 1)
        cv2.circle(mapview, (123+int(x0), 475-int(y0)), 3, (0, 0, 220), 2)
        cv2.line(mapview, (123+int(x0), 475-int(y0)), (123+int(x0+dx), 475-int(y0+dy)), (0, 0, 220), 2)
        for l in range(len(L)):
            lxy = (123+int(L[l, 0]/a), 475-int(L[l, 1]/a))
            cv2.circle(mapview, lxy, 3, (0, 128, 255), 3)
            cv2.putText(mapview, "%d" % l, (lxy[0] + 3, lxy[1] + 3), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)

        for n, z in enumerate(data[i][1]):
            zz = np.arctan(z[0])
            if prelabels and len(prelabels) > i:
                j = prelabels[i][n]
            else:
                break
            ll = L[j]
            ll = ll/a
            cv2.line(mapview, (123+int(x0), 475-int(y0)), (123+int(ll[0]), 475-int(ll[1])), (255,128,0), 1)
            p = cv2.fisheye.distortPoints(np.array([[z]], np.float32), camera_matrix, dist_coeffs)[0][0]
            cv2.circle(bgr, (int(p[0]), int(p[1])), 8, (255, 200, 0), cv2.FILLED)

            R = np.eye(3) * 5
            R[0, 0] = 0.02  # 1*(d[2][n] - 1)

            ll = L[j]
            x, P, LL = ekf.update_lm_bearing(x, P, -zz, ll[0], ll[1], R)
            ll = ll/a
            cv2.line(mapview, (123+int(x0), 475-int(y0)), (123+int(ll[0]), 475-int(ll[1])), (0,255,0), 1)

        cv2.putText(mapview, "%d" % i, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        if VIDEO:
            mapview[:240, 160:480] = bgr[::2, ::2]
            vidout.write(mapview)
        else:
            cv2.imshow("raw", bgr)
            cv2.imshow("map", mapview)
            k = cv2.waitKey(1)
            if k == ord('q'):
                break

        i += 1

    f.close()


if __name__ == '__main__':
    main()
