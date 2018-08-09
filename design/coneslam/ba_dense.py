# bundle adjustment with sparse cholesky solver / levenberg-marquardt
import numpy as np
import pickle

np.set_printoptions(suppress=True)


def initial_pose(data):
    # (tstamp, throttle, steering, accel, gyro, servo, wheels, periods)
    gyro = np.array([d[0][4] for d in data])
    wheels = np.array([d[0][6] for d in data])

    gyro_bias = np.mean(gyro[0:25, 2])
    dt = 1.0 / 30
    theta = np.concatenate([[0], np.cumsum(gyro[:-1, 2] - gyro_bias) * dt])
    ds = wheels[1:] - wheels[:-1]
    ds = 0.5*ds[:, 0] + 0.25*ds[:, 1] + 0.25*ds[:, 3]  # hack for broken wheel encoder
    xy = np.zeros((len(data), 2))
    xy[1:, 0] = np.cumsum(np.cos(theta[1:]) * ds)
    xy[1:, 1] = np.cumsum(np.sin(theta[1:]) * ds)
    # plt.plot(xy[:, 0], xy[:, 1])

    # constraints between poses:
    #  - gyro constraint (theta_t - theta_t-1 = gyro[t]*dt)
    #  - distance constraint (xy_t - xy_t-1)**2 = ds[t]
    #  - velocity constraint (xy_t - xy_t-1)**2 = v[t]*dt
    #      ^ these can be combined into a weighted estimate of ds
    # constraints on landmarks:

    return np.vstack([xy.T, theta]).T


def solvemodel(data, X, L):
    # gyro = np.array([d[0][4] for d in data])
    wheels = np.array([d[0][6] for d in data])
    ds = wheels[1:] - wheels[:-1]
    ds = 0.5*ds[:, 0] + 0.25*ds[:, 1] + 0.25*ds[:, 3]  # hack for broken wheel encoder
    # dt = 1.0/30.

    num_constraints = 2*(len(data)-1) # one for distance, one for angle for each state transition
    for d in data:
        num_constraints += len(d[1])  # number of landmarks
    print 'num_constraints', num_constraints

    def J_and_r(L):  # jacobian and residuals
        # x is the concatenation of L and X
        err = 0
        # our error function is:
        #  for each data point t=1..N:
        #   = |X[t, 0] - X[t-1, 1]|^2 - ds[X[t]]^2
        #   + X[t, 2] - X[t-1, 2] - gyro[t]*dt
        #   + for each landmark l:
        #     min(l - z(L[i,:2], X[t,:2], theta)
        JJ = np.zeros((4, 2, 2))
        Jr = np.zeros((4, 2))
        for i, d in enumerate(data):
            s, c = np.sin(X[i, 2]), np.cos(X[i, 2])
            x1 = L[:, 0] - X[i, 0]
            x3 = L[:, 1] - X[i, 1]
            z = x1*c - s*x3
            x5 = z**(-2)

            if np.all(z <= 0):
                continue
            validj = np.arange(4)[z > 0]
            for n, l in enumerate(d[1]):  # landmarks
                # compute error for all landmarks, find lowest one
                r = (-s*x1 - c*x3)/z - l[0]
                e = r**2  # squared errors
                jj = np.argmin(e[z > 0])
                j = validj[jj]
                # print i, l[0], e, j, z
                if e[j] > 5:  # ignore outliers
                    continue
                w = float(d[2][n])**-2  # relative weight
                err += w*e[j]
                J = np.array([x3[j]*x5[j], -x1[j]*x5[j]])
                JJ[j] += w*np.outer(J, J)
                Jr[j] += w*np.dot(J, r[j])
        return err, JJ, Jr

    damp = 1e-5
    lasterr = 1e10
    lastL = L
    while True:
        err, JJ, Jr = J_and_r(L)
        if err > lasterr:
            L = lastL
            damp *= 2
            lasterr = err
            continue
        else:
            damp *= 0.5

        errstep = 0
        for j in range(4):
            adj = np.linalg.solve(JJ[j] + np.eye(2) * damp, Jr[j])
            errstep += np.dot(adj, adj)
            L[j] -= adj
        print err, errstep, damp
        lastL, lasterr = L, err
        if errstep < 1e-6:
            break
    print L


data = pickle.load(open("185924.cones.pickle"))
X = initial_pose(data)
L = np.array([[150.0, 210], [150, 0], [-200, 200], [-200, 0]])
solvemodel(data, X, L)
