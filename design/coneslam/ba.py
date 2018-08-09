# bundle adjustment with sparse cholesky solver / levenberg-marquardt
import numpy as np
import scipy.sparse
import scipy.sparse.linalg
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
    ds = 0.25*np.sum(ds, 1)
    xy = np.zeros((len(data), 2))
    xy[1:, 0] = np.cumsum(np.cos(theta[0:-1]) * ds)
    xy[1:, 1] = np.cumsum(np.sin(theta[0:-1]) * ds)
    # plt.plot(xy[:, 0], xy[:, 1])

    # constraints between poses:
    #  - gyro constraint (theta_t - theta_t-1 = gyro[t]*dt)
    #  - distance constraint (xy_t - xy_t-1)**2 = ds[t]
    #  - velocity constraint (xy_t - xy_t-1)**2 = v[t]*dt
    #      ^ these can be combined into a weighted estimate of ds
    # constraints on landmarks:

    return np.vstack([xy.T, theta]).T


def solvemodel(data, X, L):
    num_constraints = 2*(len(data)-1) # one for distance, one for angle for each state transition
    for d in data:
        num_constraints += len(d[1])  # number of landmarks
    print 'num_constraints', num_constraints

    gyro = np.array([d[0][4] for d in data])
    wheels = np.array([d[0][6] for d in data])

    gyro_bias = np.mean(gyro[0:25, 2])
    dt = 1.0 / 30
    ds = wheels[1:] - wheels[:-1]
    ds = 0.5*ds[:, 0] + 0.25*ds[:, 1] + 0.25*ds[:, 3]  # hack for broken wheel encoder

    def J_and_r(poses, landmarks):  # jacobian and residuals
        err = 0
        # our error function is:
        #  for each data point t=1..N:
        #   = |X[t, 0] - X[t-1, 1]|^2 - ds[X[t]]^2
        #   + X[t, 2] - X[t-1, 2] - gyro[t]*dt
        #   + for each landmark l:
        #     min(l - z(L[i,:2], X[t,:2], theta)
        row_ind, row = [], 0
        col_ind = []
        # Nposes = np.len(data)
        Nlandmarks = len(landmarks)
        off_L = 0
        off_X = 2*Nlandmarks
        Ncols = off_X + 3*len(X)
        B = []
        # so we have 3*Nposes + 2*Nlandmarks
        Adata = []
        for i, d in enumerate(data):
            s, c = np.sin(poses[i, 2]), np.cos(poses[i, 2])
            x1 = landmarks[:, 0] - poses[i, 0]
            x3 = landmarks[:, 1] - poses[i, 1]
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
                # w = 5*float(d[2][n] - 3)**-2  # relative weight
                w = 0.1
                err += w*e[j]

                # Landmark bearing constraint
                if i == 0:
                    Adata.extend([w*x3[j]*x5[j], -w*x1[j]*x5[j]])  # Lx, Ly
                    row_ind.extend([row, row])
                    col_ind.extend([off_L + 2*j, off_L + 2*j+1])
                    B.append(w*r[j])  # residual
                    row += 1

                else:
                    Adata.extend([w*x3[j]*x5[j], -w*x1[j]*x5[j],  # Lx, Ly
                                  -w*x3[j]*x5[j], w*x1[j]*x5[j],
                                  -(x1[j]**2 + x3[j]**2)*x5[j]])  # Xx, Xy, Xtheta
                    row_ind.extend([row, row, row, row, row])
                    col_ind.extend([off_L + 2*j, off_L + 2*j+1,
                                    off_X + 3*i, off_X + 3*i+1, off_X + 3*i+2])
                    B.append(w*r[j])  # residual
                    row += 1

            if i > 1:
                # Velocity constraint (X0+ds*cos(theta) - x1)**2 + (Y0+ds*sin(theta) - Y1)**2 = 0
                X1, Y1 = X[i, 0], X[i, 1]
                X0, Y0 = X[i-1, 0], X[i-1, 1]
                theta0, theta1 = X[i-1, 2], X[i, 2]
                v = ds[i-1]
                sin, cos = np.sin(theta0), np.cos(theta0)

                # jacobian X0, Y0, theta0
                Wodom = 1  # stddev is about 1 encoder tick on this one
                Adata.extend(Wodom * np.array([1, -1, -v*sin]))
                row_ind.extend([row, row, row])
                col_ind.extend([off_X + (i-1)*3, off_X + i*3, off_X + (i-1)*3 + 2])  # x0, x1, theta0
                B.append(Wodom * (X0+v*cos - X1))
                err += Wodom*(X0+v*cos - X1)**2
                row += 1

                Adata.extend(Wodom * np.array([1, -1, v*cos]))
                row_ind.extend([row, row, row])
                col_ind.extend([off_X + (i-1)*3 + 1, off_X + i*3 + 1, off_X + (i-1)*3 + 2])  # y0, y1, theta0
                B.append(Wodom * (Y0+v*sin - Y1))
                err += Wodom*(Y0+v*sin - Y1)**2
                row += 1
                
                # FIXME: weight / error measure
                # Gyro constraint (theta1 - theta0 = (gyro - gyro_bias)*dt)
                Wgyro = 100  # gyro errors are pretty small
                Adata.extend(Wgyro * np.array([1, -1]))
                row_ind.extend([row, row])
                col_ind.extend([off_X + i*3 + 2, off_X + (i-1)*3 + 2])
                B.append(Wgyro*(theta1 - theta0 - dt*(gyro[i-1, 2] - gyro_bias)))
                err += Wgyro*(theta1 - theta0 - dt*(gyro[i-1, 2] - gyro_bias))**2
                row += 1

        A = scipy.sparse.csr_matrix((Adata, (row_ind, col_ind)), shape=(row, Ncols))
        return err, A, np.array(B)

    damp = 1e-6
    lasterr = 1e10
    lastL = L
    lastX = X
    iterno = 0
    while True:
        err, J, r = J_and_r(X, L)

        if err > lasterr:
            print 'backing up', err, damp
            L = lastL
            X = lastX
            err = lasterr
            damp *= 1.5
        else:
            damp /= 1.5

        result = scipy.sparse.linalg.lsqr(J, r, damp)
        adj = result[0]
        r1norm = result[3]
        L -= adj[:2*len(L)].reshape((-1, 2))
        X -= adj[2*len(L):].reshape((-1, 3))
        errstep = np.dot(adj, adj)
        print err, r1norm, errstep, damp
        print 'X step', np.dot(adj[2*len(L):], adj[2*len(L):])
        print 'L step', np.dot(adj[:2*len(L)], adj[:2*len(L)])
        lastL, lastX, lasterr = L, X, err
        if errstep < 1e-6:
            break
        iterno += 1
        if (iterno % 20) == 0:
            print 'saving...'
            pickle.dump(X, open('Xlast.pickle', 'wb'))
            pickle.dump(L, open('Llast.pickle', 'wb'))
    return X, L


if __name__ == '__main__':
    data = pickle.load(open("190858.cones.pickle"))
    X = initial_pose(data)
    L = np.array([[150.0, 210], [150, 20], [-200, 200], [-200, 0]])
    X, L = solvemodel(data, X, L)
    pickle.dump(X, open('X.pickle', 'wb'))
    pickle.dump(L, open('L.pickle', 'wb'))
