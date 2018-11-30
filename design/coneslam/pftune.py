# particle filter coneslam (just using FastSLAM algorithm but without any trees
# as we have a relatively small number of landmarks)
import numpy as np
import pickle
import pf


def main(data, seed):
    np.random.seed(seed)
    Np = 1000
    X = np.zeros((3, Np))
    X[:2] = 30*np.random.randn(2, Np)
    X[2] = np.random.randn(Np) * 0.2
    last_wheels = data[0][0][6]
    tstamp = data[0][0][0] - 1.0 / 30

    totalL = 0

    for d in data:
        ts = d[0][0]
        dt = tstamp - ts
        tstamp = ts
        gyro = d[0][4][2]
        dw = d[0][6] - last_wheels
        last_wheels = d[0][6]
        ds = 0.25*np.sum(dw)
        pf.step(X, dt, ds, gyro)

        for n, z in enumerate(d[1]):
            # mark the cone on the original view
            zz = np.arctan(z[0])
            j, LL = pf.likeliest_lm(X, pf.L, zz)
            # score the upper quartile of particles
            # tradeoff between exploration and exploitation here
            totalL += np.sum(np.sort(LL)[-10:])
            # resample the particles based on their landmark likelihood
            X = pf.resample_particles(X, LL)

    print totalL
    return totalL


if __name__ == '__main__':
    #data = pickle.load(open("20180804-194415.cones.pickle"))
    #data = pickle.load(open("20180817-232656.cones.pickle"))
    #data = pickle.load(open("20180817-232546.cones.pickle"))
    Ls = []
    for seed in range(10):
        Ls.append(main(data, seed))

    print np.mean(Ls), 'std', np.std(Ls)
    print 'score for angular', pf.NOISE_ANGULAR, 'longitudinal', pf.NOISE_LONG, 'lateral', pf.NOISE_LAT, 'landmark selectivity', pf.LM_SELECTIVITY
    print '2-std score', np.mean(Ls) - 2*np.std(Ls)
    print 'worst case', np.min(Ls)
