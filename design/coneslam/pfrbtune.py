# particle filter coneslam (just using FastSLAM algorithm but without any trees
# as we have a relatively small number of landmarks)
import numpy as np
import pickle
import pfrb


def main(data, seed):
    np.random.seed(seed)
    Np = 1000
    X = np.zeros((3, Np))
    L = np.zeros((5, pfrb.NUM_CONES, Np))
    L[0] = 400
    L[2] = 1000**2
    L[4] = 1000**2
    L[:2, 0] = (pfrb.L0.T*pfrb.a)[:, 7, None]
    L[2, 0] = 0.001  # anchor the first seen cone location
    L[4, 0] = 0.001

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
        ds = 0.5 * np.sum(dw[:2])  # front wheels only!
        pfrb.step(X, dt, ds, gyro)

        LL = None
        for n, z in enumerate(d[1]):
            # mark the cone on the original view
            zz = np.arctan(z[0])
            newLL = pfrb.pf_update(X, L, zz, pfrb.NOISE_BEARING)
            if LL is None:
                LL = newLL
            else:
                LL += newLL

        # resample particles after all observations
        if LL is not None:
            # resample the particles based on their landmark likelihood
            X, L = pfrb.resample_particles(X, L, LL)

            # for mapping, we only care about the single best particle
            # for localization we need a different explore/exploit tradeoff so
            # our mean position is always reasonable
            totalL += np.max(LL)

    print totalL
    return totalL


if __name__ == '__main__':
    data = pickle.load(open("20180804-194415.cones.pickle"))
    Ls = []
    for seed in range(10):
        Ls.append(main(data, seed))

    print np.mean(Ls), 'std', np.std(Ls)
    print 'score for angular', pfrb.NOISE_ANGULAR, 'longitudinal', pfrb.NOISE_LONG, 'lateral', pfrb.NOISE_LAT, 'bearing measure error', pfrb.NOISE_BEARING
    print '2-std score', np.mean(Ls) - 2*np.std(Ls)
    print 'worst case', np.min(Ls)
