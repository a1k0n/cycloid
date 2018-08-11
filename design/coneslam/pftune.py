# particle filter coneslam (just using FastSLAM algorithm but without any trees
# as we have a relatively small number of landmarks)
import numpy as np
import pickle
import pf


def main(data, seed):
    np.random.seed(seed)
    Np = 1000
    X = np.zeros((3, Np))
    last_wheels = data[0][0][6]
    dt = 1.0 / 30

    totalL = 0

    for d in data:
        gyro = d[0][4][2]
        dw = d[0][6] - last_wheels
        last_wheels = d[0][6]
        ds = 0.5 * np.sum(dw[:2])  # front wheels only!
        pf.step(X, dt, ds, gyro)

        for n, z in enumerate(d[1]):
            # mark the cone on the original view
            zz = np.arctan(z[0])
            j, LL = pf.likeliest_lm(X, pf.L, zz)
            totalL += np.max(LL)
            # resample the particles based on their landmark likelihood
            X = pf.resample_particles(X, LL)

    print totalL
    return totalL


if __name__ == '__main__':
    data = pickle.load(open("20180804-194415.cones.pickle"))
    Ls = []
    for seed in range(10):
        Ls.append(main(data, seed))

    print np.mean(Ls), 'std', np.std(Ls)
    print '2-std score', np.mean(Ls) + 2*np.std(Ls)
    print 'worst case', np.min(Ls)
