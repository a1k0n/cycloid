import numpy as np
import scipy.linalg

# curvature map learned from the track
KMAP_ENTRIES_PER_METER = 5
kmap = np.loadtxt("kmap.txt").T
KMAP_ENTRIES = len(kmap[0])


def prob_s_given_k(k, prec):
    # i need to integrate this term over the distribution of N(k, 1/prec)
    # this is seat of the pants, not sure if it's right.
    # l = kmap[1] * np.exp(-(kmap[0] - k)**2 * (kmap[1] * prec))
    # do i want to include kmap's precision? i am not sure
    l = -(kmap[0] - k)**2 * prec
    l -= np.max(l)
    l = np.exp(l)
    if np.any(l < 0):
        print 'negative likelihood?!', k, prec
        print l
    s = np.sum(l)
    if s == 0:
        return np.ones(KMAP_ENTRIES) / KMAP_ENTRIES
    return l / s


def ds_distribution(x, P, Delta_t):
    v, _, y_e, psi_e, kappa = x[:5]
    x0 = np.cos(psi_e)
    x1 = -kappa*y_e + 1
    x2 = Delta_t/x1
    x3 = x0*x2
    x4 = Delta_t*v*x0/x1**2

    ds = v*x3
    J = np.array([x3, 0, kappa*x4, -v*x2*np.sin(psi_e), x4*y_e])

    cov = np.dot(J, np.dot(P[:5, :5], J.T))

    return ds, cov


def predict(prob_s, x, P, Delta_T):
    ds, cov = ds_distribution(x, P, Delta_T)

    ds *= KMAP_ENTRIES_PER_METER
    cov *= KMAP_ENTRIES_PER_METER**2
    dsi = np.floor(ds)
    dsf = ds - dsi

    print '--------PREDICT ds', ds, 'cov', cov

    N = KMAP_ENTRIES
    a = np.minimum(np.arange(N), np.arange(N, 0, -1))
    a[N//2:] = -a[N//2:]
    dist_conv = scipy.linalg.toeplitz(-a, a)

    # do a gaussian convolution, shifting by the integer part of ds
    ps = np.exp(-(dist_conv - dsi)**2 / cov)
    ps = (ps.T / np.sum(ps, axis=1))

    # now shift it by the fractional part of ds
    ps = np.dot(ps, prob_s)
    ps_rot = np.concatenate([ps[-1:], ps[:-1]])

    if dsf < 0:
        print '**** NEGATIVE dsf!', ds, dsi, dsf

    return (1-dsf)*ps + dsf*ps_rot
