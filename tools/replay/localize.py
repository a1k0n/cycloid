import numpy as np
import scipy.linalg
import cv2

TRACK = "../../src/drive/oakland"

# curvature map learned from the track
KMAP_ENTRIES_PER_METER = 5
kmap = np.loadtxt(TRACK + "_track_k.txt", comments=",")
KMAP_ENTRIES = len(kmap)
trackx = np.loadtxt(TRACK + "_track_x.txt", comments=",").reshape((-1, 2))
tracku = np.loadtxt(TRACK + "_track_u.txt", comments=",").reshape((-1, 2))


def prob_s_given_k(k, prec):
    # i need to integrate this term over the distribution of N(k, 1/prec)
    # this is seat of the pants, not sure if it's right.
    # l = kmap[1] * np.exp(-(kmap[0] - k)**2 * (kmap[1] * prec))
    # do i want to include kmap's precision? i am not sure
    l = -(kmap - k)**2 * prec
    l -= np.max(l)
    l = np.exp(l)
    s = np.sum(l)
    if s == 0:
        return np.ones(KMAP_ENTRIES) / KMAP_ENTRIES
    return l / s


def rotate_prob(p, k):
    ''' rotate a discrete probability distribution by k (which can be
    fractional) elements, linearly interpolating '''
    ki = int(np.floor(k))
    kf = k - ki
    ps = np.concatenate([p[-ki:], p[:-ki]])
    ps_rot = np.concatenate([ps[-1:], ps[:-1]])
    return (1-kf)*ps + kf*ps_rot


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


def estimate_kappa_ds(a, b, c, y_c, Rk):
    # we could get an error estimate on ds and convolve the probability
    # distribution too. hmm.
    pass


def update(prob_s, a, b, c, y_c, Rk):
    tmp0 = a*y_c
    tmp1 = b + 2*tmp0
    tmp2 = tmp1**2 + 1
    tmp5 = 2*tmp2**(-1.5)
    tmp9 = 2*a
    tmp10 = tmp2**(-2.5)
    tmp11 = 12.0*tmp1*tmp10

    # wait, i need to know s = y_c / cos(phi)(?)
    kappa = a*tmp5

    Mk = np.array(
        [-tmp0*tmp11 + tmp5, -tmp10*tmp9*(3.0*b + 6.0*tmp0), 0, -a**2*tmp11])
    R = np.dot(Mk, np.dot(Rk, Mk.T))

    return prob_s * prob_s_given_k(kappa, 0.01 / R)
    # kappa, ds, R = estimate_kappa_ds(a, b, c, y_c, Rk)
    # return prob_s * rotate_prob(
    #     prob_s_given_k(kappa, 0.01 / R),
    #     ds * KMAP_ENTRIES_PER_METER)


def predict(prob_s, x, P, Delta_T):
    ds, cov = ds_distribution(x, P, Delta_T)

    ds *= KMAP_ENTRIES_PER_METER
    cov *= KMAP_ENTRIES_PER_METER**2
    dsi = np.floor(ds)
    dsf = ds - dsi

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

    return (1-dsf)*ps + dsf*ps_rot


def drawpath(img, scale, offset, trackx, skip, color, thick):
    N = len(trackx)
    for i in range(0, N, skip):
        xy1 = np.int32(trackx[i] * scale + offset)
        xy2 = np.int32(trackx[(i+1)%N] * scale + offset)
        cv2.line(img, (xy1[0], xy1[1]), (xy2[0], xy2[1]), color, thick)


def drawtrack(scale, offset, img):
    # draw start line
    x0 = np.int32((trackx[0] + tracku[0]*0.9) * scale + offset)
    x1 = np.int32((trackx[0] - tracku[0]*0.9) * scale + offset)
    cv2.line(img, (x0[0], x0[1]), (x1[0], x1[1]), (0, 0, 0), 3)
    cv2.line(img, (x0[0], x0[1]), (x1[0], x1[1]), (0, 255, 255), 1)

    drawpath(img, scale, offset, trackx, 3, (0, 0, 0), 3)
    drawpath(img, scale, offset, trackx, 3, (0, 255, 255), 1)
    drawpath(img, scale, offset, trackx + tracku*0.9, 1, (0, 0, 0), 3)
    drawpath(img, scale, offset, trackx + tracku*0.9, 1, (255, 255, 255), 1)
    drawpath(img, scale, offset, trackx - tracku*0.9, 1, (0, 0, 0), 3)
    drawpath(img, scale, offset, trackx - tracku*0.9, 1, (255, 255, 255), 1)


vidout = None  # hack hack hack
def drawstate(x, Sdist):
    global vidout
    scale = 15
    offset = 200
    v, delta, ye, psi_e, kappa = x[:5]
    img = np.zeros((400, 640, 3), np.uint8)
    img[:, :, 1] = 255   # green screen

    drawtrack(scale, offset, img)

    # s = np.argmax(Sdist)
    pmax = np.max(Sdist)
    for p, s in sorted([(p, i) for i, p in enumerate(Sdist)])[-10:]:
        if s >= len(trackx):  # we have a slight mismatch in lengths
            s = 0
        x0 = trackx[s] + tracku[s] * ye
        R = np.array([
            [-np.sin(psi_e), np.cos(psi_e)],
            [-np.cos(psi_e), -np.sin(psi_e)]
        ])
        x1 = x0 + 0.2 * v * np.dot(R, tracku[s])
        x0 = np.int32(scale * x0) + offset
        x1 = np.int32(scale * x1) + offset
        # cv2.line(img, (x0[0], x0[1]), (x1[0], x1[1]), (255, 255, 255), 1)
        color = int(255 * p / pmax)
        cv2.circle(img, (x0[0], x0[1]), 2, (color, color, color), 2)

    if vidout is None:
        vidout = cv2.VideoWriter("track.h264", cv2.VideoWriter_fourcc(
            'X', '2', '6', '4'), 30, (img.shape[1], img.shape[0]), True)
    cv2.imshow("track", img)
    vidout.write(img)
