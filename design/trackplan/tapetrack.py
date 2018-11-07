import numpy as np

''' use the data exported from ../trackplan.py and quickly find the track
parameters for any point on the plane '''


def trackparams(xy, T):
    # reconstructed by following code in drive/trajtrack.cc
    l0 = T[3:5]
    l1 = T[5:7]
    dp = l1 - l0
    tnum = np.sum(dp.T*(xy[:2] - l0.T), axis=1)
    tden = np.sum(dp.T**2, axis=1)
    t = np.clip(tnum / tden, 0, 1)
    p = (1-t)*l0 + t*l1
    dist = np.sum((p.T - xy[:2])**2, axis=1)
    i = np.argmin(dist)
    mint = t[i]
    if mint == 1:
        i = (i+1) % T.shape[1]
        mint = 0

    if mint == 0:  # on arc, not straight line
        # recompute closest x, y from circle
        p = T[:2, i]
        r = T[2, i]
        dp = xy[:2] - p
        norm = np.sqrt(np.dot(dp, dp))
        closest = p + np.abs(r) * dp / norm
        norm = -dp*np.sign(r) / norm
        kappa = 1.0 / r
    else:
        closest = p[:, i]
        norm = -T[7:9, i]
        kappa = 0

    return closest, norm, kappa


def tracksubdiv(T, N):
    # arc length = r*theta
    Tp = np.hstack([T[:, -1:], T[:, :-1]])
    lp = Tp[5:7]
    l0 = T[3:5]
    ln = T[5:7]
    linelen = np.sqrt(np.sum((ln - l0)**2, axis=0))
    # rotate l0 into the frame of lp
    p = T[:2]
    l0 = l0 - p
    lp = lp - p
    c, s = lp / np.linalg.norm(lp, axis=0)
    x = c*l0[0] + s*l0[1]
    y = -s*l0[0] + c*l0[1]
    angles = np.abs(np.arctan2(y, x))
    r = np.abs(T[2])

    # interleave angles, lines
    L = np.vstack([r*angles, linelen]).T.reshape(-1)
    totallen = np.sum(L)
    seglen = totallen / N
    offsets = np.cumsum(L/seglen)
    print offsets

    print 'seglen', seglen, (r*angles)/seglen

    a0 = np.arctan2(lp[1], lp[0])  # not actually sure this is right

    # n1 = np.ceil((r*angles)[0] / seglen)
    # print offsets % 1.0

    # norm = np.linalg.norm(T[5:7, 0] - T[3:5, 0])
    # n2 = np.ceil((L[1] - offsets[0]) / seglen)

    pts = np.zeros((N, 5))

    s = 0
    j = 0
    for i in range(N):
        k = j >> 1
        if (j & 1) == 0:
            # we're on a circle
            rr = np.abs(T[2, k])
            a = a0[k] - s/T[2, k]
            dp = np.array([np.cos(a), np.sin(a)])
            pts[i, :2] = p[:2, k] + dp*rr
            # fill in normal and curvature
            pts[i, 2:4] = dp*np.sign(T[2, k])  # normal
            pts[i, 4] = 1.0 / rr  # curvature
        else:
            t = s / L[j]
            pts[i, :2] = (l0+p)[:, k]*(1-t) + ln[:, k]*t
            nn = (l0+p-ln)[:, k]
            nn /= np.linalg.norm(nn)
            pts[i, 2:4] = np.array([nn[1], -nn[0]])  # normal
            pts[i, 4] = 0  # curvature
        s += seglen
        if s >= L[j]:
            s -= L[j]
            j += 1

    return pts
