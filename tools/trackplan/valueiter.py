import numpy as np
import cv2
import tqdm

# FIXME / TODO:
#  - wrap this whole thing up with functions
#  - add a main
#  - input track as circles / lines, not as a bunch of points
#    and normals from tapetrack
#  - checkpoint V.npy more frequently
#  - downsample V.npy when saving and interpolate when using

f = open("track.txt")
N = int(f.readline())
track = np.zeros((N, 5))
for i in range(N):
    track[i] = [float(x) for x in f.readline().strip().split()]
f.close()

f = open("lm.txt")
N = int(f.readline())
cones = np.zeros((N, 2))
for i in range(N):
    cones[i] = [float(x) for x in f.readline().strip().split()]
homex, homey, hometheta = [float(x) for x in f.readline().strip().split()[1:]]
f.close()

# assumptions: working with a 2cm/pixel grid
# control curvature range -1.3..1.3 m^-1
amax = 8
vmax = 10
action_dx = np.float32([1, 15, 15, 15, 15, 15, 15])*.02
action_dy = np.float32([0, -3, -2, -1,  1,  2,  3])*.02
# delta angle on a 96-angle scale (1/15 slope works out
# almost exactly to 1/96th of a circle)
action_dang = np.int32([0, -3, -2, -1,  1,  2,  3])

dp2 = action_dx**2 + action_dy**2
action_k = 2*action_dy / dp2
action_ds = np.zeros(7)
action_ds[1:] = dp2[1:]*np.arcsin(action_dy[1:]/np.sqrt(dp2[1:]))/action_dy[1:]
action_ds[0] = action_dx[0]
action_dx, action_dy, action_ds
action_oov = np.zeros(action_k.shape)
action_oov[0] = 1.0/vmax
action_oov[1:] = np.sqrt(np.abs(action_k[1:]) / amax)


def conepenalty(w, h):
    xy = np.mgrid[:h, :w] * 0.02
    x = xy[1]
    y = -xy[0]
    # for each x, y, find closest point on track
    dxy = (x[:, :, None] - cones[:, 0])**2 + (y[:, :, None] - cones[:, 1])**2
    return np.min(dxy, axis=2)


def initgrid(w, h):
    # x, y coordinates of entire track
    xy = np.mgrid[:h, :w] * 0.02
    x = xy[1].reshape(-1)
    y = -xy[0].reshape(-1)
    # for each x, y, find closest point on track
    dxy = (x[:, None] - track[:, 0])**2 + (y[:, None] - track[:, 1])**2
    tracki = np.argmin(dxy, axis=1)
    tdata = track[tracki].T
    ye = (x - tdata[0])*tdata[2] + (y - tdata[1])*tdata[3]
    ye = ye.reshape((h, w))
    tk = tdata[4].reshape((h, w))
    tN = tdata[2:4].reshape((2, h, w))
    return ye, tk, tN


def genpathcost(w, h):
    ye, tk, tN = initgrid(w, h)
    tang = np.arctan2(tN[0], tN[1])
    angs = np.arange(96)*2*np.pi/96.
    yelim = 0.99 / np.max(np.abs(track[:, 4]))
    coneradius = conepenalty(w, h)
    pathcost = ((1 - tk*np.clip(ye, -yelim, yelim))[:, :, None] /
                np.clip(np.cos(angs+tang[:, :, None]), 1e-2, 1))
    pathcost = pathcost.transpose((2, 0, 1))
    # penalty for going outside the lines
    pathcost[:, ye > yelim] += 100
    # penalty for going within the inside of the track
    pathcost[:, ye < -yelim] = 1000

    # major penalty for hitting cones
    pathcost[:, coneradius < 0.3**2] = 1000
    return pathcost


# last thing before we can solve this: generate the actual remap table for each move
def remaptable(w, h, dx, dy, angle):  # dx and dy in world coordinates
    xy = np.mgrid[:h, :w].astype(np.float32)
    map1 = np.stack([xy[1], xy[0]])
    C, S = np.cos(angle*np.pi/48), np.sin(angle*np.pi/48)
    dx, dy = np.dot([[C, -S], [S, C]], [dx, dy])
    map1[0] += dx / .02
    map1[1] -= dy / .02
    map1 = map1.transpose(1, 2, 0)
    return map1


# okay, then here's our first iteration
# we generate a remap table for each of 7 moves to each of 7 destination angles
# the result is:
# np.amin(action_ds * action_oov * pathcost + remap(V[ang+dang...], <delta move table>))
def computeremaps(w, h, pathcost):
    rm1 = np.zeros((96, 7, h, w, 2), np.int16)
    rm2 = np.zeros((96, 7, h, w), np.int16)
    pcosts = np.zeros((96, 7, h, w), np.float32)
    for ang in range(96):
        for a in range(7):
            rmf = remaptable(w, h, action_dx[a], action_dy[a], ang)
            rm1[ang, a], rm2[ang, a] = cv2.convertMaps(rmf, None, cv2.CV_16SC2)
            pcosts[ang, a] = action_ds[a] * action_oov[a] * pathcost[ang]
    return rm1, rm2, pcosts


def runiter(V, rm1, rm2, pcosts):
    for ang in range(96):
        for a in range(7):
            Vd = cv2.remap(V[(ang + action_dang[a]) % 96], rm1[ang, a], rm2[ang, a], cv2.INTER_LINEAR, borderValue=1000.)
            V[ang] = np.minimum(V[ang], pcosts[ang, a] + Vd)


def main():
    try:
        print("resuming V.npy; delete to start over")
        V = np.load("V.npy")
    except Exception:
        V = 1000.*np.ones((96, 330, 600), np.float32)  # we're using 96 angles, stride dx 15, dy -3..+3
    # initialize finish line
    finishx, finishy = homex/.02, homey/.02
    # finish line is pointing right, but we can cover any right-facing angle, -12..+12?
    V[:24, int(-finishy-.5/.02):int(-finishy+.5/.02), int(finishx):int(finishx+2)] = 0
    V[-24:, int(-finishy-.5/.02):int(-finishy+.5/.02), int(finishx):int(finishx+2)] = 0

    print("precomputing path costs...")
    pathcost = genpathcost(V.shape[2], V.shape[1])
    print("precomputing remappings...")
    rm1, rm2, pcosts = computeremaps(V.shape[2], V.shape[1], pathcost)

    v0 = np.sum(V)
    s = tqdm.trange(500)
    s.set_postfix_str(str(v0))
    for i in s:
        runiter(V, rm1, rm2, pcosts)
        v1 = np.sum(V)
        if v1 == v0:
            break
        dv = v1 - v0
        v0 = v1
        s.set_postfix_str(str(v0) + " dv " + str(dv))
    np.save("V.npy", V)


if __name__ == '__main__':
    main()
