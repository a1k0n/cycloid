import pycuda.driver as cuda
from pycuda import gpuarray
import pycuda.autoinit
from pycuda.compiler import SourceModule
import numpy as np
from tqdm import trange
import struct

TIMESTEP = 1.0/16
# STEER_LIMIT_K = 0.66
STEER_LIMIT_K = 1.2
NANGLES = 48
NSPEEDS = 12
GRID_RES = 0.05
TRACK_HALFWIDTH = 0.7
CONE_RADIUS = 0.3  # 30cm radius keep-out zone around cones
XSIZE = 13  # track size, meters
YSIZE = 9
VMIN = 2
VMAX = 13
AxMAX = 8
AyMAX = 12


def savebin(V):
    f = open("vf4.bin", "wb")
    v, a, h, w = V.shape
    # header:
    #  - uint16 num velocities
    #  - uint16 num angles
    #  - uint16 height
    #  - uint16 width
    #  - float  pixel scale (pixels/meter)
    #  - float  vmin
    #  - float  vscale
    hlen = 4*2 + 3*4
    f.write(struct.pack("=4sIHHHHfff",
                        b'VFN4', hlen, v, a, h, w,
                        1.0/GRID_RES, VMIN, 1))
    f.write(V.astype(np.float16).tobytes())
    f.close()
    print("vf4.bin saved")


#f = open("track.txt")
#N = int(f.readline())
#track = np.zeros((N, 5))
#for i in range(N):
#    track[i] = [float(x) for x in f.readline().strip().split()]
#f.close()
track = np.load("trackdata.npy")
track[:, 2:4] = np.stack([-track[:, 3], track[:, 2]]).T

if False: # FIRST HALF
    track = np.roll(track, -61, axis=0)[-125:]
    homex, homey = track[-1, :2]
    hometheta = 0
else: # SECOND HALF
    track = np.roll(track, -61, axis=0)[:146]
    homex, homey = track[0, :2]
    hometheta = 0


#f = open("lm.txt")
#N = int(f.readline())
#cones = np.zeros((N, 2))
#for i in range(N):
#    cones[i] = [float(x) for x in f.readline().strip().split()]
#homex, homey, hometheta = [float(x) for x in f.readline().strip().split()[1:]]
#f.close()

cones = np.zeros((0, 2))


xsize = (int(XSIZE/GRID_RES) + 15) & (~15)
ysize = (int(YSIZE/GRID_RES) + 15) & (~15)


def initgrid(w, h):
    # x, y coordinates of entire track
    xy = np.mgrid[:h, :w] * GRID_RES
    x = xy[1].reshape(-1)
    y = -xy[0].reshape(-1)
    # for each x, y, find closest point on track
    dxy = (x[:, None] - track[:, 0])**2 + (y[:, None] - track[:, 1])**2
    tracki = np.argmin(dxy, axis=1)
    tdata = track[tracki].T
    ye = (x - tdata[0])*tdata[2] + (y - tdata[1])*tdata[3]
    ye = ye.reshape((h, w)).astype(np.float32).copy()
    tk = tdata[4].reshape((h, w)).astype(np.float32).copy()
    tN = tdata[2:4].reshape((2, h, w)).astype(np.float32).copy()
    return ye, tk, tN


ye, _, _ = initgrid(xsize, ysize)
np.save("ye.npy", ye)


mod = SourceModule("""
// just hardcode these for now, we can make them format args eventually
const float STEER_LIMIT_K = %f;
const int NANGLES = %d;
const int NSPEEDS = %d;
const float GRID_RES = %f;
const int XSIZE = %d;
const int YSIZE = %d;
const int VMIN = %d;
const int VMAX = %d;
const float TRACK_HALFWIDTH = 0.55f;  // 0.76f; artifically reduce this to leave room for the car
const float AxMAX = %f;
const float AyMAX = %f;
const float dt = %f;
const float finishx = %f, finishy = %f;

__device__ bool viability(float *yebuf, float x, float y) {
    float fx = x * (1.0/GRID_RES);
    float fy = -y * (1.0/GRID_RES);
    int ix = floor(fx);
    int iy = floor(fy);
    if (ix < 0 || ix >= XSIZE-1 || iy < 0 || iy >= YSIZE-1) return false;

    int idx_xy = ix + iy*XSIZE;

    float ye = yebuf[idx_xy];
    //if (x < 7.5) {
    //    return ye < TRACK_HALFWIDTH*1.5 && ye > -TRACK_HALFWIDTH;
    //}
    return ye < TRACK_HALFWIDTH && ye > -TRACK_HALFWIDTH;
}

__device__ float vlookup(float *V, float x, float y, float theta, float v) {
    // check for terminal state (crossing finish line)
    float C = cos(theta);
    if (y >= (finishy - TRACK_HALFWIDTH) && y <= (finishy + TRACK_HALFWIDTH) &&
      x < finishx && x + v*dt*C >= finishx) {
        // solve for exact time we hit the finish line
        // x0 + v*t*C = finishx
        return (finishx - x) / (v*C);
    }

    float ftheta = (theta * NANGLES / (2*M_PI));
    if (ftheta >= NANGLES) ftheta -= NANGLES;
    if (ftheta < 0) ftheta += NANGLES;
    int itheta = floor(ftheta);
    ftheta -= itheta;
    // due to fp precision issues, we might still be rounded to NANGLES here
    if (itheta >= NANGLES) itheta -= NANGLES;
    float fv = min(max((float)(v - VMIN), 0.0f), NSPEEDS-1.0f);
    int iv = floor(fv);
    fv -= iv;

    float fx = x * (1.0/GRID_RES);
    int ix = floor(fx);
    fx -= ix;

    float fy = -y * (1.0/GRID_RES);
    int iy = floor(fy);
    fy -= iy;

    if (ix < 0 || ix >= XSIZE-1 || iy < 0 || iy >= YSIZE-1) return 1000.0f;

    int idx_xy = ix + iy*XSIZE;
    int di = idx_xy + itheta*XSIZE*YSIZE + iv*XSIZE*YSIZE*NANGLES;

    int nexttheta = itheta < NANGLES-1 ? XSIZE*YSIZE : -XSIZE*YSIZE*(NANGLES-1);

    //     vtyx
    float V0000 = V[di];
    float V0001 = V[di+1];
    float V0010 = V[di+XSIZE];
    float V0011 = V[di+XSIZE+1];
    float V0100 = V[di+nexttheta];
    float V0101 = V[di+nexttheta+1];
    float V0110 = V[di+nexttheta+XSIZE];
    float V0111 = V[di+nexttheta+XSIZE+1];
    float V1000 = V0000;
    float V1001 = V0001;
    float V1010 = V0010;
    float V1011 = V0011;
    float V1100 = V0100;
    float V1101 = V0101;
    float V1110 = V0110;
    float V1111 = V0111;
    if (iv < NSPEEDS-1) {
        V1000 = V[di+XSIZE*YSIZE*NANGLES];
        V1001 = V[di+XSIZE*YSIZE*NANGLES+1];
        V1010 = V[di+XSIZE*YSIZE*NANGLES+XSIZE];
        V1011 = V[di+XSIZE*YSIZE*NANGLES+XSIZE+1];
        V1100 = V[di+XSIZE*YSIZE*NANGLES+nexttheta];
        V1101 = V[di+XSIZE*YSIZE*NANGLES+nexttheta+1];
        V1110 = V[di+XSIZE*YSIZE*NANGLES+nexttheta+XSIZE];
        V1111 = V[di+XSIZE*YSIZE*NANGLES+nexttheta+XSIZE+1];
    }
    // lerp
    return dt+
    (1-fv)*((1-ftheta)*((1-fy)*((1-fx)*V0000 + fx*V0001) + fy*((1-fx)*V0010 + fx*V0011)) +
                ftheta*((1-fy)*((1-fx)*V0100 + fx*V0101) + fy*((1-fx)*V0110 + fx*V0111))) +
        fv*((1-ftheta)*((1-fy)*((1-fx)*V1000 + fx*V1001) + fy*((1-fx)*V1010 + fx*V1011)) +
                ftheta*((1-fy)*((1-fx)*V1100 + fx*V1101) + fy*((1-fx)*V1110 + fx*V1111)));

}

__global__ void valueiter(float *V, float *Vprev, float *yebuf) {
    int ix = threadIdx.x + blockIdx.x * blockDim.x;
    int iy = threadIdx.y + blockIdx.y * blockDim.y;
    int ithetav = threadIdx.z + blockIdx.z * blockDim.z;
    int itheta = ithetav %% NANGLES;
    int iv = ithetav / NANGLES;

    if (ix >= XSIZE) return;
    if (iy >= YSIZE) return;
    if (itheta >= NANGLES) return;
    if (iv >= NSPEEDS) return;

    float theta = itheta * 2 * M_PI / NANGLES;
    float v = iv + VMIN;
    float x = ix * GRID_RES;
    float y = -iy * GRID_RES;

    int idx_xy = ix + iy*XSIZE;
    int di = idx_xy + itheta*XSIZE*YSIZE + iv*XSIZE*YSIZE*NANGLES;

    // TODO: more realistic acceleration/braking bias

    float bestcost = 1e5;
    bool viable = false;
    for (int a = 0; a < 16; a++) {
        float fa = a*M_PI/8.0;
        // go around traction circle and try different moves
        float v1 = max(min(v + AxMAX*dt*cos(fa), (float)VMAX), (float)VMIN);
        float k1 = max(min(sin(fa)*AyMAX/(v*v), STEER_LIMIT_K), -STEER_LIMIT_K);
        float t1 = theta + k1*v*dt;
        float dx = v1*cos(t1)*dt;
        float dy = v1*sin(t1)*dt;
        bool via1 = viability(yebuf, x+dx, y+dy);
        if (viable && !via1) {  // if we already have a viable move, anything better must be viable
            continue;
        }
        float c = vlookup(Vprev, x+dx, y+dy, t1, v1);
        if (!via1) c += 10;
        if (c < bestcost || (via1 && !viable)) {
            bestcost = c;
            viable = via1;
        }
    }

    V[di] = bestcost;
}
""" % (STEER_LIMIT_K, NANGLES, NSPEEDS, GRID_RES, xsize, ysize, VMIN, VMAX,
       AxMAX, AyMAX, TIMESTEP, homex, homey))

valueiter = mod.get_function("valueiter")
V0_gpu = cuda.mem_alloc(NSPEEDS*NANGLES*xsize*ysize*4)
V = np.zeros((NSPEEDS, NANGLES, ysize, xsize), np.float32) + 1000.
cuda.memcpy_htod(V0_gpu, V)

ye_in = gpuarray.to_gpu(ye)
del ye

s = trange(20)
v0 = np.sum(V, dtype=np.float64)
for j in s:
    for i in range(20):
        valueiter(V0_gpu, V0_gpu, ye_in, block=(16, 8, 1),
                  grid=(xsize//16, ysize//8, NANGLES*NSPEEDS))
    cuda.memcpy_dtoh(V, V0_gpu)
    v1 = np.sum(V, dtype=np.float64)
    dv = v1 - v0
    v0 = v1
    if dv == 0:
        s.set_postfix_str("converged, lap time %f" % np.min(V[0, 0, :, 155]))
        break
    s.set_postfix_str("dv %f lap time %f" % (dv, np.min(V[0, 0, :, 155])))

savebin(V)
np.save("V.npy", V)
