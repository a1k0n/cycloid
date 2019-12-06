import pycuda.driver as cuda
from pycuda import gpuarray
import pycuda.autoinit
from pycuda.compiler import SourceModule
import numpy as np
from tqdm import trange
import struct

TIMESTEP = 1.0/8
STEER_LIMIT_K = 0.66
NANGLES = 48
NSPEEDS = 12
GRID_RES = 0.05
TRACK_HALFWIDTH = 0.76
CONE_RADIUS = 0.3  # 30cm radius keep-out zone around cones
XSIZE = 20  # track size, meters
YSIZE = 10
VMIN = 2
VMAX = 13
AxMAX = 8
AyMAX = 10


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

xsize = int(XSIZE/GRID_RES)
ysize = int(YSIZE/GRID_RES)


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

ye, tk, tN = initgrid(400, 200)
tang = np.arctan2(tN[0], tN[1]).astype(np.float32)

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
const float TRACK_HALFWIDTH = 0.65f;  // 0.76f; artifically reduce this to leave room for the car
const float AxMAX = %f;
const float AyMAX = %f;
const float dt = %f;
const float finishx = %f, finishy = %f;

__device__ float vlookup(float *V, float x, float y, float theta, float v) {
    // check for terminal state (crossing finish line)
    float C = cos(theta);
    if (y >= (finishy - TRACK_HALFWIDTH) && y <= (finishy + TRACK_HALFWIDTH) &&
      x < finishx && x + v*dt*cos(theta) >= finishx) {
        // solve for exact time we hit the finish line
        // x0 + v*t*C = finishx
        return (finishx - x) / (v*C);
    }

    float ftheta = theta * NANGLES / (2*M_PI);
    if (ftheta >= NANGLES) ftheta -= NANGLES;
    if (ftheta < 0) ftheta += NANGLES;
    int itheta = ftheta;
    ftheta -= itheta;
    // due to fp precision issues, we might still be rounded to NANGLES here
    if (itheta >= NANGLES) itheta -= NANGLES;
    float fv = min(max((int)(v - VMIN), 0), NSPEEDS-1);
    int iv = fv;
    fv -= iv;
    float fx = x * (1.0/GRID_RES);
    int ix = fx;
    fx -= ix;
    float fy = -y * (1.0/GRID_RES);
    int iy = fy;
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
    return
    (1-fv)*((1-ftheta)*((1-fy)*((1-fx)*V0000 + fx*V0001) + fy*((1-fx)*V0010 + fx*V0011)) +
                ftheta*((1-fy)*((1-fx)*V0100 + fx*V0101) + fy*((1-fx)*V0110 + fx*V0111))) +
        fv*((1-ftheta)*((1-fy)*((1-fx)*V1000 + fx*V1001) + fy*((1-fx)*V1010 + fx*V1011)) +
                ftheta*((1-fy)*((1-fx)*V1100 + fx*V1101) + fy*((1-fx)*V1110 + fx*V1111)));

}

__global__ void valueiter(float *V, float *Vprev, float *yebuf, float *tk, float *tang) {
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

    float ye = yebuf[idx_xy];
    //float ye_clamp = min(max(ye, -TRACK_HALFWIDTH), TRACK_HALFWIDTH);
    //float cphi = max(cos(tang[idx_xy]+theta), 1e-2);
    float pathcost = dt;  //*(1 - tk[idx_xy]*ye_clamp)/(cphi);
    float penalty = ye > TRACK_HALFWIDTH ? 100 :
        ye < -TRACK_HALFWIDTH ? 1000 : 0;

    // TODO: more realistic acceleration/braking bias
    //static const float dv[4] = {-AxMAX*dt, AxMAX*dt, 0, 0};
    //float kmax = min(AyMAX/(v*v), STEER_LIMIT_K);
    //float kappa[4] = {0, 0, -kmax, kmax};

    float bestcost = 1000;
    for (int a = 0; a < 16; a++) {
        float fa = a*M_PI/8.0;
        // go around traction circle and try different moves
        float v1 = max(min(v + AxMAX*dt*cos(fa), (float)VMAX), (float)VMIN);
        float k1 = max(min(sin(fa)*AyMAX/(v*v), STEER_LIMIT_K), -STEER_LIMIT_K);
        float t1 = theta + k1*v*dt;
        float c = vlookup(Vprev, x+v1*cos(t1)*dt, y+v1*sin(t1)*dt, t1, v1, dt);
        bestcost = min(bestcost, c);
    }

    V[di] = pathcost + penalty + bestcost;
}
  """ % (STEER_LIMIT_K, NANGLES, NSPEEDS, GRID_RES, xsize, ysize, VMIN, VMAX, AxMAX, AyMAX, TIMESTEP, homex, homey))


valueiter = mod.get_function("valueiter")
V0_gpu = cuda.mem_alloc(NSPEEDS*NANGLES*xsize*ysize*4)
V = np.zeros((NSPEEDS, NANGLES, ysize, xsize), np.float32) + 1000.
cuda.memcpy_htod(V0_gpu, V)

ye_in = gpuarray.to_gpu(ye)
tk_in = gpuarray.to_gpu(tk)
tang_in = gpuarray.to_gpu(tang)
del ye
del tk
del tang

s = trange(110)
v0 = np.sum(V, dtype=np.float64)
for j in s:
    for i in range(20):
        valueiter(V0_gpu, V0_gpu, ye_in, tk_in, tang_in,
                  block=(16, 8, 1), grid=(400//16, 200//8, NANGLES*NSPEEDS))
    cuda.memcpy_dtoh(V, V0_gpu)
    v1 = np.sum(V, dtype=np.float64)
    dv = v1 - v0
    v0 = v1
    s.set_postfix_str("dv %f lap time %f" % (dv, np.min(V[0, 0, :, 155])))

savebin(V)
np.save("V.npy", V)
