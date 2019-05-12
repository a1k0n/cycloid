# -*- coding: utf-8 -*-
from __future__ import print_function

import numpy as np
import glfw
import OpenGL.GL as gl
import imgui
import time
from imgui.integrations.glfw import GlfwRenderer


def load_texture(im):
    # gl.glEnable(gl.GL_TEXTURE_2D)
    texid = gl.glGenTextures(1)
    gl.glBindTexture(gl.GL_TEXTURE_2D, texid)
    gl.glTexParameteri(gl.GL_TEXTURE_2D,
                       gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR)
    gl.glTexParameteri(gl.GL_TEXTURE_2D,
                       gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)
    gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_RGBA,
                    im.shape[1], im.shape[0], 0,
                    gl.GL_BGR, gl.GL_UNSIGNED_BYTE, im)
    return texid


def unload_texture(texid):
    gl.glDeleteTextures([texid])


def impl_glfw_init():
    width, height = 1280, 720
    window_name = "cycloid replay viewer"

    if not glfw.init():
        print("Could not initialize OpenGL context")
        exit(1)

    # OS X supports only forward-compatible core profiles from 3.2
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)

    glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, gl.GL_TRUE)

    # Create a windowed mode window and its OpenGL context
    window = glfw.create_window(
        int(width), int(height), window_name, None, None
    )
    glfw.make_context_current(window)

    if not window:
        glfw.terminate()
        print("Could not initialize Window")
        exit(1)

    return window


class Car:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.v = 0
        self.w = 0

        self.maxa = 10
        self.mk1 = 10
        self.mk2 = 1

    def clone(self):
        c = Car()
        c.x, c.y, c.theta, c.v, c.w = self.x, self.y, self.theta, self.v, self.w
        return c

    def step(self, u_v, u_delta, dt):
        ''' stupid simple car model; just understeers if a_y > maxa '''
        V = (1 + np.sign(u_v))/2
        dc = np.abs(u_v)
        dv = self.mk1*V*dc - self.mk2*self.v*dc
        k = u_delta
        self.w = 0.8*self.w + 0.2*(k*self.v)
        # underster?
        v = self.v + dv*dt*0.5
        a = v**2 * np.abs(k)
        if a > self.maxa:
            # a = v^2 k
            # maxk = a/v^2
            k = np.sign(k) * self.maxa / v**2
            # apply backwards force on speed too

        dv -= 100*dt*np.cos(k)

        v = self.v + dv*dt*0.5
        theta = self.theta + dt*self.w*0.5

        self.v += dv*dt
        if self.v < 0:
            self.v = 0

        self.theta += self.w*dt
        self.theta %= 2*np.pi
        self.x += v*np.cos(theta)*dt
        self.y += v*np.sin(theta)*dt

    def kstep(self, k, dt):
        v = 10
        if k != 0:
            v = np.sqrt(self.maxa / np.abs(k))
        dv = v - self.v
        self.step(np.clip(dv, -1, 1), k, dt)


class SimGUI:
    def __init__(self):
        try:
            f = open("lm.txt", "r")
            n = int(f.readline())
            self.lm = np.zeros((n, 2))
            for i in range(n):
                self.lm[i] = [float(x) for x in f.readline().strip().split()]
            self.home = [float(x) for x in f.readline().strip().split()[1:]]
            f.close()
        except IOError:
            print("no lm.txt found; skipping")
        try:
            f = open("track.txt", "r")
            n = int(f.readline())
            self.track = np.zeros((n, 5))
            for i in range(n):
                self.track[i] = [
                    float(x) for x in f.readline().strip().split()]
            f.close()
        except IOError:
            print("no track.txt found; skipping")
        # FIXME: add lanewidith to track def
        self.lanewidth = 0.8
        self.car = Car()
        self.car.v = 0
        self.car.x = self.home[0]
        self.car.y = self.home[1]
        self.car.theta = self.home[2]
        self.Kp = 3.0
        self.Kd = 1.0
        self.u = 0
        self.k = 0
        self.shots = None
        self.laptimes = []
        self.lookstep = 0.01
        self.V = np.load("V.npy")
        self.init_vis()

    def init_vis(self):
        self.Vtex = []
        for i in range(96):
            h = 16*(self.V.shape[1]//16)
            w = 16*(self.V.shape[2]//16)
            vt = np.zeros((h, w, 3), np.uint8)
            vt[:] = (255 - self.V[i].clip(0, 20)*255.0/20.0)[:h, :w, None]
            self.Vtex.append(load_texture(vt))

    def render(self, play):
        imgui.begin("sim")
        dl = imgui.get_window_draw_list()
        meterwidth = max(np.max(self.track[:, 0]), np.max(self.lm[:, 0]))

        if play:
            if imgui.button("pause"):
                play = False
        else:
            if imgui.button("play"):
                play = True

        # sliders
        _, xy = imgui.slider_float2("car xy", self.car.x, -self.car.y, 0, meterwidth)
        self.car.x, self.car.y = xy[0], -xy[1]
        _, self.car.theta = imgui.slider_float("car theta", self.car.theta, 0, 2*np.pi)
        imgui.slider_float("car v", self.car.v, 0, 10)
        imgui.slider_float2("controls", self.u, self.k, -1, 1)
        _, self.lanewidth = imgui.slider_float("lane width", self.lanewidth, 0.1, 2.0)

        # render overview
        pos = imgui.get_cursor_screen_pos()
        siz = imgui.get_content_region_available()
        if siz[1] <= 0:
            siz = [400, 400]
        imgui.invisible_button("overview", siz[0], siz[1])
        origin = pos
        scale = siz[0] / (meterwidth * 1.1)
        conecolor = imgui.get_color_u32_rgba(1, 0.7, 0, 1)
        for lm in self.lm:
            dl.add_rect_filled(
                origin[0] + lm[0]*scale - 2,
                origin[1] - lm[1]*scale - 2,
                origin[0] + lm[0]*scale + 2,
                origin[1] - lm[1]*scale + 2,
                conecolor, 3)
        trackcolor = imgui.get_color_u32_rgba(1, 1, 0.3, 0.5)
        for i in range(0, len(self.track), 2):
            j = (i+1) % len(self.track)
            ti = self.track[i] * scale
            tj = self.track[j] * scale
            dl.add_line(origin[0] + ti[0], origin[1] - ti[1],
                        origin[0] + tj[0], origin[1] - tj[1], trackcolor, 1.5)

        for i in range(0, len(self.track)):
            j = (i+1) % len(self.track)
            ti = self.track[i] * scale
            tj = self.track[j] * scale
            Li = ti[2:4] * self.lanewidth
            Lj = tj[2:4] * self.lanewidth
            dl.add_line(origin[0] + ti[0] + Li[0], origin[1] - ti[1] - Li[1],
                        origin[0] + tj[0] + Lj[0], origin[1] - tj[1] - Lj[1],
                        trackcolor, 3)
            dl.add_line(origin[0] + ti[0] - Li[0], origin[1] - ti[1] + Li[1],
                        origin[0] + tj[0] - Lj[0], origin[1] - tj[1] + Lj[1],
                        trackcolor, 3)

        mx = scale*self.car.x
        my = scale*self.car.y
        # draw forward projections
        for traj in self.shots:
            for i in range(len(traj)-1):
                dl.add_line(
                    origin[0] + traj[i, 0] * scale,
                    origin[1] - traj[i, 1] * scale,
                    origin[0] + traj[i+1, 0] * scale,
                    origin[1] - traj[i+1, 1] * scale,
                    imgui.get_color_u32_rgba(0, 1, 1, 1), 1)

        # draw car + velocity
        vxy = scale * self.car.v * .1 * \
            np.array([np.cos(self.car.theta), np.sin(self.car.theta)])
        dl.add_line(origin[0] + mx, origin[1] - my,
                    origin[0] + mx + vxy[0],
                    origin[1] - my - vxy[1],
                    imgui.get_color_u32_rgba(0, 1, 0, 1), 1)
        imgui.end()
        return play

    def reward(self, c, tracki, dt):
        Nt = self.track.shape[0]
        x, y = c.x, c.y
        tx, ty, nx, ny, nk = self.track[tracki]
        tx1, ty1, nx1, ny1, nk1 = self.track[(tracki+1) % Nt]
        if min((self.lm[:, 0] - x)**2 + (self.lm[:, 1] - y)**2) < 0.15**2:
            return -1000, tracki
        while ((x - tx)**2 + (y - ty)**2) > ((x - tx1)**2 + (y - ty)**2):
            tracki = (tracki + 1) % Nt
            tx, ty, nx, ny, nk = tx1, ty1, nx1, ny1, nk1
            tx1, ty1, nx1, ny1, nk1 = self.track[(tracki+1) % Nt]
        ye = (x - tx) * nx + (y - ty) * ny
        C, S = np.cos(c.theta), np.sin(c.theta)
        Cp = -S*nx + C*ny
        ds = dt * c.v * Cp / (1 - nk*ye)
        if np.abs(ye) > self.lanewidth:
            ds -= 100 # + np.abs(ye)
        # return ds - 100*abs(max(0, ye - 0.25))**2, tracki % Nt
        return ds, tracki

    def shoot(self, tracki, N, dt, k):
        ''' shoot a single trajectory, count its cost '''
        c = self.car.clone()

        R = 0
        poss = [[c.x, c.y]]
        for _ in range(N):
            c.kstep(k, dt)
            poss.append([c.x, c.y])
            dR, tracki = self.reward(c, tracki, dt)
            R += dR
        return R, np.array(poss)

    def search(self, c0, k, tracki, N, dt, path, finalpts):
        ''' exhaustive search minimum time for N forward points '''
        # TODO: heuristic for branch-and-bound
        bestR, bestdk = None, None
        bestpath = None
        for dk in [0.5, 0, -0.5]:
            if np.abs(k + dk) > 2.0:
                continue
                # dk = np.sign(k)*2.0 - k
            c1 = c0.clone()
            c1.kstep(k + dk, dt)
            R, trackj = self.reward(c1, tracki, dt)
            if N > 0:
                _, dR = self.search(c1, k+dk, trackj, N-1, dt, path[1:], finalpts)
                R += dR
            else:
                finalpts.append([c1.x, c1.y])
            if bestR is None or R > bestR:
                bestR, bestdk = R, dk
                bestpath = path[1:].copy()
                path[0] = [c1.x, c1.y]
            # if N == 10:
            #     imgui.slider_float("R_%0.1f" % dk, R, -10, 10)
        path[1:] = bestpath
        return bestdk, bestR

    def plan(self):
        ''' choose an action which optimizes our speed around the track '''
        x, y = self.car.x, self.car.y
        closesti = np.argmin(
            (self.track[:, 0] - x)**2 + (self.track[:, 1] - y)**2)
        tx, ty, nx, ny, nk = self.track[closesti]
        c, s = np.cos(self.car.theta), np.sin(self.car.theta)
        ye = (x - tx) * nx + (y - ty) * ny
        Cp = -s*nx + c*ny
        Sp = s*ny + c*nx
        imgui.slider_float2("psie", Cp, Sp, -1, 1)
        imgui.slider_float("ye", ye, -2, 2)

        Np = 6
        path = np.zeros((Np+1, 2))
        pts = []
        bestdk, R = self.search(self.car, self.k, closesti, Np, 0.11, path, pts)
        imgui.slider_float("R", R, -10, 10)
        imgui.slider_float("searched pts", len(pts), 0, 1024)
        #for dk in np.linspace(-3, 3, 21):
        #    # R, posk = self.shoot(closesti, 30, 0.05, self.k + dk)
        #    poss.append(posk)
        #    imgui.slider_float("R_%0.1f" % dk, R, -10, 10)
        #    if bestR is None or R > bestR:
        #        bestdk, bestR = dk, R
        self.shots = [path, np.array(pts)]

        return np.clip(self.k + bestdk, -2, 2)

    def Value(self, x, y, theta):
        # test: downsample
        if not hasattr(self, 'Vd'):
            # FIXME: area resampling; be careful of how angles are handled though
            #self.Vd = self.V[::4, ::5, ::5].astype(np.float16)
            self.Vd = self.V.astype(np.float16)
            #self.Vd = self.V[:, ::5, ::5].astype(np.float16)
        Vd = self.Vd
        x = np.clip(x/.02, 0, self.V.shape[2]-2)
        y = np.clip(-y/.02, 0, self.V.shape[1]-2)
        ang = (theta*Vd.shape[0]/2/np.pi) % Vd.shape[0]
        #x = np.clip(x/.1, 0, Vd.shape[2]-2)
        #y = np.clip(-y/.1, 0, Vd.shape[1]-2)
        x0, y0, a0 = int(x), int(y), int(ang)
        fx, fy, fa = x - x0, y - y0, ang - a0
        # trilinear interpolation
        vv = Vd[[a0, (a0+1) % Vd.shape[0]], y0:y0+2, x0:x0+2]
        vv = (1-fa)*vv[0] + fa*vv[1]
        vv = (1-fy)*vv[0] + fy*vv[1]
        vv = (1-fx)*vv[0] + fx*vv[1]
        return vv

    def bestkv(self, car, dt):
        bestk, bestV, bestC = None, None, None
        for k in np.linspace(-1.3, 1.3, 7):
            c = car.clone()
            c.kstep(k, dt)
            v = self.Value(c.x, c.y, c.theta)
            if bestV is None or v < bestV:
                bestk, bestV, bestC = k, v, c
        return bestk, bestV, bestC

    def Vplan(self):
        w, h = imgui.get_window_size()
        h = self.V.shape[1] * w / self.V.shape[2]
        _, self.lookstep = imgui.slider_float("lookahead step", self.lookstep, 0, 1, power=1.5)
        imgui.image(self.Vtex[int(48*self.car.theta/np.pi % 96)], w, h)
        shots = []
        c = self.car
        bestk = None
        for i in range(50):
            k, v, c = self.bestkv(c, self.lookstep)
            if bestk is None:
                bestk = k
            shots.append([c.x, c.y])
        self.shots = np.array([shots])

        return bestk

    def step(self, dt):
        k = self.Vplan()
        self.k = k
        self.car.kstep(k, dt)

    def idle(self):
        self.Vplan()


def sim():
    imgui.create_context()
    window = impl_glfw_init()
    impl = GlfwRenderer(window)

    sg = SimGUI()
    t0 = time.time()
    play = True
    while not glfw.window_should_close(window):
        glfw.poll_events()
        impl.process_inputs()
        imgui.new_frame()

        if imgui.begin_main_menu_bar():
            if imgui.begin_menu("File", True):
                clicked_quit, _ = imgui.menu_item(
                    "Quit", 'Cmd+Q', False, True)
                if clicked_quit:
                    exit(0)
                imgui.end_menu()
            imgui.end_main_menu_bar()

        t = time.time()
        if play:
            sg.step(min(t - t0, 0.03))
        else:
            sg.idle()
        t0 = t
        play = sg.render(play)

        gl.glClearColor(0, 0, 0, 1)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)

        imgui.render()
        impl.render(imgui.get_draw_data())
        glfw.swap_buffers(window)

    impl.shutdown()
    glfw.terminate()


if __name__ == "__main__":
    sim()
