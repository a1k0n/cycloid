# -*- coding: utf-8 -*-
from __future__ import print_function

import cv2
import numpy as np
import glfw
import OpenGL.GL as gl
import imgui
from imgui.integrations.glfw import GlfwRenderer
import time

import recordreader

K, dist, frontmap = None, None, None


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


def motor_step_response(ks, N, dt=1.0/30):
    k1, k2, k3 = ks
    vs = np.zeros(N, np.float32)
    v = 0
    u = 1
    for i in range(N):
        V = i < N/2 and 1 or 0
        dvdt = k1*u*V - k2*v*u - k3*v
        v += dvdt*dt
        vs[i] = v
    return vs


def renderfront(bgr):
    global K, dist
    if K is None:
        K = np.load("../../tools/camcal/camera_matrix.npy")
        K[:2] /= 4.05
        dist = np.load("../../tools/camcal/dist_coeffs.npy")
    Knew = np.array([
        [-250, 0, 320],
        [0, 250, 260],
        [0, 0, 1]
    ])
    R = cv2.Rodrigues(np.array([0, (22.-90)*np.pi/180., 0]))[0]
    R = np.dot(np.array([[0, 1, 0], [1, 0, 0], [0, 0, 1]]), R)
    return cv2.fisheye.undistortImage(
        bgr, K=K, D=dist, Knew=np.dot(Knew, R), new_size=(640, 360))


class ReplayGUI:
    def __init__(self, fname):
        self.f = open(fname, "rb")
        print("scanning ", fname, "...")
        self.scanner = recordreader.RecordScanner(self.f)
        self.frametexid = None
        self.fronttexid = None
        self.f.seek(0, 0)
        self.controlstate = []
        self.controls = []
        self.carstate = []
        self.ts = []
        self.learn_controls = False
        self.lap_timer = False
        self.show_frontview = False
        self.startlinexy = np.array([450/50.0, 160/60.0])
        for frdata in recordreader.RecordIterator(self.f):
            self.ts.append(frdata['tstamp'])
            (throttle, steering, accel, gyro, servo,
             wheels, periods) = frdata['carstate']
            self.carstate.append(frdata['carstate'])
            self.controls.append([throttle, steering])
            self.controlstate.append(frdata['controldata2'])
        self.controlstate = np.float32(self.controlstate)
        self.controls = np.float32(self.controls)
        self.ts = np.array(self.ts)
        self.loadframe(0)
        self.playing = False
        print("done")
        self.lm = None
        self.track = None
        self.unloadlist = []
        try:
            f = open("lm.txt", "r")
            n = int(f.readline())
            self.lm = np.zeros((n, 2))
            for i in range(n):
                self.lm[i] = [float(x) for x in f.readline().strip().split()]
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

    def loadframe(self, i):
        if self.frametexid is not None:
            self.unloadlist.append(self.frametexid)
        if self.fronttexid is not None:
            self.unloadlist.append(self.fronttexid)
        self.i = i
        self.frame = self.scanner.frame(i)
        yuv420 = self.frame['yuv420']
        if 'activations' in self.frame:
            act = self.frame['activations'].astype(np.float32)
            act[1:] -= act[:-1]
            self.acts = act
        else:
            self.acts = None
        im = cv2.cvtColor(yuv420, cv2.COLOR_YUV2BGR_I420)
        self.frametexid = load_texture(im)
        frontim = renderfront(im)
        self.fronttexid = load_texture(frontim)

    def nextframe(self):
        if self.i < self.scanner.num_frames() - 1:
            self.loadframe(self.i+1)

    def render_timeline(self):
        imgui.begin("timeline")
        tstamp = self.frame['tstamp']
        if imgui.button("<"):
            self.playing = False
            if self.i > 0:
                self.loadframe(self.i - 1)
        imgui.same_line()
        if self.playing:
            if (self.i == self.scanner.num_frames()-1) or imgui.button("stop"):
                self.playing = False
            elif time.time() >= self.ts[self.i+1] - self.t0:
                self.nextframe()
        elif imgui.button("play"):
            self.playing = True
            self.t0 = tstamp - time.time()
        imgui.same_line()
        if imgui.button(">"):
            self.playing = False
            self.nextframe()
        tsfrac = tstamp - int(tstamp)
        tstring = time.strftime("%H:%M:%S.", time.localtime(
            tstamp)) + "%02d" % (tsfrac*100)
        imgui.same_line()
        imgui.text(tstring)

        w = imgui.get_window_width()
        if self.show_frontview:
            imgui.image(self.fronttexid, w, w/2)  # 2:1 aspect for front view
        else:  # 4:3 aspect
            imgui.image(self.frametexid, w, 3*w/4)

        if self.acts is not None:
            imgui.plot_lines("activations", self.acts)

        nCtrlAngles = (len(self.controlstate[self.i]) - 12) // 2
        cc = self.controlstate[self.i][-nCtrlAngles:]
        imgui.plot_lines(
            "control costs", np.clip(cc, 0, 100))

        # make a histogram of expected cone locations
        if self.acts is not None:
            hist = np.zeros(len(self.acts)*2, np.float32)
            np.add.at(hist, self.frame['c0'], 1)
            np.add.at(hist, self.frame['c1'], -1)
            hist = np.cumsum(hist)
            hist = hist[:len(self.acts)] + hist[-len(self.acts):]
            imgui.plot_lines("expected cone locations", hist)

        changed, i = imgui.slider_int(
            "frame", self.i, 0, self.scanner.num_frames()-1)
        if changed:
            self.playing = False
            self.loadframe(i)
        imgui.end()

    def render_graphs(self):
        imgui.begin("graphs")
        i = self.i
        dl = imgui.get_window_draw_list()
        mi = max(0, i - 30*3)
        temp = self.controlstate[mi:i+1, 3].copy()
        imgui.plot_lines("velocity", temp)
        temp = self.controlstate[mi:i+1, 8].copy()
        imgui.plot_lines("target v", temp)
        temp = self.controls[mi:i+1, 0].copy()
        imgui.plot_lines("control v", temp)
        temp = self.controls[mi:i+1, 1].copy()
        imgui.plot_lines("control steer", temp)
        temp = self.controlstate[mi:i+1, 9].copy()
        imgui.plot_lines("target w", temp)
        temp = self.controlstate[mi:i+1, 4].copy()
        imgui.plot_lines("yaw rate", temp)

        # live variables
        maxv = int(np.ceil(np.max(self.controlstate[:, 3]) * 1.1))
        imgui.slider_float("velocity", self.controlstate[i, 3], 0, maxv)
        imgui.slider_float("target_v", self.controlstate[i, 8], 0, maxv)

        imgui.slider_float("control motor", self.controls[i, 0]/127., -1, 1)
        imgui.slider_float("control steer", self.controls[i, 1]/127., -1, 1)

        # for yaw rate and curvature, set the limits backwards
        # so that turning right is to the right
        maxw = int(np.ceil(np.max(np.abs(self.controlstate[:, 4])) * 1.1))
        imgui.slider_float("yaw rate", self.controlstate[i, 4], maxw, -maxw)
        imgui.slider_float("target w", self.controlstate[i, 9], maxw, -maxw)
        v = self.controlstate[i, 3]
        if v > 0.5:
            k = self.controlstate[i, 4] / v
        else:
            k = 0
        imgui.slider_float("curvature", k, 2, -2)

        nCtrlAngles = (len(self.controlstate[self.i]) - 12) // 2
        cc = self.controlstate[self.i][-nCtrlAngles:]
        targetK = self.controlstate[self.i][12 + np.argmin(cc)]
        imgui.slider_float("target k", targetK, 2, -2)

        imgui.slider_float("windup k", self.controlstate[i, 7], -1, 1)

        # render overview
        pos = imgui.get_cursor_screen_pos()
        siz = imgui.get_content_region_available()
        if siz[1] == 0:
            siz = [400, 300]
        imgui.invisible_button("overview", siz[0], siz[0]*0.7)
        origin = pos
        meterwidth = max(np.max(self.track[:, 0]), np.max(self.lm[:, 0]))
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

        particlecolor = imgui.get_color_u32_rgba(1, 1, 1, 0.3)
        dl.add_rect(pos[0], pos[1], pos[0]+siz[0],
                    pos[1]+siz[1], particlecolor, 1)
        if 'particles' in self.frame:
            ps = self.frame['particles']
            for p in ps:
                dl.add_rect_filled(
                    origin[0] + p[0]*scale, origin[1] - p[1]*scale,
                    origin[0] + p[0]*scale + 1, origin[1] - p[1]*scale + 1,
                    particlecolor)
            # also draw a mean velocity vector
            mxy = scale*np.mean(ps[:, :2], axis=0)
            vxy = scale * v * .1 * np.array([
                np.mean(np.cos(ps[:, 3])),
                np.mean(np.sin(ps[:, 3]))])
            dl.add_line(origin[0] + mxy[0], origin[1] - mxy[1],
                        origin[0] + mxy[0] + vxy[0],
                        origin[1] - mxy[1] - vxy[1],
                        imgui.get_color_u32_rgba(0, 1, 0, 1), 1)
        else:
            x = self.controlstate[self.i, 0]
            y = self.controlstate[self.i, 1]
            theta = self.controlstate[self.i, 2]
            imgui.slider_float("x", x, 0, 20)
            imgui.slider_float("y", y, -10, 0)
            imgui.slider_float("theta", theta % (2*np.pi), -np.pi, np.pi)
            v = 0.3*self.controlstate[self.i, 3]
            S, C = np.sin(theta), np.cos(theta)
            dl.add_rect_filled(origin[0] + x*scale - 3, origin[1] - y*scale - 3,
                        origin[0] + scale*x + 3,
                        origin[1] - scale*y + 3,
                        imgui.get_color_u32_rgba(0, 1, 0, 1), 1)
            dl.add_line(origin[0] + x*scale, origin[1] - y*scale,
                        origin[0] + scale*(x + v*C),
                        origin[1] - scale*(y + v*S),
                        imgui.get_color_u32_rgba(0, 1, 0, 1), 1)

        accel = self.carstate[self.i][2]
        ox, oy = origin[0] + scale*3, origin[1] + scale*9
        for i in range(100):
            t0 = i*2*np.pi / 100
            t1 = (i+1)*2*np.pi / 100
            dl.add_line(
                ox + 2*scale*np.cos(t0), oy - 2*scale*np.sin(t0),
                ox + 2*scale*np.cos(t1), oy - 2*scale*np.sin(t1),
                imgui.get_color_u32_rgba(0.7, 0.7, 0.7, 1), 1)
            dl.add_line(
                ox + scale*np.cos(t0), oy - scale*np.sin(t0),
                ox + scale*np.cos(t1), oy - scale*np.sin(t1),
                imgui.get_color_u32_rgba(0.7, 0.7, 0.7, 1), 1)
        dl.add_line(ox, oy, ox + scale*accel[1], oy - scale*accel[0],
                    imgui.get_color_u32_rgba(0.3, 1, 0.5, 1), 3)

        imgui.end()

    def render_laptimer(self):
        _, self.lap_timer = imgui.begin("lap timer", True)
        if not self.lap_timer:
            imgui.end()
            return

        # _, self.startlinexy[0] = imgui.slider_float("start line x", self.startlinexy[0], 0, 20)
        # _, self.startlinexy[1] = imgui.slider_float("top lane y", self.startlinexy[0], 0, 10)
        x = self.controlstate[:, 0]
        y = self.controlstate[:, 1]
        lapidxs = np.nonzero((x[:-1] < self.startlinexy[0]) &
                             (x[1:] > self.startlinexy[0]) & (
                                 y[1:] < self.startlinexy[1]))[0]

        n = 0
        for i in range(1, len(lapidxs)):
            if self.i < lapidxs[i]:
                break
            dt = self.ts[lapidxs[i]] - self.ts[lapidxs[i-1]]
            n += 1
            imgui.text("Lap %d: %d.%03d" % (n, int(dt), 1000*(dt-int(dt))))
        if self.i > lapidxs[0]:
            dt = self.ts[self.i] - self.ts[lapidxs[n]]
            imgui.text("Lap %d: %d.%03d" % (n+1, int(dt), 1000*(dt-int(dt))))
        imgui.set_scroll_here(1.0)
        imgui.end()

    def render_controllearn(self):
        _, self.learn_controls = imgui.begin("control system tuning", True)
        if not self.learn_controls:
            imgui.end()
            return

        n = max(self.i+1, 10)
        # use history from 0..i to learn motor model
        # dv/dt = k1*DC*V + k2*DC*v + k3*v
        XTX = np.eye(2)
        XTY = np.array([5., 1])
        v = self.controlstate[1:n, 3].copy()
        dv = v.copy()
        dv[1:] = dv[1:] - dv[:-1]
        u = self.controls[:n-1, 0]
        DC = np.abs(u/127.0)
        V = (1+np.sign(u))/2
        dt = 1.0/30  # FIXME
        X = np.vstack([DC*V, -DC*v])
        Y = dv/dt
        XTX += np.dot(X, X.T)
        XTY += np.dot(X, Y.T)
        ks = np.linalg.lstsq(XTX, XTY, rcond=None)[0]
        imgui.slider_float("motor k1", ks[0], 0, 10)
        imgui.slider_float("motor k2", ks[1], 0, 2)

        imgui.plot_lines("dv/dt", dv/dt)
        imgui.plot_lines("DC*V", DC*V)
        imgui.plot_lines("v", v)
        imgui.plot_lines("step response", motor_step_response(
            [ks[0], ks[1], 0], 120))

        # let's also solve for steering ratios
        XTX = np.eye(2)
        XTY = np.array([1., 0])
        w = self.controlstate[1:n, 4].copy()
        u = self.controls[:n-1, 1] / 127.0
        X = np.vstack([u*v, v])
        XTX += np.dot(X, X.T)
        XTY += np.dot(X, w.T)
        ks = np.linalg.lstsq(XTX, XTY, rcond=None)[0]
        imgui.slider_float("servo ratio", ks[0], -2, 2)
        imgui.slider_float("servo bias", ks[1], -1, 1)

        imgui.end()

    def render(self):
        for t in self.unloadlist:
            unload_texture(t)
        self.unloadlist = []
        self.render_timeline()
        self.render_graphs()
        if self.learn_controls:
            self.render_controllearn()
        if self.lap_timer:
            self.render_laptimer()


def replay(fname):
    imgui.create_context()
    window = impl_glfw_init()
    impl = GlfwRenderer(window)
    replaygui = ReplayGUI(fname)

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
            if imgui.begin_menu("View", True):
                _, replaygui.show_frontview = imgui.menu_item(
                    "Front view", selected=replaygui.show_frontview)
                imgui.end_menu()
            if imgui.begin_menu("Tools", True):
                clicked, _ = imgui.menu_item("Learn control params")
                if clicked:
                    replaygui.learn_controls = True
                clicked, _ = imgui.menu_item("Lap timer")
                if clicked:
                    replaygui.lap_timer = True
                imgui.end_menu()
            imgui.end_main_menu_bar()

        replaygui.render()
        gl.glClearColor(0, 0, 0, 1)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)

        imgui.render()
        impl.render(imgui.get_draw_data())
        glfw.swap_buffers(window)

    impl.shutdown()
    glfw.terminate()


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("usage:", sys.argv[0], "[replay.rec]")
        exit(1)

    replay(sys.argv[1])
