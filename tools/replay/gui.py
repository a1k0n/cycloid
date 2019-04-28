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


class ReplayGUI:
    def __init__(self, fname):
        self.f = open(fname, "rb")
        print("scanning ", fname, "...")
        self.scanner = recordreader.RecordScanner(self.f)
        self.frametexid = None
        self.f.seek(0, 0)
        self.controlstate = []
        self.controls = []
        self.ts = []
        self.learn_controls = False
        for frdata in recordreader.RecordIterator(self.f):
            self.ts.append(frdata['tstamp'])
            (throttle, steering, accel, gyro, servo,
             wheels, periods) = frdata['carstate']
            self.controls.append([throttle, steering])
            self.controlstate.append(frdata['controldata'])
        self.controlstate = np.float32(self.controlstate)
        self.controls = np.float32(self.controls)
        self.ts = np.array(self.ts)
        self.loadframe(0)
        self.playing = False
        print("done")
        self.lm = None
        self.track = None
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
                self.track[i] = [float(x) for x in f.readline().strip().split()]
            f.close()
        except IOError:
            print("no track.txt found; skipping")

    def loadframe(self, i):
        if self.frametexid is not None:
            unload_texture(self.frametexid)
        self.i = i
        self.frame = self.scanner.frame(i)
        yuv420 = self.frame['yuv420']
        act = self.frame['activations'].astype(np.float32)
        act[1:] -= act[:-1]
        self.acts = act
        im = cv2.cvtColor(yuv420, cv2.COLOR_YUV2BGR_I420)
        self.frametexid = load_texture(im)

    def nextframe(self):
        if self.i < self.scanner.num_frames() - 1:
            self.loadframe(self.i+1)

    def render_timeline(self):
        imgui.begin("timeline")
        tstamp = self.frame['tstamp']
        if self.playing:
            if (self.i == self.scanner.num_frames()-1) or imgui.button("stop"):
                self.playing = False
            elif time.time() >= self.ts[self.i+1] - self.t0:
                self.nextframe()
        elif imgui.button("play"):
            self.playing = True
            self.t0 = tstamp - time.time()
        tsfrac = tstamp - int(tstamp)
        tstring = time.strftime("%H:%M:%S.", time.localtime(
            tstamp)) + "%02d" % (tsfrac*100)
        imgui.same_line()
        imgui.text(tstring)

        w = imgui.get_window_width()
        imgui.image(self.frametexid, w, 480*w/640)

        imgui.plot_lines("activations", self.acts)

        # make a histogram of expected cone locations
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
        temp = self.controlstate[mi:i+1, 10].copy()
        imgui.plot_lines("target v", temp)
        temp = self.controls[mi:i+1, 0].copy()
        imgui.plot_lines("control v", temp)
        temp = self.controls[mi:i+1, 1].copy()
        imgui.plot_lines("control steer", temp)
        temp = self.controlstate[mi:i+1, 11].copy()
        imgui.plot_lines("target w", temp)
        temp = self.controlstate[mi:i+1, 5].copy()
        imgui.plot_lines("yaw rate", temp)

        # live variables
        maxv = int(np.ceil(np.max(self.controlstate[:, 3]) * 1.1))
        imgui.slider_float("velocity", self.controlstate[i, 3], 0, maxv)
        imgui.slider_float("target_v", self.controlstate[i, 10], 0, maxv)

        imgui.slider_float("control motor", self.controls[i, 0]/127., -1, 1)
        imgui.slider_float("control steer", self.controls[i, 1]/127., -1, 1)

        # for yaw rate and curvature, set the limits backwards
        # so that turning right is to the right
        maxw = int(np.ceil(np.max(np.abs(self.controlstate[:, 5])) * 1.1))
        imgui.slider_float("yaw rate", self.controlstate[i, 5], maxw, -maxw)
        imgui.slider_float("target w", self.controlstate[i, 11], maxw, -maxw)
        imgui.slider_float("target k", self.controlstate[i, 9], 2, -2)
        v = self.controlstate[i, 3]
        if v > 0.5:
            k = self.controlstate[i, 5] / v
        else:
            k = 0
        imgui.slider_float("curvature", k, 2, -2)

        imgui.slider_float("windup v", self.controlstate[i, 6], -5, 5)
        imgui.slider_float("windup w", self.controlstate[i, 7], -1, 1)

        # render overview
        pos = imgui.get_cursor_screen_pos()
        siz = imgui.get_content_region_available()
        imgui.invisible_button("overview", siz[0], siz[1])
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
        imgui.end()

    def render_controllearn(self):
        _, self.learn_controls = imgui.begin("control system tuning", True)
        if not self.learn_controls:
            imgui.end()
            return

        n = max(self.i+1, 10)
        # use history from 0..i to learn motor model
        # dv/dt = k1*DC*V + k2*DC*v + k3*v
        XTX = np.eye(3)
        XTY = np.array([5., 1, 1])
        v = self.controlstate[1:n, 3].copy()
        dv = v.copy()
        dv[1:] = dv[1:] - dv[:-1]
        u = self.controls[:n-1, 0]
        DC = np.abs(u/127.0)
        V = (1+np.sign(u))/2
        dt = 1.0/30  # FIXME
        X = np.vstack([DC*V, -DC*v, -v])
        Y = dv/dt
        XTX += np.dot(X, X.T)
        XTY += np.dot(X, Y.T)
        ks = np.linalg.lstsq(XTX, XTY, rcond=None)[0]
        imgui.slider_float("k1", ks[0], 0, 10)
        imgui.slider_float("k2", ks[1], 0, 2)
        imgui.slider_float("k3", ks[2], 0, 2)

        imgui.plot_lines("dv/dt", dv/dt)
        imgui.plot_lines("DC*V", DC*V)
        imgui.plot_lines("DC*v", DC*v)
        imgui.plot_lines("v", v)
        imgui.plot_lines("step response", motor_step_response(ks, 120))

        imgui.end()

    def render(self):
        self.render_timeline()
        self.render_graphs()
        if self.learn_controls:
            self.render_controllearn()


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
            if imgui.begin_menu("Tools", True):
                clicked, _ = imgui.menu_item("Learn control params")
                if clicked:
                    replaygui.learn_controls = True
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
