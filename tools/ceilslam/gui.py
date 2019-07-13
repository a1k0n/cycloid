# -*- coding: utf-8 -*-
from __future__ import print_function

import cv2
import numpy as np
import time
import glfw
import OpenGL.GL as gl
import imgui
from imgui.integrations.glfw import GlfwRenderer

import ceiltrack
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


class SLAMGUI:
    def __init__(self, fname):
        self.unloadlist = []
        self.f = open(fname, "rb")
        print("scanning ", fname, "...")
        self.scanner = recordreader.RecordScanner(self.f)
        self.frametexid = None
        self.playing = False
        self.ts = []
        self.camdata = ceiltrack.genlut()
        self.f.seek(0, 0)

        # do a full tracking here on load
        B = np.zeros(3)  # this is the initial state
        self.track = []
        match_time = 0
        opt_time = 0
        first = True
        for frdata in recordreader.RecordIterator(self.f):
            self.ts.append(frdata['tstamp'])
            yuv420 = frdata['yuv420']
            t0 = time.time()
            xy = ceiltrack.match(yuv420[:480], *self.camdata)
            tm = time.time()
            if first:
                first = False
                for i in range(6):
                    cost, dB = ceiltrack.cost(xy, *B)
                    B += dB
                B_straight, cost_straight = B, cost
                B = np.array([0., 0., np.pi/2])
                for i in range(6):
                    cost, dB = ceiltrack.cost(xy, *B)
                    B += dB
                if cost_straight < cost:
                    B = B_straight
            else:
                for i in range(2):
                    c, dB = ceiltrack.cost(xy, *B)
                    B += dB
            topt = time.time()
            match_time += tm - t0
            opt_time += topt - tm
            self.track.append(B.copy())
        self.ts = np.array(self.ts)
        self.track = np.array(self.track)

        self.loadframe(0)
        print("done,", match_time, "secs match_time", opt_time, "sec opt_time")

    def loadframe(self, i):
        if self.frametexid is not None:
            self.unloadlist.append(self.frametexid)
        self.i = i
        self.frame = self.scanner.frame(i)
        yuv420 = self.frame['yuv420']
        # optional: front view and annotated ceiling view?
        im = cv2.cvtColor(yuv420, cv2.COLOR_YUV2BGR_I420)

        for gp in ceiltrack.mkgrid(ceiltrack.X_GRID, ceiltrack.Y_GRID, 31, *-self.track[i])[0]:
            cv2.circle(im, (int(gp[0]), int(gp[1])), 3, (255, 0, 0), 1)

        self.frametexid = load_texture(im)

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
            if (self.i == len(self.ts)-1) or imgui.button("stop"):
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
        imgui.image(self.frametexid, w, 480*w/640)

        changed, i = imgui.slider_int(
            "frame", self.i, 0, self.scanner.num_frames()-1)
        if changed:
            self.playing = False
            self.loadframe(i)
        imgui.end()

    def render_map(self):
        imgui.begin("map")
        imgui.slider_float("x", self.track[self.i, 0], -10, 10)
        imgui.slider_float("y", self.track[self.i, 1], -10, 10)
        imgui.slider_float("theta", self.track[self.i, 2] % (np.pi*2), -7, 7)

        dl = imgui.get_window_draw_list()
        pos = imgui.get_cursor_screen_pos()
        siz = imgui.get_content_region_available()
        if siz[1] == 0:
            siz = [400, 300]
        imgui.invisible_button("overview", siz[0], siz[1])
        origin = [pos[0] + siz[0]/2, pos[1] + siz[1]/2]
        scale = min(*siz) / 10.0
        trackcolor = imgui.get_color_u32_rgba(0.3, 0.5, 0.3, 1)
        for i in range(1, self.i):
            dl.add_line(
                origin[0] - scale * self.track[i-1, 0],
                origin[1] + scale * self.track[i-1, 1],
                origin[0] - scale * self.track[i, 0],
                origin[1] + scale * self.track[i, 1],
                trackcolor, 1.5)

        carcolor = imgui.get_color_u32_rgba(0, 1, 0.6, 1)
        B = self.track[self.i]
        dl.add_line(
            origin[0] - scale * B[0],
            origin[1] + scale * B[1],
            origin[0] - scale * (B[0] - np.cos(B[2])),
            origin[1] + scale * (B[1] + np.sin(B[2])),
            carcolor, 1.5)

        imgui.end()

    def render(self):
        for t in self.unloadlist:
            unload_texture(t)
        self.unloadlist = []
        self.render_timeline()
        self.render_map()


def main(recfile):
    imgui.create_context()
    window = impl_glfw_init()
    impl = GlfwRenderer(window)
    slamgui = SLAMGUI(recfile)

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

        slamgui.render()
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
        print("usage:", sys.argv[0], "[cycloid-x.rec]")
        exit(1)

    main(sys.argv[1])
