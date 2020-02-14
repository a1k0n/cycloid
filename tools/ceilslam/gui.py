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

# starting position for localization
# negative x because we also mirror the track about X
HOME = [ceiltrack.X_GRID*-2.5, ceiltrack.Y_GRID*0.5]


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
        self.camdata = ceiltrack.ceillut()
        self.f.seek(0, 0)
        self.ceilheight = ceiltrack.CEIL_HEIGHT

        # do a full tracking here on load
        B = np.float32([HOME[0], HOME[1], 0])
        self.track = []
        match_time = 0
        opt_time = 0
        first = True
        floordata = []
        floormask = None
        for frdata in recordreader.RecordIterator(self.f):
            if 'yuv420' not in frdata:
                continue
            self.ts.append(frdata['tstamp'])
            yuv420 = frdata['yuv420']
            gray = yuv420[:480]
            bgr = cv2.cvtColor(yuv420, cv2.COLOR_YUV2BGR_I420)
            t0 = time.time()
            xy = ceiltrack.match(gray, *self.camdata)
            tm = time.time()
            if first:
                first = False
                for i in range(6):
                    cost, dB = ceiltrack.cost(xy, *B)
                    B += dB
                #B_straight, cost_straight = B, cost
                #B = np.float32([HOME[0], HOME[1], np.pi/2])
                #for i in range(6):
                #    cost, dB = ceiltrack.cost(xy, *B)
                #    B += dB
                #if cost_straight < cost:
                #    B = B_straight
                # we need an example frame to initialize the floor lookup table
                # to filter out the visible body posts
                self.floorlut = ceiltrack.floorlut(gray)
                floormask = self.floorlut[0]
            else:
                for i in range(2):
                    c, dB = ceiltrack.cost(xy, *B)
                    B += dB
            topt = time.time()
            match_time += tm - t0
            opt_time += topt - tm
            self.track.append(B.copy())
            floordata.append(bgr[floormask])
        self.ts = np.array(self.ts)
        self.track = np.array(self.track)
        self.origtrack = self.track.copy()
        self.track[:, 0] = -self.track[:, 0]
        self.track[:, 2] = -self.track[:, 2]
        # mirror the floor-pixel lookup table x coordinates also
        self.floorlut[1][0] = -self.floorlut[1][0]
        self.floordata = np.array(floordata)

        self.loadframe(0)
        print("done,", match_time, "secs match_time", opt_time, "sec opt_time")
        floorimg = ceiltrack.render_floor(
            self.track, self.floordata, self.floorlut[1])
        if True:
            xgm = ceiltrack.X_GRID * ceiltrack.CEIL_HEIGHT
            ygm = ceiltrack.Y_GRID * ceiltrack.CEIL_HEIGHT
            Z = 50   # pixels per meter
            for x in range(0, 1+int(1000 / (xgm*Z))):
                for y in range(0, 1+int(500 / (ygm*Z))):
                    cv2.circle(floorimg, (int(x*xgm*Z), int(y*ygm*Z)), int(0.25*Z), (255, 255, 0))
        cv2.imwrite("map.png", floorimg)
        self.floortex = load_texture(floorimg)
        print("home location:", HOME)

    def loadframe(self, i):
        if self.frametexid is not None:
            self.unloadlist.append(self.frametexid)
        self.i = i
        self.frame = self.scanner.frame(i)
        if 'yuv420' not in self.frame:
            return
        yuv420 = self.frame['yuv420']
        # optional: front view and annotated ceiling view?
        im = cv2.cvtColor(yuv420, cv2.COLOR_YUV2BGR_I420)

        xg = ceiltrack.X_GRID * self.ceilheight / ceiltrack.CEIL_HEIGHT
        yg = ceiltrack.Y_GRID * self.ceilheight / ceiltrack.CEIL_HEIGHT
        gray = yuv420[:480]
        xy = ceiltrack.match(gray, *self.camdata)
        B = self.origtrack[self.i]
        for i in range(6):
            cost, dB = ceiltrack.costxyg(xg, yg, xy, *B)
            B += dB

        for gp in ceiltrack.mkgrid(xg, yg, 31, *-B)[0]:
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
        imgui.slider_float("x (m)", self.track[self.i, 0] * ceiltrack.CEIL_HEIGHT, -80, 80)
        imgui.slider_float("y (m)", self.track[self.i, 1] * ceiltrack.CEIL_HEIGHT, -80, 80)
        imgui.slider_float("theta", self.track[self.i, 2] % (np.pi*2), -7, 7)
        imgui.slider_float("x (grid)", self.track[self.i, 0] / ceiltrack.X_GRID, -10, 10)
        imgui.slider_float("y (grid)", self.track[self.i, 1] / ceiltrack.X_GRID, -10, 10)

        changed, self.ceilheight = imgui.slider_float("ceiling height (m)", self.ceilheight, 2, 4)
        if changed:
            self.loadframe(self.i)

        dl = imgui.get_window_draw_list()
        pos = imgui.get_cursor_screen_pos()
        siz = imgui.get_content_region_available()
        if siz[1] == 0:
            siz = [400, 300]
        # just use a fixed size
        w = siz[0]
        imgui.image_button(self.floortex, w, w/2, frame_padding=0)
        # imgui.image_button(self.floortex, siz[0], siz[0])
        origin = [pos[0], pos[1]]
        scale = 50 * ceiltrack.CEIL_HEIGHT * w/1000
        trackcolor = imgui.get_color_u32_rgba(0.3, 0.5, 0.3, 1)
        for i in range(1, self.i):
            dl.add_line(
                origin[0] + scale * self.track[i-1, 0],
                origin[1] + scale * self.track[i-1, 1],
                origin[0] + scale * self.track[i, 0],
                origin[1] + scale * self.track[i, 1],
                trackcolor, 1.5)

        carcolor = imgui.get_color_u32_rgba(0, 1, 0.6, 1)
        B = self.track[self.i]
        dl.add_line(
            origin[0] + scale * B[0],
            origin[1] + scale * B[1],
            origin[0] + scale * (B[0] + np.cos(B[2])),
            origin[1] + scale * (B[1] - np.sin(B[2])),
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
