# -*- coding: utf-8 -*-
from __future__ import print_function

import cv2
import numpy as np
import pickle
import glfw
import OpenGL.GL as gl
import imgui
from imgui.integrations.glfw import GlfwRenderer

import tapetrack
import track_opt


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


def zoomtip(imtex, imdim, mag=1.0):
    if imgui.is_item_hovered():
        w, h = imgui.get_window_size()
        h = imdim[0] * w / imdim[1]
        rectmin = imgui.get_item_rect_min()
        mousepos = imgui.get_mouse_pos()
        u = float(mousepos[0] - rectmin[0]) / w
        v = float(mousepos[1] - rectmin[1]) / h
        imgui.begin_tooltip()
        tw = 32. / imdim[1] / mag
        th = 32. / imdim[0] / mag
        imgui.image(imtex, 64, 64, uv0=(u-tw, v-th), uv1=(u+tw, v+th))
        dl = imgui.get_window_draw_list()
        rm = imgui.get_item_rect_min()
        col = imgui.get_color_u32_rgba(1, 1, 0, 1)
        dl.add_line(rm[0], rm[1]+32, rm[0]+64, rm[1]+32, col)
        dl.add_line(rm[0]+32, rm[1], rm[0]+32, rm[1]+64, col)
        imgui.end()


class TrackplanWindow:
    def __init__(self, mapim):
        # state on init
        self.mapim = mapim
        self.maptex = load_texture(mapim)

        # ephemeral state
        self.editmode = 'cone'
        self.selectedpt = None
        self.kcurv = 0.5
        self.kdist = 0.5
        self.opttrack = None
        self.start_opt = False

        # state to save
        self.lanewidth = 100
        self.pts = {
            'cone': [],
            'turn': [],
            'home': [],
        }
        self.scaleref = 0.02
        self.offsetlock = False
        self.homeangle = 0.0

    def getstate(self):
        return (self.lanewidth, self.pts,
                self.scaleref, self.offsetlock, self.homeangle)

    def setstate(self, s):
        (self.lanewidth, self.pts, self.scaleref,
            self.offsetlock, self.homeangle) = s
        self.selectedpt = None

    def render_turns(self, scale, rectmin):
        turns = self.pts['turn']
        Nturns = len(turns)
        if Nturns < 3:
            return
        t = np.float32(turns).T
        t[1] = -t[1]  # invert y
        tx = tapetrack.trackexport(t)
        N = 100*Nturns
        s = (tapetrack.tracksubdiv(tx, N) * scale)
        s[:, 0] += rectmin[0]
        s[:, 1] = rectmin[1] - s[:, 1]  # uninvert y
        s[:, 3] = -s[:, 3]
        L = s[:, :2] + s[:, 2:4]*self.lanewidth
        R = s[:, :2] - s[:, 2:4]*self.lanewidth
        dl = imgui.get_window_draw_list()
        cyan = imgui.get_color_u32_rgba(0, 1, 1, 1)
        yellow = imgui.get_color_u32_rgba(1, 1, 0, 1)
        for i in range(1+N):
            a = i % N
            b = (i+1) % N
            if i & 1:
                dl.add_line(s[a, 0], s[a, 1], s[b, 0], s[b, 1], yellow)
            dl.add_line(L[a, 0], L[a, 1], L[b, 0], L[b, 1], cyan)
            dl.add_line(R[a, 0], R[a, 1], R[b, 0], R[b, 1], cyan)
        return s

    def start_optimization(self):
        self.start_opt = True

    def render_opttrack(self, scale, rectmin):
        turns = self.pts['turn']
        Nturns = len(turns)
        if Nturns < 3:
            return
        t = np.float32(turns).T
        t[1] = -t[1]  # invert y
        tx = tapetrack.trackexport(t)
        s = tapetrack.tracksubdiv(tx, 100)
        if self.start_opt:
            self.start_opt = False
            sscale = np.max(s[:, :2]) - np.min(s[:, :2])
            u = s[:, 0] + 1j*s[:, 1]
            ye, val, stuff = track_opt.OptimizeTrack(
                u/sscale, lanewidth=2*self.lanewidth/sscale, kcurv=self.kcurv, kdist=self.kdist)
            print(u[:30]/sscale)
            print(ye[:30])
            print(ye[:30]*sscale)
            print(self.lanewidth / sscale)
            self.opttrack = ye*sscale

        if self.opttrack is None:
            return

        ye = self.opttrack
        s *= scale
        s[:, 0] += rectmin[0]
        s[:, 1] = rectmin[1] - s[:, 1]  # uninvert y
        s[:, 3] = -s[:, 3]
        L = s[:, :2] + (s[:, 2:4].T*ye.T).T
        dl = imgui.get_window_draw_list()
        magenta = imgui.get_color_u32_rgba(1, 0, 1, 1)
        N = len(ye)
        for i in range(1+N):
            a = i % N
            b = (i+1) % N
            dl.add_line(L[a, 0], L[a, 1], L[b, 0], L[b, 1], magenta)

    def render(self, changed):
        imgui.begin("birdseye")
        _, self.scaleref = imgui.input_float(
            "map scale (m/pixel)", self.scaleref)

        for i, mode in enumerate(["cone", "turn", "home"]):
            if i != 0:
                imgui.same_line()
            if imgui.radio_button(mode, self.editmode == mode):
                self.editmode = mode
                self.selectedpt = None

        imgui.same_line()
        if imgui.button("delete last"):
            self.pts[self.editmode] = self.pts[self.editmode][:-1]

        self.render_editor()

        imgui.end()

    def add_point(self, u, v):
        if self.editmode == 'turn':
            self.pts['turn'].append([u, v, 250])
        else:
            if self.editmode == 'home':
                # only one of these
                self.pts['home'] = []
            self.pts[self.editmode].append([u, v])
        self.selectedpt = len(self.pts[self.editmode]) - 1

    def select_point(self, u, v):
        if len(self.pts[self.editmode]) == 0:
            return
        pts = np.array(self.pts[self.editmode]).T
        dist = (pts[0] - u)**2 + (pts[1] - v)**2
        if np.min(dist) < 40*40:
            self.selectedpt = np.argmin(dist)
        else:
            self.selectedpt = None

    def move_point(self, u, v):
        self.pts[self.editmode][self.selectedpt][0] = u
        self.pts[self.editmode][self.selectedpt][1] = v

    def render_ptlist(self, dl, ptlist, rectmin, scale, col, r,
                      select=False, lines=False):
        lastuv = None
        for i, p in enumerate(ptlist):
            u = p[0] * scale + rectmin[0]
            v = p[1] * scale + rectmin[1]
            dl.add_rect_filled(u - r, v - r, u + r, v + r, col, r)
            if select and i == self.selectedpt:
                dl.add_rect(u - r+1.5, v - r+1.5, u + r+1.5, v + r+1.5,
                            imgui.get_color_u32_rgba(1, 1, 1, 1), r+1)
            else:
                dl.add_rect(u - r+.5, v - r+.5, u + r+.5, v + r+.5,
                            imgui.get_color_u32_rgba(0, 0, 0, 1), r+1)
            if lines and lastuv is not None:
                dl.add_line(lastuv[0], lastuv[1], u, v, col)
            lastuv = (u, v)

    def render_editor(self):
        if imgui.button("optimize track"):
            self.start_optimization()
        imgui.same_line()
        _, ks = imgui.slider_float2(
            "kcurv, kdist", self.kcurv, self.kdist,
            min_value=0, max_value=100.0, power=2)
        self.kcurv, self.kdist = ks
        _, self.lanewidth = imgui.slider_float(
            "lane width", self.lanewidth, min_value=10, max_value=1000,
            power=1.5)
        # use a child region just to keep appearing/disappearing widgets at the
        # top from shifting stuff around
        imgui.begin_child("region", 0, 25, border=False)
        if (self.editmode == 'turn' and self.selectedpt is not None
                and self.selectedpt < len(self.pts['turn'])):
            i = self.selectedpt
            T = self.pts['turn']
            _, T[i][2] = imgui.slider_float(
                "turn %d radius" % (i+1), T[i][2], min_value=-1000.,
                max_value=1000., power=1.2)
        if self.editmode == 'home':
            _, self.homeangle = imgui.slider_float(
                "home position angle", self.homeangle, -3.141, 3.141)
        imgui.end_child()

        w, h = imgui.get_window_size()
        h = self.mapim.shape[0] * w / self.mapim.shape[1]
        imgui.image_button(self.maptex, w, h, frame_padding=0)
        rectmin = imgui.get_item_rect_min()
        if imgui.is_item_clicked(0):
            mxy = imgui.get_mouse_pos()
            u = (mxy[0] - rectmin[0]) * self.mapim.shape[1] / w
            v = (mxy[1] - rectmin[1]) * self.mapim.shape[1] / w

            if imgui.is_mouse_double_clicked(0):
                self.add_point(u, v)
            else:
                self.select_point(u, v)

        if (imgui.is_item_hovered() and self.selectedpt is not None and
                imgui.is_mouse_dragging(0)):
            mxy = imgui.get_mouse_pos()
            u = (mxy[0] - rectmin[0]) * self.mapim.shape[1] / w
            v = (mxy[1] - rectmin[1]) * self.mapim.shape[1] / w
            self.move_point(u, v)

        scale = w / self.mapim.shape[1]
        dl = imgui.get_window_draw_list()
        self.render_ptlist(dl, self.pts['cone'], rectmin, scale,
                           imgui.get_color_u32_rgba(1, 0.7, 0, 1), 4,
                           self.editmode == 'cone')
        self.render_ptlist(dl, self.pts['turn'], rectmin, scale,
                           imgui.get_color_u32_rgba(0, 0.3, 1, 1), 3,
                           self.editmode == 'turn')
        self.render_ptlist(dl, self.pts['home'], rectmin, scale,
                           imgui.get_color_u32_rgba(1, 1, 1, 1), 10,
                           self.editmode == 'home')
        # render home position angle
        if len(self.pts['home']) > 0:
            h = self.pts['home'][0]
            S = 100*np.sin(self.homeangle)
            C = 100*np.cos(self.homeangle)
            dl.add_line(rectmin[0] + h[0]*scale, rectmin[1] + h[1]*scale,
                        rectmin[0] + h[0]*scale + C, rectmin[1] + h[1]*scale - S,
                        imgui.get_color_u32_rgba(1, 1, 1, 1))

        self.render_turns(scale, rectmin)
        self.render_opttrack(scale, rectmin)

        if self.editmode == "cone":
            zoomtip(self.maptex, self.mapim.shape, 2)

    def save_cones(self, fname):
        scale = self.scaleref
        cones = np.array(self.pts['cone']) * scale
        f = open(fname, "w")
        f.write("%d\n" % len(cones))
        for c in cones:
            f.write("%f %f\n" % (c[0], -c[1]))
        home = np.array(self.pts['home']) * scale
        if len(home) > 0:
            f.write("home %f %f %f\n" % (
                home[0][0], -home[0][1], self.homeangle))
        f.close()

    def save_track(self, fname):
        scale = self.scaleref
        turns = self.pts['turn']
        Nturns = len(turns)
        if Nturns < 3:
            return
        t = np.float32(turns).T
        t[1] = -t[1]  # invert y
        tx = tapetrack.trackexport(t)
        s = tapetrack.tracksubdiv(tx, 100)
        u = (s[:, 0] + 1j*s[:, 1]) * scale
        ye = self.opttrack * scale
        N = 1j*track_opt.TrackNormal(u)
        x = x = u + ye*N
        TN = 1j*track_opt.TrackNormal(x)
        traj = np.vstack([np.real(x), np.imag(x), np.real(TN),
                          np.imag(TN), track_opt.TrackCurvature(x)]).T

        f = open(fname, "w")
        f.write("%d\n" % len(traj))
        for t in traj:
            f.write(" ".join(map(str, t)))
            f.write("\n")
        f.close()


def main(mapim):
    imgui.create_context()
    window = impl_glfw_init()
    impl = GlfwRenderer(window)

    mw = TrackplanWindow(cv2.imread(mapim))

    status = 'Ready'
    while not glfw.window_should_close(window):
        glfw.poll_events()
        impl.process_inputs()

        imgui.new_frame()

        loaded = False
        if imgui.begin_main_menu_bar():
            if imgui.begin_menu("File", True):
                load1, _ = imgui.menu_item("Load pts")
                if load1:
                    try:
                        f = open("trackstate.pkl", "rb")
                        rwstate = pickle.load(f)
                        mw.setstate(rwstate)
                        loaded = True
                        f.close()
                    except IOError:
                        status = 'Unable to load ptlist.pkl'

                save1, _ = imgui.menu_item("Save pts")
                if save1:
                    f = open("trackstate.pkl", "wb")
                    pickle.dump((mw.getstate()), f)
                    f.close()
                    status = 'Saved point list'

                exportlm, _ = imgui.menu_item("Export cones / home")
                if exportlm:
                    mw.save_cones("lm.txt")
                    status = 'Exported lm.txt'

                exporttrack, _ = imgui.menu_item("Export track")
                if exporttrack:
                    if mw.opttrack is None:
                        status = 'Optimized track not defined!'
                    else:
                        mw.save_track("track.txt")
                        status = 'Exported track.txt'

                clicked_quit, _ = imgui.menu_item(
                    "Quit", 'Cmd+Q', False, True)
                if clicked_quit:
                    exit(0)

                imgui.end_menu()
            imgui.end_main_menu_bar()

        mw.render(loaded)

        imgui.set_next_window_position(
            0, imgui.get_io().display_size[1] - 30)
        imgui.begin("status", flags=0x7f)
        imgui.text(status)
        imgui.end()

        gl.glClearColor(0., 0., 0., 1)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)

        imgui.render()
        impl.render(imgui.get_draw_data())
        glfw.swap_buffers(window)

    impl.shutdown()
    glfw.terminate()


def impl_glfw_init():
    width, height = 1280, 720
    window_name = "cycloid track planner"

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


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("usage:", sys.argv[0], "[mapimg]")
        print("mapimg is exported from ceilslam or any other tool, 2cm/pix")
        exit(1)

    main(sys.argv[1])
