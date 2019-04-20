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


def imagepicker(name, im, imtex, ptlist):
    imgui.begin(name)
    changed = False
    if imgui.button("delete last"):
        ptlist = ptlist[:-1]
        changed = True
    w, h = imgui.get_window_size()
    h = im.shape[0] * w / im.shape[1]
    imgui.image_button(imtex, w, h, frame_padding=0)
    rectmin = imgui.get_item_rect_min()

    # add selected points to draw list
    dl = imgui.get_window_draw_list()
    for p in ptlist:
        u = p[0] * w / im.shape[1] + rectmin[0]
        v = p[1] * w / im.shape[1] + rectmin[1]
        dl.add_rect_filled(u - 4, v - 4, u + 4, v + 4,
                           imgui.get_color_u32_rgba(1, 0.8, 0.1, 1), 4)
        dl.add_rect(u - 4.5, v - 4.5, u + 4.5, v + 4.5,
                    imgui.get_color_u32_rgba(0, 0, 0, 1), 5)

    zoomtip(imtex, im.shape)

    if imgui.is_item_clicked(0):
        # print("clicked", name, imgui.get_mouse_pos(), imgui.get_item_rect_min())
        # fixme: drag points
        # for now, you just have to be careful where you double-click
        if imgui.is_mouse_double_clicked(0):
            # print("double-clicked", name, imgui.get_mouse_pos(), imgui.get_item_rect_min())
            mxy = imgui.get_mouse_pos()
            u = (mxy[0] - rectmin[0]) * im.shape[1] / w
            v = (mxy[1] - rectmin[1]) * im.shape[1] / w
            ptlist.append((u, v))
            changed = True

    imgui.end()
    return ptlist, changed


def Rmatrix(N):
    R2 = np.eye(3)
    R2[2] = N.reshape(-1)
    R2[0] = np.cross(R2[0], R2[2])
    R2[0] /= np.linalg.norm(R2[0])
    R2[1] = np.cross(R2[2], R2[0])
    R2[1] /= np.linalg.norm(R2[1])
    return R2


class RemapWindow:
    def __init__(self, im1, K):
        self.remappedtex = None
        self.remappedim = None
        self.im1 = im1
        self.K = K
        self.editmode = 'cone'
        self.offxy = (0, 0)
        self.remapscale = 200
        self.lanewidth = 100
        self.pts = {
            'cone': [],
            'turn': [],
            'scaleref': [],
            'home': [],
        }
        self.scaleref = 1.0
        self.offsetlock = False
        self.selectedpt = None
        self.kcurv = 0.5
        self.kdist = 0.5
        self.opttrack = None
        self.start_opt = False

    def getstate(self):
        return (self.offxy, self.remapscale, self.lanewidth, self.pts,
                self.scaleref, self.offsetlock)

    def setstate(self, s):
        (self.offxy, self.remapscale, self.lanewidth, self.pts, self.scaleref,
         self.offsetlock) = s
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

    def render_opttrack(self, s):
        N = s.shape[0]
        if self.start_opt:
            self.start_opt = False
            scale = np.max(s[:, :2]) - np.min(s[:, :2])
            u = s[:, 0] - 1j*s[:, 1]
            ye, val, stuff = track_opt.OptimizeTrack(
                u/scale, lanewidth=2*self.lanewidth/scale, kcurv=self.kcurv, kdist=self.kdist)
            print(u[:30]/scale)
            print(ye[:30])
            print(ye[:30]*scale)
            print(self.lanewidth / scale)
            self.opttrack = ye*scale

        if self.opttrack is None:
            return

        ye = self.opttrack
        L = s[:, :2] - (s[:, 2:4].T*ye.T).T
        dl = imgui.get_window_draw_list()
        magenta = imgui.get_color_u32_rgba(1, 0, 1, 1)
        for i in range(1+N):
            a = i % N
            b = (i+1) % N
            dl.add_line(L[a, 0], L[a, 1], L[b, 0], L[b, 1], magenta)

    def render(self, ptlist1, ptlist2, changed):
        imgui.begin("birdseye")
        if len(ptlist1) < 4:
            imgui.text("please add at least 4 points to each image")
            imgui.end()
            return
        if len(ptlist1) != len(ptlist2):
            imgui.text("please add the same number of points to each image")
            imgui.end()
            return

        _, self.offsetlock = imgui.checkbox("offset lock", self.offsetlock)
        offsetchanged = False
        scalechanged = False
        if not self.offsetlock:
            offsetchanged, self.offxy = imgui.slider_float2(
                "offset xy", *self.offxy, min_value=-10, max_value=10)
            scalechanged, self.remapscale = imgui.slider_float(
                "scale", self.remapscale, min_value=10, max_value=10000,
                power=2)
            _, self.scaleref = imgui.input_float(
                "scale reference dist (m)", self.scaleref)

        for i, mode in enumerate(["cone", "turn", "scaleref", "home"]):
            if i != 0:
                imgui.same_line()
            if imgui.radio_button(mode, self.editmode == mode):
                self.editmode = mode
                self.selectedpt = None

        imgui.same_line()
        if imgui.button("delete last"):
            self.pts[self.editmode] = self.pts[self.editmode][:-1]

        if changed or offsetchanged or scalechanged:
            self.recompute(ptlist1, ptlist2)

        if self.remappedtex is not None:
            self.render_editor()

        imgui.end()

    def add_point(self, u, v):
        if self.editmode == 'turn':
            self.pts['turn'].append([u, v, 250])
        else:
            if self.editmode == 'scaleref':
                # only two of these at once
                self.pts['scaleref'] = self.pts['scaleref'][-1:]
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

    def render_ptlist(self, ptlist, rectmin, scale, col, r,
                      select=False, lines=False):
        dl = imgui.get_window_draw_list()
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
            min_value=0, max_value=10.0, power=2)
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
        imgui.end_child()

        w, h = imgui.get_window_size()
        h = self.remappedim.shape[0] * w / self.remappedim.shape[1]
        rectmin = imgui.get_item_rect_min()
        imgui.image_button(self.remappedtex, w, h, frame_padding=0)
        if imgui.is_item_clicked(0):
            mxy = imgui.get_mouse_pos()
            u = (mxy[0] - rectmin[0]) * self.im1.shape[1] / w
            v = (mxy[1] - rectmin[1]) * self.im1.shape[1] / w

            if imgui.is_mouse_double_clicked(0):
                self.add_point(u, v)
            else:
                self.select_point(u, v)

        if (imgui.is_item_hovered() and self.selectedpt is not None and
                imgui.is_mouse_dragging(0)):
            mxy = imgui.get_mouse_pos()
            u = (mxy[0] - rectmin[0]) * self.im1.shape[1] / w
            v = (mxy[1] - rectmin[1]) * self.im1.shape[1] / w
            self.move_point(u, v)

        scale = w / self.im1.shape[1]
        self.render_ptlist(self.pts['cone'], rectmin, scale,
                           imgui.get_color_u32_rgba(1, 0.7, 0, 1), 4,
                           self.editmode == 'cone')
        self.render_ptlist(self.pts['scaleref'], rectmin, scale,
                           imgui.get_color_u32_rgba(0, 1, 0.3, 1), 2,
                           self.editmode == 'scaleref', True)
        self.render_ptlist(self.pts['turn'], rectmin, scale,
                           imgui.get_color_u32_rgba(0, 0.3, 1, 1), 3,
                           self.editmode == 'turn')
        self.render_ptlist(self.pts['home'], rectmin, scale,
                           imgui.get_color_u32_rgba(1, 1, 1, 1), 10,
                           self.editmode == 'home')

        trackdat = self.render_turns(scale, rectmin)
        self.render_opttrack(trackdat)

        if self.editmode == "cone":
            zoomtip(self.remappedtex, self.remappedim.shape, 2)
        elif self.editmode == "scaleref":
            zoomtip(self.remappedtex, self.remappedim.shape, 5)

    def recompute(self, ptlist1, ptlist2):
        H, _ = cv2.findHomography(np.float32(ptlist1), np.float32(ptlist2))
        n, R, t, N = cv2.decomposeHomographyMat(H, self.K)
        # refpts = np.hstack([ptlist1, np.ones((len(ptlist1), 1))])
        n = N[np.argmin(np.array(N)[:, 1, 0])]
        R2 = Rmatrix(n)
        size = 1000
        M2 = np.float32([
            [-self.remapscale, 0, size/2 - self.remapscale*self.offxy[0]],
            [0, self.remapscale, size/2 - self.remapscale*self.offxy[1]],
            [0, 0, 1]
        ])
        MM = np.dot(M2, np.dot(R2, np.linalg.inv(self.K)))
        self.remappedim = cv2.warpPerspective(self.im1, MM, (size, size))
        if self.remappedtex is not None:
            unload_texture(self.remappedtex)
        self.remappedtex = load_texture(self.remappedim)


def main(im1, im2, K):
    im1 = cv2.imread(im1)
    im2 = cv2.imread(im2)

    imgui.create_context()
    window = impl_glfw_init()
    impl = GlfwRenderer(window)

    imtex1 = load_texture(im1)
    imtex2 = load_texture(im2)

    ptlist1 = []
    ptlist2 = []

    rw = RemapWindow(im1, K)

    status = 'Ready'
    while not glfw.window_should_close(window):
        glfw.poll_events()
        impl.process_inputs()

        imgui.new_frame()

        loaded = False
        if imgui.begin_main_menu_bar():
            if imgui.begin_menu("File", True):
                clicked_quit, _ = imgui.menu_item(
                    "Quit", 'Cmd+Q', False, True)
                if clicked_quit:
                    exit(0)
                load1, _ = imgui.menu_item("Load pts")
                if load1:
                    try:
                        f = open("ptlist.pkl", "rb")
                        ptlist1, ptlist2, rwstate = pickle.load(f)
                        rw.setstate(rwstate)
                        loaded = True
                        f.close()
                    except IOError:
                        status = 'Unable to load ptlist.pkl'
                save1, _ = imgui.menu_item("Save pts")
                if save1:
                    f = open("ptlist.pkl", "wb")
                    pickle.dump((ptlist1, ptlist2, rw.getstate()), f)
                    f.close()
                    status = 'Saved point list'
                imgui.end_menu()
            imgui.end_main_menu_bar()

        ptlist1, changed1 = imagepicker("image 1", im1, imtex1, ptlist1)
        ptlist2, changed2 = imagepicker("image 2", im2, imtex2, ptlist2)

        rw.render(ptlist1, ptlist2, loaded or changed1 or changed2)

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
    window_name = "minimal ImGui/GLFW3 example"

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

    # need camera calibration matrix
    K = np.load("../../tools/camcal/flat/camera_matrix.npy")

    if len(sys.argv) < 3:
        print("usage:", sys.argv[0], "[img1] [img2]")
        exit(1)

    main(sys.argv[1], sys.argv[2], K)
