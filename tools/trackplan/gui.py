# -*- coding: utf-8 -*-
from __future__ import print_function

import cv2
import numpy as np
import pickle
import glfw
import OpenGL.GL as gl
import imgui
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
        self.offxy = (0, 0)
        self.im1 = im1
        self.K = K
        self.editmode = 'cone'
        self.cones = []
        self.turns = []
        self.scalerefpts = []
        self.scaleref = 1.0

    def render_turns(self):
        # TODO
        pass

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

        offsetchanged, self.offxy = imgui.slider_float2(
            "offset xy", *self.offxy, min_value=-10, max_value=10)

        for i, mode in enumerate(["cone", "turn", "scaleref"]):
            if i != 0:
                imgui.same_line()
            if imgui.radio_button(mode, self.editmode == mode):
                self.editmode = mode
                print('editmode', mode)

        imgui.same_line()
        if imgui.button("delete last"):
            if self.editmode == "cone":
                self.cones = self.cones[:-1]
            if self.editmode == "turn":
                self.turns = self.turns[:-1]

        _, self.scaleref = imgui.input_float("scale reference dist (m)", self.scaleref)

        # i don't think we actually need the button now
        if imgui.button("recompute mapping") or changed or offsetchanged:
            self.recompute(ptlist1, ptlist2)

        if self.remappedtex is not None:
            self.render_editor()

        imgui.end()

    def render_editor(self):
        w, h = imgui.get_window_size()
        h = self.remappedim.shape[0] * w / self.remappedim.shape[1]
        imgui.image_button(self.remappedtex, w, h, frame_padding=0)

        if self.editmode == "cone" or self.editmode == "scaleref":
            zoomtip(self.remappedtex, self.remappedim.shape, 5)

    def recompute(self, ptlist1, ptlist2):
        H, _ = cv2.findHomography(np.float32(ptlist1), np.float32(ptlist2))
        n, R, t, N = cv2.decomposeHomographyMat(H, self.K)
        # refpts = np.hstack([ptlist1, np.ones((len(ptlist1), 1))])
        n = N[np.argmin(np.array(N)[:, 1, 0])]
        R2 = Rmatrix(n)
        scale = 200
        size = 1000
        M2 = np.float32([
            [-scale, 0, size/2 - scale*self.offxy[0]],
            [0, scale, size/2 - scale*self.offxy[1]],
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
                load1, _ = imgui.menu_item("Load pts", 'Cmd+L')
                if load1:
                    try:
                        f = open("ptlist.pkl", "rb")
                        ptlist1, ptlist2 = pickle.load(f)
                        loaded = True
                        f.close()
                    except IOError:
                        status = 'Unable to load ptlist.pkl'
                save1, _ = imgui.menu_item("Save pts", 'Cmd+S')
                if save1:
                    f = open("ptlist.pkl", "wb")
                    pickle.dump((ptlist1, ptlist2), f)
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
