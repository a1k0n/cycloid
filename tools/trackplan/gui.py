# -*- coding: utf-8 -*-
from __future__ import print_function

import cv2
import glfw
import OpenGL.GL as gl
import imgui
from imgui.integrations.glfw import GlfwRenderer


def imagepicker(name, im, imtex, ptlist):
    imgui.begin(name)
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

    if imgui.is_item_hovered():
        mousepos = imgui.get_mouse_pos()
        u = float(mousepos[0] - rectmin[0]) / w
        v = float(mousepos[1] - rectmin[1]) / h
        imgui.begin_tooltip()
        tw = 32. / im.shape[1]
        th = 32. / im.shape[0]
        imgui.image(imtex, 64, 64, uv0=(u-tw, v-th), uv1=(u+tw, v+th))
        dl = imgui.get_window_draw_list()
        rm = imgui.get_item_rect_min()
        col = imgui.get_color_u32_rgba(1, 1, 0, 1)
        dl.add_line(rm[0], rm[1]+32, rm[0]+64, rm[1]+32, col)
        dl.add_line(rm[0]+32, rm[1], rm[0]+32, rm[1]+64, col)
        imgui.end()
    if imgui.is_item_clicked(0):
        # print("clicked", name, imgui.get_mouse_pos(), imgui.get_item_rect_min())
        # fixme: drag points
        # for now, you just have to be careful where you double-click
        if imgui.is_mouse_double_clicked(0):
            # print("double-clicked", name, imgui.get_mouse_pos(), imgui.get_item_rect_min())
            mxy = imgui.get_mouse_pos()
            u = (mxy[0] - rectmin[0]) * im.shape[1] / w
            v = (mxy[1] - rectmin[1]) * im.shape[1] / w
            print("added", name, "point", u, v)
            ptlist.append((u, v))

    imgui.end()


def main(im1, im2):
    im1 = cv2.imread(im1)
    im2 = cv2.imread(im2)

    imgui.create_context()
    window = impl_glfw_init()
    impl = GlfwRenderer(window)

    imtex1 = load_texture(im1)
    imtex2 = load_texture(im2)

    ptlist1 = []
    ptlist2 = []

    while not glfw.window_should_close(window):
        glfw.poll_events()
        impl.process_inputs()

        imgui.new_frame()

        if imgui.begin_main_menu_bar():
            if imgui.begin_menu("File", True):
                clicked_quit, selected_quit = imgui.menu_item(
                    "Quit", 'Cmd+Q', False, True)
                if clicked_quit:
                    exit(0)
                imgui.end_menu()
            imgui.end_main_menu_bar()

        imagepicker("image 1", im1, imtex1, ptlist1)
        imagepicker("image 2", im2, imtex2, ptlist2)

        gl.glClearColor(0., 0., 0., 1)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)

        imgui.render()
        impl.render(imgui.get_draw_data())
        glfw.swap_buffers(window)

    impl.shutdown()
    glfw.terminate()


def load_texture(im):
    # gl.glEnable(gl.GL_TEXTURE_2D)
    texid = gl.glGenTextures(1)
    gl.glBindTexture(gl.GL_TEXTURE_2D, texid)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)
    gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_RGBA,
                    im.shape[1], im.shape[0], 0,
                    gl.GL_BGR, gl.GL_UNSIGNED_BYTE, im)
    return texid


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

    if len(sys.argv) < 3:
        print("usage:", sys.argv[0], "[img1] [img2]")
        exit(1)

    main(sys.argv[1], sys.argv[2])
