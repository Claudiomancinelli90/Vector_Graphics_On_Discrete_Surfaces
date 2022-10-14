#include <GLFW/glfw3.h>

#include "drawing_window.h"

void init_callbacks(my_Window& win) {
  if (win.my_callbacks.mouse_move) {
    glfwSetCursorPosCallback(win.glfw,
        [](GLFWwindow* glfw, double xpos, double ypos) {
          auto win = (my_Window*)glfwGetWindowUserPointer(glfw);
          win->mouse_move((float)xpos, (float)ypos, win->mouse_hold);
        });
  }

  if (win.callbacks.key && !win.my_callbacks.key) {
    win.my_callbacks.key = win.callbacks.key;
    win.callbacks.key = nullptr;
  }

  if (win.my_callbacks.key) {
    glfwSetKeyCallback(win.glfw,
        [](GLFWwindow* glfw, int key, int scancode, int action, int mods) {
          auto win = (my_Window*)glfwGetWindowUserPointer(glfw);
          win->key(window::Key(key), (bool)action);
        });
  }

  if (win.callbacks.click && !win.my_callbacks.click) {
    win.my_callbacks.click = win.callbacks.click;
    win.callbacks.click = nullptr;
  }

  if (win.my_callbacks.click) {
    glfwSetMouseButtonCallback(
        win.glfw, [](GLFWwindow* glfw, int button, int action, int mods) {
          auto win = (my_Window*)glfwGetWindowUserPointer(glfw);
          win->click(button == GLFW_MOUSE_BUTTON_LEFT, (bool)action);
        });
  }

  if (win.callbacks.scroll && !win.my_callbacks.scroll) {
    win.my_callbacks.scroll = win.callbacks.scroll;
    win.callbacks.scroll = nullptr;
  }

  if (win.my_callbacks.scroll) {
    glfwSetScrollCallback(
        win.glfw, [](GLFWwindow* glfw, double xoffset, double yoffset) {
          auto win = (my_Window*)glfwGetWindowUserPointer(glfw);
          win->scroll((float)yoffset);
        });
  }

  if (win.callbacks.drop && !win.my_callbacks.drop) {
    win.my_callbacks.drop = win.callbacks.drop;
    win.callbacks.drop = nullptr;
  }

  if (win.my_callbacks.drop) {
    glfwSetDropCallback(
        win.glfw, [](GLFWwindow* glfw, int num, const char** paths) {
          auto win   = (my_Window*)glfwGetWindowUserPointer(glfw);
          auto pathv = std::vector<std::string>();
          for (auto i = 0; i < num; i++) pathv.push_back(paths[i]);
          win->drop(pathv);
        });
  }

  if (win.callbacks.focus && !win.my_callbacks.focus) {
    win.my_callbacks.focus = win.callbacks.focus;
    win.callbacks.focus = nullptr;
  }

  if (win.my_callbacks.focus) {
    glfwSetWindowFocusCallback(win.glfw, [](GLFWwindow* glfw, int focus) {
      auto win = (my_Window*)glfwGetWindowUserPointer(glfw);
      win->focus(focus);
    });
  }
}


void run_draw_loop(my_Window& win, function<void(my_Window&)> draw, bool wait) {
  while (!should_window_close(win)) {
    update_window_size(win);
    update_input(win.input, win);
    update_joystick_input(win);
    draw(win);
    swap_buffers(win);
    poll_events(win, wait);
  }
}