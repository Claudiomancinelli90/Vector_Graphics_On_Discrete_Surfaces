#pragma once

#include <realtime/window.h>

using namespace std;
using namespace window;

struct my_Window;

struct my_Callbacks {
  // Called when a key is pressed or release.
  function<void(my_Window&, Key key, bool pressed)> key;

  // Called when a mouse button is pressed or release.
  function<void(my_Window&, bool left, bool pressed)> click;

  // Called when scroll is preformed.
  function<void(my_Window&, float amount)> scroll;

  // Called when drag and dropping files onto window.
  function<void(my_Window&, const vector<string>&)> drop;

  // Called when window gains or lose focus.
  function<void(my_Window&, int focused)> focus;

  function<void(my_Window&, float xpos, float ypos, bool mouse_hold)> mouse_move;
};

struct my_Window : public Window {
  unsigned short frames = 0;

  my_Callbacks my_callbacks = {};
  bool         mouse_hold = false;

  void drop(const vector<string>& names) { my_callbacks.drop(*this, names); }
  void key(Key key, bool pressed) { my_callbacks.key(*this, key, pressed); }
  void click(bool left, bool pressed) { my_callbacks.click(*this, left, pressed); }
  void scroll(float amount) { my_callbacks.scroll(*this, amount); }
  void focus(int focused) { my_callbacks.focus(*this, focused); }
  void mouse_move(float xpos, float ypos, bool mouse_hold) { my_callbacks.mouse_move(*this, xpos, ypos, mouse_hold); }
};

void init_callbacks(my_Window& win);

void run_draw_loop(my_Window& win, function<void(my_Window&)> draw, bool wait);