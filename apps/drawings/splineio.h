#pragma once

#include "geometry.h"

bool load_mesh(const string &filename, bezier_mesh &mesh, string &error);

using Svg_Path = vector<array<vec2f, 4>>;
struct Svg_Shape {
  vec3f color = {};
  vector<Svg_Path> paths = {};
};
using Svg = vector<Svg_Shape>;

Svg load_svg(const string &filename);
