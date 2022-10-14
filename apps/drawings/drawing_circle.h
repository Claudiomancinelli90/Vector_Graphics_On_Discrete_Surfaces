#pragma once

#include "struct.h"
#include <realtime/gpu.h>
#include <vector>
#include <yocto/yocto_mesh.h>

using gpu::Shape, gpu::Shader;
using std::vector;
using yocto::mesh_point, yocto::vec3i, yocto::vec3f, yocto::mat4f;

struct closed_curve {
  vector<int> strip = {};
  vector<float> lerps = {};
};
struct circle_tids {
  float lerp = -1;
  int offset = -1;
};
typedef vector<closed_curve> Isoline;

struct Circle {
  mesh_point center;
  float radius;
  Isoline isoline;
  vector<int> tids = {};
  vector<vec3f> pos = {};
  vector<float> distances = {};
  float theta = 0.f;
  int primitive = -1;
  vec2i crop_range = vec2i{-1, -1};
  vector<mesh_point> vertices = {};
  vector<vector<mesh_point>> spider_vertices = {};
  vector<vector<vec3f>> arcs = {};
  int petals = -1;
  bool with_inner_circle = false;
  int garland_vertices = -1;
  int levels = -1;
};
struct Ellipse {
  mesh_point midpoint;
  float scale_factor;
  Isoline isoline;
  vector<float> distances_from_midpoint = {};
};

void draw_isoline(unordered_map<string, Shader> &shaders,
                  const Isoline &isoline, mat4f &view, mat4f &projection);

vector<vec3f> closed_curve_positions(const closed_curve &curve,
                                     const vector<vec3i> &triangles,
                                     const vector<vec3f> &positions,
                                     const vector<vec3i> &adjacencies,
                                     const vec2i &range = {-1, -1});

mesh_point closest_point_on_circle(const bezier_mesh &mesh,
                                   const Circle *circle,
                                   const mesh_point &point);

vector<vec3f> circle_positions(const vector<vec3i> &triangles,
                               const vector<vec3f> &positions,
                               const vector<vec3i> &adjacencies,
                               const Circle &c0);

closed_curve create_closed_curve(const bezier_mesh &mesh,
                                 unordered_map<int, vec3f> &tids);

closed_curve create_closed_curve(const bezier_mesh &mesh, const Circle &circle,
                                 const unordered_map<int, float> &tids);

unordered_map<int, vec3f>
create_tids(const bezier_mesh &mesh, const vector<float> &distances0,
            float length0, const vector<float> &distances1, float length1);

unordered_map<int, vec3f> create_tids(const bezier_mesh &mesh,
                                      const vector<float> &distances0,
                                      const vector<float> &distances1);

unordered_map<int, vec3f> create_tids(const bezier_mesh &mesh,
                                      const vector<float> &distances,
                                      const float &radius);

Isoline create_isoline(const bezier_mesh &mesh, const vector<float> &distances,
                       const float &radius);

Isoline create_isoline(const bezier_mesh &mesh, const vector<float> &distances0,
                       const vector<float> &distances1);

// -------------------------------------------------------------------------------------
mesh_point get_closed_curve_point(const vector<vec3i> &triangles,
                                  const vector<vec3f> &positions,
                                  const vector<vec3i> &adjacencies,
                                  const closed_curve &curve, int ix);

bool set_radius(const bezier_mesh &mesh, Circle *circle, const float &radius,
                const mesh_point &seed = {});

bool set_center(const bezier_mesh &mesh, Circle *circle,
                const mesh_point &center);

Circle create_circle(const bezier_mesh &mesh, const mesh_point &center,
                     const mesh_point &point);

Circle create_circle(const bezier_mesh &mesh, const mesh_point &center,
                     const float &radius);
vec2i cropping_range(const bezier_mesh &mesh, const Circle &circle,
                     const vector<mesh_point> &range);

std::pair<vec2i, bool> cropping_range(const bezier_mesh &mesh,
                                      const Circle &circle,
                                      const vector<mesh_point> &range,
                                      const mesh_point &interpolating);
vec2i cropping_range_generic_points(const bezier_mesh &mesh,
                                    const Circle &circle,
                                    vector<mesh_point> &range);
vector<vec3f> crop_circle(const vector<vec3f> &pos, const vec2i &range);

vector<vec3f> crop_circle(const bezier_mesh &mesh, const vector<vec3f> &pos,
                          const vec2i &range,
                          const vector<mesh_point> &extreme);
vector<vector<vec3f>> crop_circle(const bezier_mesh &mesh,
                                  const vector<vec3f> &pos,
                                  const vector<vec2i> &ranges,
                                  const vector<vector<mesh_point>> &extrema);
vector<vec3f> crop_outside_circle(const bezier_mesh &mesh,
                                  const Circle &curr_circle,
                                  const Circle &container,
                                  const mesh_point &start,
                                  const int start_entry, const mesh_point &end,
                                  const int end_entry);
mesh_point find_center_for_spider_net(const bezier_mesh &mesh,
                                      const mesh_point &a, const mesh_point &b,
                                      const Circle *c);