#pragma once

#include "VTP/geodesic_algorithm_exact.h"
#include "VTP/geodesic_mesh.h"
#include <cstring>
#include <realtime/window.h>

#include "drawing_circle.h"
#include "drawing_functions.h"
#include "drawing_polygon.h"
#include "geometry.h"
#include "logging.h"

using namespace logging;
using logging::info;
using window::Window;

vector<vec3f> path_positions(const geodesic_path &path,
                             const bezier_mesh &mesh);

// -------------------------------------------------------------------------------------
mesh_point eval_mesh_point(const vector<vec3i> &triangles,
                           const vector<vec3f> &positions, const int &face,
                           const vec3f &point);

mesh_point eval_mesh_point(const vector<vec3i> &triangles,
                           const vector<vec3f> &positions,
                           const vector<int> &strip, const vec3f &point);

int path_point_entry(const geodesic_path &path, const mesh_point &point);

float max_of_field(const vector<float> &distances);

mesh_point intersect_segments(const vector<vec3i> &triangles,
                              const vector<vec3f> &positions, const int &tid,
                              const vec3f &start1, const vec3f &end1,
                              const vec3f &start2, const vec3f &end2);

mesh_point intersect_segments(const vector<vec3i> &triangles,
                              const vector<vec3f> &positions, const int &tid,
                              const mesh_point &start1, const mesh_point &end1,
                              const mesh_point &start2, const mesh_point &end2);

// Intersections with straightest_geodesic approach
void init_intersect_geodesic(const bezier_mesh &mesh, const mesh_point &start,
                             vec3f &dir, vec3f &prev, int &vid, int &tid,
                             vec3f &bary);

geodesic_path find_intersecting_geodesic(const bezier_mesh &mesh,
                                         const vector<geodesic_path> &ends,
                                         const int &angle,
                                         const mesh_point &start,
                                         const vec3f &direction, const int &n,
                                         const float &max_length = 1.0F);

pair<geodesic_path, geodesic_path>
find_intersecting_geodesics(const bezier_mesh &mesh, const mesh_point &start1,
                            const vec3f &direction1, const mesh_point &start2,
                            const vec3f &direction2,
                            const float &max_length = 1.0F);

bool find_intersection(const bezier_mesh &mesh, const int &ix1,
                       const geodesic_path &path1, const int &ix2,
                       const geodesic_path &path2, const int &tid,
                       mesh_point &intersection);

geodesic_path reverse(const geodesic_path &path);

float interpolate_distances(const bezier_mesh &mesh,
                            const vector<float> &distances,
                            const mesh_point &point);
// -------------------------------------------------------------------------------------
float angle(const vector<vec3i> &triangles, const vector<vec3f> &positions,
            const mesh_point &p, const mesh_point &a, const mesh_point &b);

vec3f rotate_in_tangent_space(const vector<vec3i> &triangles,
                              const vector<vec3f> &positions,
                              const mesh_point &from, const vec3f &d,
                              const float &angle);

vec3f rotate_in_tangent_space(const vec3f &axis, const vec3f &d,
                              const float &angle);

vec3f direction_in_tangent_space(const vector<vec3i> &triangles,
                                 const vector<vec3f> &positions,
                                 const mesh_point &p, const vec3f &dir);

vec2f direction_2D_in_tangent_space(const vector<vec3i> &triangles,
                                    const vector<vec3f> &positions,
                                    const int &tid, const vec3f &p,
                                    const vec3f &dir);

vec2f direction_2D_in_tangent_space(const vector<vec3i> &triangles,
                                    const vector<vec3f> &positions,
                                    const mesh_point &p, const vec3f &dir);

vec2f mouse_direction_in_tangent_space(const ray3f &ray,
                                       const vector<vec3i> &triangles,
                                       const vector<vec3f> &positions,
                                       Window &win, const int &tid,
                                       const vec3f &p);

vec2f mouse_direction_in_tangent_space(const ray3f &ray,
                                       const vector<vec3i> &triangles,
                                       const vector<vec3f> &positions,
                                       Window &win, const mesh_point &p);

vec3f geodesic_path_direction(const bezier_mesh &mesh,
                              const geodesic_path &path, int tid);

vec3f geodesic_path_direction(const bezier_mesh &mesh,
                              const geodesic_path &path, bool reverse = false);

// -------------------------------------------------------------------------------------
// TODO: change from mesh_point to vec3f return
mesh_point get_geodesic_path_point(const bezier_mesh &mesh,
                                   const geodesic_path &path, float l);

mesh_point get_geodesic_path_point(const bezier_mesh &mesh,
                                   const geodesic_path &path, int ix);

mesh_point get_geodesic_path_point(const bezier_mesh &mesh,
                                   const geodesic_path &path, int ix, float l);

pair<mesh_point, mesh_point>
get_common_end_points(const vector<vec3i> &triangles,
                      const vector<vec3f> &positions, const geodesic_path &a,
                      const geodesic_path &b);

pair<mesh_point, mesh_point>
get_common_end_points(const vector<vec3i> &triangles,
                      const vector<vec3f> &positions, const geodesic_path &a,
                      const geodesic_path &b, bool &isEnd1, bool &isEnd2);

// -------------------------------------------------------------------------------------
vector<float> get_geodesic_distances(const bezier_mesh &mesh,
                                     const int &source);

vector<float> get_geodesic_distances(const bezier_mesh &mesh,
                                     const mesh_point &source);

vector<float> exact_geodesic_distance(const vector<vec3i> &triangles,
                                      const vector<vec3f> &positions,
                                      const vector<vector<int>> &v2t,
                                      const int &source);

vector<float> exact_geodesic_distance(const vector<vec3i> &triangles,
                                      const vector<vec3f> &positions,
                                      const vector<vector<int>> &v2t,
                                      const mesh_point &source);

mesh_point select_closest(const mesh_point &point,
                          const vector<mesh_point> &list,
                          const vector<vec3i> &triangles,
                          const vector<vec3f> &positions, const float &epsilon);

mesh_point select_closest(const ray3f &ray, const Isoline &isoline,
                          const vector<vec3i> &triangles,
                          const vector<vec3f> &positions,
                          const vector<vec3i> &adjacencies,
                          const float &epsilon);

// find a value in a vector or vecs
template <typename T> inline int find_in_vec(const T &vec, int x) {
  for (int i = 0; i < size(vec); i++)
    if (vec[i] == x)
      return i;
  return -1;
}

inline vector<mesh_point> straightest_geodesic(const bezier_mesh &mesh,
                                               const mesh_point &from,
                                               const vec3f &v, const float &l) {
  return straightest_geodesic(mesh.solver, mesh.triangles, mesh.positions,
                              mesh.normals, mesh.adjacencies, mesh.v2t,
                              mesh.angles, mesh.total_angles, from, v, l);
}

template <typename T> vector<T> reverse_copy(vector<T> v) {
  auto copy = vector<T>();

  for (int i = v.size() - 1; i >= 0; --i) {
    copy.push_back(v[i]);
  }

  return copy;
}

// -------------------------------------------------------------------------------------
// project a point p onto a plane having n as normal and c as center
inline vec3f project_point(const vec3f &p, const vec3f &c, const vec3f &n) {
  auto v = p - c;
  auto proj = n * dot(v, n);

  return c + v - proj;
}

// -------------------------------------------------------------------------------------

inline float get_distance(const vec3i &triangle, const vec3f bary,
                          const vector<float> &distances) {
  return dot(bary, vec3f{distances[triangle.x], distances[triangle.y],
                         distances[triangle.z]});
}

inline float get_distance(const mesh_point &point, const bezier_mesh &mesh,
                          const vector<float> &distances) {
  auto triangle = mesh.triangles[point.face];

  return dot(get_bary(point.uv),
             vec3f{distances[triangle.x], distances[triangle.y],
                   distances[triangle.z]});
}

// Intersect a ray with a point (approximate)
inline bool intersect_point(const ray3f &ray, const vec3f &p, float &dist) {
  // find parameter for line-point minimum distance
  auto w = p - ray.o;
  auto t = dot(w, ray.d) / dot(ray.d, ray.d);

  // exit if not within bounds
  if (t < ray.tmin - flt_eps || t > ray.tmax + flt_eps) {
    return false;
  }

  // test for line-point distance vs point radius
  auto rp = ray.o + ray.d * t;
  auto prp = p - rp;
  if (dot(prp, prp) > flt_eps * flt_eps)
    return false;

  // intersection occurred: set params and exit
  dist = t;
  return true;
}

// Intersect a ray with a line
inline bool intersect_line(const ray3f &ray, const vec3f &p0, const vec3f &p1,
                           float &dist) {
  // setup intersection params
  auto u = ray.d;
  auto v = p1 - p0;
  auto w = ray.o - p0;

  // compute values to solve a linear system
  auto a = dot(u, u);
  auto b = dot(u, v);
  auto c = dot(v, v);
  auto d = dot(u, w);
  auto e = dot(v, w);
  auto det = a * c - b * b;

  // check determinant and exit if lines are parallel
  // (could use EPSILONS if desired)
  if (det == 0)
    return false;

  // compute Parameters on both ray and segment
  auto t = (b * e - c * d) / det;
  auto s = (a * e - b * d) / det;

  // exit if not within bounds
  if (t < ray.tmin - 1e-4F || t > ray.tmax + 1e-4F || s < -1e-4F ||
      s > 1 + 1e-4F) {
    return false;
  }

  dist = t;
  return true;
}

// Intersect a ray with a plane
inline bool intersect_plane(const ray3f &ray, const vec3f &p, const vec3f &n,
                            float &dist) {
  auto den = dot(ray.d, n);

  if (den == 0)
    return false;

  auto t = dot(p - ray.o, n) / den;

  if (t < ray.tmin || t > ray.tmax)
    return false;

  dist = t;
  return true;
}

inline float distance(const vector<vec3i> &triangles,
                      const vector<vec3f> &positions, const mesh_point &a,
                      const mesh_point &b) {
  return distance(eval_position(triangles, positions, a),
                  eval_position(triangles, positions, b));
}

inline bool validate_points(std::initializer_list<mesh_point> list) {
  for (auto point : list) {
    if (point.face == -1)
      return false;
  }

  return true;
}
inline bool validate_points(const vector<mesh_point> &list) {
  for (auto point : list) {
    if (point.face == -1)
      return false;
  }

  return true;
}
inline bool validate_paths(std::initializer_list<geodesic_path> list) {
  for (auto path : list) {
    if (path.start.face == -1 || path.end.face == -1 || path.strip.size() == 0)
      return false;
  }

  return true;
}

inline bool validate_circles(std::initializer_list<Circle> list) {
  for (auto circle : list) {
    if (circle.center.face == -1 || circle.radius <= 0)
      return false;
  }

  return true;
}

inline bool validate_curves(std::initializer_list<closed_curve> list) {
  for (auto curve : list) {
    if (curve.strip.size() <= 2)
      return false;
  }

  return true;
}

geodesic_path straightest_geodesic_path(const bezier_mesh &mesh,
                                        const mesh_point &from, const vec3f &v,
                                        const float &l);

geodesic_path join_paths(const geodesic_path &a, const geodesic_path &b);
