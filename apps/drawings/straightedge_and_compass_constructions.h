#pragma once

#include "drawing_circle.h"
#include "geometry.h"
#include <realtime/gpu.h>
#include <vector>
#include <yocto/yocto_mesh.h>
#include <yocto_gui/yocto_shade.h>

using namespace std;
using namespace yocto;

enum scc_state {
  SCC_NONE,
  GEODESIC,
  CIRCLE,
  ELLIPSE,
  INTERSECT,
  BISECTOR,
  MIDPOINT,
  PERPENDICULAR,
  MIRROR,
  TANGENT,
  CIRCLE_3,
  BISECTOR_ANGLE,
  PARALLEL,
  CROP_CIRCLE,
  TRIANGLE,
  SQUARE,
  RHOMBUS,
  RECTANGLE,
  PARALLELOGRAM,
  PENTAGON,
  HEXAGON,
  OCTAGON,
  DECAGON,
  SPIDER_NET,
  CRUX,
  FLOWER
};
enum primitives {
  cir,
  tri,
  square,
  rhomb,
  rett,
  parallelog,
  pent,
  hex,
  oct,
  deca,
  garland_tri,
  garland_sq,
  garland_pent,
  garland_hex,
  spider,
  crux,
  flower
};
enum solution { Greek, Intermediate, Expected };
enum intersection { TwoCurves, TwoCircles, Curve_Circle };
enum triangle_types { IsosceleSameLenth, IsosceleBis, Equilateral, Scalene };
enum rectangle_type { Diagonal, Euclidean, SameLengths };
struct Added_Path {
  vector<vec3f> positions;
  vec3f color;
  float radius;
  shade_instance *instance;
};

struct Added_Points {
  vector<mesh_point> points;
  vec3f color;
  float radius;
  shade_instance *instance;
};

struct SCC {
  scc_state state = scc_state::SCC_NONE;
  bool show_expected = true;

  vector<mesh_point> points = vector<mesh_point>();
  vector<mesh_point> cropping_range = {};
  vector<geodesic_path> paths = {geodesic_path{}};
  vector<Circle> circles = {Circle()};
  vector<Ellipse> Ellipses = {Ellipse()};
  unordered_map<int, vector<int>> circles_to_shape = {};

  mesh_point result_point = mesh_point();
  geodesic_path result_path = geodesic_path();
  Circle result_circle = Circle();

  mesh_point expected_point = mesh_point();
  geodesic_path expected_path = geodesic_path();
  Circle expected_circle = Circle();
  vector<float> lambdas = {}; // could be anything
  vector<Added_Points *> points_shape = {};
  vector<Shape> paths_shape = vector<Shape>();
  vector<Added_Path *> construction_shape = {};
  vector<Added_Path *> input_shape = {};
  vector<Added_Path *> output_shape = {};

  vector<Added_Path *> circle_shape = {};
  Added_Path *result_shape = nullptr;
  Added_Path *geodesic_shape = nullptr;
  Added_Points *samples_shape = nullptr;
  vector<vec3f> result_pos = {};
  vector<vec3f> expected_pos = {};
};

void draw_scc(unordered_map<string, Shader> &shaders, SCC &scc, mat4f &view,
              mat4f &projection);

void clear(SCC &scc);

void reset_triangle(SCC &scc);

void add_path(const bezier_mesh &mesh, SCC &scc, const geodesic_path &path);

void add_point(const bezier_mesh &mesh, SCC &scc, const mesh_point &point);

bool set_result(const vector<vec3i> &triangles, const vector<vec3f> &positions,
                SCC &scc, const mesh_point &point);

bool set_result(const vector<vec3i> &triangles, const vector<vec3f> &positions,
                const vector<vec3i> &adjacencies, SCC &scc,
                const geodesic_path &path);

bool set_result(const vector<vec3i> &triangles, const vector<vec3f> &positions,
                const vector<vec3i> &adjacencies, SCC &scc,
                const Circle &circle);

bool set_expected(const vector<vec3i> &triangles,
                  const vector<vec3f> &positions, SCC &scc,
                  const mesh_point &point);

bool set_expected(const vector<vec3i> &triangles,
                  const vector<vec3f> &positions,
                  const vector<vec3i> &adjacencies, SCC &scc,
                  const geodesic_path &path);

bool set_expected(const vector<vec3i> &triangles,
                  const vector<vec3f> &positions,
                  const vector<vec3i> &adjacencies, SCC &scc,
                  const Circle &circle);

// TODO: remove from header
bool is_same_point(const vector<vec3i> &triangles,
                   const vector<vec3f> &positions,
                   const vector<vec3i> &adjacencies, const mesh_point &a,
                   const mesh_point &b, const float &tol = 5e-4);

mesh_point closer_point(const vector<vec3i> &triangles,
                        const vector<vec3f> &positions,
                        const vector<vec3i> &adjacencies, const mesh_point &p,
                        const mesh_point &a, const mesh_point &b,
                        const float &tol = 5e-4);

mesh_point further_point(const vector<vec3i> &triangles,
                         const vector<vec3f> &positions,
                         const vector<vec3i> &adjacencies, const mesh_point &p,
                         const mesh_point &a, const mesh_point &b,
                         const float &tol = 5e-4);

mesh_point intersect(const bezier_mesh &mesh, const geodesic_path &first,
                     const geodesic_path &second);

pair<mesh_point, mesh_point> intersect(const bezier_mesh &mesh,
                                       const vector<mesh_point> &path,
                                       const Isoline &isoline);

pair<mesh_point, mesh_point> intersect(const bezier_mesh &mesh,
                                       const geodesic_path &path,
                                       const Circle &circle);

pair<mesh_point, mesh_point> intersect(const bezier_mesh &mesh,
                                       const geodesic_path &path,
                                       const Isoline &isoline);

pair<mesh_point, mesh_point>
intersect(const bezier_mesh &mesh, const Circle &first, const Circle &second);

std::tuple<geodesic_path, Circle, Circle>
find_segment_bisector(const bezier_mesh &mesh, const geodesic_path &path);

Isoline find_segment_bisector_isolines(const bezier_mesh &mesh,
                                       const geodesic_path &path);

mesh_point find_midpoint(const bezier_mesh &mesh, const geodesic_path &path);

std::tuple<geodesic_path, Circle, Circle, Circle>
find_angle_bisector(const bezier_mesh &mesh, const geodesic_path &first,
                    const geodesic_path &second,
                    const float &scale_factor = (float)2 / (float)3);

mesh_point find_mirror_point(const bezier_mesh &mesh, const geodesic_path &path,
                             const mesh_point &point, const float &tol = 9e-2F);

geodesic_path find_perpendicular(const bezier_mesh &mesh,
                                 const geodesic_path &path,
                                 const mesh_point &point,
                                 const float &tol = 9e-2F);

geodesic_path find_tangent_to_circle(const bezier_mesh &mesh,
                                     const Circle &circle,
                                     const mesh_point &point,
                                     const float &tol = 9e-2F);

std::tuple<Circle, geodesic_path, geodesic_path>
find_circle_3_points(const bezier_mesh &mesh, const mesh_point &a,
                     const mesh_point &b, const mesh_point &c);

geodesic_path find_parallel(const bezier_mesh &mesh, const geodesic_path &path,
                            const mesh_point &point);

void parallel_transp_in_path(const bezier_mesh &mesh, const geodesic_path &path,
                             vec3f &v, const int &from, const int &to);

geodesic_path expected_segment_bisector(const bezier_mesh &mesh,
                                        const geodesic_path &path);

mesh_point expected_midpoint(const bezier_mesh &mesh,
                             const geodesic_path &path);

mesh_point expected_mirror_point(const bezier_mesh &mesh,
                                 const geodesic_path &path,
                                 const mesh_point &point, float tol = 9e-2F);

geodesic_path expected_perpendicular(const bezier_mesh &mesh,
                                     const geodesic_path &path,
                                     const mesh_point &point,
                                     const float &len = -1, float tol = 9e-2F);

geodesic_path expected_parallel(const bezier_mesh &mesh,
                                const geodesic_path &path,
                                const mesh_point &point);

geodesic_path expected_angle_bisector(const bezier_mesh &mesh,
                                      const geodesic_path &first,
                                      const geodesic_path &second);

geodesic_path expected_tangent_to_circle(const bezier_mesh &mesh,
                                         const Circle &circle,
                                         const mesh_point &point,
                                         const float &tol = 9e-2F);

std::tuple<Circle, Isoline, Isoline>
expected_circle_3_points(const bezier_mesh &mesh, const mesh_point &a,
                         const mesh_point &b, const mesh_point &c);

geodesic_path expected_parallel(const bezier_mesh &mesh,
                                const geodesic_path &path,
                                const mesh_point &point);

std::tuple<vector<mesh_point>, Circle, Circle>
equilateral_triangle(const bezier_mesh &mesh, const mesh_point &a,
                     const mesh_point &b, const bool flipped);
vector<mesh_point> equilateral_triangle_tangent_space(const bezier_mesh &mesh,
                                                      const Circle *circle,
                                                      const float teta = 0.f);
vector<mesh_point> square_in_tangent_space(const bezier_mesh &mesh,
                                           const Circle *circle,
                                           const float &teta = 0.f);

vector<mesh_point> rhombus_tangent_space(const bezier_mesh &mesh,
                                         const Circle *circle,
                                         const float &lambda,
                                         const float &teta = 0.f);
vector<mesh_point> parallelogram_tangent_space(const bezier_mesh &mesh,
                                               const Circle *circle,
                                               const float &sigma,
                                               const float &lambda,
                                               const float &teta = 0.f);
vector<mesh_point> pentagon_tangent_space(const bezier_mesh &mesh,
                                          const Circle *circle,
                                          const float &teta = 0.f);

vector<mesh_point> hexagon_tangent_space(const bezier_mesh &mesh,
                                         const Circle *circle,
                                         const float &teta = 0.f);

vector<mesh_point> octagon_tangent_space(const bezier_mesh &mesh,
                                         const Circle *circle);

vector<mesh_point> decagon_tangent_space(const bezier_mesh &mesh,
                                         const Circle *circle);

vector<vector<mesh_point>> make_garland(const bezier_mesh &mesh, Circle *c,
                                        const vector<mesh_point> &vertices);

vector<Circle> make_concentric_circles(const bezier_mesh &mesh, Circle *c);

vector<float> subdivide_radius(const float &radius, const int k);

std::tuple<vector<vector<mesh_point>>, vector<vector<vec3f>>>
spider_net(const bezier_mesh &mesh, const Circle *c,
           const bool just_vertices = false);

vector<vector<vec3f>> make_cross(const bezier_mesh &mesh, Circle *c,
                                 const bool no_arcs);
vector<vector<vec3f>> make_flower(const bezier_mesh &mesh, const Circle *circle,
                                  const int k, const bool no_arcs,
                                  const float &scale_factor = 0.5,
                                  const bool &with_inner_circle = false);
vector<vector<mesh_point>>
make_concentric_n_gon(const bezier_mesh &mesh, const Circle *c,
                      const float &sigma, const float &lambda,
                      const vector<mesh_point> &vertices);

std::tuple<vector<mesh_point>, Circle, Circle>
same_lengths_isoscele_triangle(const bezier_mesh &mesh, const mesh_point &a,
                               const mesh_point &b, const float &len,
                               const bool flipped);
std::tuple<vector<mesh_point>, geodesic_path, Circle>
altitude_isoscele_triangle(const bezier_mesh &mesh, const mesh_point &a,
                           const mesh_point &b, const float &len,
                           const bool flipped);

std::tuple<vector<mesh_point>, geodesic_path, Circle, Circle, Circle>
euclidean_rectangle(const bezier_mesh &mesh, const geodesic_path &base,
                    const float &height);

std::tuple<vector<mesh_point>, geodesic_path, geodesic_path, Circle, Circle>
same_lengths_rectangle(const bezier_mesh &mesh, const geodesic_path &base,
                       const float &height);

std::tuple<vector<mesh_point>, geodesic_path, geodesic_path, geodesic_path>
diagonal_rectangle(const bezier_mesh &mesh, const geodesic_path &base,
                   const geodesic_path &diagonal);

pair<mesh_point, int>
sample_on_curve(const vector<vec3i> &triangles, const vector<vec3f> &positions,
                const vector<vec3i> &adjacencies, const geodesic_path &path,
                const vector<float> &path_parameter_t, const float &t);

pair<geodesic_path, vector<float>>
cut_curve(const geodesic_path &path, const vector<float> &path_parameter_t,
          int entry, const mesh_point &sample, bool keep_the_first_portion);

Isoline draw_ellipse(const bezier_mesh &mesh, const geodesic_path &major_axis,
                     const float &dist_from_mid_point);
inline mesh_point eval_point_from_lerp(const int tid, const int offset,
                                       const float &lerp) {
  auto bary = zero3f;
  bary[offset] = 1 - lerp;
  bary[(offset + 1) % 3] = lerp;
  return {tid, vec2f{bary.y, bary.z}};
}
inline vec3f transp_vect_along_path(const bezier_mesh &mesh,
                                    const mesh_point &from,
                                    const mesh_point &to, const vec3f &v) {
  return tranport_vector(mesh.solver, mesh.angles, mesh.total_angles,
                         mesh.triangles, mesh.positions, mesh.adjacencies,
                         mesh.v2t, v, mesh.normals, from, to);
}
inline vec2f tangent_path_direction(const bezier_mesh &mesh,
                                    const geodesic_path &path,
                                    bool start = true) {
  auto find = [](const vec3i &vec, int x) {
    for (int i = 0; i < size(vec); i++)
      if (vec[i] == x)
        return i;
    return -1;
  };

  auto direction = vec2f{};

  if (start) {
    auto start_tr =
        triangle_coordinates(mesh.triangles, mesh.positions, path.start);

    if (path.lerps.empty()) {
      direction = interpolate_triangle(start_tr[0], start_tr[1], start_tr[2],
                                       path.end.uv);
    } else {
      auto x = path.lerps[0];
      auto k = find(mesh.adjacencies[path.strip[0]], path.strip[1]);
      direction = lerp(start_tr[k], start_tr[(k + 1) % 3], x);
    }
  } else {
    auto end_tr =
        triangle_coordinates(mesh.triangles, mesh.positions, path.end);
    if (path.lerps.empty()) {
      direction =
          interpolate_triangle(end_tr[0], end_tr[1], end_tr[2], path.start.uv);
    } else {
      auto x = path.lerps.back();
      auto k = find(mesh.adjacencies[path.strip.rbegin()[0]],
                    path.strip.rbegin()[1]);
      direction = lerp(end_tr[k], end_tr[(k + 1) % 3], 1 - x);
    }
  }
  return normalize(direction);
}
