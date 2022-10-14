
#include <iostream>
#include <realtime/ext/imgui/imgui.h>
//#include <realtime/gpu.h>
#include <yocto/yocto_commonio.h>
#include <yocto/yocto_math.h>
//#include <yocto/yocto_trace.h>
#include <yocto_gui/yocto_opengl.h>

#include "app.h"
#include "drawing_functions.h"
#include "drawing_polygon.h"
#include "geometry.h"
#include "imgui_extension.h"
#include "logging.h"
#include "splineio.h"
#include "straightedge_and_compass_constructions.h"

using namespace logging;
using namespace window;

using std::array;

enum struct gradient_type {
  pw_linear,
  pw_constant,
};
using App = App_base;

inline void intersect_circle_center(App &app, const vec2f &mouse) {
  // Find index of clicked control point.
  float min_dist = flt_max;
  float threshold = 0.1;

  for (auto i = 0; i < app.scc.circles.size() - 1; ++i) {
    auto &circ = app.scc.circles[i];
    auto &point = circ.center;
    auto pos = eval_position(app.mesh.triangles, app.mesh.positions, point);
    auto pos_ss = screenspace_from_worldspace(app, pos);
    float dist = length(pos_ss - mouse);
    if (dist <= min_dist) {
      app.selected_circle = &circ;
      min_dist = dist;
      app.selected_circle_entries = app.scc.circles_to_shape.at(i);
      app.curr_color = (circ.primitive != primitives::cir)
                           ? app.added_paths[app.selected_circle_entries[0] + 1]
                                 ->instance->material->color
                           : app.added_paths[app.selected_circle_entries[0]]
                                 ->instance->material->color;
      // app.max_radius = max_of_field(app.mesh, circ.distances);
    }
  }
  app.overlapping_circles.clear();
  for (auto i = 0; i < app.scc.circles.size() - 1; ++i) {
    if (app.scc.circles[i].center.face == app.selected_circle->center.face)
      app.overlapping_circles.push_back(i);
  }
}
mesh_point intersect_geodesic_point(App &app, const mesh_point &point) {
  float min_dist = flt_max;
  float threshold = 0.1;
  auto closest_point = mesh_point{};
  auto curr_pos = eval_position(app.mesh.triangles, app.mesh.positions, point);
  for (auto &path : app.scc.paths) {
    if (path.end.face == -1)
      continue;
    auto pos = path_positions(path, app.mesh);
    for (auto j = 0; j < pos.size(); ++j) {
      float dist = length(curr_pos - pos[j]);
      if (dist < threshold && dist < min_dist) {
        closest_point = get_geodesic_path_point(app.mesh, path, j);
        min_dist = dist;
      }
    }
  }
  return closest_point;
}
inline mesh_point intersect_circle_point(App &app, const vec2f &mouse) {
  // Find index of clicked control point.
  float min_dist = flt_max;
  float threshold = 0.1;
  auto closest_point = mesh_point{};
  if (app.selected_circle->radius == 0.f)
    return mesh_point{};

  for (auto &curve : app.selected_circle->isoline) {
    auto curve_pos = closed_curve_positions(
        curve, app.mesh.triangles, app.mesh.positions, app.mesh.adjacencies);

    for (auto i = 0; i < curve_pos.size(); ++i) {
      auto pos_ss = screenspace_from_worldspace(app, curve_pos[i]);
      float dist = length(pos_ss - mouse);
      if (dist < min_dist) {
        closest_point =
            get_closed_curve_point(app.mesh.triangles, app.mesh.positions,
                                   app.mesh.adjacencies, curve, i);

        min_dist = dist;
      }
    }
  }
  return closest_point;
}
inline mesh_point intersect_isoline_point(App &app, const mesh_point &curr) {
  float min_dist = flt_max;
  float threshold = 0.1;
  auto closest_point = mesh_point{};
  auto curr_pos = eval_position(app.mesh.triangles, app.mesh.positions, curr);
  if (app.isoline.size() == 0.f)
    return mesh_point{};

  for (auto &curve : app.isoline) {
    auto curve_pos = closed_curve_positions(
        curve, app.mesh.triangles, app.mesh.positions, app.mesh.adjacencies);

    for (auto i = 0; i < curve_pos.size(); ++i) {
      float dist = length(curr_pos - curve_pos[i]);
      if (dist < threshold && dist < min_dist) {
        closest_point =
            get_closed_curve_point(app.mesh.triangles, app.mesh.positions,
                                   app.mesh.adjacencies, curve, i);

        min_dist = dist;
      }
    }
  }
  return closest_point;
}
inline mesh_point intersect_isoline_point(App &app, const vec3f &curr_pos) {
  float min_dist = flt_max;
  float threshold = 0.1;
  auto closest_point = mesh_point{};
  if (app.isoline.size() == 0.f)
    return mesh_point{};

  for (auto &curve : app.isoline) {
    auto curve_pos = closed_curve_positions(
        curve, app.mesh.triangles, app.mesh.positions, app.mesh.adjacencies);

    for (auto i = 0; i < curve_pos.size(); ++i) {
      float dist = length(curr_pos - curve_pos[i]);
      if (dist < threshold && dist < min_dist) {
        closest_point =
            get_closed_curve_point(app.mesh.triangles, app.mesh.positions,
                                   app.mesh.adjacencies, curve, i);

        min_dist = dist;
      }
    }
  }
  return closest_point;
}
inline bool triangle_drawn_by_draggin(const App &app) {
  if (app.type_of_triangle == Scalene || app.type_of_triangle == IsosceleBis)
    return true;

  return false;
}
inline bool height_and_base_ratio_can_be_edited(const App &app) {
  if (app.selected_circle->primitive == rett ||
      app.selected_circle->primitive == parallelog)
    return true;

  return false;
}
inline bool levels_can_be_edited(const App &app) {
  if (app.selected_circle == nullptr)
    return false;
  if (app.selected_circle_entries.size() != 1)
    return false;
  if (app.selected_circle->primitive == cir &&
      app.selected_circle_entries[0] + app.selected_circle->levels ==
          app.added_paths.size())
    return true;
  if (app.selected_circle_entries[0] + app.selected_circle->levels ==
      app.added_paths.size() - 1)
    return true;

  return false;
}
inline bool flower_can_be_edited(const App &app) {
  if (app.selected_circle == nullptr)
    return false;
  if (app.selected_circle_entries.size() != 1)
    return false;
  if (app.selected_circle->primitive == flower &&
      app.selected_circle->with_inner_circle &&
      app.selected_circle_entries[0] + app.selected_circle->petals + 2 ==
          app.added_paths.size())
    return true;
  if (app.selected_circle->primitive == flower &&
      !app.selected_circle->with_inner_circle &&
      app.selected_circle_entries[0] + app.selected_circle->petals + 1 ==
          app.added_paths.size())
    return true;

  return false;
}
inline bool affine_transofrmation(const App &app) {
  if (app.input().rotating || app.input().translating || app.input().scaling)
    return true;

  return false;
}
inline string curr_primitive_name(const App &app) {
  if (app.selected_circle == nullptr)
    return "None Selected";
  auto curr = app.selected_circle->primitive;
  if (curr == primitives::tri)
    return "Triangle";
  else if (curr == square)
    return "Square";
  else if (curr == primitives::rett)
    return "Rectangle";
  else if (curr == primitives::rhomb)
    return "Rhombus";
  else if (curr == primitives::pent)
    return "Pentagon";
  else if (curr == primitives::hex)
    return "Hexagon";
  else if (curr == primitives::cir)
    return "Circle";
  else if (curr == primitives::parallelog)
    return "Parallelogram";
  else if (curr == primitives::spider)
    return "Spider Net";
  else if (curr == primitives::crux)
    return "Cross";
  else if (curr == primitives::flower)
    return "Flower";

  return "None Selected";
}
int find_circle_from_shape(const App_base &app, const int entry) {
  for (auto i = 0; i < app.scc.circles.size(); ++i) {
    auto entries = app.scc.circles_to_shape.at(i);
    if (entries[0] == entry)
      return i;
  }
  return -1;
}
vec3f vector_bary_coords(const vector<vec3i> &triangles,
                         const vector<vec3f> &positions, const int tid,
                         const vec3f &v) {
  auto A = positions[triangles[tid].x];
  auto B = positions[triangles[tid].y];
  auto C = positions[triangles[tid].z];

  auto p = A + v;
  auto bary = tri_bary_coords(A, B, C, p);
  return bary;
}

vec2f from_3d_to_2d_vector(const vector<vec3i> &triangles,
                           const vector<vec3f> &positions, const vec3f &v,
                           const int tid, const unfold_triangle &flat_tid) {
  auto baryv = vector_bary_coords(triangles, positions, tid, v);

  return baryv.y * (flat_tid[1] - flat_tid[0]) +
         baryv.z * (flat_tid[2] - flat_tid[0]);
}

vec2f direction_in_tangent_space(const bezier_mesh &mesh, const int &tid,
                                 const vec3f &p, const vec3f &dir) {
  auto n = tid_normal(mesh.triangles, mesh.positions, tid);
  auto d = normalize(dir);

  auto t = mesh.triangles[tid];
  auto y_axis = normalize(mesh.positions[t.y] - mesh.positions[t.x]);
  auto x_axis = cross(y_axis, n);

  auto y = clamp(dot(y_axis, d), -1.0F, 1.0F);
  if (y < -1 || y > 1) {
    error("direction acos()");
  }
  auto x = std::sin(std::acos(y));

  return vec2f{dot(x_axis, d) < 0 ? -x : x, y};
}

vec2f direction_in_tangent_space(const bezier_mesh &mesh, const mesh_point &p,
                                 const vec3f &dir) {
  return direction_in_tangent_space(
      mesh, p.face, eval_position(mesh.triangles, mesh.positions, p), dir);
}

vector<Added_Path *> add_circle_shape(App &app, const Circle &circle) {
  auto result = vector<Added_Path *>(circle.isoline.size());

  for (auto i = 0; i < circle.isoline.size(); ++i) {
    auto curr =
        closed_curve_positions(circle.isoline[i], app.mesh.triangles,
                               app.mesh.positions, app.mesh.adjacencies);
    result[i] =
        add_path_shape(app, curr, app.curve_size * 0.0001, app.curr_color);
  }
  return result;
}
void update_circle_shape(App &app, const Circle *circle, vector<int> &entries) {
  if (entries.size() == circle->isoline.size()) {
    for (auto i = 0; i < circle->isoline.size(); ++i) {

      auto curr =
          closed_curve_positions(circle->isoline[i], app.mesh.triangles,
                                 app.mesh.positions, app.mesh.adjacencies);
      update_path_shape(app.added_paths, app.mesh, curr,
                        app.curve_size * 0.0001, entries[i]);
    }
  } else if (entries.size() < circle->isoline.size()) {
    auto new_entries = entries;
    for (auto i = 0; i < circle->isoline.size(); ++i) {
      if (i < entries.size()) {
        auto curr =
            closed_curve_positions(circle->isoline[i], app.mesh.triangles,
                                   app.mesh.positions, app.mesh.adjacencies);
        update_path_shape(app.added_paths, app.mesh, curr,
                          app.curve_size * 0.0001, entries[i]);
      } else {
        auto curr =
            closed_curve_positions(circle->isoline[i], app.mesh.triangles,
                                   app.mesh.positions, app.mesh.adjacencies);
        new_entries.push_back((int)app.added_paths.size());
        add_path_shape(app, curr, app.curve_size * 0.0001, app.curr_color);
      }
    }
    entries = new_entries;
  } else {
    auto new_entries = entries;
    for (auto i = 0; i < entries.size(); ++i) {
      if (i < circle->isoline.size()) {
        auto curr =
            closed_curve_positions(circle->isoline[i], app.mesh.triangles,
                                   app.mesh.positions, app.mesh.adjacencies);
        update_path_shape(app.added_paths, app.mesh, curr,
                          app.curve_size * 0.0001, entries[i]);
      } else {
        new_entries.erase(new_entries.begin() + i);
        clear_shape(app.added_paths[entries[i]]->instance->shape);
        app.added_paths.erase(app.added_paths.begin() + i);
      }
    }
    entries = new_entries;
  }
}
void update_circle_shape(App &app, const Circle *circle, const int entry) {

  if (circle->isoline.size() == 1) {
    auto curr =
        closed_curve_positions(circle->isoline[0], app.mesh.triangles,
                               app.mesh.positions, app.mesh.adjacencies);
    update_path_shape(app.added_paths, app.mesh, curr, app.curve_size * 0.0001,
                      entry);
  }
}

std::pair<vector<Added_Path *>, vector<int>>
add_isoline_shape(App &app, Isoline &isoline) {
  auto result = vector<Added_Path *>(isoline.size());
  auto entries = vector<int>(isoline.size());
  for (auto i = 0; i < isoline.size(); ++i) {
    auto curr =
        closed_curve_positions(isoline[i], app.mesh.triangles,
                               app.mesh.positions, app.mesh.adjacencies);
    entries[i] = (int)app.added_paths.size();
    result[i] =
        add_path_shape(app, curr, app.curve_size * 0.00008, app.curr_color);
  }
  return {result, entries};
}
void update_isoline_shape(App &app, const Isoline &isoline,
                          vector<int> &entries) {
  if (entries.size() == isoline.size()) {
    for (auto i = 0; i < isoline.size(); ++i) {
      auto curr =
          closed_curve_positions(isoline[i], app.mesh.triangles,
                                 app.mesh.positions, app.mesh.adjacencies);
      update_path_shape(app.added_paths, app.mesh, curr,
                        app.curve_size * 0.0001, entries[i]);
    }
  } else if (entries.size() < isoline.size()) {
    auto new_entries = entries;
    for (auto i = 0; i < isoline.size(); ++i) {
      if (i < entries.size()) {
        auto curr =
            closed_curve_positions(isoline[i], app.mesh.triangles,
                                   app.mesh.positions, app.mesh.adjacencies);
        update_path_shape(app.added_paths, app.mesh, curr,
                          app.curve_size * 0.0001, entries[i]);
      } else {
        auto curr =
            closed_curve_positions(isoline[i], app.mesh.triangles,
                                   app.mesh.positions, app.mesh.adjacencies);
        new_entries.push_back((int)app.added_paths.size());
        add_path_shape(app, curr, app.curve_size * 0.0001, app.curr_color);
      }
    }
    entries = new_entries;
  } else {
    auto erased_entries = vector<int>((int)(entries.size() - isoline.size()));
    for (auto i = 0; i < entries.size(); ++i) {
      if (i < isoline.size()) {
        auto curr =
            closed_curve_positions(isoline[i], app.mesh.triangles,
                                   app.mesh.positions, app.mesh.adjacencies);
        update_path_shape(app.added_paths, app.mesh, curr,
                          app.curve_size * 0.0001, entries[i]);
      } else {
        erased_entries[i - isoline.size()] = i;
      }
    }
    entries.erase(entries.begin() + erased_entries[0], entries.end());
    for (auto j = 0; j < erased_entries.size(); ++j) {
      clear_shape(app.added_paths[erased_entries[j]]->instance->shape);
    }
    app.added_paths.erase(app.added_paths.begin() + erased_entries[0],
                          app.added_paths.begin() + erased_entries.back() + 1);
  }
}

Added_Path *add_triangle_shape(App &app, const vector<mesh_point> &vertices) {
  auto pos = vector<vec3f>{};
  if (validate_points({vertices[0], vertices[1], vertices[2]})) {
    for (auto i = 0; i < 3; ++i) {
      auto curr_pos = path_positions(
          compute_geodesic_path(app.mesh, vertices[i], vertices[(i + 1) % 3]),
          app.mesh);
      pos.insert(pos.end(), curr_pos.begin(), curr_pos.end());
    }
    return add_path_shape(app, pos, app.curve_size * 0.0001, app.curr_color);
  }
  return nullptr;
}
void update_triangle_shape(App &app, const vector<mesh_point> &vertices) {
  auto pos = vector<vec3f>{};
  if (validate_points({vertices[0], vertices[1], vertices[2]})) {
    for (auto i = 0; i < 3; ++i) {
      auto curr_pos = path_positions(
          compute_geodesic_path(app.mesh, vertices[i], vertices[(i + 1) % 3]),
          app.mesh);
      pos.insert(pos.end(), curr_pos.begin(), curr_pos.end());
    }
    update_path_shape(app.scc.result_shape->instance->shape, app.mesh, pos,
                      app.curve_size * 0.0001);
  }
}
void update_triangle_shape(App &app, const vector<mesh_point> &vertices,
                           const int entry) {
  auto pos = vector<vec3f>{};
  if (validate_points({vertices[0], vertices[1], vertices[2]})) {
    for (auto i = 0; i < 3; ++i) {
      auto curr_pos = path_positions(
          compute_geodesic_path(app.mesh, vertices[i], vertices[(i + 1) % 3]),
          app.mesh);
      pos.insert(pos.end(), curr_pos.begin(), curr_pos.end());
    }
    update_path_shape(app.added_paths, app.mesh, pos, app.curve_size * 0.0001,
                      entry);
  }
}
Added_Path *add_quadrilateral_shape(App &app,
                                    const vector<mesh_point> &vertices) {
  auto pos = vector<vec3f>{};
  if (validate_points({vertices[0], vertices[1], vertices[2], vertices[3]})) {
    for (auto i = 0; i < 4; ++i) {
      auto curr_pos = path_positions(
          compute_geodesic_path(app.mesh, vertices[i], vertices[(i + 1) % 4]),
          app.mesh);
      pos.insert(pos.end(), curr_pos.begin(), curr_pos.end());
    }
    return add_path_shape(app, pos, app.curve_size * 0.0001, app.curr_color);
  }
  return nullptr;
}
void update_quadrilateral_shape(App &app, const vector<mesh_point> &vertices) {
  auto pos = vector<vec3f>{};
  if (validate_points({vertices[0], vertices[1], vertices[2], vertices[3]})) {
    for (auto i = 0; i < 4; ++i) {
      auto curr_pos = path_positions(
          compute_geodesic_path(app.mesh, vertices[i], vertices[(i + 1) % 4]),
          app.mesh);
      pos.insert(pos.end(), curr_pos.begin(), curr_pos.end());
    }
    update_path_shape(app.scc.result_shape->instance->shape, app.mesh, pos,
                      app.curve_size * 0.0001);
  }
}
void update_quadrilateral_shape(App &app, const vector<mesh_point> &vertices,
                                const int entry) {
  auto pos = vector<vec3f>{};
  if (validate_points({vertices[0], vertices[1], vertices[2], vertices[3]})) {
    for (auto i = 0; i < 4; ++i) {
      auto curr_pos = path_positions(
          compute_geodesic_path(app.mesh, vertices[i], vertices[(i + 1) % 4]),
          app.mesh);
      pos.insert(pos.end(), curr_pos.begin(), curr_pos.end());
    }
    update_path_shape(app.added_paths, app.mesh, pos, app.curve_size * 0.0001,
                      entry);
  }
}
Added_Path *add_pentagonal_shape(App &app, const vector<mesh_point> &vertices) {
  auto pos = vector<vec3f>{};
  if (validate_points(
          {vertices[0], vertices[1], vertices[2], vertices[3], vertices[4]})) {
    for (auto i = 0; i < 5; ++i) {
      auto curr_pos = path_positions(
          compute_geodesic_path(app.mesh, vertices[i], vertices[(i + 1) % 5]),
          app.mesh);
      pos.insert(pos.end(), curr_pos.begin(), curr_pos.end());
    }
    return add_path_shape(app, pos, app.curve_size * 0.0001, app.curr_color);
  }
  return nullptr;
}
void update_pentagonal_shape(App &app, const vector<mesh_point> &vertices,
                             const int entry) {
  auto pos = vector<vec3f>{};
  if (validate_points(
          {vertices[0], vertices[1], vertices[2], vertices[3], vertices[4]})) {
    for (auto i = 0; i < 5; ++i) {
      auto curr_pos = path_positions(
          compute_geodesic_path(app.mesh, vertices[i], vertices[(i + 1) % 5]),
          app.mesh);
      pos.insert(pos.end(), curr_pos.begin(), curr_pos.end());
    }
    update_path_shape(app.added_paths, app.mesh, pos, app.curve_size * 0.0001,
                      entry);
  }
}
Added_Path *add_hexagonal_shape(App &app, const vector<mesh_point> &vertices) {
  auto pos = vector<vec3f>{};
  if (validate_points({vertices[0], vertices[1], vertices[2], vertices[3],
                       vertices[4], vertices[5]})) {
    for (auto i = 0; i < 6; ++i) {
      auto curr_pos = path_positions(
          compute_geodesic_path(app.mesh, vertices[i], vertices[(i + 1) % 6]),
          app.mesh);
      pos.insert(pos.end(), curr_pos.begin(), curr_pos.end());
    }
    return add_path_shape(app, pos, app.curve_size * 0.0001, app.curr_color);
  }
  return nullptr;
}

void update_hexagonal_shape(App &app, const vector<mesh_point> &vertices,
                            const int entry) {
  auto pos = vector<vec3f>{};
  if (validate_points({vertices[0], vertices[1], vertices[2], vertices[3],
                       vertices[4], vertices[5]})) {
    for (auto i = 0; i < 6; ++i) {
      auto curr_pos = path_positions(
          compute_geodesic_path(app.mesh, vertices[i], vertices[(i + 1) % 6]),
          app.mesh);
      pos.insert(pos.end(), curr_pos.begin(), curr_pos.end());
    }
    update_path_shape(app.added_paths, app.mesh, pos, app.curve_size * 0.0001,
                      entry);
  }
}
Added_Path *add_octagonal_shape(App &app, const vector<mesh_point> &vertices) {
  auto pos = vector<vec3f>{};
  if (validate_points(vertices)) {
    for (auto i = 0; i < 8; ++i) {
      auto curr_pos = path_positions(
          compute_geodesic_path(app.mesh, vertices[i], vertices[(i + 1) % 8]),
          app.mesh);
      pos.insert(pos.end(), curr_pos.begin(), curr_pos.end());
    }
    return add_path_shape(app, pos, app.curve_size * 0.0001, app.curr_color);
  }
  return nullptr;
}
void update_octagonal_shape(App &app, const vector<mesh_point> &vertices,
                            const int entry) {
  auto pos = vector<vec3f>{};
  if (validate_points(vertices)) {
    for (auto i = 0; i < 8; ++i) {
      auto curr_pos = path_positions(
          compute_geodesic_path(app.mesh, vertices[i], vertices[(i + 1) % 8]),
          app.mesh);
      pos.insert(pos.end(), curr_pos.begin(), curr_pos.end());
    }
    update_path_shape(app.added_paths, app.mesh, pos, app.curve_size * 0.0001,
                      entry);
  }
}
Added_Path *add_decagonal_shape(App &app, const vector<mesh_point> &vertices) {
  auto pos = vector<vec3f>{};
  if (validate_points(vertices)) {
    for (auto i = 0; i < 10; ++i) {
      auto curr_pos = path_positions(
          compute_geodesic_path(app.mesh, vertices[i], vertices[(i + 1) % 10]),
          app.mesh);
      pos.insert(pos.end(), curr_pos.begin(), curr_pos.end());
    }
    return add_path_shape(app, pos, app.curve_size * 0.0001, app.curr_color);
  }
  return nullptr;
}
void update_decagonal_shape(App &app, const vector<mesh_point> &vertices,
                            const int entry) {
  auto pos = vector<vec3f>{};
  if (validate_points(vertices)) {
    for (auto i = 0; i < 10; ++i) {
      auto curr_pos = path_positions(
          compute_geodesic_path(app.mesh, vertices[i], vertices[(i + 1) % 10]),
          app.mesh);
      pos.insert(pos.end(), curr_pos.begin(), curr_pos.end());
    }
    update_path_shape(app.added_paths, app.mesh, pos, app.curve_size * 0.0001,
                      entry);
  }
}
void add_generic_shape(App_base &app) {
  auto entry = app.selected_circle_entries[0];
  if (app.selected_circle->primitive == tri) {
    auto vertices = equilateral_triangle_tangent_space(
        app.mesh, app.selected_circle, app.selected_circle->theta);
    add_triangle_shape(app, vertices);
  } else if (app.selected_circle->primitive == square) {
    auto vertices = rhombus_tangent_space(app.mesh, app.selected_circle, 1.f,
                                          app.selected_circle->theta);
    add_quadrilateral_shape(app, vertices);
  } else if (app.selected_circle->primitive == rett) {
    auto vertices =
        parallelogram_tangent_space(app.mesh, app.selected_circle, app.sigma,
                                    1.f, app.selected_circle->theta);
    add_quadrilateral_shape(app, vertices);
  } else if (app.selected_circle->primitive == rhomb) {
    auto vertices = rhombus_tangent_space(
        app.mesh, app.selected_circle, app.lambda, app.selected_circle->theta);
    add_quadrilateral_shape(app, vertices);
  } else if (app.selected_circle->primitive == parallelog) {
    auto vertices =
        parallelogram_tangent_space(app.mesh, app.selected_circle, app.sigma,
                                    app.lambda, app.selected_circle->theta);
    add_quadrilateral_shape(app, vertices);
  } else if (app.selected_circle->primitive == pent) {
    auto vertices = pentagon_tangent_space(app.mesh, app.selected_circle,
                                           app.selected_circle->theta);
    add_pentagonal_shape(app, vertices);
  } else if (app.selected_circle->primitive == primitives::hex) {
    auto vertices = hexagon_tangent_space(app.mesh, app.selected_circle,
                                          app.selected_circle->theta);
    add_hexagonal_shape(app, vertices);
  } else if (app.selected_circle->primitive == primitives::oct) {
    auto vertices = octagon_tangent_space(app.mesh, app.selected_circle);
    add_octagonal_shape(app, vertices);
  } else if (app.selected_circle->primitive == primitives::deca) {
    auto vertices = decagon_tangent_space(app.mesh, app.selected_circle);
    add_decagonal_shape(app, vertices);
  }
}
void add_generic_shape(App_base &app, const vector<mesh_point> &vertices) {
  if (vertices.size() == 3)
    add_triangle_shape(app, vertices);
  else if (vertices.size() == 4)
    add_quadrilateral_shape(app, vertices);
  else if (vertices.size() == 5)
    add_pentagonal_shape(app, vertices);
  else if (vertices.size() == 6)
    add_hexagonal_shape(app, vertices);
  else if (vertices.size() == 8)
    add_octagonal_shape(app, vertices);
  else if (vertices.size() == 10)
    add_decagonal_shape(app, vertices);
}
void update_generic_shape(App_base &app, const vector<mesh_point> &vertices,
                          const int entry) {
  if (vertices.size() == 3)
    update_triangle_shape(app, vertices, entry);
  else if (vertices.size() == 4)
    update_quadrilateral_shape(app, vertices, entry);
  else if (vertices.size() == 5)
    update_pentagonal_shape(app, vertices, entry);
  else if (vertices.size() == 6)
    update_hexagonal_shape(app, vertices, entry);
  else if (vertices.size() == 8)
    update_octagonal_shape(app, vertices, entry);
  else if (vertices.size() == 10)
    update_decagonal_shape(app, vertices, entry);
}
void update_generic_shape(App_base &app) {
  auto entry = app.selected_circle_entries[0];
  if (app.selected_circle->primitive == tri) {
    auto vertices = equilateral_triangle_tangent_space(
        app.mesh, app.selected_circle, app.selected_circle->theta);
    update_triangle_shape(app, vertices, entry + 1);
  } else if (app.selected_circle->primitive == square) {
    auto vertices = rhombus_tangent_space(app.mesh, app.selected_circle, 1.f,
                                          app.selected_circle->theta);
    update_quadrilateral_shape(app, vertices, entry + 1);
  } else if (app.selected_circle->primitive == rett) {
    auto vertices =
        parallelogram_tangent_space(app.mesh, app.selected_circle, app.sigma,
                                    1.f, app.selected_circle->theta);
    update_quadrilateral_shape(app, vertices, entry + 1);
  } else if (app.selected_circle->primitive == rhomb) {
    auto vertices = rhombus_tangent_space(
        app.mesh, app.selected_circle, app.lambda, app.selected_circle->theta);
    update_quadrilateral_shape(app, vertices, entry + 1);
  } else if (app.selected_circle->primitive == parallelog) {
    auto vertices =
        parallelogram_tangent_space(app.mesh, app.selected_circle, app.sigma,
                                    app.lambda, app.selected_circle->theta);
    update_quadrilateral_shape(app, vertices, entry + 1);
  } else if (app.selected_circle->primitive == pent) {
    auto vertices = pentagon_tangent_space(app.mesh, app.selected_circle,
                                           app.selected_circle->theta);
    update_pentagonal_shape(app, vertices, entry + 1);
  } else if (app.selected_circle->primitive == primitives::hex) {
    auto vertices = hexagon_tangent_space(app.mesh, app.selected_circle,
                                          app.selected_circle->theta);
    update_hexagonal_shape(app, vertices, entry + 1);
  } else if (app.selected_circle->primitive == primitives::oct) {
    auto vertices = octagon_tangent_space(app.mesh, app.selected_circle);
    update_octagonal_shape(app, vertices, entry + 1);
  } else if (app.selected_circle->primitive == primitives::deca) {
    auto vertices = decagon_tangent_space(app.mesh, app.selected_circle);
    update_decagonal_shape(app, vertices, entry + 1);
  }
}
std::pair<Added_Path *, int>
add_result_shape(App &app, const vector<mesh_point> &vertices) {
  auto entry = (int)app.added_paths.size();
  auto result = new Added_Path;
  switch ((int)vertices.size()) {
  case 3:
    result = add_triangle_shape(app, vertices);
    break;
  case 4:
    result = add_quadrilateral_shape(app, vertices);
  default:
    break;
  }

  return {result, entry};
}
vector<Added_Path *> add_garland_shape(App_base &app,
                                       const vector<mesh_point> &vertices) {
  auto garland = make_garland(app.mesh, app.selected_circle, vertices);
  auto result = vector<Added_Path *>{};
  if (app.selected_circle->garland_vertices == 3) {
    result.resize(20);
    for (auto i = 0; i < 20; ++i) {
      result[i] = add_triangle_shape(app, garland[i]);
    }
  } else if (app.selected_circle->garland_vertices == 4) {
    result.resize(5);
    for (auto i = 0; i < 5; ++i) {
      result[i] = add_quadrilateral_shape(app, garland[i]);
    }
  } else if (app.selected_circle->garland_vertices == 5) {
    result.resize(5);
    for (auto i = 0; i < 6; ++i) {
      result[i] = add_pentagonal_shape(app, garland[i]);
    }
  } else if (app.selected_circle->garland_vertices == 6) {
    result.resize(4);
    for (auto i = 0; i < 4; ++i) {
      result[i] = add_hexagonal_shape(app, garland[i]);
    }
  }

  return result;
}
void update_garland_shape(App_base &app,
                          const vector<vector<mesh_point>> &vertices) {
  auto entry = app.selected_circle_entries[0] + 1;
  if (app.selected_circle->garland_vertices == 3) {
    for (auto i = 0; i < 20; ++i) {
      update_triangle_shape(app, vertices[i], entry + i);
    }
  } else if (app.selected_circle->garland_vertices == 4) {
    for (auto i = 0; i < 5; ++i) {
      update_quadrilateral_shape(app, vertices[i], entry + i);
    }
  } else if (app.selected_circle->garland_vertices == 5) {
    for (auto i = 0; i < 6; ++i) {
      update_pentagonal_shape(app, vertices[i], entry + i);
    }
  } else if (app.selected_circle->garland_vertices == 6) {
    for (auto i = 0; i < 4; ++i) {
      update_hexagonal_shape(app, vertices[i], entry + i);
    }
  }
}
void add_concentric_shape(App_base &app, const vector<mesh_point> &vertices) {
  if (vertices.size() == 0) {
    auto concentric_circles =
        make_concentric_circles(app.mesh, app.selected_circle);
    for (auto i = 0; i < concentric_circles.size(); ++i) {
      add_circle_shape(app, concentric_circles[i]);
    }
  } else {
    auto concentric_polygons = make_concentric_n_gon(
        app.mesh, app.selected_circle, app.sigma, app.lambda, vertices);
    for (auto i = 0; i < concentric_polygons.size(); ++i) {
      add_generic_shape(app, concentric_polygons[i]);
    }
  }
}
void update_concentric_circles(App_base &app,
                               const vector<Circle> &concentric_circles) {
  auto levels = app.selected_circle->levels;
  if (levels != concentric_circles.size()) {
    std::cerr << "Error: levels and number of circles are different"
              << std::endl;
  }
  auto first_entry = app.selected_circle_entries[0];
  for (auto i = 0; i < levels; ++i) {
    update_circle_shape(app, &concentric_circles[i], first_entry + i);
  }
}
void update_concentric_polygons(
    App_base &app, const vector<vector<mesh_point>> &concentric_polygons) {
  auto levels = app.selected_circle->levels;
  if (levels != concentric_polygons.size()) {
    std::cerr << "Error: levels and number of polygons are different"
              << std::endl;
  }
  auto first_entry = app.selected_circle_entries[0] + 1;
  for (auto i = 0; i < levels; ++i) {
    update_generic_shape(app, concentric_polygons[i], first_entry + i);
  }
}
void add_spider_net_shape(App &app, const vector<vector<mesh_point>> &vertices,
                          const vector<vector<vec3f>> &arcs,
                          const bool just_vertices) {
  auto center = app.selected_circle->center;
  for (auto i = 0; i < 8; ++i) {
    auto path = compute_geodesic_path(app.mesh, center, vertices[0][i]);
    add_path_shape(app, path, app.curve_size * 0.0001, app.curr_color);
  }
  if (!just_vertices) {
    for (auto i = 0; i < 24; ++i) {
      add_path_shape(app, arcs[i], app.curve_size * 0.0001, app.curr_color);
    }
  }
}
void update_spider_net_shape(App &app,
                             const vector<vector<mesh_point>> &vertices,
                             const vector<vector<vec3f>> &arcs,
                             const bool just_vertices) {
  auto entry = app.selected_circle_entries[0] + 1;
  auto center = app.selected_circle->center;
  for (auto i = 0; i < 8; ++i) {
    auto path = compute_geodesic_path(app.mesh, center, vertices[0][i]);
    update_path_shape(app.added_paths, app.mesh, path_positions(path, app.mesh),
                      app.curve_size * 0.0001, entry + i);
  }
  if (!just_vertices) {
    for (auto i = 0; i < 24; ++i) {
      if (arcs[i].size() > 0)
        update_path_shape(app.added_paths, app.mesh, arcs[i],
                          app.curve_size * 0.0001, entry + 8 + i);
    }
  }
}
void add_cross_shape(App &app, const vector<vector<vec3f>> &arcs) {
  for (auto i = 0; i < arcs.size(); ++i) {
    if (arcs[i].size() > 0)
      add_path_shape(app, arcs[i], app.curve_size * 0.0001, app.curr_color);
  }
}
void update_cross_shape(App &app, const vector<vector<vec3f>> &arcs,
                        const bool no_arcs) {
  auto entry = app.selected_circle_entries[0] + 1;
  if (no_arcs) {
    for (auto i = 0; i < 4; ++i) {
      update_path_shape(app.added_paths, app.mesh, arcs[i],
                        app.curve_size * 0.0001, entry + i);
    }
  } else {
    for (auto i = 0; i < 12; ++i) {
      if (arcs.size() > 0)
        update_path_shape(app.added_paths, app.mesh, arcs[i],
                          app.curve_size * 0.0001, entry + i);
    }
  }
}
void add_flower_shape(App &app, const vector<vector<vec3f>> &arcs) {
  for (auto i = 0; i < arcs.size(); ++i) {
    if (arcs[i].size() > 0)
      add_path_shape(app, arcs[i], app.curve_size * 0.0001, app.curr_color);
  }
}
void update_flower_shape(App &app, const vector<vector<vec3f>> &arcs,
                         const bool no_arcs, const bool with_inner_circle) {
  auto entry = app.selected_circle_entries[0] + 1;
  auto n = -1;
  if (no_arcs || !with_inner_circle)
    n = app.selected_circle->petals;
  else
    n = app.selected_circle->petals + 1;

  for (auto i = 0; i < n; ++i) {
    if (arcs.size() > 0)
      update_path_shape(app.added_paths, app.mesh, arcs[i],
                        app.curve_size * 0.0001, entry + i);
  }
}
void clear_scc(App &app) {
  clear(app.scc);
  app.isoline.clear();
  app.curr_path = app.curr_circle = 0;
  if (app.added_paths.size() != 0) {
    for (auto &path : app.added_paths) {
      clear_shape(path->instance->shape);
    }
    app.added_paths.clear();
  }
  if (app.added_points.size() != 0) {
    for (auto &points : app.added_points) {
      clear_shape(points->instance->shape);
    }

    app.added_points.clear();
  }
}
void draw_scc(App &app) {
  auto &scc = app.scc;
  if (app.input().cropping && app.selected_circle != nullptr) {
    if (scc.cropping_range.size() == 2 &&
        app.selected_circle_entries.size() == 1) {
      app.selected_circle->crop_range =
          cropping_range(app.mesh, *app.selected_circle, scc.cropping_range);
      auto &circle_shape = app.added_paths.at(app.selected_circle_entries[0]);

      auto pos = (app.reverse_cropping)
                     ? crop_circle(circle_shape->positions,
                                   {app.selected_circle->crop_range.y,
                                    app.selected_circle->crop_range.x})
                     : crop_circle(circle_shape->positions,
                                   app.selected_circle->crop_range);
      update_path_shape(app.added_paths, app.mesh, pos, app.curve_size * 0.0001,
                        app.selected_circle_entries[0]);
      scc.cropping_range.clear();
      app.input().cropping = false;
      app.selected_circle = nullptr;
      scc.points.pop_back();
      scc.points.pop_back();
      app.isoline.clear();
    }
  } else if (affine_transofrmation(app) && app.selected_circle != nullptr &&
             app.selected_circle_entries.size() == 1) {
    auto entry = app.selected_circle_entries[0] + 1;
    if (app.selected_circle->primitive == cir) {
      if (app.selected_circle->levels == 1)
        update_circle_shape(app, app.selected_circle,
                            app.selected_circle_entries);
      else {
        auto circles = make_concentric_circles(app.mesh, app.selected_circle);
        update_concentric_circles(app, circles);
      }

    } else if (app.selected_circle->primitive == tri) {
      auto vertices = equilateral_triangle_tangent_space(
          app.mesh, app.selected_circle, app.selected_circle->theta);
      app.selected_circle->vertices = vertices;
      if (app.selected_circle->levels == 1)
        update_triangle_shape(app, vertices, entry);
      else {
        auto polygons = make_concentric_n_gon(app.mesh, app.selected_circle,
                                              app.sigma, app.lambda, vertices);
        update_concentric_polygons(app, polygons);
      }

    } else if (app.selected_circle->primitive == square) {
      auto vertices = rhombus_tangent_space(app.mesh, app.selected_circle, 1.f,
                                            app.selected_circle->theta);
      app.selected_circle->vertices = vertices;
      if (app.selected_circle->levels == 1)
        update_quadrilateral_shape(app, vertices, entry);
      else {
        auto polygons = make_concentric_n_gon(app.mesh, app.selected_circle,
                                              app.sigma, app.lambda, vertices);
        update_concentric_polygons(app, polygons);
      }
    } else if (app.selected_circle->primitive == rhomb) {
      auto vertices =
          rhombus_tangent_space(app.mesh, app.selected_circle, app.lambda,
                                app.selected_circle->theta);
      app.selected_circle->vertices = vertices;
      if (app.selected_circle->levels == 1)
        update_quadrilateral_shape(app, vertices, entry);
      else {
        auto polygons = make_concentric_n_gon(app.mesh, app.selected_circle,
                                              app.sigma, app.lambda, vertices);
        update_concentric_polygons(app, polygons);
      }
    } else if (app.selected_circle->primitive == rett) {
      auto vertices =
          parallelogram_tangent_space(app.mesh, app.selected_circle, app.sigma,
                                      1.f, app.selected_circle->theta);
      app.selected_circle->vertices = vertices;
      if (app.selected_circle->levels == 1)
        update_quadrilateral_shape(app, vertices, entry);
      else {
        auto polygons = make_concentric_n_gon(app.mesh, app.selected_circle,
                                              app.sigma, app.lambda, vertices);
        update_concentric_polygons(app, polygons);
      }
    } else if (app.selected_circle->primitive == parallelog) {
      auto vertices =
          parallelogram_tangent_space(app.mesh, app.selected_circle, app.sigma,
                                      app.lambda, app.selected_circle->theta);
      app.selected_circle->vertices = vertices;
      if (app.selected_circle->levels == 1)
        update_quadrilateral_shape(app, vertices, entry);
      else {
        auto polygons = make_concentric_n_gon(app.mesh, app.selected_circle,
                                              app.sigma, app.lambda, vertices);
        update_concentric_polygons(app, polygons);
      }
    } else if (app.selected_circle->primitive == pent) {
      auto vertices = pentagon_tangent_space(app.mesh, app.selected_circle,
                                             app.selected_circle->theta);
      app.selected_circle->vertices = vertices;
      if (app.selected_circle->levels == 1)
        update_pentagonal_shape(app, vertices, entry);
      else {
        auto polygons = make_concentric_n_gon(app.mesh, app.selected_circle,
                                              app.sigma, app.lambda, vertices);
        update_concentric_polygons(app, polygons);
      }
    } else if (app.selected_circle->primitive == primitives::hex) {
      auto vertices = hexagon_tangent_space(app.mesh, app.selected_circle,
                                            app.selected_circle->theta);
      app.selected_circle->vertices = vertices;
      if (app.selected_circle->levels == 1)
        update_hexagonal_shape(app, vertices, entry);
      else {
        auto polygons = make_concentric_n_gon(app.mesh, app.selected_circle,
                                              app.sigma, app.lambda, vertices);
        update_concentric_polygons(app, polygons);
      }
    } else if (app.selected_circle->primitive == primitives::oct) {
      auto vertices = octagon_tangent_space(app.mesh, app.selected_circle);
      app.selected_circle->vertices = vertices;
      if (app.selected_circle->levels == 1)
        update_octagonal_shape(app, vertices, entry);
      else {
        auto polygons = make_concentric_n_gon(app.mesh, app.selected_circle,
                                              app.sigma, app.lambda, vertices);
        update_concentric_polygons(app, polygons);
      }
    } else if (app.selected_circle->primitive == primitives::deca) {
      auto vertices = decagon_tangent_space(app.mesh, app.selected_circle);
      app.selected_circle->vertices = vertices;
      if (app.selected_circle->levels == 1)
        update_decagonal_shape(app, vertices, entry);
      else {
        auto polygons = make_concentric_n_gon(app.mesh, app.selected_circle,
                                              app.sigma, app.lambda, vertices);
        update_concentric_polygons(app, polygons);
      }
    } else if (app.selected_circle->primitive == garland_tri) {
      auto vertices = equilateral_triangle_tangent_space(
          app.mesh, app.selected_circle, app.selected_circle->theta);
      scc.circles[app.curr_circle].vertices = vertices;
      auto garland = make_garland(app.mesh, app.selected_circle, vertices);
      update_garland_shape(app, garland);
    } else if (app.selected_circle->primitive == garland_sq) {
      auto vertices = rhombus_tangent_space(app.mesh, app.selected_circle, 1.f,
                                            app.selected_circle->theta);
      scc.circles[app.curr_circle].vertices = vertices;
      auto garland = make_garland(app.mesh, app.selected_circle, vertices);
      update_garland_shape(app, garland);
    } else if (app.selected_circle->primitive == garland_pent) {
      auto vertices = pentagon_tangent_space(app.mesh, app.selected_circle,
                                             app.selected_circle->theta);
      scc.circles[app.curr_circle].vertices = vertices;
      auto garland = make_garland(app.mesh, app.selected_circle, vertices);
      update_garland_shape(app, garland);
    } else if (app.selected_circle->primitive == garland_hex) {
      auto vertices = hexagon_tangent_space(app.mesh, app.selected_circle,
                                            app.selected_circle->theta);
      scc.circles[app.curr_circle].vertices = vertices;
      auto garland = make_garland(app.mesh, app.selected_circle, vertices);
      update_garland_shape(app, garland);
    } else if (app.selected_circle->primitive == spider) {
      auto [vertices, arcs] = spider_net(app.mesh, app.selected_circle, true);
      update_spider_net_shape(app, vertices, arcs, true);
      auto entry = app.selected_circle_entries[0] + 9;
      for (auto i = 0; i < 24; ++i) {
        clear_shape(app.added_paths[entry + i]->instance->shape);
      }
    } else if (app.selected_circle->primitive == crux) {
      auto arcs = make_cross(app.mesh, app.selected_circle, true);
      update_cross_shape(app, arcs, true);
      auto entry = app.selected_circle_entries[0] + 1;
      for (auto i = 4; i < 12; ++i) {
        clear_shape(app.added_paths[entry + i]->instance->shape);
      }
    } else if (app.selected_circle->primitive == flower) {
      auto arcs = make_flower(app.mesh, app.selected_circle,
                              app.selected_circle->petals, true);
      auto entry = app.selected_circle_entries[0] + 1;
      auto n = (app.selected_circle->with_inner_circle)
                   ? app.selected_circle->petals + 1
                   : app.selected_circle->petals;
      for (auto i = 0; i < n; ++i) {
        clear_shape(app.added_paths[entry + i]->instance->shape);
      }
      update_flower_shape(app, arcs, true, false);
    }

  } else if (app.input().duplicating && app.selected_circle != nullptr &&
             app.selected_circle_entries.size() == 1) {
    auto entry = app.selected_circle_entries[0];
    scc.circles[app.curr_circle] = *app.selected_circle;
    scc.circles_to_shape[app.curr_circle] =
        vector<int>{(int)app.added_paths.size()};
    scc.circles[app.curr_circle].primitive = app.selected_circle->primitive;
    scc.circle_shape = add_circle_shape(app, scc.circles[app.curr_circle]);

    if (app.selected_circle->primitive == garland_tri) {
      clear_shape(scc.circle_shape[0]->instance->shape);
      auto vertices = equilateral_triangle_tangent_space(
          app.mesh, app.selected_circle, app.selected_circle->theta);
      add_garland_shape(app, vertices);
    } else if (app.selected_circle->primitive == garland_sq) {
      clear_shape(scc.circle_shape[0]->instance->shape);
      auto vertices = rhombus_tangent_space(app.mesh, app.selected_circle, 1.0,
                                            app.selected_circle->theta);
      add_garland_shape(app, vertices);
    } else if (app.selected_circle->primitive == garland_pent) {
      clear_shape(scc.circle_shape[0]->instance->shape);
      auto vertices = pentagon_tangent_space(app.mesh, app.selected_circle,
                                             app.selected_circle->theta);
      add_garland_shape(app, vertices);
    } else if (app.selected_circle->primitive == garland_hex) {
      clear_shape(scc.circle_shape[0]->instance->shape);
      auto vertices = hexagon_tangent_space(app.mesh, app.selected_circle,
                                            app.selected_circle->theta);
      add_garland_shape(app, vertices);
    } else if (app.selected_circle->primitive == spider) {
      clear_shape(scc.circle_shape[0]->instance->shape);
      auto [vertices, arcs] = spider_net(app.mesh, app.selected_circle, false);
      scc.circles[app.curr_circle].spider_vertices = vertices;
      scc.circles[app.curr_circle].arcs = arcs;
      add_spider_net_shape(app, vertices, arcs, false);
    } else if (app.selected_circle->primitive == crux) {
      clear_shape(scc.circle_shape[0]->instance->shape);
      auto arcs = make_cross(app.mesh, app.selected_circle, false);
      scc.circles[app.curr_circle].arcs = arcs;
      add_cross_shape(app, arcs);

    } else if (app.selected_circle->primitive == flower) {
      clear_shape(scc.circle_shape[0]->instance->shape);
      auto arcs = make_flower(app.mesh, app.selected_circle,
                              app.selected_circle->petals, false,
                              app.scale_factor_inner_circle_radius,
                              app.selected_circle->with_inner_circle);
      scc.circles[app.curr_circle].arcs = arcs;
      scc.circles[app.curr_circle].with_inner_circle =
          app.selected_circle->with_inner_circle;
      add_flower_shape(app, arcs);
    } else {
      if (app.selected_circle->primitive != cir)
        clear_shape(scc.circle_shape[0]->instance->shape);
      if (app.selected_circle->levels == 1)
        add_generic_shape(app);
      else
        add_concentric_shape(app, app.selected_circle->vertices);
    }

    ++app.curr_circle;
    scc.circles.push_back(Circle());
    scc.circle_shape = {};
    app.selected_circle = nullptr;
    app.input().duplicating = false;
  } else if (app.input().deleting && app.curr_circle > 0) {
    auto entries = app.scc.circles_to_shape.at(app.curr_circle - 1);
    for (auto i = entries[0]; i < app.added_paths.size(); ++i) {
      clear_shape(app.added_paths[i]->instance->shape);
    }
    app.added_paths.erase(app.added_paths.begin() + entries[0],
                          app.added_paths.end());
    app.scc.circles.erase(app.scc.circles.begin() + app.curr_circle - 1);
    --app.curr_circle;
    app.input().deleting = false;
    app.selected_circle = nullptr;
  } else if (scc.state == scc_state::GEODESIC &&
             scc.paths[app.curr_path].start.face != -1) {
    if (scc.geodesic_shape == nullptr) {
      scc.geodesic_shape =
          add_path_shape(app, scc.paths[app.curr_path], app.curve_size * 0.0001,
                         app.curr_color);
    } else
      update_path_shape(scc.geodesic_shape->instance->shape, app.mesh,
                        scc.paths[app.curr_path], app.curve_size * 0.0001);

    if (scc.points.size() == 2) {
      add_points_shape(app, scc.points, app.curve_size * 0.0003, zero3f);
      clear_shape(app.added_paths.back()->instance->shape);
      app.added_paths.pop_back();
      if (!app.straighthest) {
        add_path_shape(app, scc.paths[app.curr_path],
                       2 * app.curve_size * 0.0001, app.curr_color);
        auto line =
            straight_line(eval_position(app.mesh.triangles, app.mesh.positions,
                                        scc.points[0]),
                          eval_position(app.mesh.triangles, app.mesh.positions,
                                        scc.points[1]));
        add_path_shape(app, line, 2 * app.curve_size * 0.0001, vec3f{1, 0, 0});
      } else {
        auto gamma = scc.paths[app.curr_path];
        scc.paths[app.curr_path] = straightest_path(
            app.mesh.triangles, app.mesh.positions, app.mesh.adjacencies,
            gamma.end, tangent_path_direction(app.mesh, gamma, false),
            app.length_of_straightest);
        add_path_shape(app, scc.paths[app.curr_path], app.curve_size * 0.0001,
                       app.curr_color);
      }
      scc.geodesic_shape = nullptr;
      scc.paths.push_back(geodesic_path{});
      scc.points.clear();
      app.curr_path += 1;
    }

  } else if (scc.state == scc_state::CIRCLE &&
             app.scc.circles[app.curr_circle].radius > 0) {
    auto circ = &app.scc.circles[app.curr_circle];

    if (scc.circle_shape.size() == 0)
      std::tie(scc.circle_shape, app.selected_circle_entries) =
          add_isoline_shape(app, circ->isoline);
    else
      update_isoline_shape(app, circ->isoline, app.selected_circle_entries);

    if (scc.points.size() == 2) {
      // clear_shape(app.added_points.back()->instance->shape);
      scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
      scc.circles[app.curr_circle].primitive = cir;
      scc.circles[app.curr_circle].levels = 1;
      ++app.curr_circle;
      scc.circles.push_back(Circle());
      scc.circle_shape = {};
      app.isoline.clear();
      scc.points = {};
    }
  } else if (scc.state == scc_state::ELLIPSE &&
             scc.paths[app.curr_path].start.face != -1) {
    if (app.scc.points.size() == 2) {

      if (scc.circle_shape.size() == 0)
        std::tie(scc.circle_shape, app.selected_circle_entries) =
            add_isoline_shape(app, app.isoline);
      else
        update_isoline_shape(app, app.isoline, app.selected_circle_entries);

    } else if (app.scc.points.size() == 3) {
      // clear_shape(app.added_points.back()->instance->shape);
      // app.added_points.pop_back();
      // clear_shape(app.added_points.back()->instance->shape);
      // app.added_points.pop_back();
      app.isoline.clear();
      app.scc.points.clear();
    }
  } else if (scc.state == scc_state::INTERSECT && app.scc.points.size() > 0) {
    add_points_shape(app, app.scc.points, 3 * app.curve_size * 0.0001,
                     vec3f{0, 1, 0});
  } else if (scc.state == scc_state::TANGENT && app.isoline.size() > 0) {
    if (scc.circle_shape.size() == 0)
      std::tie(scc.circle_shape, app.selected_circle_entries) =
          add_isoline_shape(app, app.isoline);
    else
      update_isoline_shape(app, app.isoline, app.selected_circle_entries);
    if (scc.result_pos.size() > 0) {
      if (scc.output_shape.size() == 0)
        scc.output_shape = {add_path_shape(
            app, scc.result_pos, app.curve_size * 0.0001, vec3f{0, 0, 0})};
      else
        update_path_shape(scc.output_shape[0]->instance->shape, app.mesh,
                          scc.result_pos, app.curve_size * 0.0001);
    }

    if (scc.expected_pos.size() > 0) {
      scc.output_shape = {add_path_shape(
          app, scc.expected_pos, app.curve_size * 0.0001, vec3f{0, 1, 0})};
    }
    //    if (scc.circles[app.curr_circle].radius > 0) {
    //      scc.circles_to_shape[app.curr_circle] = app.added_paths.size() -
    //      1; scc.input_shape.push_back(scc.circle_shape); scc.circle_shape =
    //      nullptr;
    //      ++app.curr_circle;
    //      scc.circles.push_back(Circle());
    //      scc.points = {};
    //    }
  } else if (scc.state == scc_state::BISECTOR) {
    if (scc.result_pos.size() > 0) {
      if (scc.output_shape.size() == 0)
        scc.output_shape = {add_path_shape(
            app, scc.result_pos, app.curve_size * 0.0001, vec3f{0, 1, 0})};
      else
        update_path_shape(scc.output_shape[0]->instance->shape, app.mesh,
                          scc.result_pos, app.curve_size * 0.0001);
      add_points_shape(app, {scc.result_pos[0], scc.result_pos.back()},
                       2 * app.curve_size * 0.0001, vec3f{0, 1, 0});
    }
    if (scc.expected_pos.size() > 0) {
      if (scc.output_shape.size() == 0)
        scc.output_shape = {add_path_shape(
            app, scc.expected_pos, app.curve_size * 0.0001, vec3f{0, 1, 0})};
      else
        update_path_shape(scc.output_shape[0]->instance->shape, app.mesh,
                          scc.expected_pos, app.curve_size * 0.0001);

      add_points_shape(app, {scc.expected_pos[0], scc.expected_pos.back()},
                       2 * app.curve_size * 0.0001, vec3f{0, 1, 0});
    }
    if (app.isoline.size() > 0) {
      auto pos = vector<vec3f>{};
      for (auto curve : app.isoline) {
        auto curr =
            closed_curve_positions(curve, app.mesh.triangles,
                                   app.mesh.positions, app.mesh.adjacencies);
        pos.insert(pos.end(), curr.begin(), curr.end());
      }

      if (scc.output_shape.size() == 0)
        scc.output_shape = {
            add_path_shape(app, pos, app.curve_size * 0.0001, vec3f{0, 1, 0})};
      else
        update_path_shape(scc.output_shape[0]->instance->shape, app.mesh, pos,
                          app.curve_size * 0.0001);
    }
  } else if (scc.state == scc_state::PERPENDICULAR) {
    if (scc.points.size() == 1 && app.scc.paths[app.curr_path].end.face != -1) {
      if (scc.geodesic_shape == nullptr) {
        scc.geodesic_shape = add_path_shape(app, scc.paths[app.curr_path],
                                            app.curve_size * 0.0001, zero3f);
      } else if (app.scc.paths[app.curr_path].end.face != -1)
        update_path_shape(scc.geodesic_shape->instance->shape, app.mesh,
                          scc.paths[app.curr_path], app.curve_size * 0.0001);
    } else if (scc.points.size() == 2) {
      update_path_shape(scc.geodesic_shape->instance->shape, app.mesh,
                        scc.paths[app.curr_path], app.curve_size * 0.0001);

    } else if (scc.points.size() == 3 && scc.result_pos.size() > 0) {
      add_path_shape(app, scc.result_pos, app.curve_size * 0.0001, {0, 1, 0});
      scc.points.clear();
      scc.paths.clear();
      scc.geodesic_shape = nullptr;
      app.curr_path = 0;
    }
  } else if (scc.state == scc_state::TANGENT && scc.expected_pos.size() > 0) {
    add_path_shape(app, scc.expected_pos, app.curve_size * 0.0001, {0, 1, 0});
    // scc.points.clear();
    // scc.paths.clear();
    // scc.geodesic_shape = nullptr;
    // app.curr_path = 0;
    // scc.expected_pos.clear();
  } else if (scc.state == scc_state::BISECTOR_ANGLE) {
    if (scc.geodesic_shape == nullptr &&
        app.scc.paths[app.curr_path].end.face != -1) {
      scc.geodesic_shape = add_path_shape(app, scc.paths[app.curr_path],
                                          app.curve_size * 0.0001, zero3f);
    } else if (app.scc.paths[app.curr_path].end.face != -1)
      update_path_shape(scc.geodesic_shape->instance->shape, app.mesh,
                        scc.paths[app.curr_path], app.curve_size * 0.0001);
    if (scc.points.size() == 2) {
      scc.input_shape.push_back(scc.geodesic_shape);
      scc.geodesic_shape = nullptr;
      scc.paths.push_back(geodesic_path{});
      scc.points.push_back(scc.points[0]);
      ++app.curr_path;
    }
    if (scc.points.size() == 4) {
      scc.input_shape.push_back(scc.geodesic_shape);
      scc.geodesic_shape = nullptr;
      scc.paths.push_back(geodesic_path{});
      scc.points.clear();
      ++app.curr_path;
    }
    if (scc.result_pos.size() > 0) {
      if (scc.output_shape.size() == 0)
        scc.output_shape = {add_path_shape(
            app, scc.result_pos, app.curve_size * 0.0001, vec3f{0, 1, 0})};
      else
        update_path_shape(scc.output_shape[0]->instance->shape, app.mesh,
                          scc.result_pos, app.curve_size * 0.0001);

      if (app.result_point == nullptr)
        app.result_point =
            add_points_shape(app, {scc.result_pos.back()},
                             2 * app.curve_size * 0.0001, vec3f{0, 1, 0});
      else
        update_points_shape(app.result_point->instance->shape,
                            {scc.result_pos.back()},
                            2 * app.curve_size * 0.0001);
      scc.result_pos.clear();
    }

    if (scc.expected_pos.size() > 0) {
      if (scc.output_shape.size() == 0)
        scc.output_shape = {add_path_shape(
            app, scc.expected_pos, app.curve_size * 0.0001, vec3f{0, 1, 0})};
      else
        update_path_shape(scc.output_shape[0]->instance->shape, app.mesh,
                          scc.expected_pos, app.curve_size * 0.0001);

      add_points_shape(app, {scc.expected_pos[0], scc.expected_pos.back()},
                       2 * app.curve_size * 0.0001, vec3f{0, 1, 0});
    }
  } else if (app.scc.state == scc_state::CIRCLE_3 && scc.points.size() == 3) {
    if (scc.result_pos.size() > 0) {
      if (scc.output_shape.size() == 0)
        scc.output_shape = {add_path_shape(
            app, scc.result_pos, app.curve_size * 0.0001, vec3f{0, 1, 0})};
      else
        update_path_shape(scc.output_shape[0]->instance->shape, app.mesh,
                          scc.result_pos, app.curve_size * 0.0001);
    }
    if (scc.expected_pos.size() > 0) {
      if (scc.output_shape.size() == 0)
        scc.output_shape = {add_path_shape(app, scc.expected_pos,
                                           app.curve_size * 0.0001, {0, 1, 0})};
      else
        update_path_shape(scc.output_shape[0]->instance->shape, app.mesh,
                          scc.expected_pos, app.curve_size * 0.0001);
    }
  } else if (scc.state == TRIANGLE && scc.points.size() > 0) {
    if (scc.points.size() == 1) {
      if (!app.use_exp_map_construction &&
          validate_paths({scc.paths[app.curr_path]})) {
        if (scc.geodesic_shape == nullptr) {
          scc.geodesic_shape =
              add_path_shape(app, scc.paths[app.curr_path],
                             app.curve_size * 0.0001, app.curr_color);
        } else if (app.scc.paths[app.curr_path].end.face != -1)
          update_path_shape(scc.geodesic_shape->instance->shape, app.mesh,
                            scc.paths[app.curr_path], app.curve_size * 0.0001);
      } else if (app.use_exp_map_construction &&
                 app.scc.circles[app.curr_circle].radius > 0) {

        app.selected_circle = &app.scc.circles[app.curr_circle];

        if (scc.circle_shape.size() == 0)
          std::tie(scc.circle_shape, app.selected_circle_entries) =
              add_isoline_shape(app, app.selected_circle->isoline);
        else
          update_isoline_shape(app, app.selected_circle->isoline,
                               app.selected_circle_entries);

        auto vertices =
            equilateral_triangle_tangent_space(app.mesh, app.selected_circle);
        scc.circles[app.curr_circle].vertices = vertices;
        if (app.garland) {
          if (app.added_paths.size() - 1 == app.selected_circle_entries[0])
            add_garland_shape(app, vertices);
          else
            update_garland_shape(
                app, make_garland(app.mesh, app.selected_circle, vertices));
        } else {
          if (scc.result_shape == nullptr)
            scc.result_shape = add_triangle_shape(app, vertices);
          else
            update_triangle_shape(app, vertices,
                                  app.selected_circle_entries[0] + 1);
        }
      }
    } else if (scc.points.size() == 2 && app.use_exp_map_construction &&
               app.selected_circle_entries.size() == 1) {
      app.scc.points.clear();

      clear_shape(scc.circle_shape[0]->instance->shape);
      scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
      if (!app.garland)
        scc.circles[app.curr_circle].primitive = tri;
      else
        scc.circles[app.curr_circle].primitive = garland_tri;
      app.selected_circle->levels = 1;
      ++app.curr_circle;
      scc.circles[app.curr_circle].levels = 1;
      scc.circles.push_back(Circle());
      app.scc.circle_shape = {};
      app.scc.result_shape = nullptr;

    } else if (scc.points.size() == 2 && !app.use_exp_map_construction) {
      if (app.type_of_triangle == Equilateral) {
        auto [vertices, c0, c1] = equilateral_triangle(
            app.mesh, app.scc.points[0], app.scc.points[1], app.flip_triangle);
        app.curr_color = vec3f{0, 1, 0};

        if (scc.result_shape == nullptr) {

          std::tie(scc.result_shape, app.result_entry) =
              add_result_shape(app, vertices);
        } else
          update_generic_shape(app, vertices, app.result_entry);
        app.curr_color = vec3f{1, 0, 1};
        std::tie(scc.circle_shape, app.selected_circle_entries) =
            add_isoline_shape(app, c0.isoline);
        scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
        scc.circles[app.curr_circle] = c0;
        std::tie(scc.circle_shape, app.selected_circle_entries) =
            add_isoline_shape(app, c1.isoline);
        scc.circles_to_shape[app.curr_circle + 1] = app.selected_circle_entries;
        scc.circles.push_back(c1);
        app.curr_circle += 2;
        scc.circles.push_back(Circle());
        scc.points = vertices;
        add_points_shape(app, {scc.points[0], scc.points[1]},
                         2 * app.curve_size * 0.0001, {1, 0, 1});
      } else if (app.type_of_triangle == IsosceleSameLenth) {
        auto len = path_length(
            compute_geodesic_path(app.mesh, scc.points[0], scc.points.back()),
            app.mesh.triangles, app.mesh.positions, app.mesh.adjacencies);
        auto [vertices, c0, c1] = same_lengths_isoscele_triangle(
            app.mesh, app.scc.points[0], app.scc.points[1], 1.5 * len,
            app.flip_triangle);
        app.curr_color = vec3f{0, 1, 0};

        if (scc.result_shape == nullptr) {

          std::tie(scc.result_shape, app.result_entry) =
              add_result_shape(app, vertices);
        } else
          update_generic_shape(app, vertices, app.result_entry);
        app.curr_color = vec3f{1, 0, 1};
        std::tie(scc.circle_shape, app.selected_circle_entries) =
            add_isoline_shape(app, c0.isoline);
        scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
        scc.circles[app.curr_circle] = c0;
        scc.circles[app.curr_circle].primitive = cir;
        std::tie(scc.circle_shape, app.selected_circle_entries) =
            add_isoline_shape(app, c1.isoline);
        scc.circles_to_shape[app.curr_circle + 1] = app.selected_circle_entries;
        scc.circles.push_back(c1);
        scc.circles.back().primitive = cir;
        app.curr_circle += 2;
        scc.circles.push_back(Circle());
        scc.points = vertices;
        add_points_shape(app, {scc.points[0], scc.points[1]},
                         2 * app.curve_size * 0.0001, {1, 0, 1});
        clear_shape(scc.geodesic_shape->instance->shape);
      } else if (app.type_of_triangle == IsosceleBis) {
        auto midpoint = find_midpoint(app.mesh, scc.paths[0]);
        auto len = path_length(scc.paths[0], app.mesh.triangles,
                               app.mesh.positions, app.mesh.adjacencies);
        auto [vertices, bis, c0] =
            altitude_isoscele_triangle(app.mesh, scc.points[0], scc.points[1],
                                       1.5 * len, app.flip_triangle);
        app.curr_color = vec3f{0, 1, 0};
        if (scc.result_shape == nullptr) {

          std::tie(scc.result_shape, app.result_entry) =
              add_result_shape(app, vertices);
        } else
          update_generic_shape(app, vertices, app.result_entry);

        add_path_shape(app, bis, 0.00008 * app.curve_size, {0, 0, 0});
        app.curr_color = vec3f{1, 0, 1};
        std::tie(scc.circle_shape, app.selected_circle_entries) =
            add_isoline_shape(app, c0.isoline);
        scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
        scc.circles[app.curr_circle] = c0;
        scc.circles[app.curr_circle].primitive = cir;
        ++app.curr_circle;
        scc.circles.push_back(Circle{});
        scc.points = vertices;
        add_points_shape(app, {scc.points[0], scc.points[1]},
                         2 * app.curve_size * 0.0001, {1, 0, 1});
      }
    }
  } else if (scc.state == SQUARE && scc.points.size() > 0) {
    if (app.use_exp_map_construction && app.scc.points.size() == 1 &&
        app.scc.circles[app.curr_circle].radius > 0) {
      app.selected_circle = &app.scc.circles[app.curr_circle];
      if (scc.circle_shape.size() == 0) {

        std::tie(scc.circle_shape, app.selected_circle_entries) =
            add_isoline_shape(app, app.selected_circle->isoline);

      } else
        update_isoline_shape(app, app.selected_circle->isoline,
                             app.selected_circle_entries);

      auto vertices = rhombus_tangent_space(app.mesh, app.selected_circle, 1.f);
      scc.circles[app.curr_circle].vertices = vertices;
      if (app.garland) {
        if (app.added_paths.size() - 1 == app.selected_circle_entries[0])
          add_garland_shape(app, vertices);
        else
          update_garland_shape(
              app, make_garland(app.mesh, app.selected_circle, vertices));
      } else {
        if (scc.result_shape == nullptr)
          scc.result_shape = add_quadrilateral_shape(app, vertices);
        else
          update_quadrilateral_shape(app, vertices,
                                     app.selected_circle_entries[0] + 1);
      }
    } else if (app.use_exp_map_construction && app.scc.points.size() == 2 &&
               app.scc.circles[app.curr_circle].radius > 0 &&
               app.selected_circle_entries.size() == 1) {
      app.scc.points.clear();
      clear_shape(scc.circle_shape[0]->instance->shape);
      scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
      if (!app.garland)
        scc.circles[app.curr_circle].primitive = square;
      else
        scc.circles[app.curr_circle].primitive = garland_sq;
      scc.circles[app.curr_circle].levels = 1;
      ++app.curr_circle;
      scc.circles.push_back(Circle());
      app.scc.circle_shape = {};
      app.scc.result_shape = nullptr;
    }
  } else if (scc.state == RHOMBUS && scc.points.size() > 0) {
    if (app.use_exp_map_construction && app.scc.points.size() == 1 &&
        app.scc.circles[app.curr_circle].radius > 0) {
      auto &circle = app.scc.circles[app.curr_circle];
      if (scc.circle_shape.size() == 0) {

        std::tie(scc.circle_shape, app.selected_circle_entries) =
            add_isoline_shape(app, circle.isoline);

      } else
        update_isoline_shape(app, circle.isoline, app.selected_circle_entries);

      auto vertices = rhombus_tangent_space(app.mesh, &circle, app.lambda);
      scc.circles[app.curr_circle].vertices = vertices;
      if (scc.result_shape == nullptr)
        scc.result_shape = add_quadrilateral_shape(app, vertices);
      else
        update_quadrilateral_shape(app, vertices);
    } else if (app.use_exp_map_construction && app.scc.points.size() == 2 &&
               app.scc.circles[app.curr_circle].radius > 0 &&
               app.selected_circle_entries.size() == 1) {
      app.scc.points.clear();
      clear_shape(scc.circle_shape[0]->instance->shape);
      scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
      scc.circles[app.curr_circle].primitive = rhomb;
      scc.circles[app.curr_circle].levels = 1;
      ++app.curr_circle;
      scc.circles.push_back(Circle());
      app.scc.circle_shape = {};
      app.scc.result_shape = nullptr;
    }
  } else if (scc.state == PARALLELOGRAM && scc.points.size() > 0) {
    if (app.use_exp_map_construction && app.scc.points.size() == 1 &&
        app.scc.circles[app.curr_circle].radius > 0) {
      auto &circle = app.scc.circles[app.curr_circle];
      if (scc.circle_shape.size() == 0) {

        std::tie(scc.circle_shape, app.selected_circle_entries) =
            add_isoline_shape(app, circle.isoline);

      } else
        update_isoline_shape(app, circle.isoline, app.selected_circle_entries);

      auto vertices =
          parallelogram_tangent_space(app.mesh, &circle, app.sigma, app.lambda);
      scc.circles[app.curr_circle].vertices = vertices;
      if (scc.result_shape == nullptr)
        scc.result_shape = add_quadrilateral_shape(app, vertices);
      else
        update_quadrilateral_shape(app, vertices,
                                   app.selected_circle_entries[0] + 1);
    } else if (app.use_exp_map_construction && app.scc.points.size() == 2 &&
               app.scc.circles[app.curr_circle].radius > 0 &&
               app.selected_circle_entries.size() == 1) {
      app.scc.points.clear();
      clear_shape(scc.circle_shape[0]->instance->shape);
      scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
      scc.circles[app.curr_circle].primitive = parallelog;
      scc.circles[app.curr_circle].levels = 1;
      ++app.curr_circle;
      scc.circles.push_back(Circle());
      app.scc.circle_shape = {};
      app.scc.result_shape = nullptr;
    }
  } else if (scc.state == PENTAGON && scc.points.size() > 0) {
    if (app.use_exp_map_construction && app.scc.points.size() == 1 &&
        app.scc.circles[app.curr_circle].radius > 0) {
      app.selected_circle = &app.scc.circles[app.curr_circle];
      if (scc.circle_shape.size() == 0) {
        std::tie(scc.circle_shape, app.selected_circle_entries) =
            add_isoline_shape(app, app.selected_circle->isoline);
      } else
        update_isoline_shape(app, app.selected_circle->isoline,
                             app.selected_circle_entries);
      auto vertices = pentagon_tangent_space(app.mesh, app.selected_circle);
      scc.circles[app.curr_circle].vertices = vertices;
      if (app.garland) {
        if (app.added_paths.size() - 1 == app.selected_circle_entries[0])
          add_garland_shape(app, vertices);
        else
          update_garland_shape(
              app, make_garland(app.mesh, app.selected_circle, vertices));
      } else {

        if (scc.result_shape == nullptr)
          scc.result_shape = add_pentagonal_shape(app, vertices);
        else
          update_pentagonal_shape(app, vertices,
                                  app.selected_circle_entries[0] + 1);
      }

    } else if (app.use_exp_map_construction && app.scc.points.size() == 2 &&
               app.scc.circles[app.curr_circle].radius > 0 &&
               app.selected_circle_entries.size() == 1) {
      app.scc.points.clear();
      clear_shape(scc.circle_shape[0]->instance->shape);
      scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
      if (!app.garland)
        scc.circles[app.curr_circle].primitive = pent;
      else
        scc.circles[app.curr_circle].primitive = garland_pent;
      scc.circles[app.curr_circle].levels = 1;
      ++app.curr_circle;
      scc.circles.push_back(Circle());
      app.scc.circle_shape = {};
      app.scc.result_shape = nullptr;
    }
  } else if (scc.state == HEXAGON && scc.points.size() > 0) {
    if (app.use_exp_map_construction && app.scc.points.size() == 1 &&
        app.scc.circles[app.curr_circle].radius > 0) {
      app.selected_circle = &app.scc.circles[app.curr_circle];
      if (scc.circle_shape.size() == 0) {
        std::tie(scc.circle_shape, app.selected_circle_entries) =
            add_isoline_shape(app, app.selected_circle->isoline);
      } else
        update_isoline_shape(app, app.selected_circle->isoline,
                             app.selected_circle_entries);

      auto vertices = hexagon_tangent_space(app.mesh, app.selected_circle);
      scc.circles[app.curr_circle].vertices = vertices;
      if (app.garland) {
        if (app.added_paths.size() - 1 == app.selected_circle_entries[0])
          add_garland_shape(app, vertices);
        else
          update_garland_shape(
              app, make_garland(app.mesh, app.selected_circle, vertices));
      } else {
        if (scc.result_shape == nullptr)
          scc.result_shape = add_hexagonal_shape(app, vertices);
        else
          update_hexagonal_shape(app, vertices,
                                 app.selected_circle_entries[0] + 1);
      }
    } else if (app.use_exp_map_construction && app.scc.points.size() == 2 &&
               app.scc.circles[app.curr_circle].radius > 0 &&
               app.selected_circle_entries.size() == 1) {
      app.scc.points.clear();
      clear_shape(scc.circle_shape[0]->instance->shape);
      scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
      if (!app.garland)
        scc.circles[app.curr_circle].primitive = primitives::hex;
      else
        scc.circles[app.curr_circle].primitive = garland_hex;
      scc.circles[app.curr_circle].levels = 1;
      ++app.curr_circle;
      scc.circles.push_back(Circle());
      app.scc.circle_shape = {};
      app.scc.result_shape = nullptr;
    }
  } else if (scc.state == OCTAGON && scc.points.size() > 0) {
    if (app.use_exp_map_construction && app.scc.points.size() == 1 &&
        app.scc.circles[app.curr_circle].radius > 0) {
      app.selected_circle = &app.scc.circles[app.curr_circle];
      if (scc.circle_shape.size() == 0) {
        std::tie(scc.circle_shape, app.selected_circle_entries) =
            add_isoline_shape(app, app.selected_circle->isoline);
      } else
        update_isoline_shape(app, app.selected_circle->isoline,
                             app.selected_circle_entries);

      auto vertices = octagon_tangent_space(app.mesh, app.selected_circle);
      scc.circles[app.curr_circle].vertices = vertices;
      // if (app.garland) {
      //   if (app.added_paths.size() - 1 == app.selected_circle_entries[0])
      //     add_garland_shape(app, vertices);
      //   else
      //     update_garland_shape(
      //         app, make_garland(app.mesh, app.selected_circle, vertices));
      // } else {
      if (scc.result_shape == nullptr)
        scc.result_shape = add_octagonal_shape(app, vertices);
      else
        update_octagonal_shape(app, vertices,
                               app.selected_circle_entries[0] + 1);
      // }
    } else if (app.use_exp_map_construction && app.scc.points.size() == 2 &&
               app.scc.circles[app.curr_circle].radius > 0 &&
               app.selected_circle_entries.size() == 1) {
      app.scc.points.clear();
      clear_shape(scc.circle_shape[0]->instance->shape);
      scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
      // if (!app.garland)
      scc.circles[app.curr_circle].primitive = primitives::oct;
      // else
      //   scc.circles[app.curr_circle].primitive = garland_hex;
      scc.circles[app.curr_circle].levels = 1;
      ++app.curr_circle;
      scc.circles.push_back(Circle());
      app.scc.circle_shape = {};
      app.scc.result_shape = nullptr;
    }
  } else if (scc.state == DECAGON && scc.points.size() > 0) {
    if (app.use_exp_map_construction && app.scc.points.size() == 1 &&
        app.scc.circles[app.curr_circle].radius > 0) {
      app.selected_circle = &app.scc.circles[app.curr_circle];
      if (scc.circle_shape.size() == 0) {
        std::tie(scc.circle_shape, app.selected_circle_entries) =
            add_isoline_shape(app, app.selected_circle->isoline);
      } else
        update_isoline_shape(app, app.selected_circle->isoline,
                             app.selected_circle_entries);

      auto vertices = decagon_tangent_space(app.mesh, app.selected_circle);
      scc.circles[app.curr_circle].vertices = vertices;
      // if (app.garland) {
      //   if (app.added_paths.size() - 1 == app.selected_circle_entries[0])
      //     add_garland_shape(app, vertices);
      //   else
      //     update_garland_shape(
      //         app, make_garland(app.mesh, app.selected_circle, vertices));
      // } else {
      if (scc.result_shape == nullptr)
        scc.result_shape = add_decagonal_shape(app, vertices);
      else
        update_decagonal_shape(app, vertices,
                               app.selected_circle_entries[0] + 1);
      // }
    } else if (app.use_exp_map_construction && app.scc.points.size() == 2 &&
               app.scc.circles[app.curr_circle].radius > 0 &&
               app.selected_circle_entries.size() == 1) {
      app.scc.points.clear();
      clear_shape(scc.circle_shape[0]->instance->shape);
      scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
      // if (!app.garland)
      scc.circles[app.curr_circle].primitive = primitives::deca;
      // else
      //   scc.circles[app.curr_circle].primitive = garland_hex;
      scc.circles[app.curr_circle].levels = 1;
      ++app.curr_circle;
      scc.circles.push_back(Circle());
      app.scc.circle_shape = {};
      app.scc.result_shape = nullptr;
    }
  } else if (scc.state == RECTANGLE && scc.points.size() > 0) {
    if (scc.points.size() == 1) {
      if (app.use_exp_map_construction && app.scc.points.size() == 1 &&
          app.scc.circles[app.curr_circle].radius > 0) {
        auto &circle = app.scc.circles[app.curr_circle];
        if (scc.circle_shape.size() == 0) {
          std::tie(scc.circle_shape, app.selected_circle_entries) =
              add_isoline_shape(app, circle.isoline);
        } else
          update_isoline_shape(app, circle.isoline,
                               app.selected_circle_entries);

        auto vertices =
            parallelogram_tangent_space(app.mesh, &circle, app.sigma, 1.f);
        scc.circles[app.curr_circle].vertices = vertices;
        if (scc.result_shape == nullptr)
          scc.result_shape = add_quadrilateral_shape(app, vertices);
        else
          update_quadrilateral_shape(app, vertices);
      } else if (scc.paths[app.curr_path].end.face != -1) {

        if (scc.geodesic_shape == nullptr) {
          scc.geodesic_shape =
              add_path_shape(app, scc.paths[app.curr_path],
                             app.curve_size * 0.0001, app.curr_color);
        } else if (app.scc.paths[app.curr_path].end.face != -1)
          update_path_shape(scc.geodesic_shape->instance->shape, app.mesh,
                            scc.paths[app.curr_path], app.curve_size * 0.0001);
      }

    } else if (scc.points.size() == 2) {
      if (app.use_exp_map_construction &&
          app.scc.circles[app.curr_circle].radius > 0 &&
          app.selected_circle_entries.size() == 1) {
        app.scc.points.clear();
        clear_shape(scc.circle_shape[0]->instance->shape);
        scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
        scc.circles[app.curr_circle].primitive = rett;
        scc.circles[app.curr_circle].levels = 1;
        ++app.curr_circle;
        scc.circles.push_back(Circle());
        app.scc.circle_shape = {};
        app.scc.result_shape = nullptr;

      } else if (app.scc.paths.back().end.face != -1) {
        switch (app.type_of_rectangle) {
        case Diagonal: {
          auto [vertices, line0, line1, line2] = diagonal_rectangle(
              app.mesh, scc.paths.rbegin()[1], scc.paths.rbegin()[0]);
          app.curr_color = vec3f{0, 1, 0};
          if (scc.result_shape == nullptr) {
            std::tie(scc.result_shape, app.result_entry) =
                add_result_shape(app, vertices);
          } else
            update_generic_shape(app, vertices, app.result_entry);
        } break;
        case Euclidean: {
          auto height = path_length(scc.paths.back(), app.mesh.triangles,
                                    app.mesh.positions, app.mesh.adjacencies);

          auto [vertices, line, c0, c1, c2] = euclidean_rectangle(
              app.mesh,
              compute_geodesic_path(app.mesh, scc.points[0], scc.points[1]),
              height);
          if (scc.result_shape == nullptr) {
            std::tie(scc.result_shape, app.result_entry) =
                add_result_shape(app, vertices);
          } else
            update_generic_shape(app, vertices, app.result_entry);

          // add_points_shape(app, vertices, 2 * app.curve_size * 0.0001,
          // zero3f);
        } break;
        case SameLengths: {
          auto height = path_length(scc.paths.back(), app.mesh.triangles,
                                    app.mesh.positions, app.mesh.adjacencies);
          auto [vertices, line0, line1, c0, c1] = same_lengths_rectangle(
              app.mesh,
              compute_geodesic_path(app.mesh, scc.points[0], scc.points[1]),
              height);
          if (scc.result_shape == nullptr) {
            std::tie(scc.result_shape, app.result_entry) =
                add_result_shape(app, vertices);
          } else
            update_generic_shape(app, vertices, app.result_entry);

          // add_points_shape(app, vertices, 2 * app.curve_size * 0.0001,
          // zero3f);

        } break;
        }
      }

    } else if (scc.points.size() == 3) {

      switch (app.type_of_rectangle) {
      case Diagonal: {
        auto [vertices, line0, line1, line2] =
            diagonal_rectangle(app.mesh, scc.paths[0], scc.paths[1]);
        clear_shape(scc.geodesic_shape->instance->shape);
        scc.geodesic_shape = nullptr;
        scc.points = vertices;
        // scc.paths = {geodesic_path{}};
        // app.curr_path = 0;
        update_generic_shape(app, vertices, app.result_entry);
        add_points_shape(app, {vertices[0], vertices.back()},
                         2 * app.curve_size * 0.0001, {1, 0, 1});
      } break;
      case Euclidean: {
        auto height = path_length(scc.paths.back(), app.mesh.triangles,
                                  app.mesh.positions, app.mesh.adjacencies);

        auto [vertices, line, c0, c1, c2] = euclidean_rectangle(
            app.mesh,
            compute_geodesic_path(app.mesh, scc.points[0], scc.points[1]),
            height);
        scc.geodesic_shape = nullptr;
        scc.points = vertices;
        // scc.paths = {geodesic_path{}};
        // app.curr_path = 0;
        update_generic_shape(app, vertices, app.result_entry);
        add_points_shape(app, vertices, 2 * app.curve_size * 0.0001, zero3f);
      } break;
      case SameLengths: {
        auto height = path_length(scc.paths.back(), app.mesh.triangles,
                                  app.mesh.positions, app.mesh.adjacencies);
        auto [vertices, line0, line1, c0, c1] = same_lengths_rectangle(
            app.mesh,
            compute_geodesic_path(app.mesh, scc.points[0], scc.points[1]),
            height);
        scc.geodesic_shape = nullptr;
        scc.points = vertices;
        // scc.paths = {geodesic_path{}};
        // app.curr_path = 0;
        update_generic_shape(app, vertices, app.result_entry);
        add_points_shape(app, vertices, 2 * app.curve_size * 0.0001, zero3f);
      } break;
      }

      // scc.result_shape = nullptr;
      // scc.points.clear();
      // add_path_shape(app, line0, 0.75 * app.curve_size * 0.0001, {0, 1,
      // 0}); add_path_shape(app, line1, 0.75 * app.curve_size * 0.0001, {0,
      // 1, 0}); add_path_shape(app, line2, 0.75 * app.curve_size * 0.0001,
      // {0, 1, 0}); add_path_shape(
      //     app,
      //     compute_geodesic_path(app.mesh, vertices[0],
      //     vertices.rbegin()[1]), 0.75 * app.curve_size * 0.0001, zero3f);
    } else if (scc.points.size() == 4) {
      app.curr_color = vec3f{0, 1, 0};
      if (scc.result_shape == nullptr) {
        std::tie(scc.result_shape, app.result_entry) =
            add_result_shape(app, scc.points);
      } else
        update_generic_shape(app, scc.points, app.result_entry);
    }
  } else if (scc.state == SPIDER_NET && scc.points.size() > 0) {
    if (app.scc.points.size() == 1 &&
        app.scc.circles[app.curr_circle].radius > 0) {
      app.selected_circle = &app.scc.circles[app.curr_circle];
      app.selected_circle->levels = 3;
      app.selected_circle->primitive = primitives::oct;
      if (scc.circle_shape.size() == 0) {

        std::tie(scc.circle_shape, app.selected_circle_entries) =
            add_isoline_shape(app, app.selected_circle->isoline);

      } else
        update_isoline_shape(app, app.selected_circle->isoline,
                             app.selected_circle_entries);
      auto [vertices, arcs] = spider_net(app.mesh, app.selected_circle, true);
      app.selected_circle->spider_vertices = vertices;
      app.selected_circle->arcs = arcs;
      if (app.added_paths.size() - 1 == app.selected_circle_entries[0])
        add_spider_net_shape(app, vertices, arcs, true);
      else
        update_spider_net_shape(app, vertices, arcs, true);
    } else if (app.scc.points.size() == 2 &&
               app.scc.circles[app.curr_circle].radius > 0 &&
               app.selected_circle_entries.size() == 1) {
      app.scc.points.clear();
      clear_shape(scc.circle_shape[0]->instance->shape);
      for (auto i = app.selected_circle_entries[0] + 1;
           i < app.added_paths.size(); ++i) {
        clear_shape(app.added_paths[i]->instance->shape);
      }
      app.added_paths.erase(app.added_paths.begin() +
                                app.selected_circle_entries[0] + 1,
                            app.added_paths.end());

      auto [vertices, arcs] = spider_net(app.mesh, app.selected_circle, false);
      app.selected_circle->spider_vertices = vertices;
      app.selected_circle->arcs = arcs;
      add_spider_net_shape(app, vertices, arcs, false);
      scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
      scc.circles[app.curr_circle].primitive = spider;
      scc.circles[app.curr_circle].levels = 3;
      ++app.curr_circle;
      scc.circles.push_back(Circle());
      app.scc.circle_shape = {};
      app.scc.result_shape = nullptr;
    }
  } else if (scc.state == CRUX && scc.points.size() > 0) {
    if (app.use_exp_map_construction && app.scc.points.size() == 1 &&
        app.scc.circles[app.curr_circle].radius > 0) {
      app.selected_circle = &app.scc.circles[app.curr_circle];
      if (scc.circle_shape.size() == 0) {
        std::tie(scc.circle_shape, app.selected_circle_entries) =
            add_isoline_shape(app, app.selected_circle->isoline);
      } else
        update_isoline_shape(app, app.selected_circle->isoline,
                             app.selected_circle_entries);

      auto arcs = make_cross(app.mesh, app.selected_circle, true);
      app.selected_circle->arcs = arcs;
      if (app.added_paths.size() - 1 == app.selected_circle_entries[0])
        add_cross_shape(app, arcs);
      else
        update_cross_shape(app, arcs, true);

    } else if (app.use_exp_map_construction && app.scc.points.size() == 2 &&
               app.scc.circles[app.curr_circle].radius > 0 &&
               app.selected_circle_entries.size() == 1) {
      app.scc.points.clear();
      clear_shape(scc.circle_shape[0]->instance->shape);
      for (auto i = app.selected_circle_entries[0] + 1;
           i < app.added_paths.size(); ++i) {
        clear_shape(app.added_paths[i]->instance->shape);
      }
      app.added_paths.erase(app.added_paths.begin() +
                                app.selected_circle_entries[0] + 1,
                            app.added_paths.end());
      app.selected_circle->arcs =
          make_cross(app.mesh, app.selected_circle, false);
      add_cross_shape(app, app.selected_circle->arcs);
      scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
      scc.circles[app.curr_circle].primitive = primitives::crux;
      scc.circles[app.curr_circle].levels = 1;
      ++app.curr_circle;
      scc.circles.push_back(Circle());
      app.scc.circle_shape = {};
      app.scc.result_shape = nullptr;
    }
  } else if (scc.state == FLOWER && scc.points.size() > 0) {
    if (app.use_exp_map_construction && app.scc.points.size() == 1 &&
        app.scc.circles[app.curr_circle].radius > 0) {
      app.selected_circle = &app.scc.circles[app.curr_circle];
      if (scc.circle_shape.size() == 0) {
        std::tie(scc.circle_shape, app.selected_circle_entries) =
            add_isoline_shape(app, app.selected_circle->isoline);
      } else
        update_isoline_shape(app, app.selected_circle->isoline,
                             app.selected_circle_entries);
      app.selected_circle->petals = 10;
      auto arcs = make_flower(app.mesh, app.selected_circle,
                              app.selected_circle->petals, true);
      app.selected_circle->arcs = arcs;
      if (app.added_paths.size() - 1 == app.selected_circle_entries[0])
        add_flower_shape(app, arcs);
      else
        update_flower_shape(app, arcs, true, false);
    } else if (app.use_exp_map_construction && app.scc.points.size() == 2 &&
               app.scc.circles[app.curr_circle].radius > 0 &&
               app.selected_circle_entries.size() == 1) {
      app.scc.points.clear();
      clear_shape(scc.circle_shape[0]->instance->shape);
      for (auto i = app.selected_circle_entries[0];
           i < app.selected_circle_entries[0] + app.selected_circle->petals + 1;
           ++i) {
        clear_shape(app.added_paths[i]->instance->shape);
      }
      app.added_paths.erase(app.added_paths.begin() +
                                app.selected_circle_entries[0] + 1,
                            app.added_paths.end());
      app.selected_circle->arcs = make_flower(
          app.mesh, app.selected_circle, app.selected_circle->petals, false,
          app.scale_factor_inner_circle_radius, app.with_inner_circle);
      add_flower_shape(app, app.selected_circle->arcs);
      scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
      scc.circles[app.curr_circle].primitive = primitives::flower;
      scc.circles[app.curr_circle].levels = 1;
      scc.circles[app.curr_circle].with_inner_circle = app.with_inner_circle;
      ++app.curr_circle;
      scc.circles.push_back(Circle());
      app.scc.circle_shape = {};
      app.scc.result_shape = nullptr;
    }
  }
}
void update_scc(App &app) {
  auto &scc = app.scc;
  if (app.input().cropping && scc.points.size() > 0) {

    scc.cropping_range.push_back(closest_point_on_circle(
        app.mesh, app.selected_circle, app.scc.points.back()));
    if (scc.cropping_range.size() == 2) {
      clear_shape(app.selected_point->instance->shape);
      clear_shape(app.added_points.rbegin()[1]->instance->shape);
      app.selected_point = nullptr;
    } else
      app.selected_point = nullptr;

  } else if (affine_transofrmation(app) && scc.points.size() == 1) {
    if (app.selected_circle->primitive == primitives::spider) {
      auto [vertices, arcs] = spider_net(app.mesh, app.selected_circle, false);
      update_spider_net_shape(app, vertices, arcs, false);
    } else if (app.selected_circle->primitive == primitives::crux) {
      auto arcs = make_cross(app.mesh, app.selected_circle, false);
      update_cross_shape(app, arcs, false);
    } else if (app.selected_circle->primitive == primitives::flower) {
      auto flower = make_flower(app.mesh, app.selected_circle,
                                app.selected_circle->petals, false,
                                app.scale_factor_inner_circle_radius,
                                app.selected_circle->with_inner_circle);
      update_flower_shape(app, flower, false,
                          app.selected_circle->with_inner_circle);
    }
    app.input().rotating = false;
    app.input().translating = false;
    app.input().scaling = false;
    app.selected_circle = nullptr;
    app.scc.points.clear();

  } else if (scc.state == scc_state::GEODESIC) {
    if (app.scc.points.size() > 0 && (int)(app.scc.points.size()) % 2 == 0) {
      app.scc.paths[app.curr_path] = compute_geodesic_path(
          app.mesh, scc.points.rbegin()[0], scc.points.rbegin()[1]);

      // add_points_shape(app, {p}, 2 * app.curve_size * 0.0001, {0, 0, 0});
    }
  } else if (scc.state == scc_state::CIRCLE) {
    if (scc.points.size() == 1 &&
        scc.circles[app.curr_circle].distances.size() == 0) {
      scc.circles[app.curr_circle] =
          create_circle(app.mesh, scc.points[0], 0.f);
      add_points_shape(app, scc.points, 3 * app.curve_size * 0.0001,
                       app.curr_color);
    } else if (scc.points.size() == 2) {

      set_radius(app.mesh, &app.scc.circles[app.curr_circle],
                 get_distance(app.mesh.triangles[scc.points.back().face],
                              get_bary(scc.points.back().uv),
                              app.scc.circles[app.curr_circle].distances),
                 scc.points[1]);
    }

  } else if (scc.state == scc_state::ELLIPSE) {
    if (app.scc.points.size() == 1)
      add_points_shape(app, scc.points, 3 * app.curve_size * 0.0001, zero3f);
    else if (app.scc.points.size() == 2 &&
             app.scc.Ellipses[app.curr_circle].distances_from_midpoint.size() ==
                 0) {
      app.scc.Ellipses[app.curr_circle].midpoint =
          eval_path_midpoint(app.scc.paths[app.curr_path], app.mesh.triangles,
                             app.mesh.positions, app.mesh.adjacencies);
      app.scc.Ellipses[app.curr_circle].distances_from_midpoint =
          compute_geodesic_distances(
              app.mesh.solver, app.mesh.triangles, app.mesh.positions,
              app.mesh.adjacencies,
              {app.scc.Ellipses[app.curr_circle].midpoint});
      add_points_shape(app, {scc.points.back()}, 3 * app.curve_size * 0.0001,
                       zero3f);
    } else if (app.scc.points.size() == 3) {
      app.scc.Ellipses[app.curr_circle].isoline = app.isoline;
    }

  } else if (scc.state == scc_state::BISECTOR && scc.points.size() == 2) {
    add_points_shape(app, scc.points, 2 * app.curve_size * 0.0001, {0, 0, 0});
    auto path = compute_geodesic_path(app.mesh, scc.points[0], scc.points[1]);
    app.scc.paths[app.curr_path] = path;
    scc.input_shape = {
        add_path_shape(app, path, app.curve_size * 0.0001, vec3f{0, 0, 0})};
  } else if (scc.state == scc_state::TANGENT) {
    if (scc.points.size() == 1 &&
        scc.circles[app.curr_circle].distances.size() == 0) {
      scc.circles[app.curr_circle] =
          create_circle(app.mesh, scc.points[0], 0.f);
      add_points_shape(app, scc.points, 2 * app.curve_size * 0.0001, {0, 0, 0});
    } else if (scc.points.size() == 2 &&
               scc.circles[app.curr_circle].distances.size() > 0) {
      set_radius(app.mesh, &app.scc.circles[app.curr_circle],
                 get_distance(app.mesh.triangles[scc.points.back().face],
                              get_bary(scc.points.back().uv),
                              app.scc.circles[app.curr_circle].distances));
      app.isoline = scc.circles[app.curr_circle].isoline;
      scc.points.back() = intersect_isoline_point(
          app, eval_position(app.mesh.triangles, app.mesh.positions,
                             scc.points.back()));
      add_points_shape(app, {scc.points.back()}, 3 * app.curve_size * 0.0001,
                       {0, 1, 0});
    }
  } else if (scc.state == scc_state::BISECTOR_ANGLE) {
    if (app.scc.points.size() > 0 && (int)(app.scc.points.size()) % 2 == 0) {
      app.scc.paths[app.curr_path] = compute_geodesic_path(
          app.mesh, scc.points.rbegin()[1], scc.points.rbegin()[0]);
      add_points_shape(app, scc.points, 2 * app.curve_size * 0.0001, {0, 0, 0});
    }
  } else if (scc.state == scc_state::PERPENDICULAR) {
    if (scc.points.size() == 2)
      add_points_shape(app, scc.points, 2 * app.curve_size * 0.0001, zero3f);
    else if (scc.points.size() == 3) {
      scc.points.back() = intersect_geodesic_point(app, scc.points.back());
    }

  } else if (scc.state == scc_state::CIRCLE_3 && app.scc.points.size() > 0) {
    add_points_shape(app, {scc.points.back()}, 2 * app.curve_size * 0.0001,
                     {1, 0, 1});
  } else if (scc.state == TRIANGLE && app.scc.points.size() > 0) {
    if (scc.points.size() == 2 && !app.use_exp_map_construction) {
      if (app.type_of_triangle == IsosceleBis && app.isoline.size() == 0)
        app.isoline =
            find_segment_bisector_isolines(app.mesh, scc.paths[app.curr_path]);

    } else if (scc.points.size() == 3) {
      if (app.type_of_triangle == IsosceleBis) {
        scc.points.back() = intersect_isoline_point(app, scc.points.back());
        clear_shape(app.selected_point->instance->shape);
        app.isoline = {};
      }
      app.selected_point = nullptr;
      scc.construction_shape.clear();

      add_points_shape(app, app.scc.points, 2 * app.curve_size * 0.0001,
                       {0, 1, 0});
    }
  } else if (scc.state == RECTANGLE && app.scc.points.size() == 2) {
    scc.paths.push_back(geodesic_path{});
    ++app.curr_path;
  }
}

//
void set_common_uniforms(const App &app, const ogl_program *program) {
  auto &view = app.matrices.view;
  auto &projection = app.matrices.projection;
  set_uniform(program, "frame", identity4x4f);
  set_uniform(program, "view", view);
  set_uniform(program, "projection", projection);
  set_uniform(program, "eye", app.ogl_camera->frame.o);
  set_uniform(program, "envlight", (int)app.envlight);
  set_uniform(program, "gamma", app.shade_params.gamma);
  set_uniform(program, "exposure", app.shade_params.exposure);
  // set_uniform(program, "size", app.line_size);
  if (app.scene->environments.size()) {
    auto &env = app.scene->environments.front();
    if (env->envlight_diffuse)
      set_uniform(program, "envlight_irradiance", env->envlight_diffuse, 6);
    if (env->envlight_specular)
      set_uniform(program, "envlight_reflection", env->envlight_specular, 7);
    if (env->envlight_brdflut)
      set_uniform(program, "envlight_brdflut", env->envlight_brdflut, 8);
  }
}

inline bool is_pressing(gui_button button) {
  return button.state == gui_button::state::pressing;
}
inline bool is_releasing(gui_button button) {
  return button.state == gui_button::state::releasing;
}
inline bool is_down(gui_button button) {
  return button.state == gui_button::state::down ||
         button.state == gui_button::state::pressing;
}
inline bool is_pressing(const gui_input &input, gui_key key) {
  return is_pressing(input.key_buttons[(int)key]);
}

bool push_disable_items(bool condition) {
  if (condition) {
    ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
  }

  return condition;
}

bool pop_disable_items(bool condition) {
  if (condition) {
    ImGui::PopItemFlag();
    ImGui::PopStyleVar();
  }

  return false;
}

void show_direction(App_base &app, const mesh_point &point, const vec3f &dir,
                    const float &angle, const string &name) {
  auto d = rotate_in_tangent_space(app.mesh.triangles, app.mesh.positions,
                                   point, dir, angle / 360 * (2 * pi));
  auto p = straightest_geodesic(app.mesh, point, d, 0.1F);

  update_glpolyline(
      app,
      trace_polyline_from_samples(p, app.mesh.triangles, app.mesh.positions),
      name);
}

bool process_camera_move(App &app, const gui_input &input) {

  auto rotating = input.modifier_shift;
  auto panning = input.modifier_alt;
  auto &camera = *app.ogl_camera;

  auto update_camera_frame = [&](frame3f &frame, float &focus, bool rotating,
                                 bool panning, bool zooming) {
    auto last_pos = input.mouse_last;
    auto mouse_pos = input.mouse_pos;
    auto mouse_left = is_down(input.mouse_left);
    auto mouse_right = is_down(input.mouse_right);
    // handle mouse and keyboard for navigation
    if (mouse_left) {
      auto dolly = 0.0f;
      auto pan = zero2f;
      auto rotate = zero2f;
      if (rotating) {
        if (mouse_left)
          rotate = (mouse_pos - last_pos) / 100.0f;
      }
      if (zooming) {
        if (mouse_right)
          dolly = (mouse_pos.y - last_pos.y) / 100.0f;
      }
      if (panning) {
        if (mouse_left)
          pan = (mouse_pos - last_pos) * focus / 200.0f;
      }
      pan.x = -pan.x;
      rotate.y = -rotate.y;
      update_turntable(frame, focus, rotate, dolly, pan);
    }
  };

  if (is_down(input.mouse_left) && (rotating || panning)) {
    update_camera_frame(camera.frame, app.camera_focus, rotating, panning,
                        false);
    return true;
  }

  // Zoom-in/out by scrolling;
  float zoom = input.scroll.y * 0.1;
  if (zoom != 0) {
    update_turntable(camera.frame, app.camera_focus, zero2f, zoom, zero2f);
    return true;
  }

  return false;
}
void process_key_input(App &app, const gui_input &input) {
  for (int i = 0; i < input.key_buttons.size(); i++) {
    auto key = gui_key(i);
    if (!is_pressing(input, key))
      continue;

    printf("%c pressed!\n", (char)key);
    if (key == gui_key('C') && app.state == app_state::EDITING) {
      app.input().cropping = true;
    }
    if (key == gui_key('R') && app.state == app_state::EDITING) {
      app.input().rotating = true;
    }
    if (key == gui_key('S') && app.state == app_state::EDITING) {
      app.input().scaling = true;
    }
    if (key == gui_key('T') && app.state == app_state::EDITING) {
      app.input().translating = true;
    }
    if (key == gui_key('D') && app.state == app_state::EDITING) {
      app.input().duplicating = true;
    }
    if (key == gui_key('Z') && app.state == app_state::EDITING) {
      app.input().deleting = true;
    }
  }
}
bool process_user_input(App &app, const gui_input &input) {

  if (is_active(app.widget))
    return false;
  if (process_camera_move(app, input)) {
    update_camera_info(app, input);
    return false;
  }
  if (app.scc.state == scc_state::SCC_NONE)
    return false;
  auto mouse = input.mouse_pos;
  auto size = vec2f{(float)input.window_size.x, (float)input.window_size.y};
  mouse = vec2f{2 * (mouse.x / size.x) - 1, 1 - 2 * (mouse.y / size.y)};
  auto editing = (app.state == app_state::EDITING);
  if (editing && is_pressing(input.mouse_left)) {
    intersect_circle_center(app, mouse);
  }

  auto point = intersect_mesh(app, mouse);
  if (is_releasing(input.mouse_left) && !input.modifier_shift &&
      !input.modifier_alt && !editing) {
    if (point.face != -1) {
      app.scc.points.push_back(point);
      update_scc(app);

      return true;
    }
  }
  if (is_releasing(input.mouse_right)) {
    if (app.input().rotating || app.input().scaling ||
        app.input().translating || app.input().cropping)
      app.scc.points.push_back(point);
    update_scc(app);
    return true;
  }
  auto drag = input.mouse_pos - input.mouse_last;
  if (editing && length(drag) > 0.05 && app.selected_circle != nullptr) {
    if (app.input().cropping) {
      point = intersect_circle_point(app, mouse);
      if (app.selected_point == nullptr)
        app.selected_point =
            add_points_shape(app, {point}, 2 * app.curve_size * 0.0001, zero3f);
      else
        update_points_shape(
            app.selected_point->instance->shape,
            {eval_position(app.mesh.triangles, app.mesh.positions, point)},
            2 * app.curve_size * 0.0001);

    } else if (app.input().rotating) {
      point = intersect_circle_point(app, mouse);
      auto center = app.selected_circle->center;
      auto path = compute_geodesic_path(app.mesh, center, point);
      auto t = tangent_path_direction(app.mesh, path);
      auto e = vec2f{1, 0};
      app.selected_circle->theta = angle(e, t);
      if (cross(e, t) < 0)
        app.selected_circle->theta = 2 * pif - app.selected_circle->theta;
    } else if (app.input().scaling && point.face != -1 &&
               !app.use_slider_for_circle) {
      set_radius(app.mesh, app.selected_circle,
                 get_distance(app.mesh.triangles[point.face],
                              get_bary(point.uv),
                              app.selected_circle->distances));
    } else if (app.input().translating && point.face != -1) {
      auto center = app.selected_circle->center;
      auto [is_vert, kv] = point_is_vert(center);
      auto path = compute_geodesic_path(app.mesh, center, point);
      vec3f v, n, e;
      if (is_vert) {
        auto vid = app.mesh.triangles[center.face][kv];

        e = polar_basis(app.mesh.solver, app.mesh.positions, app.mesh.normals,
                        vid);
        n = app.mesh.normals[vid];
        v = rot_vect(e, n, app.selected_circle->theta);
        parallel_transp(app.mesh.solver, app.mesh.angles, app.mesh.total_angles,
                        app.mesh.triangles, app.mesh.positions,
                        app.mesh.adjacencies, app.mesh.v2t, v, app.mesh.normals,
                        vid, path.start.face, V2T);
      } else {
        e = polar_basis(app.mesh.triangles, app.mesh.positions, center.face);
        n = tid_normal(app.mesh.triangles, app.mesh.positions, center.face);
        v = rot_vect(e, n, app.selected_circle->theta);
      }

      p_transp_along_path(app.mesh, path, v);
      // transp_vect_along_path(app.mesh, center, point, v);
      std::tie(is_vert, kv) = point_is_vert(point);
      if (is_vert) {
        auto vid = app.mesh.triangles[point.face][kv];
        app.selected_circle->theta =
            angle_in_tangent_space(app.mesh.triangles, app.mesh.positions,
                                   app.mesh.v2t, v, vid, app.mesh.normals[vid]);
        if (app.selected_circle->theta < 0)
          app.selected_circle->theta = 2 * pif - app.selected_circle->theta;
      } else {
        auto e =
            polar_basis(app.mesh.triangles, app.mesh.positions, point.face);
        app.selected_circle->theta = angle(e, v);
        if (dot(cross(e, v), n) < 0)
          app.selected_circle->theta = 2 * pif - app.selected_circle->theta;
      }

      set_center(app.mesh, app.selected_circle, point);
    }
    return true;
  }

  if (app.state == app_state::STRAIGHTEDGE && !editing) {
    if (app.scc.state == scc_state::CIRCLE) {
      if (app.scc.points.size() == 1 && point.face != -1) {
        auto circle = &app.scc.circles[app.curr_circle];

        set_radius(app.mesh, circle,
                   get_distance(app.mesh.triangles[point.face],
                                get_bary(point.uv), circle->distances),
                   point);
      }
    } else if (app.scc.state == scc_state::ELLIPSE &&
               app.scc.points.size() > 0) {
      if (app.scc.points.size() == 1) {
        if (validate_points({point}) &&
            !point_are_too_close(app.mesh, app.scc.points.back(), point))
          app.scc.paths[app.curr_path] =
              compute_geodesic_path(app.mesh, app.scc.points.back(), point);
      } else if (app.scc.points.size() == 2 && length(drag) > 0.1) {
        auto dist = interpolate_distances(
            app.mesh, app.scc.Ellipses[app.curr_circle].distances_from_midpoint,
            point);
        app.isoline =
            draw_ellipse(app.mesh, app.scc.paths[app.curr_path], dist);
      }
    } else if (app.scc.state == scc_state::TANGENT) {
      if (app.scc.points.size() == 1 &&
          app.scc.circles[app.curr_circle].radius == 0.0F) {
        auto circle = Circle(app.scc.circles[app.curr_circle]);
        set_radius(app.mesh, &circle,
                   get_distance(app.mesh.triangles[point.face],
                                get_bary(point.uv), circle.distances));

        // auto view = make_view_matrix(app.camera);
        // auto projection = make_projection_matrix(app.camera, win.size);

        app.isoline = circle.isoline;
      } else if (app.scc.points.size() == 1 &&
                 app.scc.circles[app.curr_circle].radius > 0) {
        point = intersect_circle_point(app, mouse);
        if (app.selected_point == nullptr)
          app.selected_point = add_points_shape(
              app, {point}, 2 * app.curve_size * 0.0001, zero3f);
        else
          update_points_shape(
              app.selected_point->instance->shape,
              {eval_position(app.mesh.triangles, app.mesh.positions, point)},
              2 * app.curve_size * 0.0001);
      }

    } else if (app.scc.state == scc_state::GEODESIC &&
               app.scc.points.size() > 0) {
      if (validate_points({point}) &&
          !point_are_too_close(app.mesh, app.scc.points.back(), point))
        app.scc.paths[app.curr_path] =
            compute_geodesic_path(app.mesh, app.scc.points.back(), point);
      // auto p = eval_path_point(app.scc.paths[app.curr_path],
      // app.mesh.triangles,
      //                          app.mesh.positions, app.mesh.adjacencies,
      //                          0.75);
      // if (app.selected_point == nullptr)
      //   app.selected_point =
      //       add_points_shape(app, {p}, 4 * app.curve_size * 0.0001, {1, 0,
      //       0});
      // else
      //   update_points_shape(
      //       app.selected_point->instance->shape,
      //       {eval_position(app.mesh.triangles, app.mesh.positions, p)},
      //       4 * app.curve_size * 0.0001);

    } else if (app.scc.state == scc_state::BISECTOR_ANGLE &&
               app.scc.points.size() > 0) {

      if (validate_points({point}) &&
          !point_are_too_close(app.mesh, app.scc.points.back(), point))
        app.scc.paths[app.curr_path] =
            compute_geodesic_path(app.mesh, app.scc.points.back(), point);
    } else if (app.scc.state == scc_state::PERPENDICULAR &&
               app.scc.points.size() > 0) {
      if (app.scc.points.size() == 1 && validate_points({point}) &&
          !point_are_too_close(app.mesh, app.scc.points.back(), point)) {
        app.scc.paths[app.curr_path] =
            compute_geodesic_path(app.mesh, app.scc.points.back(), point);
      } else if (app.scc.points.size() == 2) {
        point = intersect_geodesic_point(app, point);
        if (app.selected_point == nullptr)
          app.selected_point = add_points_shape(
              app, {point}, 2 * app.curve_size * 0.0001, {0, 1, 0});
        else
          update_points_shape(
              app.selected_point->instance->shape,
              {eval_position(app.mesh.triangles, app.mesh.positions, point)},
              2 * app.curve_size * 0.0001);
      }
    }
    return true;
  } else if (app.state == app_state::POLYGON) {
    if (app.use_exp_map_construction && app.scc.points.size() == 1) {
      if (validate_points({point}) &&
          !point_are_too_close(app.mesh, app.scc.points.back(), point)) {
        auto &circle = app.scc.circles[app.curr_circle];
        if (circle.distances.size() > 0)
          set_radius(app.mesh, &circle,
                     get_distance(app.mesh.triangles[point.face],
                                  get_bary(point.uv), circle.distances));
        else
          circle = create_circle(app.mesh, app.scc.points[0], point);
        return true;
      }
    } else if (app.scc.state == scc_state::RECTANGLE &&
               app.scc.points.size() > 0 && length(drag) > 0.1) {
      if (validate_points({point}) &&
          !point_are_too_close(app.mesh, app.scc.points.back(), point)) {
        if (app.scc.points.size() == 1)
          app.scc.paths[app.curr_path] =
              compute_geodesic_path(app.mesh, app.scc.points.back(), point);
        else if (app.scc.points.size() == 2) {
          if (app.type_of_rectangle == Diagonal)
            app.scc.paths[app.curr_path] =
                compute_geodesic_path(app.mesh, app.scc.points[0], point);
          else
            app.scc.paths[app.curr_path] =
                compute_geodesic_path(app.mesh, app.scc.points.back(), point);
        }
      }
      return true;
    } else if (app.scc.state == scc_state::TRIANGLE &&
               app.scc.points.size() > 0) {
      if (validate_points({point}) &&
          !point_are_too_close(app.mesh, app.scc.points.back(), point)) {
        if (app.scc.points.size() == 1) {
          if (!app.use_exp_map_construction)
            app.scc.paths[app.curr_path] =
                compute_geodesic_path(app.mesh, app.scc.points.back(), point);
        } else if (app.scc.points.size() == 2 && length(drag) > 0.1 &&
                   !app.use_exp_map_construction &&
                   app.type_of_triangle == Scalene) {

          // if (app.type_of_triangle == IsosceleBis) {
          //   point = intersect_isoline_point(app, point);
          //   if (app.selected_point == nullptr)
          //     app.selected_point = add_points_shape(
          //         app, {point}, 2 * app.curve_size * 0.0001, zero3f);
          //   else
          //     update_points_shape(app.selected_point->instance->shape,
          //                         {eval_position(app.mesh.triangles,
          //                                        app.mesh.positions,
          //                                        point)},
          //                         2 * app.curve_size * 0.0001);
          // }

          if (app.scc.result_shape == nullptr) {
            std::tie(app.scc.result_shape, app.result_entry) = add_result_shape(
                app, {app.scc.points[0], app.scc.points[1], point});
          } else
            update_generic_shape(app,
                                 {app.scc.points[0], app.scc.points[1], point},
                                 app.result_entry);
        }
      }
      return true;
    }
  } else if (app.state == app_state::MACROS) {
    if (app.scc.points.size() == 1 && validate_points({point}) &&
        !point_are_too_close(app.mesh, app.scc.points.back(), point)) {
      auto &circle = app.scc.circles[app.curr_circle];
      if (circle.distances.size() > 0)
        set_radius(app.mesh, &circle,
                   get_distance(app.mesh.triangles[point.face],
                                get_bary(point.uv), circle.distances));
      else
        circle = create_circle(app.mesh, app.scc.points[0], point);
      return true;
    }
  }

  return false;
};
void update_app(App &app, const gui_input &input) {
  // process_gui_input(app, input); TODO(giacomo)

  if (is_active(app.widget))
    return;

  app.window_size = input.window_size;
  // TODO(giacomo): make this optional, curve evaluation viz
  /*if (app.input().selected_spline != -1 && app.state.splines.size()) {
    double i = 0;
    auto   t = modf(input.time_now * 0.1, &i);
    if (app.spline().curves.size()) {
      auto segment = bezier_segment{
          app.spline().control_points[0],
          app.spline().control_points[1],
          app.spline().control_points[2],
          app.spline().control_points[3],
      };
      app.input().eval_point = eval_bezier_point(app.mesh, segment, t);
    }
  }*/
  process_key_input(app, input);
  process_user_input(app, input);
  draw_scc(app);
  auto tasks = vector<vec2i>{};
}
void draw_scene(const App &app, const vec4i &viewport) {
  clear_ogl_framebuffer(vec4f{0, 0, 0, 1});

  // Draw mesh and environment.
  draw_scene(app.scene, app.ogl_camera, viewport, app.shade_params);

  auto camera_aspect = (float)viewport.z / (float)viewport.w;
  auto camera_yfov =
      camera_aspect >= 0
          ? (2 * yocto::atan(app.ogl_camera->film /
                             (camera_aspect * 2 * app.ogl_camera->lens)))
          : (2 *
             yocto::atan(app.ogl_camera->film / (2 * app.ogl_camera->lens)));
  auto view = frame_to_mat(inverse(app.ogl_camera->frame));
  auto projection = perspective_mat(
      camera_yfov, camera_aspect, app.shade_params.near, app.shade_params.far);
  if (app.show_edges) {
    auto program = &app.ogl_shaders.at("lines");
    bind_program(program);
    set_common_uniforms(app, program);
    set_uniform(program, "color", vec3f{0, 0, 0});
    draw_shape(&app.edges_shape);
  }
}

void draw(const gui_input &input, void *data) {
  auto &app = *(App *)data;
  auto &polygon = app.polygon;
  auto &mesh = app.mesh;
  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;
  auto &adjacencies = mesh.adjacencies;
  auto &v2t = mesh.v2t;
  auto &[i0, i1] = app.intersections;
  auto &scc = app.scc;
  auto widget = app.widget;
  app.started = true;

  update_camera_info(app, input);

  // Do everything

  update_app(app, input);

  // Draw mesh, lines and stuff.
  draw_scene(app, input.framebuffer_viewport);
  begin_widget(widget, "Drawings");
  ImGui::PushItemWidth(ImGui::GetFontSize() * 8.0F);

  auto macro_id = ImGui::GetID("Macros");
  if (app.state != app_state::MACROS &&
      ImGui::GetStateStorage()->GetInt(macro_id)) {
    ImGui::GetStateStorage()->SetInt(macro_id, 0);
  }
  if (ImGui::TreeNode("Macros")) {
    app.state = app_state::MACROS;

    auto spid = ImGui::GetID("Spider Net");
    if (scc.state != scc_state::SPIDER_NET &&
        ImGui::GetStateStorage()->GetInt(spid)) {
      ImGui::GetStateStorage()->SetInt(spid, 0);
    }
    if (ImGui::TreeNode("Spider Net")) {
      scc.state = scc_state::SPIDER_NET;
      ImGui::TreePop();
    } else if (scc.state == scc_state::SPIDER_NET) {
      scc.state = scc_state::SCC_NONE;
    }
    auto crid = ImGui::GetID("Cross");
    if (scc.state != scc_state::CRUX &&
        ImGui::GetStateStorage()->GetInt(crid)) {
      ImGui::GetStateStorage()->SetInt(crid, 0);
    }
    if (ImGui::TreeNode("Cross")) {
      scc.state = scc_state::CRUX;
      ImGui::TreePop();
    } else if (scc.state == scc_state::CRUX) {
      scc.state = scc_state::SCC_NONE;
    }

    auto flid = ImGui::GetID("Flower");
    if (scc.state != scc_state::FLOWER &&
        ImGui::GetStateStorage()->GetInt(flid)) {
      ImGui::GetStateStorage()->SetInt(flid, 0);
    }
    if (ImGui::TreeNode("Flower")) {
      scc.state = scc_state::FLOWER;
      ImGui::TreePop();
    } else if (scc.state == scc_state::FLOWER) {
      scc.state = scc_state::SCC_NONE;
    }
    ImGui::TreePop();
  } else if (app.state == app_state::MACROS) {
    app.state = app_state::APP_NONE;
    info("app none");
  }

  auto p_id = ImGui::GetID("Polygon");

  if (app.state != app_state::POLYGON &&
      ImGui::GetStateStorage()->GetInt(p_id)) {
    app.selected_circle = nullptr;
    app.input().rotating = false;
    app.input().translating = false;
    app.input().scaling = false;
    app.input().duplicating = false;
    app.input().deleting = false;
    ImGui::GetStateStorage()->SetInt(p_id, 0);
  }

  if (ImGui::TreeNode("Polygon")) {
    app.state = app_state::POLYGON;
    draw_checkbox(widget, "Tangent Space Construction",
                  app.use_exp_map_construction);
    draw_checkbox(widget, "Garland", app.garland);
    // CIRCLES
    auto cid = ImGui::GetID("Circles");
    if (scc.state != scc_state::CIRCLE &&
        ImGui::GetStateStorage()->GetInt(cid)) {
      ImGui::GetStateStorage()->SetInt(cid, 0);
    }
    if (ImGui::TreeNode("Circles")) {
      scc.state = scc_state::CIRCLE;
      ImGui::TreePop();
    } else if (scc.state == scc_state::CIRCLE) {
      scc.state = scc_state::SCC_NONE;
    }
    auto id = ImGui::GetID("Triangle");
    if (scc.state != scc_state::TRIANGLE &&
        ImGui::GetStateStorage()->GetInt(id)) {
      // clear(scc);
      // app.curr_circle = app.curr_path = 0;
      ImGui::GetStateStorage()->SetInt(id, 0);
    }

    if (ImGui::TreeNode("Triangle")) {
      scc.state = scc_state::TRIANGLE;

      auto disabled = push_disable_items(scc.result_shape == nullptr);

      auto triangle_type = vector<string>{
          "Isoscele-SameLengths", "Isoscele-Bis", "Equilateral", "Scalene"};
      if (draw_combobox(widget, "Type", app.type_of_triangle, triangle_type)) {

        for (auto point : app.added_points) {
          clear_shape(point->instance->shape);
        }
        app.added_points.clear();
        for (auto path : app.added_paths) {
          clear_shape(path->instance->shape);
        }
        app.added_paths.clear();
        clear_shape(scc.result_shape->instance->shape);
        scc.result_shape = nullptr;
        app.curr_circle = 0;
        scc.circles = {Circle{}};
        scc.circles_to_shape.clear();
        scc.points.pop_back();
      }

      if (draw_checkbox(widget, "Flip Triangle", app.flip_triangle)) {
        for (auto point : app.added_points) {
          clear_shape(point->instance->shape);
        }
        app.added_points.clear();
        for (auto path : app.added_paths) {
          clear_shape(path->instance->shape);
        }
        app.added_paths.clear();
        clear_shape(scc.result_shape->instance->shape);
        scc.result_shape = nullptr;
        app.curr_circle = 0;
        scc.circles = {Circle{}};
        scc.circles_to_shape.clear();
        scc.points.pop_back();
      }
      disabled = pop_disable_items(disabled);
      ImGui::TreePop();
    } else if (scc.state == scc_state::TRIANGLE) {
      // clear(scc);
      // app.curr_circle = app.curr_path = 0;
      scc.state = scc_state::SCC_NONE;
    }
    auto sqid = ImGui::GetID("Square");
    if (scc.state != scc_state::SQUARE &&
        ImGui::GetStateStorage()->GetInt(sqid)) {
      // clear(scc);
      // app.curr_circle = app.curr_path = 0;
      ImGui::GetStateStorage()->SetInt(sqid, 0);
    }
    if (ImGui::TreeNode("Square")) {
      scc.state = scc_state::SQUARE;
      ImGui::TreePop();
    } else if (scc.state == scc_state::SQUARE) {
      // clear(scc);
      // app.curr_circle = app.curr_path = 0;
      scc.state = scc_state::SCC_NONE;
    }

    auto rhid = ImGui::GetID("Rhombus");
    if (scc.state != scc_state::RHOMBUS &&
        ImGui::GetStateStorage()->GetInt(rhid)) {
      // clear(scc);
      // app.curr_circle = app.curr_path = 0;
      ImGui::GetStateStorage()->SetInt(rhid, 0);
    }
    if (ImGui::TreeNode("Rhombus")) {
      scc.state = scc_state::RHOMBUS;
      ImGui::TreePop();
    } else if (scc.state == scc_state::RHOMBUS) {
      // clear(scc);
      // app.curr_circle = app.curr_path = 0;
      scc.state = scc_state::SCC_NONE;
    }

    auto rid = ImGui::GetID("Rectangle");
    if (scc.state != scc_state::RECTANGLE &&
        ImGui::GetStateStorage()->GetInt(rid)) {
      clear(scc);
      app.curr_circle = app.curr_path = 0;
      ImGui::GetStateStorage()->SetInt(rid, 0);
    }

    if (ImGui::TreeNode("Rectangle")) {
      scc.state = scc_state::RECTANGLE;

      auto rectangle_type =
          vector<string>{"Diagonal", "Euclidean", "Same Lenghts"};
      auto disabled = push_disable_items(scc.result_shape == nullptr);
      if (draw_combobox(widget, "Type", app.type_of_rectangle,
                        rectangle_type)) {
        for (auto point : app.added_points) {
          clear_shape(point->instance->shape);
        }
        app.added_points.clear();
        for (auto path : app.added_paths) {
          clear_shape(path->instance->shape);
        }
        app.added_paths.clear();
        clear_shape(scc.result_shape->instance->shape);
        scc.result_shape = nullptr;
        app.curr_circle = 0;
        scc.circles = {Circle{}};
        scc.circles_to_shape.clear();

        switch (app.type_of_rectangle) {
        case Euclidean: {
          auto height = path_length(
              compute_geodesic_path(app.mesh, scc.points.rbegin()[1],
                                    scc.points.back()),
              mesh.triangles, mesh.positions, mesh.adjacencies);
          auto [vertices, line, c0, c1, c2] = euclidean_rectangle(
              mesh,
              compute_geodesic_path(app.mesh, scc.points[0], scc.points.back()),
              height);
          scc.points = vertices;
          app.curr_color = vec3f{1, 0, 1};
          add_points_shape(app, {scc.points[0], scc.points.back()},
                           2 * app.curve_size * 0.0001, app.curr_color);
          std::tie(scc.circle_shape, app.selected_circle_entries) =
              add_isoline_shape(app, c0.isoline);
          scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
          scc.circles[app.curr_circle] = c0;
          scc.circles[app.curr_circle].primitive = cir;
          std::tie(scc.circle_shape, app.selected_circle_entries) =
              add_isoline_shape(app, c1.isoline);
          scc.circles_to_shape[app.curr_circle + 1] =
              app.selected_circle_entries;
          scc.circles.push_back(c1);
          scc.circles.back().primitive = cir;
          std::tie(scc.circle_shape, app.selected_circle_entries) =
              add_isoline_shape(app, c2.isoline);
          scc.circles_to_shape[app.curr_circle + 2] =
              app.selected_circle_entries;
          scc.circles.push_back(c2);
          scc.circles.back().primitive = cir;
          scc.circles.push_back(Circle());
          app.curr_circle += 3;
          app.curr_color = zero3f;
          add_path_shape(app, line, 0.00008 * app.curve_size, zero3f);
          scc.circles.push_back(Circle());
          app.curr_color = {0, 1, 0};

        } break;
        case SameLengths: {
          auto height = path_length(
              compute_geodesic_path(app.mesh, scc.points.rbegin()[1],
                                    scc.points.back()),
              mesh.triangles, mesh.positions, mesh.adjacencies);

          auto [vertices, line0, line1, c0, c1] = same_lengths_rectangle(
              mesh,
              compute_geodesic_path(app.mesh, scc.points[0], scc.points.back()),
              height);
          scc.points = vertices;
          app.curr_color = vec3f{1, 0, 1};
          add_points_shape(app, {scc.points[0], scc.points.back()},
                           2 * app.curve_size * 0.0001, app.curr_color);
          std::tie(scc.circle_shape, app.selected_circle_entries) =
              add_isoline_shape(app, c0.isoline);
          scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
          scc.circles[app.curr_circle] = c0;
          scc.circles[app.curr_circle].primitive = cir;
          std::tie(scc.circle_shape, app.selected_circle_entries) =
              add_isoline_shape(app, c1.isoline);
          scc.circles_to_shape[app.curr_circle + 1] =
              app.selected_circle_entries;
          scc.circles.push_back(c1);
          scc.circles.back().primitive = cir;

          scc.circles.push_back(Circle());
          app.curr_circle += 2;
          app.curr_color = zero3f;
          add_path_shape(app, line0, 0.00008 * app.curve_size, zero3f);
          add_path_shape(app, line1, 0.00008 * app.curve_size, zero3f);
          scc.circles.push_back(Circle());
        } break;
        case Diagonal: {
          auto [vertices, line0, line1, line2] = diagonal_rectangle(
              app.mesh, scc.paths.rbegin()[1], scc.paths.rbegin()[0]);
          auto len = path_length(scc.paths.rbegin()[1], mesh.triangles,
                                 mesh.positions, mesh.adjacencies);
          add_points_shape(app, {vertices[0], vertices.rbegin()[1]},
                           2 * app.curve_size * 0.0001, vec3f{1, 0, 1});
          add_path_shape(app, line0, 0.00008 * app.curve_size, zero3f);
          add_path_shape(app, line1, 0.00008 * app.curve_size, zero3f);
          add_path_shape(app, line2, 0.00008 * app.curve_size, zero3f);
          add_path_shape(app, scc.paths.back(), 0.00008 * app.curve_size,
                         zero3f);
          auto base = straightest_path(
              app.mesh.triangles, app.mesh.positions, app.mesh.adjacencies,
              vertices[0], tangent_path_direction(mesh, scc.paths.rbegin()[1]),
              1.5 * len);
          add_path_shape(app, base, 0.00008 * app.curve_size, zero3f);
          scc.points = vertices;
        } break;
        }
      }
      disabled = pop_disable_items(disabled);

      ImGui::TreePop();
    } else if (scc.state == scc_state::RECTANGLE) {
      clear(scc);
      app.curr_circle = app.curr_path = 0;
      scc.state = scc_state::SCC_NONE;
    }
    auto parapid = ImGui::GetID("Parallelogram");
    if (scc.state != scc_state::PARALLELOGRAM &&
        ImGui::GetStateStorage()->GetInt(parapid)) {
      // clear(scc);
      // app.curr_circle = app.curr_path = 0;
      ImGui::GetStateStorage()->SetInt(parapid, 0);
    }
    if (ImGui::TreeNode("Parallelogram")) {
      scc.state = scc_state::PARALLELOGRAM;
      ImGui::TreePop();
    } else if (scc.state == scc_state::PARALLELOGRAM) {
      // clear(scc);
      // app.curr_circle = app.curr_path = 0;
      scc.state = scc_state::SCC_NONE;
    }

    auto pentaid = ImGui::GetID("Pentagon");
    if (scc.state != scc_state::PENTAGON &&
        ImGui::GetStateStorage()->GetInt(pentaid)) {
      // clear(scc);
      // app.curr_circle = app.curr_path = 0;
      ImGui::GetStateStorage()->SetInt(pentaid, 0);
    }
    if (ImGui::TreeNode("Pentagon")) {
      scc.state = scc_state::PENTAGON;
      ImGui::TreePop();
    } else if (scc.state == scc_state::PENTAGON) {
      // clear(scc);
      // app.curr_circle = app.curr_path = 0;
      scc.state = scc_state::SCC_NONE;
    }

    auto hexid = ImGui::GetID("Hexagon");
    if (scc.state != scc_state::HEXAGON &&
        ImGui::GetStateStorage()->GetInt(hexid)) {
      // clear(scc);
      // app.curr_circle = app.curr_path = 0;
      ImGui::GetStateStorage()->SetInt(hexid, 0);
    }
    if (ImGui::TreeNode("Hexagon")) {
      scc.state = scc_state::HEXAGON;
      ImGui::TreePop();
    } else if (scc.state == scc_state::HEXAGON) {
      // clear(scc);
      // app.curr_circle = app.curr_path = 0;
      scc.state = scc_state::SCC_NONE;
    }
    auto octid = ImGui::GetID("Octagon");
    if (scc.state != scc_state::OCTAGON &&
        ImGui::GetStateStorage()->GetInt(octid)) {
      // clear(scc);
      // app.curr_circle = app.curr_path = 0;
      ImGui::GetStateStorage()->SetInt(octid, 0);
    }
    if (ImGui::TreeNode("Octagon")) {
      scc.state = scc_state::OCTAGON;
      ImGui::TreePop();
    } else if (scc.state == scc_state::OCTAGON) {
      // clear(scc);
      // app.curr_circle = app.curr_path = 0;
      scc.state = scc_state::SCC_NONE;
    }
    auto decid = ImGui::GetID("Decagon");
    if (scc.state != scc_state::DECAGON &&
        ImGui::GetStateStorage()->GetInt(decid)) {
      // clear(scc);
      // app.curr_circle = app.curr_path = 0;
      ImGui::GetStateStorage()->SetInt(decid, 0);
    }
    if (ImGui::TreeNode("Decagon")) {
      scc.state = scc_state::DECAGON;
      ImGui::TreePop();
    } else if (scc.state == scc_state::DECAGON) {
      // clear(scc);
      // app.curr_circle = app.curr_path = 0;
      scc.state = scc_state::SCC_NONE;
    }

    ImGui::TreePop();
  } else if (app.state == app_state::POLYGON) {
    app.state = app_state::APP_NONE;
    info("app none");
  }

  auto s_id = ImGui::GetID("Straightedge and compass");

  if (app.state != app_state::STRAIGHTEDGE &&
      ImGui::GetStateStorage()->GetInt(s_id)) {
    app.selected_circle = nullptr;
    app.input().rotating = false;
    app.input().translating = false;
    app.input().scaling = false;
    app.input().duplicating = false;
    app.input().deleting = false;
    ImGui::GetStateStorage()->SetInt(s_id, 0);
  }

  if (ImGui::TreeNode("Straightedge and compass")) {
    app.state = app_state::STRAIGHTEDGE;

    static auto disabled = false;
    // GEODESIC
    auto id = ImGui::GetID("Geodesic");
    if (scc.state != scc_state::GEODESIC &&
        ImGui::GetStateStorage()->GetInt(id)) {
      ImGui::GetStateStorage()->SetInt(id, 0);
    }

    if (ImGui::TreeNode("Geodesic")) {
      scc.state = scc_state::GEODESIC;
      draw_checkbox(widget, "Straightest", app.straighthest);
      disabled =
          push_disable_items(!app.straighthest || app.curr_path == 0 ||
                             scc.paths[app.curr_path - 1].start.face == -1);
      if (draw_slider(widget, "Length", app.length_of_straightest, 0.0001, 5)) {
        auto v = tangent_path_direction(app.mesh, scc.paths[app.curr_path - 1]);
        scc.paths[app.curr_path - 1] = straightest_path(
            app.mesh.triangles, app.mesh.positions, app.mesh.adjacencies,
            scc.paths[app.curr_path - 1].start, v, app.length_of_straightest);
        update_path_shape(
            app.added_paths, app.mesh,
            path_positions(scc.paths[app.curr_path - 1], app.mesh),
            app.curve_size * 0.00010, app.added_paths.size() - 1);
      }
      disabled = pop_disable_items(disabled);
      ImGui::TreePop();
    } else if (scc.state == scc_state::GEODESIC) {

      scc.state = scc_state::SCC_NONE;
    }
    // ELLIPSE
    id = ImGui::GetID("Ellipse");
    if (scc.state != scc_state::ELLIPSE &&
        ImGui::GetStateStorage()->GetInt(id)) {
      clear(scc);
      app.curr_circle = app.curr_path = 0;
      ImGui::GetStateStorage()->SetInt(id, 0);
    }
    if (ImGui::TreeNode("Ellipse")) {
      scc.state = scc_state::ELLIPSE;
      ImGui::TreePop();
    } else if (scc.state == scc_state::ELLIPSE) {
      clear(scc);
      app.curr_circle = app.curr_path = 0;
      scc.state = scc_state::SCC_NONE;
    }
    // INTERSECT
    id = ImGui::GetID("Intersect");
    if (scc.state != scc_state::INTERSECT &&
        ImGui::GetStateStorage()->GetInt(id)) {
      clear(scc);
      app.curr_circle = app.curr_path = 0;
      ImGui::GetStateStorage()->SetInt(id, 0);
    }
    if (ImGui::TreeNode("Intersect")) {
      scc.state = scc_state::INTERSECT;
      if (ImGui::Button("Compute")) {

        auto c0 = mesh_point{532620, vec2f{0.201439455, 0.297493041}};
        // auto c1 = mesh_point{42584, vec2f{0.490420878, 0.483200759}};
        // auto m0 = mesh_point{44195, vec2f{0.326937556, 0.359278232}};
        auto radius = 0.438741982;
        auto c_curr = create_circle(app.mesh, c0, radius);
        app.scc.circles[app.curr_circle] = c_curr;
        ++app.curr_circle;
        app.scc.circles.push_back(Circle{});
        auto tid_c =
            tid_centroid(app.mesh.triangles, app.mesh.positions, 674455);
        auto sec_tid_c =
            tid_centroid(app.mesh.triangles, app.mesh.positions, 677586);
        // auto c_next = create_circle(app.mesh, c1, radius);
        // auto [w0, w1] = intersect(app.mesh, c_curr, c_next);
        // mesh_point s1 = {44196, vec2f{0, 0.803679108}};
        // mesh_point e1 = {44196, vec2f{0.138437271, 0.861562729}};

        // mesh_point s2 = {44196, vec2f{0, 0.0614414811}};
        // mesh_point e2 = {44196, vec2f{0.76080358, 0}};
        // add_points_shape(app, {w0, w1}, 0.000006 * app.curve_size, {0, 1,
        // 0});

        add_circle_shape(app, c_curr);
        // add_circle_shape(app, c_next);
        add_points_shape(app, {tid_c, sec_tid_c}, 0.000006 * app.curve_size,
                         {0, 1, 0});
        // add_points_shape(app, {m1}, 0.00006 * app.curve_size, {1, 0, 0});
        // auto f = cropping_range_generic_points(mesh, c, v);
        // auto new_c = find_center_for_spider_net(app.mesh, m0, m1, &c,
        // radius);
        // auto radii = subdivide_radius(radius, 3);
        // auto tetas = subdivide_angles(16);
        // auto [is_vert, kv] = point_is_vert(center);
        // vec3f n, e = zero3f;
        // if (is_vert) {
        //   auto vid = mesh.triangles[center.face][kv];
        //   n = mesh.normals[vid];
        //   e = polar_basis(mesh.solver, mesh.positions, mesh.normals, vid);

        // } else {
        //   e = polar_basis(mesh.triangles, mesh.positions, center.face);
        //   n = tid_normal(mesh.triangles, mesh.positions, center.face);
        // }

        // auto gamma = straightest_geodesic(app.mesh, center,
        //                                   rot_vect(e, n, tetas[1]),
        //                                   radius);
        // // auto middle = eval_point(mesh, gamma, 0.8 * radii[0] / radius);
        // auto p = vec3f{-0.110754222, -0.0129426122, 0.486753523};
        // auto q = vec3f{-0.111215852, -0.0145259351, 0.48649472};
        // auto initial = vec3f{-0.111092918, -0.0141042816, 0.486563653};

        // auto scale_factors = vector<float>{1.25, 1.5, 1.75};

        // auto gamma1 = straightest_geodesic(
        //     app.mesh, center, rot_vect(e, n, tetas[3]), 2 * radius);
        // auto curr_circle =
        //     create_circle(app.mesh, gamma1.back(), radius *
        //     scale_factors[0]);
        // auto pos_gamma = vector<vec3f>(gamma.size());
        // auto pos_gamma1 = vector<vec3f>(gamma1.size());
        // for (auto i = 0; i < gamma.size(); ++i) {
        //   pos_gamma[i] =
        //       eval_position(app.mesh.triangles, app.mesh.positions,
        //       gamma[i]);
        // }
        // for (auto i = 0; i < gamma1.size(); ++i) {
        //   pos_gamma1[i] =
        //       eval_position(app.mesh.triangles, app.mesh.positions,
        //       gamma1[i]);
        // }
        // add_path_shape(app, pos_gamma, 0.00003 * app.curve_size, zero3f);
        // add_path_shape(app, pos_gamma1, 0.000003 * app.curve_size, zero3f);
        // add_path_shape(app,
        //                circle_positions(app.mesh.triangles,
        //                app.mesh.positions,
        //                                 app.mesh.adjacencies, curr_circle),
        //                0.000003 * app.curve_size, {1, 0, 0});

        // auto [i0, i1] = intersect(app.mesh, gamma,
        // curr_circle.isoline); if (i0.face != -1)
        // add_points_shape(app, {tid_c, sec_tid_c}, 0.00006 *
        // app.curve_size,
        //{0, 1, 1});
        // if (i1.face != -1)
        //   add_points_shape(app, {i1}, 0.000006 *
        //   app.curve_size, {1, 1, 0});
        // auto points = intersect(app.mesh, path,
        // app.isoline);
        //   switch (app.type_of_intersection) {
        //   case TwoCurves: {
        //     if (app.scc.paths.size() < 3)
        //       std::cout << "Please draw two paths to compute
        //       intersection"
        //                 << std::endl;
        //     else {
        //       app.scc.points = {intersect(app.mesh,
        //       app.scc.paths.rbegin()[1],
        //                                   app.scc.paths.rbegin()[2])};
        //     }
        //   } break;
        //   case TwoCircles: {
        //     if (app.scc.circles.size() < 3)
        //       std::cout << "Please draw two paths to compute
        //       intersection"
        //                 << std::endl;
        //     else {
        //       auto points = intersect(app.mesh,
        //       app.scc.circles.rbegin()[1],
        //                               app.scc.circles.rbegin()[2]);
        //       app.scc.points.clear();
        //       if (validate_points({points.first}))
        //         app.scc.points.push_back(points.first);
        //       if (validate_points({points.second}))
        //         app.scc.points.push_back(points.second);
        //     }
        //   } break;
        //   case Curve_Circle: {
        //     if (app.scc.circles.size() == 1 ||
        //     app.scc.paths.size() == 1)
        //       std::cout
        //           << "Please draw a circle and a paths to
        //           compute intersection"
        //           << std::endl;
        //     else {
        //       app.scc.points.clear();
        //       auto points = intersect(app.mesh,
        //       app.scc.paths.rbegin()[1],
        //                               app.scc.circles.rbegin()[1]);
        //       if (validate_points({points.first}))
        //         app.scc.points.push_back(points.first);
        //       if (validate_points({points.second}))
        //         app.scc.points.push_back(points.second);
        //     }
        //   } break;
        //   }
        //
      }
      disabled = pop_disable_items(disabled);

      ImGui::TreePop();
    } else if (scc.state == scc_state::INTERSECT) {
      clear(scc);
      app.curr_circle = app.curr_path = 0;
      scc.state = scc_state::SCC_NONE;
    }
    // BISECTOR
    id = ImGui::GetID("Bisector of segment");

    if (scc.state != scc_state::BISECTOR &&
        ImGui::GetStateStorage()->GetInt(id)) {
      clear(scc);
      app.curr_circle = app.curr_path = 0;
      ImGui::GetStateStorage()->SetInt(id, 0);
    }

    if (ImGui::TreeNode("Bisector of segment")) {
      scc.state = scc_state::BISECTOR;

      disabled = push_disable_items(scc.paths.size() == 1);
      ImGui::Text("Create geodesic");
      disabled = pop_disable_items(disabled);

      disabled = push_disable_items(scc.paths.size() < 1);
      if (ImGui::Button("Caculate")) {
        auto &path = scc.paths[app.curr_path];
        switch (app.solution) {
        case Greek: {
          app.curr_color = vec3f{1, 0, 1};
          auto [result, c0, c1] =
              find_segment_bisector(mesh, scc.paths[app.curr_path]);
          std::tie(scc.circle_shape, app.selected_circle_entries) =
              add_isoline_shape(app, c0.isoline);
          scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
          scc.circles[app.curr_circle] = c0;
          ++app.curr_circle;
          std::tie(scc.circle_shape, app.selected_circle_entries) =
              add_isoline_shape(app, c1.isoline);
          scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
          scc.circles.push_back(c1);
          ++app.curr_circle;
          scc.circles.push_back(Circle{});
          app.curr_color = zero3f;
          if (!set_result(triangles, positions, adjacencies, scc, result)) {
            info("Construction not found");
          }
          auto x = intersect(app.mesh, result, path);
          add_points_shape(app, {x}, 2 * app.curve_size * 0.0001,
                           vec3f{0, 1, 0});
        } break;
        case Intermediate: {
          app.isoline =
              find_segment_bisector_isolines(mesh, scc.paths[app.curr_path]);
          auto x = intersect(app.mesh, path, app.isoline);
          add_points_shape(app, {x.first}, 2 * app.curve_size * 0.0001,
                           vec3f{0, 1, 0});
        } break;
        case Expected: {
          auto iso =
              find_segment_bisector_isolines(mesh, scc.paths[app.curr_path]);
          auto midpoint = intersect(app.mesh, scc.paths[app.curr_path], iso);
          add_points_shape(app, {midpoint.first}, 2 * app.curve_size * 0.0001,
                           vec3f{0, 1, 0});
          if (!set_expected(triangles, positions, adjacencies, scc,
                            expected_perpendicular(mesh,
                                                   scc.paths[app.curr_path],
                                                   midpoint.first))) {
            info("Expected not found");
          }

        } break;
        }
      }
      disabled = pop_disable_items(disabled);
      auto solution_names = vector<string>{"Greek", "Intermediate", "Expected"};
      if (draw_combobox(widget, "Solution", app.solution, solution_names)) {
        if (scc.paths.back().start.face != -1) {
          auto a = scc.paths[0].start;
          auto b = scc.paths[0].end;
          clear_scc(app);
          auto path = compute_geodesic_path(app.mesh, a, b);
          app.scc.paths = {path};
          add_path_shape(app, path, app.curve_size * 0.00010, vec3f{0, 0, 0});
          add_points_shape(app, {a, b}, 2 * app.curve_size * 0.00010,
                           vec3f{0, 0, 0});
          switch (app.solution) {
          case Greek: {
            auto [result, c0, c1] = find_segment_bisector(mesh, scc.paths[0]);
            auto entries = app.scc.circles_to_shape.at(app.curr_circle - 2);
            update_isoline_shape(app, c0.isoline, entries);
            entries = app.scc.circles_to_shape.at(app.curr_circle - 1);
            update_isoline_shape(app, c1.isoline, entries);
            entries = app.scc.circles_to_shape.at(app.curr_circle - 1);
            scc.circles[app.curr_circle - 2] = c0;
            scc.circles[app.curr_circle - 1] = c1;
            if (!set_result(triangles, positions, adjacencies, scc, result)) {
              info("Construction not found");
            }

            auto x = intersect(app.mesh, result, path);
            add_points_shape(app, {x}, 2 * app.curve_size * 0.0001,
                             vec3f{0, 1, 0});

          } break;
          case Intermediate: {
            app.isoline = find_segment_bisector_isolines(mesh, scc.paths[0]);
            auto x = intersect(app.mesh, path, app.isoline);
            add_points_shape(app, {x.first}, 2 * app.curve_size * 0.0001,
                             vec3f{0, 1, 0});
          } break;
          case Expected: {
            auto iso = find_segment_bisector_isolines(mesh, scc.paths[0]);
            auto midpoint = intersect(app.mesh, scc.paths[0], iso);
            add_points_shape(app, {midpoint.first}, 2 * app.curve_size * 0.0001,
                             vec3f{0, 1, 0});
            if (!set_expected(triangles, positions, adjacencies, scc,
                              expected_perpendicular(mesh, scc.paths[0],
                                                     midpoint.first))) {
              info("Expected not found");
            }
          } break;
          }
        }
      }
      ImGui::TreePop();
    } else if (scc.state == scc_state::BISECTOR) {
      clear(scc);
      app.curr_circle = app.curr_path = 0;
      scc.state = scc_state::SCC_NONE;
    }

    // MID POINT
    id = ImGui::GetID("Midpoint of a segment");

    if (scc.state != scc_state::MIDPOINT &&
        ImGui::GetStateStorage()->GetInt(id)) {
      clear(scc);
      app.curr_circle = app.curr_path = 0;
      ImGui::GetStateStorage()->SetInt(id, 0);
    }

    if (ImGui::TreeNode("Midpoint of a segment")) {
      scc.state = scc_state::MIDPOINT;

      disabled = push_disable_items(scc.paths.size() == 1);
      ImGui::Text("Create geodesic");
      disabled = pop_disable_items(disabled);

      disabled = push_disable_items(scc.paths.size() < 1);
      if (ImGui::Button("Caculate")) {
        if (!set_expected(triangles, positions, scc,
                          expected_midpoint(mesh, scc.paths[0]))) {
          info("Expected not found");
        }
        if (!set_result(triangles, positions, scc,
                        find_midpoint(mesh, scc.paths[0]))) {
          info("Construction not found");
        }
      }
      disabled = pop_disable_items(disabled);

      ImGui::TreePop();
    } else if (scc.state == scc_state::MIDPOINT) {
      clear(scc);
      app.curr_path = app.curr_circle = 0;
      scc.state = scc_state::SCC_NONE;
    }

    // PERPENDICULAR
    id = ImGui::GetID("Perpendicular from point");

    if (scc.state != scc_state::PERPENDICULAR &&
        ImGui::GetStateStorage()->GetInt(id)) {
      clear(scc);
      app.curr_circle = app.curr_path = 0;
      ImGui::GetStateStorage()->SetInt(id, 0);
    }

    if (ImGui::TreeNode("Perpendicular from point")) {
      scc.state = scc_state::PERPENDICULAR;

      auto disabled = push_disable_items(scc.geodesic_shape == nullptr &&
                                         app.scc.points.size() != 3);
      if (ImGui::Button("Caculate")) {
        if (!set_result(triangles, positions, adjacencies, scc,
                        expected_perpendicular(mesh, scc.paths[0],
                                               scc.points[2], 0.1))) {
          info("Construction not found");
        }
      }
      disabled = pop_disable_items(disabled);

      ImGui::TreePop();
    } else if (scc.state == scc_state::PERPENDICULAR) {
      clear(scc);
      app.curr_circle = app.curr_path = 0;
      scc.state = scc_state::SCC_NONE;
    }

    // BISECTOR_ANGLE
    id = ImGui::GetID("Angle bisector");

    if (scc.state != scc_state::BISECTOR_ANGLE &&
        ImGui::GetStateStorage()->GetInt(id)) {
      clear(scc);
      app.curr_circle = app.curr_path = 0;
      ImGui::GetStateStorage()->SetInt(id, 0);
    }

    if (ImGui::TreeNode("Angle bisector")) {
      scc.state = scc_state::BISECTOR_ANGLE;

      disabled = push_disable_items(app.curr_path < 2);
      if (draw_slider(widget, "Circle Radius",
                      app.scale_factor_circle_for_angle_bisector, 0.01, 0.99)) {
        auto &path0 = app.scc.paths[app.curr_path - 2];
        auto &path1 = app.scc.paths[app.curr_path - 1];
        scc.result_pos.clear();
        scc.points.clear();
        auto [result, s, c0, c1] = find_angle_bisector(
            mesh, path0, path1, app.scale_factor_circle_for_angle_bisector);
        auto pos = path_positions(result, app.mesh);
        auto entries = app.scc.circles_to_shape.at(app.curr_circle - 3);
        update_isoline_shape(app, s.isoline, entries);
        entries = app.scc.circles_to_shape.at(app.curr_circle - 2);
        update_isoline_shape(app, c0.isoline, entries);
        entries = app.scc.circles_to_shape.at(app.curr_circle - 1);
        update_isoline_shape(app, c1.isoline, entries);
        update_path_shape(scc.output_shape[0]->instance->shape, app.mesh, pos,
                          app.curve_size * 0.0001);
        scc.circles[app.curr_circle - 3] = s;
        scc.circles[app.curr_circle - 2] = c0;
        scc.circles[app.curr_circle - 1] = c1;
        update_points_shape(app.result_point->instance->shape, {pos.back()},
                            2 * app.curve_size * 0.0001);
      }

      if (ImGui::Button("Caculate")) {
        auto &path0 = app.scc.paths[app.curr_path - 2];
        auto &path1 = app.scc.paths[app.curr_path - 1];

        switch (app.solution) {
        case Greek: {
          auto [result, s, c0, c1] = find_angle_bisector(
              mesh, path0, path1, app.scale_factor_circle_for_angle_bisector);
          if (!set_result(triangles, positions, adjacencies, scc, result)) {
            info("Result not found");
          } else {
            app.curr_color = vec3f{1, 0, 1};
            scc.circles[app.curr_circle] = s;
            std::tie(scc.circle_shape, app.selected_circle_entries) =
                add_isoline_shape(app, s.isoline);
            scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
            ++app.curr_circle;
            scc.circles.push_back(c0);
            std::tie(scc.circle_shape, app.selected_circle_entries) =
                add_isoline_shape(app, c0.isoline);
            scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
            ++app.curr_circle;
            scc.circles.push_back(c1);
            std::tie(scc.circle_shape, app.selected_circle_entries) =
                add_isoline_shape(app, c1.isoline);
            scc.circles_to_shape[app.curr_circle] = app.selected_circle_entries;
            ++app.curr_circle;
            scc.circles.push_back(Circle());
          }
        }

        break;
        case Intermediate: {
          if (!set_expected(triangles, positions, adjacencies, scc,
                            expected_angle_bisector(mesh, path0, path1))) {
            info("Expected not found");
          }
          break;
        }
        }
      }
      disabled = pop_disable_items(disabled);
      auto solution_names = vector<string>{"Greek", "Intermediate"};
      if (draw_combobox(widget, "Solution", app.solution, solution_names)) {
        if (app.curr_path >= 2) {
          auto path0 = app.scc.paths[app.curr_path - 2];
          auto path1 = app.scc.paths[app.curr_path - 1];
          clear_scc(app);
          add_path_shape(app, path0, app.curve_size * 0.0001, zero3f);
          add_path_shape(app, path1, app.curve_size * 0.0001, zero3f);
          add_points_shape(app, {path0.start, path0.end, path1.end},
                           2 * app.curve_size * 0.0001, zero3f);
          switch (app.solution) {
          case Greek: {
            auto [result, s, c0, c1] = find_angle_bisector(mesh, path0, path1);
            if (!set_result(triangles, positions, adjacencies, scc, result)) {
              info("Result not found");
            } else {
              scc.construction_shape = {
                  add_path_shape(app,
                                 circle_positions(mesh.triangles,
                                                  mesh.positions,
                                                  mesh.adjacencies, c0),
                                 app.curve_size * 0.0001, vec3f{1, 0, 1}),
                  add_path_shape(app,
                                 circle_positions(mesh.triangles,
                                                  mesh.positions,
                                                  mesh.adjacencies, c1),
                                 app.curve_size * 0.0001, vec3f{1, 0, 1})};
            }
          }

          break;
          case Intermediate: {
            if (!set_expected(triangles, positions, adjacencies, scc,
                              expected_angle_bisector(mesh, path0, path1))) {
              info("Expected not found");
            }
            break;
          }
          }
        }
      }
      ImGui::TreePop();
    } else if (scc.state == scc_state::BISECTOR_ANGLE) {
      clear_scc(app);
      app.curr_circle = app.curr_path = 0;
      scc.state = scc_state::SCC_NONE;
    }

    // MIRROR
    id = ImGui::GetID("Mirror a point");

    if (scc.state != scc_state::MIRROR &&
        ImGui::GetStateStorage()->GetInt(id)) {
      clear(scc);
      app.curr_circle = app.curr_path = 0;
      ImGui::GetStateStorage()->SetInt(id, 0);
    }

    if (ImGui::TreeNode("Mirror a point")) {
      scc.state = scc_state::MIRROR;

      disabled = push_disable_items(scc.paths.size() > 0);
      ImGui::Text("Create geodesic");
      disabled = pop_disable_items(disabled);

      disabled = push_disable_items(scc.points.size() != 2);
      ImGui::Text("Select point");
      disabled = pop_disable_items(disabled);

      disabled =
          push_disable_items(scc.paths.size() < 1 && scc.points.size() < 3);
      if (ImGui::Button("Caculate")) {
        if (!set_expected(
                triangles, positions, scc,
                expected_mirror_point(mesh, scc.paths[0], scc.points[2]))) {
          info("Expected not found");
        }
        if (!set_result(triangles, positions, scc,
                        find_mirror_point(mesh, scc.paths[0], scc.points[2]))) {
          info("Construction not found");
        }
      }
      disabled = pop_disable_items(disabled);

      ImGui::TreePop();
    } else if (scc.state == scc_state::MIRROR) {
      clear(scc);
      app.curr_circle = app.curr_path = 0;
      scc.state = scc_state::SCC_NONE;
    }

    // TANGENT
    id = ImGui::GetID("Tangent to circle");

    if (scc.state != scc_state::TANGENT &&
        ImGui::GetStateStorage()->GetInt(id)) {
      clear(scc);
      app.curr_circle = app.curr_path = 0;
      ImGui::GetStateStorage()->SetInt(id, 0);
    }

    if (ImGui::TreeNode("Tangent to circle")) {
      scc.state = scc_state::TANGENT;

      // auto disabled = push_disable_items(scc.points.size() != 2);
      if (ImGui::Button("Caculate")) {
        auto sol = expected_tangent_to_circle(
            mesh, scc.circles[app.curr_circle], scc.points[1]);
        if (!set_expected(triangles, positions, adjacencies, scc, sol)) {
          info("Expected not found");
        }
      }
      // disabled = pop_disable_items(disabled);

      ImGui::TreePop();
    } else if (scc.state == scc_state::TANGENT) {
      clear(scc);
      app.curr_circle = app.curr_path = 0;
      scc.state = scc_state::SCC_NONE;
    }

    // CIRCLE_3
    id = ImGui::GetID("Circle through 3 points");

    if (scc.state != scc_state::CIRCLE_3 &&
        ImGui::GetStateStorage()->GetInt(id)) {
      clear(scc);
      app.curr_circle = app.curr_path = 0;
      ImGui::GetStateStorage()->SetInt(id, 0);
    }

    if (ImGui::TreeNode("Circle through 3 points")) {
      scc.state = scc_state::CIRCLE_3;

      disabled = push_disable_items(scc.points.size() != 3);
      if (ImGui::Button("Caculate")) {
        scc.paths[app.curr_path] =
            compute_geodesic_path(app.mesh, scc.points[0], scc.points[1]);
        add_path_shape(app, scc.paths[app.curr_path], app.curve_size * 0.0001,
                       zero3f);
        ++app.curr_path;
        scc.paths.push_back(
            compute_geodesic_path(app.mesh, scc.points[1], scc.points[2]));
        add_path_shape(app, scc.paths.back(), app.curve_size * 0.0001, zero3f);
        ++app.curr_path;
        if (app.solution == Greek) {
          auto [result, bisec0, bisec1] = find_circle_3_points(
              mesh, scc.points[0], scc.points[1], scc.points[2]);

          if (!set_result(triangles, positions, adjacencies, scc, result)) {
            info("Construction not found");
            return;
          }

          scc.paths.push_back(bisec0);
          add_path_shape(app, bisec0, app.curve_size * 0.00008, {0, 0, 0});
          ++app.curr_path;
          scc.paths.push_back(bisec1);
          add_path_shape(app, bisec1, app.curve_size * 0.00008, {0, 0, 0});
          ++app.curr_path;
          scc.paths.push_back(geodesic_path{});
          add_points_shape(app, {result.center}, app.curve_size * 0.0003,
                           {0, 1, 0});
        } else {
          auto [result, iso0, iso1] = expected_circle_3_points(
              mesh, scc.points[0], scc.points[1], scc.points[2]);
          if (!set_expected(triangles, positions, adjacencies, scc, result)) {
            info("Expected not found");
            return;
          }
          add_isoline_shape(app, iso0);
          add_isoline_shape(app, iso1);
          clear_shape(app.added_points.back()->instance->shape);
          add_points_shape(app, {result.center}, app.curve_size * 0.0003,
                           {0, 1, 0});
        }
      }
      disabled = pop_disable_items(disabled);
      auto solution_names = vector<string>{"Greek", "Intermediate"};
      if (draw_combobox(widget, "Solution", app.solution, solution_names)) {
        if (app.scc.points.size() == 3) {
          auto a = app.scc.points.rbegin()[0];
          auto b = app.scc.points.rbegin()[1];
          auto c = app.scc.points.rbegin()[2];
          clear_scc(app);
          scc.points = {a, b, c};
          add_points_shape(app, {a, b, c}, 2 * app.curve_size * 0.0001,
                           vec3f{1, 0, 1});
          scc.paths[app.curr_path] = compute_geodesic_path(app.mesh, a, b);
          add_path_shape(app, scc.paths[app.curr_path], app.curve_size * 0.0001,
                         zero3f);
          ++app.curr_path;
          scc.paths.push_back(compute_geodesic_path(app.mesh, b, c));
          add_path_shape(app, scc.paths.back(), app.curve_size * 0.0001,
                         zero3f);
          ++app.curr_path;
          if (app.solution == Greek) {
            auto [result, bisec0, bisec1] = find_circle_3_points(mesh, a, b, c);
            if (!set_result(triangles, positions, adjacencies, scc, result)) {
              info("Construction not found");
              return;
            }

            scc.paths.push_back(bisec0);
            add_path_shape(app, bisec0, app.curve_size * 0.00008, {0, 1, 0});
            ++app.curr_path;
            scc.paths.push_back(bisec1);
            add_path_shape(app, bisec1, app.curve_size * 0.00008, {0, 1, 0});
            ++app.curr_path;
            scc.paths.push_back(geodesic_path{});
          } else {
            auto [result, iso0, iso1] = expected_circle_3_points(mesh, a, b, c);
            if (!set_expected(triangles, positions, adjacencies, scc, result)) {
              info("Expected not found");
              return;
            }
            add_isoline_shape(app, iso0);
            add_isoline_shape(app, iso1);
            // clear_shape(app.added_points.back()->instance->shape);
            add_points_shape(app, {result.center}, app.curve_size * 0.0003,
                             {0, 1, 0});
          }
        }
      }
      ImGui::TreePop();
    } else if (scc.state == scc_state::CIRCLE_3) {
      clear(scc);
      app.curr_circle = app.curr_path = 0;
      scc.state = scc_state::SCC_NONE;
    }

    // PARALLEL
    id = ImGui::GetID("Parallel line through point");

    if (scc.state != scc_state::PARALLEL &&
        ImGui::GetStateStorage()->GetInt(id)) {
      clear(scc);
      app.curr_circle = app.curr_path = 0;
      ImGui::GetStateStorage()->SetInt(id, 0);
    }

    if (ImGui::TreeNode("Parallel line through point")) {
      scc.state = scc_state::PARALLEL;

      disabled = push_disable_items(scc.paths.size() > 0);
      ImGui::Text("Create geodesic");
      disabled = pop_disable_items(disabled);

      disabled = push_disable_items(scc.points.size() != 2);
      ImGui::Text("Select point");
      disabled = pop_disable_items(disabled);

      disabled =
          push_disable_items(scc.paths.size() < 1 && scc.points.size() < 3);
      if (ImGui::Button("Caculate")) {
        if (!set_expected(
                triangles, positions, adjacencies, scc,
                expected_parallel(mesh, scc.paths[0], scc.points[2]))) {
          info("Expected not found");
        }
        if (!set_result(triangles, positions, adjacencies, scc,
                        find_parallel(mesh, scc.paths[0], scc.points[2]))) {
          info("Construction not found");
        }
      }
      disabled = pop_disable_items(disabled);

      ImGui::TreePop();
    } else if (scc.state == scc_state::PARALLEL) {
      clear(scc);
      app.curr_circle = app.curr_path = 0;
      scc.state = scc_state::SCC_NONE;
    }

    // ImGui::Checkbox("Show expected", &scc.show_expected);

    auto intersection_names =
        vector<string>{"Curves", "Circles", "Curve-Circle"};
    draw_combobox(widget, "Type of Inctersection", app.type_of_intersection,
                  intersection_names);

    ImGui::TreePop();
  } else if (app.state == app_state::STRAIGHTEDGE) {
    app.state = app_state::APP_NONE;
    delete_shape(app.shapes["black0"]);
    info("no straight");
  }
  auto e_id = ImGui::GetID("Editing");

  if (app.state != app_state::EDITING &&
      ImGui::GetStateStorage()->GetInt(e_id)) {
    ImGui::GetStateStorage()->SetInt(e_id, 0);
  }
  if (ImGui::TreeNode("Editing")) {
    app.state = app_state::EDITING;

    static auto disabled = false;

    if (app.overlapping_circles.size() > 1) {
      if (draw_slider(widget, "Select Primitives", app.curr_primitive, 0,
                      (int)(app.overlapping_circles.size() - 1))) {
        auto entry = app.overlapping_circles[app.curr_primitive];
        app.selected_circle = &app.scc.circles[entry];
        app.selected_circle_entries = app.scc.circles_to_shape.at(entry);
      }
    }

    disabled = push_disable_items(!levels_can_be_edited(app));
    app.curr_primitive_name = curr_primitive_name(app);
    draw_label(widget, "Selected Primitive", app.curr_primitive_name);
    if (draw_slider(widget, "Concentric Levels", app.concentric_levels, 1,
                    50)) {
      auto circle_shape = app.added_paths[app.selected_circle_entries[0]];
      for (auto i = app.selected_circle_entries[0]; i < app.added_paths.size();
           ++i)
        clear_shape(app.added_paths[i]->instance->shape);
      app.added_paths.erase(app.added_paths.begin() +
                                app.selected_circle_entries[0],
                            app.added_paths.end());
      app.selected_circle->levels = app.concentric_levels;
      app.added_paths.push_back(circle_shape);
      add_concentric_shape(app, app.selected_circle->vertices);
    }
    disabled = pop_disable_items(disabled);
    draw_separator(widget);
    disabled = push_disable_items(app.selected_circle == nullptr ||
                                  app.selected_circle_entries.size() != 1);
    if (draw_checkbox(widget, "Reverse", app.reverse_cropping)) {
      auto curr_pos =
          circle_positions(app.mesh.triangles, app.mesh.positions,
                           app.mesh.adjacencies, *app.selected_circle);
      auto pos =
          (app.reverse_cropping)
              ? crop_circle(curr_pos, {app.selected_circle->crop_range.y,
                                       app.selected_circle->crop_range.x})
              : crop_circle(curr_pos, app.selected_circle->crop_range);

      // clear_scc(app);

      update_path_shape(app.added_paths, app.mesh, pos, app.curve_size * 0.0001,
                        app.selected_circle_entries[0]);
    }
    disabled = pop_disable_items(disabled);
    draw_separator(widget);
    draw_checkbox(widget, "Use Slider For Circle Radius",
                  app.use_slider_for_circle);
    if (app.use_slider_for_circle && app.selected_circle != nullptr) {
      if (app.scale_factor_radius == -1) {
        app.max_dist = max_of_field(app.selected_circle->distances);
        app.scale_factor_radius = app.selected_circle->radius / app.max_dist;
      }
      if (draw_doubleinput(widget, "Scale Factor", app.scale_factor_radius,
                           0.001, 1)) {
        set_radius(app.mesh, app.selected_circle,
                   app.scale_factor_radius * app.max_dist);
        update_isoline_shape(app, app.selected_circle->isoline,
                             app.selected_circle_entries);
      }
    }

    // disabled = push_disable_items(
    //     app.selected_circle == nullptr ||
    //     app.scc.circles_to_primitve.at(app.selected_circle_entry) !=
    //     cir);
    // draw_slider(widget, "Circle Radius", app.selected_circle->radius,
    // 0.0001,
    //             app.max_radius);
    // disabled = pop_disable_items(disabled);
    disabled = push_disable_items(app.selected_circle == nullptr ||
                                  app.selected_circle->primitive != rhomb);
    if (draw_slider(widget, "Axis Ratio", app.lambda, 0.01, 1.f)) {
      auto vertices =
          rhombus_tangent_space(app.mesh, app.selected_circle, app.lambda,
                                app.selected_circle->theta);
      update_quadrilateral_shape(app, vertices,
                                 app.selected_circle_entries[0] + 1);
    }
    disabled = pop_disable_items(disabled);
    disabled = push_disable_items(app.selected_circle == nullptr ||
                                  !height_and_base_ratio_can_be_edited(app));
    if (draw_slider(widget, "H/B Ratio", app.sigma, 0.01, 1.f)) {
      auto vertices = vector<mesh_point>{};
      if (app.selected_circle->primitive == parallelog)
        vertices = parallelogram_tangent_space(app.mesh, app.selected_circle,
                                               app.sigma, app.lambda,
                                               app.selected_circle->theta);
      else
        vertices = parallelogram_tangent_space(app.mesh, app.selected_circle,
                                               app.sigma, 1.f,
                                               app.selected_circle->theta);

      update_quadrilateral_shape(app, vertices,
                                 app.selected_circle_entries[0] + 1);
    }
    disabled = pop_disable_items(disabled);
    disabled = push_disable_items(app.selected_circle == nullptr ||
                                  app.selected_circle->primitive != parallelog);
    if (draw_slider(widget, "Diagonal Ratio", app.lambda, 0.01, 1.f)) {
      auto vertices =
          parallelogram_tangent_space(app.mesh, app.selected_circle, app.sigma,
                                      app.lambda, app.selected_circle->theta);
      update_quadrilateral_shape(app, vertices,
                                 app.selected_circle_entries[0] + 1);
    }

    disabled = pop_disable_items(disabled);

    ImGui::TreePop();
  } else if (app.state == app_state::EDITING) {
    app.state = app_state::APP_NONE;
    scc.cropping_range.clear();
  }

  ImGui::NewLine();

  draw_separator(widget);
  auto disabled = push_disable_items(!flower_can_be_edited(app));
  if (draw_slider(widget, "Petals", app.petals, 3, 10)) {
    for (auto i = app.selected_circle_entries[0] + 1;
         i < app.added_paths.size(); ++i)
      clear_shape(app.added_paths[i]->instance->shape);
    app.added_paths.erase(app.added_paths.begin() +
                              app.selected_circle_entries[0] + 1,
                          app.added_paths.end());
    app.selected_circle->petals = app.petals;
    app.selected_circle->arcs = make_flower(
        app.mesh, app.selected_circle, app.petals, false,
        app.scale_factor_inner_circle_radius, app.with_inner_circle);
    add_flower_shape(app, app.selected_circle->arcs);
  }
  if (draw_checkbox(widget, "With Inner Circle", app.with_inner_circle)) {
    if (app.selected_circle->with_inner_circle != app.with_inner_circle) {
      app.selected_circle->with_inner_circle = app.with_inner_circle;
      if (app.with_inner_circle) {
        auto c0 = create_circle(app.mesh, app.selected_circle->center,
                                app.scale_factor_inner_circle_radius *
                                    app.selected_circle->radius);
        auto color = app.added_paths[app.selected_circle_entries[0] + 1]
                         ->instance->material->color;
        add_path_shape(app,
                       circle_positions(app.mesh.triangles, app.mesh.positions,
                                        app.mesh.adjacencies, c0),
                       app.curve_size * 0.0001, color);
      } else {
        clear_shape(app.added_paths.back()->instance->shape);
        app.added_paths.pop_back();
      }
    }
  }
  disabled = pop_disable_items(disabled);
  disabled = push_disable_items(app.selected_circle == nullptr ||
                                app.selected_circle->primitive != flower ||
                                !app.selected_circle->with_inner_circle);

  if (draw_slider(widget, "Inner Circle Radius",
                  app.scale_factor_inner_circle_radius, 0.1, 1)) {
    auto new_inner_circle = create_circle(app.mesh, app.selected_circle->center,
                                          app.scale_factor_inner_circle_radius *
                                              app.selected_circle->radius);
    update_path_shape(app.added_paths, app.mesh,
                      circle_positions(app.mesh.triangles, app.mesh.positions,
                                       app.mesh.adjacencies, new_inner_circle),
                      app.curve_size * 0.0001,
                      app.selected_circle_entries[0] + 1 +
                          app.selected_circle->petals);
  }
  draw_checkbox(widget, "Inner Circle Color", app.edit_inner_circle);
  disabled = pop_disable_items(disabled);

  disabled = push_disable_items(app.selected_circle == nullptr);
  if (draw_slider(widget, "Curve Size", app.curve_size, 0, 50)) {
    auto &entries = app.selected_circle_entries;
    if (app.selected_circle->primitive == cir) {
      if (app.selected_circle->levels == 1) {
        for (auto i = 0; i < entries.size(); ++i) {
          update_path_shape(app.added_paths, app.mesh,
                            app.added_paths[entries[i]]->positions,
                            app.curve_size * 0.0001, entries[i]);
        }
      } else {
        auto entry = app.selected_circle_entries[0];
        for (auto i = 0; i < app.selected_circle->levels; ++i) {
          auto curr_entry = entry + i;
          update_path_shape(app.added_paths, app.mesh,
                            app.added_paths[curr_entry]->positions,
                            app.curve_size * 0.0001, curr_entry);
        }
      }

    } else if (app.selected_circle->primitive == garland_tri) {
      auto primitives_entries = vector<int>(20);
      for (auto i = 0; i < 20; ++i) {
        primitives_entries[i] = entries[0] + 1 + i;
      }
      update_path_shape(app.added_paths, app.mesh, app.curve_size * 0.0001,
                        primitives_entries);
    } else if (app.selected_circle->primitive == garland_sq) {
      auto primitives_entries = vector<int>(5);
      for (auto i = 0; i < 5; ++i) {
        primitives_entries[i] = entries[0] + 1 + i;
      }
      update_path_shape(app.added_paths, app.mesh, app.curve_size * 0.0001,
                        primitives_entries);
    } else if (app.selected_circle->primitive == garland_pent) {
      auto primitives_entries = vector<int>(6);
      for (auto i = 0; i < 6; ++i) {
        primitives_entries[i] = entries[0] + 1 + i;
      }
      update_path_shape(app.added_paths, app.mesh, app.curve_size * 0.0001,
                        primitives_entries);
    } else if (app.selected_circle->primitive == garland_hex) {
      auto primitives_entries = vector<int>(4);
      for (auto i = 0; i < 4; ++i) {
        primitives_entries[i] = entries[0] + 1 + i;
      }
      update_path_shape(app.added_paths, app.mesh, app.curve_size * 0.0001,
                        primitives_entries);
    } else if (app.selected_circle->primitive == spider) {
      auto entry = app.selected_circle_entries[0] + 1;
      for (auto i = 0; i < 8; ++i) {
        update_path_shape(app.added_paths, app.mesh,
                          app.added_paths[entry + i]->positions,
                          app.curve_size * 0.0001, entry + i);
      }
      for (auto i = 0; i < 24; ++i) {
        update_path_shape(app.added_paths, app.mesh,
                          app.added_paths[entry + 8 + i]->positions,
                          app.curve_size * 0.0001, entry + 8 + i);
      }

    } else if (app.selected_circle->primitive == crux) {
      auto entry = app.selected_circle_entries[0] + 1;
      for (auto i = 0; i < 12; ++i) {
        update_path_shape(app.added_paths, app.mesh,
                          app.added_paths[entry + i]->positions,
                          app.curve_size * 0.0001, entry + i);
      }
    } else if (app.selected_circle->primitive == flower) {
      auto entry = app.selected_circle_entries[0] + 1;
      if (!app.edit_inner_circle) {
        for (auto i = 0; i < app.selected_circle->petals; ++i) {
          update_path_shape(app.added_paths, app.mesh,
                            app.added_paths[entry + i]->positions,
                            app.curve_size * 0.0001, entry + i);
        }
      } else
        update_path_shape(
            app.added_paths, app.mesh,
            app.added_paths[entry + app.selected_circle->petals]->positions,
            app.curve_size * 0.0001, entry + app.selected_circle->petals);
    } else {
      if (app.selected_circle->levels == 1)
        update_generic_shape(app);
      else {
        auto entry = app.selected_circle_entries[0] + 1;
        for (auto i = 0; i < app.selected_circle->levels; ++i) {
          auto curr_entry = entry + i;
          update_path_shape(app.added_paths, app.mesh,
                            app.added_paths[curr_entry]->positions,
                            app.curve_size * 0.0001, curr_entry);
        }
      }
    }
  }
  disabled = pop_disable_items(disabled);
  if (draw_coloredit(widget, "Current Color", app.curr_color)) {
    if (app.selected_circle != nullptr) {
      auto &entries = app.selected_circle_entries;
      if (app.selected_circle->primitive == cir) {
        if (app.selected_circle->levels == 1) {
          for (auto i = 0; i < entries.size(); ++i)
            update_path_shape(app.added_paths, app.mesh, app.curr_color,
                              entries[i]);
        } else {
          auto entry = entries[0];
          for (auto i = 0; i < app.selected_circle->levels; ++i)
            update_path_shape(app.added_paths, app.mesh, app.curr_color,
                              entry + i);
        }
      } else if (app.selected_circle->primitive == garland_tri) {
        auto primitives_entries = vector<int>(20);
        for (auto i = 0; i < 20; ++i) {
          primitives_entries[i] = entries[0] + 1 + i;
        }
        update_path_shape(app.added_paths, app.mesh, app.curr_color,
                          primitives_entries);
      } else if (app.selected_circle->primitive == garland_sq) {
        auto primitives_entries = vector<int>(5);
        for (auto i = 0; i < 5; ++i) {
          primitives_entries[i] = entries[0] + 1 + i;
        }
        update_path_shape(app.added_paths, app.mesh, app.curr_color,
                          primitives_entries);
      } else if (app.selected_circle->primitive == garland_pent) {
        auto primitives_entries = vector<int>(6);
        for (auto i = 0; i < 6; ++i) {
          primitives_entries[i] = entries[0] + 1 + i;
        }
        update_path_shape(app.added_paths, app.mesh, app.curr_color,
                          primitives_entries);
      } else if (app.selected_circle->primitive == garland_hex) {
        auto primitives_entries = vector<int>(4);
        for (auto i = 0; i < 4; ++i) {
          primitives_entries[i] = entries[0] + 1 + i;
        }
        update_path_shape(app.added_paths, app.mesh, app.curr_color,
                          primitives_entries);
      } else if (app.selected_circle->primitive == spider) {
        auto primitives_entries = vector<int>(32);
        for (auto i = 0; i < 32; ++i) {
          primitives_entries[i] = entries[0] + 1 + i;
        }
        update_path_shape(app.added_paths, app.mesh, app.curr_color,
                          primitives_entries);

      } else if (app.selected_circle->primitive == crux) {
        auto primitives_entries = vector<int>(12);
        for (auto i = 0; i < 12; ++i) {
          primitives_entries[i] = entries[0] + 1 + i;
        }
        update_path_shape(app.added_paths, app.mesh, app.curr_color,
                          primitives_entries);
      } else if (app.selected_circle->primitive == flower) {
        if (!app.edit_inner_circle) {
          auto primitives_entries = vector<int>(app.selected_circle->petals);
          for (auto i = 0; i < app.selected_circle->petals; ++i) {
            primitives_entries[i] = entries[0] + 1 + i;
          }
          update_path_shape(app.added_paths, app.mesh, app.curr_color,
                            primitives_entries);
        } else
          update_path_shape(app.added_paths, app.mesh, app.curr_color,
                            entries[0] + app.selected_circle->petals + 1);

      } else {
        if (app.selected_circle->levels == 1)
          update_path_shape(app.added_paths, app.mesh, app.curr_color,
                            entries[0] + 1);
        else {
          auto entry = entries[0] + 1;
          for (auto i = 0; i < app.selected_circle->levels; ++i) {
            update_path_shape(app.added_paths, app.mesh, app.curr_color,
                              entry + i);
          }
        }
      }
    }
  }
  if (draw_button(widget, "Done with editing")) {
    app.selected_circle = nullptr;
    app.concentric_levels = 1;
    app.sigma = 0.75;
    app.lambda = 0.75;
  }

  ImGui::Checkbox("Show mesh edges", &app.show_edges);
  draw_textinput(widget, "Scene Name", app.scene_name);
  if (ImGui::Button("Export Scene")) {
    export_scene(app, vec3f{0.9, 0.9, 0.9}, app.scene_name, false,
                 app.export_just_the_paths);
  }
  continue_line(widget);
  draw_checkbox(widget, "Export Only Curves", app.export_just_the_paths);
  ImGui::NewLine();
  draw_separator(widget);
  if (ImGui::Button("Clear mesh")) {
    clear_mesh(app);
    clear(scc);
    app.input().cropping = false;
    app.input().rotating = false;
    app.curr_circle = app.curr_path = 0;
    if (scc.result_shape != nullptr)
      clear_shape(scc.result_shape->instance->shape);
    if (scc.geodesic_shape != nullptr)
      clear_shape(scc.geodesic_shape->instance->shape);
    if (scc.result_shape != nullptr)
      clear_shape(scc.result_shape->instance->shape);
    scc.result_shape = nullptr;
    scc.geodesic_shape = nullptr;
    scc.result_shape = nullptr;
    if (app.added_paths.size() != 0) {
      for (auto &path : app.added_paths) {
        clear_shape(path->instance->shape);
      }
      app.added_paths.clear();
    }
    if (app.added_points.size() != 0) {
      for (auto &points : app.added_points) {
        clear_shape(points->instance->shape);
      }

      app.added_points.clear();
    }
  }
  ImGui::NewLine();
  ImGuiIO &io = ImGui::GetIO();
  ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
  end_widget(widget);
}

int main(int argc, const char *argv[]) {
  string input_filename = "bunny.ply";
  bool time = true;
  bool infolog = true;
  bool quiet = false;
  bool log_colors = true;
  auto app = App();
  auto cli = make_cli("bezier", "interactive viewer for mesh processing");
  add_option(cli, "mesh", app.filename, "Model filenames", true);
  add_option(cli, "--time/--no-time", time, "Log times");
  add_option(cli, "--info/--no-info", infolog, "Log info");
  add_option(cli, "--quiet", quiet, "Disable logs");
  add_option(cli, "--colors/--no-colors", log_colors, "Colored logs");

  parse_cli(cli, argc, argv);

  set_log_colors(log_colors);
  set_log_time(time);
  set_log_info(infolog);
  set_log_quiet(quiet);
  set_log_check(true);

  if (!load_mesh(app.filename, app.mesh, app.error))
    print_fatal(app.error);
  init_bvh(app);
  // Init window.
  auto win = new gui_window();
  init_window(win, {1080, 720}, "mesh viewer", true);
  win->user_data = &app;

  init_gpu(app, app.envlight);

  init_widget(app.widget, win);

  run_ui(win, draw);
  clear_window(win);
}
