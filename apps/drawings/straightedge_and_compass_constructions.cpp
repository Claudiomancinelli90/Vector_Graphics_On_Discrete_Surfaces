
#include "logging.h"

#include "drawing_functions.h"
#include "straightedge_and_compass_constructions.h"
#include <algorithm>
#include <yocto/yocto_math.h>

using namespace logging;
using std::tie;

void draw_scc(unordered_map<string, Shader> &shaders, SCC &scc, mat4f &view,
              mat4f &projection) {
  //   bind_shader(shaders["points"]);
  //   set_uniform(shaders["points"], Uniform("frame", identity4x4f),
  //               Uniform("view", view), Uniform("projection", projection));

  //   set_uniform(shaders["points"], Uniform("color", vec3f{0, 0, 0}));

  //   switch (scc.state) {
  //   case scc_state::BISECTOR: {
  //   }
  //   case scc_state::MIDPOINT: {
  //     for (auto i = 0; i < yocto::min(scc.points_shape.size(), 2); ++i) {
  //       draw_shape(scc.points_shape[i]);
  //     }

  //     if (scc.paths_shape.size() > 0) {
  //       draw_shape(scc.paths_shape[0]);
  //     }

  //     break;
  //   }
  //   case scc_state::PERPENDICULAR: {
  //   }
  //   case scc_state::MIRROR: {
  //   }
  //   case scc_state::PARALLEL: {
  //     for (auto i = 0; i < yocto::min(scc.points_shape.size(), 3); ++i) {
  //       draw_shape(scc.points_shape[i]);
  //     }

  //     for (auto i = 0; i < yocto::min(scc.paths_shape.size(), 1); ++i) {
  //       draw_shape(scc.paths_shape[i]);
  //     }

  //     break;
  //   }
  //   case scc_state::BISECTOR_ANGLE: {
  //     for (auto i = 0; i < yocto::min(scc.points_shape.size(), 3); ++i) {
  //       draw_shape(scc.points_shape[i]);
  //     }

  //     for (auto i = 0; i < yocto::min(scc.paths_shape.size(), 2); ++i) {
  //       draw_shape(scc.paths_shape[i]);
  //     }

  //     break;
  //   }
  //   case scc_state::TANGENT: {
  //     for (auto i = 0; i < yocto::min(scc.points_shape.size(), 2); ++i) {
  //       draw_shape(scc.points_shape[i]);
  //     }

  //     for (auto curve : scc.circle.isoline) {
  //       draw_shape(curve.shape);
  //     }

  //     break;
  //   }
  //   case scc_state::SCC_NONE: {
  //   }
  //   case scc_state::CIRCLE_3: {
  //     for (auto i = 0; i < yocto::min(scc.points_shape.size(), 3); ++i) {
  //       draw_shape(scc.points_shape[i]);
  //     }

  //     break;
  //   }
  //   }

  //   set_uniform(shaders["points"], Uniform("color", vec3f{1, 0, 0}));
  //   for (auto curve : scc.result_circle.isoline) {
  //     draw_shape(curve.shape);
  //   }
  //   draw_shape(scc.result_pos);

  //   if (scc.show_expected) {
  //     set_uniform(shaders["points"], Uniform("color", vec3f{0, 0, 1}));
  //     for (auto curve : scc.expected_circle.isoline) {
  //       draw_shape(curve.shape);
  //     }
  //     draw_shape(scc.expected_pos);
  //   }
}

void clear(SCC &scc) {
  scc.points.clear();
  scc.paths = {geodesic_path{}};
  scc.circles = {Circle()};
  scc.Ellipses = {Ellipse()};
  scc.cropping_range.clear();
  scc.result_point = mesh_point();
  scc.result_path = geodesic_path();
  scc.result_circle = Circle();

  scc.expected_point = mesh_point();
  scc.expected_path = geodesic_path();
  scc.expected_circle = Circle();
  scc.result_pos.clear();
  scc.expected_pos.clear();
  for (auto point : scc.points_shape) {
    clear_shape(point->instance->shape);
  }
  scc.points_shape.clear();

  for (auto shape : scc.paths_shape) {
    delete_shape(shape);
  }
  for (auto primitive : scc.construction_shape) {
    clear_shape(primitive->instance->shape);
  }
  for (auto primitive : scc.input_shape) {
    clear_shape(primitive->instance->shape);
  }
  for (auto primitive : scc.output_shape) {
    clear_shape(primitive->instance->shape);
  }
  if (scc.circle_shape.size() != 0) {
    for (auto &circ : scc.circle_shape)
      clear_shape(circ->instance->shape);
  }

  if (scc.geodesic_shape != nullptr)
    clear_shape(scc.geodesic_shape->instance->shape);

  scc.circle_shape = {};
  scc.geodesic_shape = nullptr;
  scc.paths_shape.clear();
  scc.construction_shape.clear();
  scc.input_shape.clear();
  scc.output_shape.clear();
  scc.result_pos.clear();
  scc.expected_pos.clear();
}
void reset_triangle(SCC &scc) {
  if (scc.points.size() == 3) {
    scc.points.pop_back();
    scc.paths.clear();
    scc.circles.clear();
    scc.paths = {geodesic_path{}};
    scc.circles = {Circle()};
  }
}
void add_path(const bezier_mesh &mesh, SCC &scc, const geodesic_path &path) {
  if (scc.paths.size() >= 2) {
    return;
  }

  scc.paths.push_back(path);
  auto points = path_positions(scc.paths.back(), mesh);
  auto shape = make_polyline_shape(points, {});

  scc.paths_shape.push_back(shape);
}

void add_point(const bezier_mesh &mesh, SCC &scc, const mesh_point &point) {
  //   if (scc.points.size() >= 3) {
  //     return;
  //   }

  //   scc.points.push_back(point);
  //   auto shape = make_points_shape(
  //       vector<vec3f>{eval_position(mesh.triangles, mesh.positions, point)});
  //   scc.points_shape.push_back(shape);

  //   auto size = scc.points.size();

  //   if (size > 1) {
  //     add_path(mesh, scc,
  //              compute_geodesic_path(mesh, scc.points[size - 2],
  //                                    scc.points[size - 1]));
  //   } else if (size == 1 && scc.state == scc_state::TANGENT) {
  //     scc.circle = create_circle(mesh, scc.points.back(), 0.0F);
  //   }
}

bool set_result(const vector<vec3i> &triangles, const vector<vec3f> &positions,
                SCC &scc, const mesh_point &point) {
  if (!validate_points({point})) {
    return false;
  }

  scc.result_point = point;
  scc.result_pos =
      vector<vec3f>{eval_position(triangles, positions, scc.result_point)};

  return true;
}

bool set_result(const vector<vec3i> &triangles, const vector<vec3f> &positions,
                const vector<vec3i> &adjacencies, SCC &scc,
                const geodesic_path &path) {
  if (!validate_paths({path})) {
    return false;
  }

  scc.result_path = path;
  scc.result_pos =
      path_positions(scc.result_path, triangles, positions, adjacencies);

  return true;
}

bool set_result(const vector<vec3i> &triangles, const vector<vec3f> &positions,
                const vector<vec3i> &adjacencies, SCC &scc,
                const Circle &circle) {
  if (!validate_circles({circle})) {
    return false;
  }
  scc.result_pos.clear();
  for (auto &curve : circle.isoline) {
    auto curve_pos =
        closed_curve_positions(curve, triangles, positions, adjacencies);
    scc.result_pos.insert(scc.result_pos.end(), curve_pos.begin(),
                          curve_pos.end());
  }

  return true;
}

bool set_expected(const vector<vec3i> &triangles,
                  const vector<vec3f> &positions, SCC &scc,
                  const mesh_point &point) {
  if (!validate_points({point})) {
    return false;
  }

  scc.expected_point = point;
  scc.expected_pos.clear();
  scc.expected_pos =
      vector<vec3f>{eval_position(triangles, positions, scc.expected_point)};

  return true;
}

bool set_expected(const vector<vec3i> &triangles,
                  const vector<vec3f> &positions,
                  const vector<vec3i> &adjacencies, SCC &scc,
                  const geodesic_path &path) {
  if (!validate_paths({path})) {
    return false;
  }

  scc.expected_path = path;
  scc.expected_pos.clear();
  scc.expected_pos =
      path_positions(scc.expected_path, triangles, positions, adjacencies);

  return true;
}

bool set_expected(const vector<vec3i> &triangles,
                  const vector<vec3f> &positions,
                  const vector<vec3i> &adjacencies, SCC &scc,
                  const Circle &circle) {
  if (!validate_circles({circle})) {
    return false;
  }
  scc.expected_pos.clear();
  for (auto &curve : circle.isoline) {
    auto curve_pos =
        closed_curve_positions(curve, triangles, positions, adjacencies);
    scc.expected_pos.insert(scc.expected_pos.end(), curve_pos.begin(),
                            curve_pos.end());
  }

  return true;
}

bool is_same_point(const vector<vec3i> &triangles,
                   const vector<vec3f> &positions,
                   const vector<vec3i> &adjacencies, const mesh_point &a,
                   const mesh_point &b, const float &tol) {
  if (a.face != b.face && find_in_vec(adjacencies[a.face], b.face) == -1) {
    return false;
  }

  return distance(eval_position(triangles, positions, a),
                  eval_position(triangles, positions, a)) <= tol;
}

mesh_point closer_point(const vector<vec3i> &triangles,
                        const vector<vec3f> &positions,
                        const vector<vec3i> &adjacencies, const mesh_point &p,
                        const mesh_point &a, const mesh_point &b,
                        const float &tol) {
  if (p.face == -1 || (a.face == -1 && b.face == -1)) {
    return mesh_point();
  }
  if (a.face == -1) {
    return b;
  }
  if (b.face == -1) {
    return a;
  }

  auto pp = eval_position(triangles, positions, p);
  auto da = distance(pp, eval_position(triangles, positions, a));
  auto db = distance(pp, eval_position(triangles, positions, b));

  auto isa =
      (p.face == a.face || find_in_vec(adjacencies[p.face], a.face) <= -1) &&
      da <= tol;
  auto isb =
      (p.face == b.face || find_in_vec(adjacencies[p.face], b.face) <= -1) &&
      db <= tol;

  if (isa && isb) {
    return da <= db ? a : b;
  } else if (isa) {
    return a;
  } else if (isb) {
    return b;
  }

  return mesh_point();
}
inline float interp_field_in_triangle(const vector<vec3i> &triangles,
                                      const vector<float> &field,
                                      const mesh_point a) {

  return (1 - a.uv.x - a.uv.y) * field[triangles[a.face].x] +
         a.uv.x * field[triangles[a.face].y] +
         a.uv.y * field[triangles[a.face].z];
}
mesh_point closer_point(const vector<vec3i> &triangles,
                        const vector<vec3f> &positions,
                        const vector<float> &distances, const mesh_point &p,
                        const mesh_point &a, const mesh_point &b) {
  if (p.face == -1 || (a.face == -1 && b.face == -1)) {
    return mesh_point();
  }
  if (a.face == -1) {
    return b;
  }
  if (b.face == -1) {
    return a;
  }

  auto dist_a = interp_field_in_triangle(triangles, distances, a);
  auto dist_b = interp_field_in_triangle(triangles, distances, b);

  return (dist_a > dist_b) ? b : a;
}
mesh_point further_point(const vector<vec3i> &triangles,
                         const vector<vec3f> &positions,
                         const vector<vec3i> &adjacencies, const mesh_point &p,
                         const mesh_point &a, const mesh_point &b,
                         const float &tol) {
  if (p.face == -1 || (a.face == -1 && b.face == -1)) {
    return mesh_point();
  }
  if (a.face == -1) {
    return b;
  }
  if (b.face == -1) {
    return a;
  }

  auto pp = eval_position(triangles, positions, p);
  auto da = distance(pp, eval_position(triangles, positions, a));
  auto db = distance(pp, eval_position(triangles, positions, b));

  auto isa =
      (p.face == a.face || find_in_vec(adjacencies[p.face], a.face) <= -1) &&
      da <= tol;
  auto isb =
      (p.face == b.face || find_in_vec(adjacencies[p.face], b.face) <= -1) &&
      db <= tol;

  if (isa && isb) {
    return da > db ? a : b;
  } else if (isa) {
    return b;
  } else if (isb) {
    return a;
  }

  return mesh_point();
}
vec3f path_pos_from_entry(const vector<vec3i> &triangles,
                          const vector<vec3f> &positions,
                          const vector<vec3i> &adjacencies,
                          const geodesic_path &path, int entry) {
  auto u = path.lerps[entry];
  auto eid = get_edge(triangles, positions, adjacencies, path.strip[entry],
                      path.strip[entry + 1]);
  if (eid.x < 0) {
    assert(path.strip.size() == 1);
    return eval_position(triangles, positions, path.end);
  }
  auto p0 = positions[eid.x];
  auto p1 = positions[eid.y];
  return (1 - u) * p0 + u * p1;
}

mesh_point mesh_point_from_isoline_entry(const vector<vec3i> &triangles,
                                         const vector<vec3i> &adjacencies,
                                         const vector<int> &strip,
                                         const vector<float> &lerps,
                                         const int entry) {
  auto lerp = lerps[entry];
  auto tid0 = strip[entry];
  auto tid1 = strip[(entry + 1) % strip.size()];
  auto offset = find_in_vec(adjacencies[tid0], tid1);
  if (offset == -1)
    return mesh_point{};

  return eval_point_from_lerp(tid0, offset, lerp);
}
vec3f path_pos_from_entry_in_isoline(const vector<vec3i> &triangles,
                                     const vector<vec3f> &positions,
                                     const vector<vec3i> &adjacencies,
                                     const vector<int> &strip,
                                     const vector<float> &lerps, int entry) {

  auto u = lerps[entry];
  auto eid = get_edge(triangles, positions, adjacencies, strip[entry],
                      strip[(entry + 1) % strip.size()]);
  auto p0 = positions[eid.x];
  auto p1 = positions[eid.y];
  return (1 - u) * p0 + u * p1;
}
mesh_point intersect(const bezier_mesh &mesh, const geodesic_path &first,
                     const geodesic_path &second) {
  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;

  vector<mesh_point> intersection;

  auto f = path_positions(first, mesh);
  auto s = path_positions(second, mesh);

  for (auto i = 0; i < first.strip.size(); ++i) {
    for (auto j = 0; j < second.strip.size(); ++j) {
      if (first.strip[i] == second.strip[j]) {
        auto point = intersect_segments(triangles, positions, first.strip[i],
                                        f[i], f[i + 1], s[j], s[j + 1]);

        if (point.face != -1) {
          return point;
        }
      }
    }
  }

  return mesh_point();
}

pair<mesh_point, mesh_point> intersect(const bezier_mesh &mesh,
                                       const geodesic_path &path,
                                       const Isoline &isoline) {

  auto crossed_by_first = vector<bool>(mesh.triangles.size(), false);
  unordered_map<int, pair<mesh_point, mesh_point>> face_to_point;

  auto first_interesection_point = mesh_point{};
  auto second_interesection_point = mesh_point{};
  auto &strip = path.strip;
  for (auto i = 0; i < strip.size(); ++i) {
    crossed_by_first[strip[i]] = true;
    if (i == 0) {
      auto k = find_in_vec(mesh.adjacencies[path.start.face], strip[1]);
      auto next = eval_point_from_lerp(path.start.face, k, path.lerps[0]);
      face_to_point[strip[i]] = std::make_pair(path.start, next);
    } else if (i == strip.size() - 1) {
      auto k = find_in_vec(mesh.adjacencies[strip.back()], path.end.face);
      auto prev = eval_point_from_lerp(strip.back(), k, path.lerps.back());
      face_to_point[strip[i]] = std::make_pair(prev, path.end);
    } else {
      auto curr = strip[i];
      auto prev = strip[i - 1];
      if (face_to_point.count(curr))
        continue;
      auto h = find_in_vec(mesh.adjacencies[curr], prev);
      auto k = find_in_vec(mesh.adjacencies[prev], curr);
      face_to_point[curr] =
          std::make_pair(eval_point_from_lerp(curr, h, 1 - path.lerps[i - 1]),
                         eval_point_from_lerp(curr, k, path.lerps[i]));
    }
  }

  for (auto &curve : isoline) {
    auto s = curve.strip.size();
    for (auto i = 0; i < s; ++i)
      if (crossed_by_first[curve.strip[i]]) {
        auto first_segment = face_to_point.at(curve.strip[i]);
        auto prev = curve.strip[(s + i - 1) % s];
        auto curr = curve.strip[i];
        auto k = find_in_vec(mesh.adjacencies[curr], prev);
        auto start2 = eval_point_from_lerp(curve.strip[i], k,
                                           1 - curve.lerps[(s + i - 1) % s]);
        auto end2 = mesh_point_from_isoline_entry(
            mesh.triangles, mesh.adjacencies, curve.strip, curve.lerps, i);
        if (end2.face == -1)
          std::cerr << "This point should be well defined" << std::endl;
        auto second_segment = std::make_pair(start2, end2);

        auto lerp = intersect_line_segments(
            mesh.triangles, mesh.positions, curve.strip[i], first_segment.first,
            first_segment.second, second_segment.first, second_segment.second);
        if (lerp < -1e-3 || lerp > 1 + 1e-3)
          continue;
        auto bary0 = second_segment.first.uv;
        auto bary1 = second_segment.second.uv;
        auto bary =
            (1 - lerp) * vec3f{1 - bary0.x - bary0.y, bary0.x, bary0.y} +
            lerp * vec3f{1 - bary1.x - bary1.y, bary1.x, bary1.y};

        if (first_interesection_point.face == -1)
          first_interesection_point =
              mesh_point{curve.strip[i], {bary.y, bary.z}};
        else
          second_interesection_point =
              mesh_point{curve.strip[i], {bary.y, bary.z}};
      }
  }
  return {first_interesection_point, second_interesection_point};
}
std::tuple<mesh_point, mesh_point, vec2i, vec2i>
intersect_with_entries(const bezier_mesh &mesh, const vector<mesh_point> &path,
                       const Isoline &isoline) {

  auto crossed_by_first = vector<bool>(mesh.triangles.size(), false);
  unordered_map<int, pair<mesh_point, mesh_point>> face_to_point;
  unordered_map<int, int> face_to_entries;
  auto range_first = zero2i;
  auto range_second = zero2i;
  auto first_interesection_point = mesh_point{};
  auto second_interesection_point = mesh_point{};

  for (auto i = 0; i < path.size(); ++i) {
    crossed_by_first[path[i].face] = true;
    if (i == 0)
      face_to_point[path[i].face] = std::make_pair(path[0], path[1]);
    else {
      auto curr = path[i];
      auto prev = path[i - 1];
      if (face_to_point.count(curr.face))
        continue;
      else if (curr.face != prev.face) {
        auto h = find_in_vec(mesh.adjacencies[curr.face], prev.face);
        auto k = find_in_vec(mesh.adjacencies[prev.face], curr.face);
        if (h == -1) {
          auto [curr_is_vert, kc] = point_is_vert(curr);
          auto [prev_is_vert, kp] = point_is_vert(prev);
          if (prev_is_vert) {
            auto vid = mesh.triangles[prev.face][kp];
            auto kv = find_in_vec(mesh.triangles[curr.face], vid);
            if (kv == -1)
              std::cerr << "This offset shouldn't be -1" << std::endl;
            face_to_point[curr.face] =
                std::make_pair(eval_point_from_lerp(curr.face, kv, 0.f), curr);
          } else
            std::cerr << "This shouldn't happens" << std::endl;
        } else {
          auto bary = vec3f{1 - prev.uv.x - prev.uv.y, prev.uv.x, prev.uv.y};
          face_to_point[curr.face] =
              std::make_pair(eval_point_from_lerp(curr.face, h, bary[k]), curr);
        }
      } else
        face_to_point[curr.face] = std::make_pair(prev, curr);
    }
    face_to_entries[path[i].face] = i;
  }

  for (auto &curve : isoline) {
    auto s = curve.strip.size();
    for (auto i = 0; i < s; ++i)
      if (crossed_by_first[curve.strip[i]]) {
        auto first_segment = face_to_point.at(curve.strip[i]);
        auto prev = curve.strip[(s + i - 1) % s];
        auto curr = curve.strip[i];
        auto k = find_in_vec(mesh.adjacencies[curr], prev);
        auto start2 = eval_point_from_lerp(curve.strip[i], k,
                                           1 - curve.lerps[(s + i - 1) % s]);
        auto end2 = mesh_point_from_isoline_entry(
            mesh.triangles, mesh.adjacencies, curve.strip, curve.lerps, i);
        if (end2.face == -1)
          std::cerr << "This point should be well defined" << std::endl;
        auto second_segment = std::make_pair(start2, end2);

        auto lerp = intersect_line_segments(
            mesh.triangles, mesh.positions, curve.strip[i], first_segment.first,
            first_segment.second, second_segment.first, second_segment.second);
        if (lerp < -1e-3 || lerp > 1 + 1e-3)
          continue;
        auto bary0 = second_segment.first.uv;
        auto bary1 = second_segment.second.uv;
        auto bary =
            (1 - lerp) * vec3f{1 - bary0.x - bary0.y, bary0.x, bary0.y} +
            lerp * vec3f{1 - bary1.x - bary1.y, bary1.x, bary1.y};

        if (first_interesection_point.face == -1) {
          first_interesection_point =
              mesh_point{curve.strip[i], {bary.y, bary.z}};
          range_second.x = i;
          range_first.x = face_to_entries.at(curr);
        } else {
          second_interesection_point =
              mesh_point{curve.strip[i], {bary.y, bary.z}};
          range_second.y = i;
          range_first.y = face_to_entries.at(curr);
        }
      }
  }
  return {first_interesection_point, second_interesection_point, range_first,
          range_second};
}
pair<mesh_point, mesh_point> intersect(const bezier_mesh &mesh,
                                       const vector<mesh_point> &path,
                                       const Isoline &isoline) {

  auto crossed_by_first = vector<bool>(mesh.triangles.size(), false);
  unordered_map<int, pair<mesh_point, mesh_point>> face_to_point;

  auto first_interesection_point = mesh_point{};
  auto second_interesection_point = mesh_point{};

  for (auto i = 0; i < path.size(); ++i) {
    crossed_by_first[path[i].face] = true;
    if (i == 0)
      face_to_point[path[i].face] = std::make_pair(path[0], path[1]);
    else {
      auto curr = path[i];
      auto prev = path[i - 1];
      if (face_to_point.count(curr.face))
        continue;
      else if (curr.face != prev.face) {
        auto h = find_in_vec(mesh.adjacencies[curr.face], prev.face);
        auto k = find_in_vec(mesh.adjacencies[prev.face], curr.face);
        if (h == -1) {
          auto [curr_is_vert, kc] = point_is_vert(curr);
          auto [prev_is_vert, kp] = point_is_vert(prev);
          if (prev_is_vert) {
            auto vid = mesh.triangles[prev.face][kp];
            auto kv = find_in_vec(mesh.triangles[curr.face], vid);
            if (kv == -1)
              std::cerr << "This offset shouldn't be -1" << std::endl;
            face_to_point[curr.face] =
                std::make_pair(eval_point_from_lerp(curr.face, kv, 0.f), curr);
          } else
            std::cerr << "This shouldn't happens" << std::endl;
        } else {
          auto bary = vec3f{1 - prev.uv.x - prev.uv.y, prev.uv.x, prev.uv.y};
          face_to_point[curr.face] =
              std::make_pair(eval_point_from_lerp(curr.face, h, bary[k]), curr);
        }
      } else
        face_to_point[curr.face] = std::make_pair(prev, curr);
    }
  }

  for (auto &curve : isoline) {
    auto s = curve.strip.size();
    for (auto i = 0; i < s; ++i)
      if (crossed_by_first[curve.strip[i]]) {
        auto first_segment = face_to_point.at(curve.strip[i]);
        auto prev = curve.strip[(s + i - 1) % s];
        auto curr = curve.strip[i];
        auto k = find_in_vec(mesh.adjacencies[curr], prev);
        auto start2 = eval_point_from_lerp(curve.strip[i], k,
                                           1 - curve.lerps[(s + i - 1) % s]);
        auto end2 = mesh_point_from_isoline_entry(
            mesh.triangles, mesh.adjacencies, curve.strip, curve.lerps, i);
        if (end2.face == -1)
          std::cerr << "This point should be well defined" << std::endl;
        auto second_segment = std::make_pair(start2, end2);

        auto lerp = intersect_line_segments(
            mesh.triangles, mesh.positions, curve.strip[i], first_segment.first,
            first_segment.second, second_segment.first, second_segment.second);
        if (lerp < -1e-3 || lerp > 1 + 1e-3)
          continue;
        auto bary0 = second_segment.first.uv;
        auto bary1 = second_segment.second.uv;
        auto bary =
            (1 - lerp) * vec3f{1 - bary0.x - bary0.y, bary0.x, bary0.y} +
            lerp * vec3f{1 - bary1.x - bary1.y, bary1.x, bary1.y};

        if (first_interesection_point.face == -1)
          first_interesection_point =
              mesh_point{curve.strip[i], {bary.y, bary.z}};
        else
          second_interesection_point =
              mesh_point{curve.strip[i], {bary.y, bary.z}};
      }
  }
  return {first_interesection_point, second_interesection_point};
}
pair<mesh_point, mesh_point> intersect(const bezier_mesh &mesh,
                                       const geodesic_path &path,
                                       const Circle &circle) {
  return intersect(mesh, path, circle.isoline);
}

pair<mesh_point, mesh_point> intersect(const bezier_mesh &mesh,
                                       const geodesic_path &path,
                                       const closed_curve &curve) {
  return intersect(mesh, path, Isoline{curve});
}

pair<mesh_point, mesh_point> intersect(const bezier_mesh &mesh,
                                       const Isoline &first,
                                       const Isoline &second) {
  auto crossed_by_first = vector<bool>(mesh.triangles.size(), false);
  unordered_map<int, pair<mesh_point, mesh_point>> face_to_point;
  auto first_interesection_point = mesh_point{};
  auto second_interesection_point = mesh_point{};
  for (auto &curve : first) {
    auto s0 = curve.strip.size();
    for (auto i = 0; i < s0; ++i) {
      crossed_by_first[curve.strip[i]] = true;
      auto prev = curve.strip[(s0 + i - 1) % s0];
      auto curr = curve.strip[i];
      auto k = find_in_vec(mesh.adjacencies[curr], prev);
      auto start1 =
          eval_point_from_lerp(curr, k, 1 - curve.lerps[(s0 + i - 1) % s0]);
      auto end1 = mesh_point_from_isoline_entry(
          mesh.triangles, mesh.adjacencies, curve.strip, curve.lerps, i);
      face_to_point[curve.strip[i]] = std::make_pair(start1, end1);
    }
  }
  for (auto &curve : second) {
    auto s1 = curve.strip.size();
    for (auto i = 0; i < s1; ++i)
      if (crossed_by_first[curve.strip[i]] == true) {
        auto first_segment = face_to_point.at(curve.strip[i]);
        auto prev = curve.strip[(s1 + i - 1) % s1];
        auto curr = curve.strip[i];
        auto k = find_in_vec(mesh.adjacencies[curr], prev);
        auto start2 =
            eval_point_from_lerp(curr, k, 1 - curve.lerps[(s1 + i - 1) % s1]);
        auto end2 = mesh_point_from_isoline_entry(
            mesh.triangles, mesh.adjacencies, curve.strip, curve.lerps, i);
        if (end2.face == -1)
          std::cerr << "This point should be well defined" << std::endl;
        auto second_segment = std::make_pair(start2, end2);

        auto lerp = intersect_line_segments(
            mesh.triangles, mesh.positions, curve.strip[i], first_segment.first,
            first_segment.second, second_segment.first, second_segment.second);
        if (lerp < -1e-3 || lerp > 1 + 1e-3)
          continue;
        auto bary0 = second_segment.first.uv;
        auto bary1 = second_segment.second.uv;
        auto bary =
            (1 - lerp) * vec3f{1 - bary0.x - bary0.y, bary0.x, bary0.y} +
            lerp * vec3f{1 - bary1.x - bary1.y, bary1.x, bary1.y};

        if (first_interesection_point.face == -1)
          first_interesection_point =
              mesh_point{curve.strip[i], {bary.y, bary.z}};
        else
          second_interesection_point =
              mesh_point{curve.strip[i], {bary.y, bary.z}};
      }
  }
  return {first_interesection_point, second_interesection_point};
}
std::tuple<mesh_point, mesh_point, vec2i, vec2i>
intersect_with_entries(const bezier_mesh &mesh, const Isoline &first,
                       const Isoline &second) {
  auto crossed_by_first = vector<bool>(mesh.triangles.size(), false);
  unordered_map<int, pair<mesh_point, mesh_point>> face_to_point;
  unordered_map<int, int> face_to_entries;
  auto first_interesection_point = mesh_point{};
  auto second_interesection_point = mesh_point{};
  vec2i crop_range_first = {};
  vec2i crop_range_second = {};
  for (auto &curve : first) {
    auto s0 = curve.strip.size();
    for (auto i = 0; i < s0; ++i) {
      crossed_by_first[curve.strip[i]] = true;
      auto prev = curve.strip[(s0 + i - 1) % s0];
      auto curr = curve.strip[i];
      auto k = find_in_vec(mesh.adjacencies[curr], prev);
      auto start1 =
          eval_point_from_lerp(curr, k, 1 - curve.lerps[(s0 + i - 1) % s0]);
      auto end1 = mesh_point_from_isoline_entry(
          mesh.triangles, mesh.adjacencies, curve.strip, curve.lerps, i);
      face_to_point[curve.strip[i]] = std::make_pair(start1, end1);
      face_to_entries[curve.strip[i]] = i;
    }
  }
  for (auto &curve : second) {
    auto s1 = curve.strip.size();
    for (auto i = 0; i < s1; ++i)
      if (crossed_by_first[curve.strip[i]] == true) {
        auto first_segment = face_to_point.at(curve.strip[i]);
        auto prev = curve.strip[(s1 + i - 1) % s1];
        auto curr = curve.strip[i];
        auto k = find_in_vec(mesh.adjacencies[curr], prev);
        auto start2 =
            eval_point_from_lerp(curr, k, 1 - curve.lerps[(s1 + i - 1) % s1]);
        auto end2 = mesh_point_from_isoline_entry(
            mesh.triangles, mesh.adjacencies, curve.strip, curve.lerps, i);
        if (end2.face == -1)
          std::cerr << "This point should be well defined" << std::endl;
        auto second_segment = std::make_pair(start2, end2);

        auto lerp = intersect_line_segments(
            mesh.triangles, mesh.positions, curve.strip[i], first_segment.first,
            first_segment.second, second_segment.first, second_segment.second);
        if (lerp < -1e-3 || lerp > 1 + 1e-3)
          continue;
        auto bary0 = second_segment.first.uv;
        auto bary1 = second_segment.second.uv;
        auto bary =
            (1 - lerp) * vec3f{1 - bary0.x - bary0.y, bary0.x, bary0.y} +
            lerp * vec3f{1 - bary1.x - bary1.y, bary1.x, bary1.y};

        if (first_interesection_point.face == -1) {
          first_interesection_point =
              mesh_point{curve.strip[i], {bary.y, bary.z}};
          crop_range_second.x = i;
          crop_range_first.x = face_to_entries.at(curve.strip[i]);
        } else {
          second_interesection_point =
              mesh_point{curve.strip[i], {bary.y, bary.z}};
          crop_range_second.y = i;
          crop_range_first.y = face_to_entries.at(curve.strip[i]);
        }
      }
  }
  return {first_interesection_point, second_interesection_point,
          crop_range_first, crop_range_second};
}
pair<mesh_point, mesh_point>
intersect(const bezier_mesh &mesh, const Circle &first, const Circle &second) {
  return intersect(mesh, first.isoline, second.isoline);
}
mesh_point intersect_inside_circle(const bezier_mesh &mesh, const Circle &first,
                                   const Circle &second,
                                   const Circle &container) {
  auto are_not_bary = [](const vec2f &bary) {
    if (bary.x < -1e-3 || bary.x > 1 + 1e3)
      return false;
    if (bary.y < -1e-3 || bary.y > 1 + 1e3)
      return false;

    return true;
  };
  auto &distances = container.distances;
  auto &radius = container.radius;
  auto [i0, i1] = intersect(mesh, first.isoline, second.isoline);
  if (i0.face == -1 || !are_not_bary(i0.uv) || i1.face == -1 ||
      !are_not_bary(i1.uv))
    std::cout << "this should not happen" << std::endl;

  auto d0 = interpolate_distances(mesh, distances, i0);
  auto d1 = interpolate_distances(mesh, distances, i1);

  return (std::abs(d0 - radius) > std::abs(d1 - radius)) ? i1 : i0;
}
std::pair<mesh_point, vec2i>
intersect_inside_circle_with_entries(const bezier_mesh &mesh,
                                     const Circle &first, const Circle &second,
                                     const Circle &container) {
  auto are_not_bary = [](const vec2f &bary) {
    if (bary.x < -1e-3 || bary.x > 1 + 1e3)
      return false;
    if (bary.y < -1e-3 || bary.y > 1 + 1e3)
      return false;

    return true;
  };
  auto &distances = container.distances;
  auto &radius = container.radius;
  auto [i0, i1, range_first, range_second] =
      intersect_with_entries(mesh, first.isoline, second.isoline);
  if (i0.face == -1 || !are_not_bary(i0.uv) || i1.face == -1 ||
      !are_not_bary(i1.uv))
    std::cout << "this should not happen" << std::endl;

  auto d0 = interpolate_distances(mesh, distances, i0);
  auto d1 = interpolate_distances(mesh, distances, i1);

  return (std::abs(d0 - radius) > std::abs(d1 - radius))
             ? std::make_pair(i1, vec2i{range_first.y, range_second.y})
             : std::make_pair(i0, vec2i{range_first.x, range_second.x});
}
std::pair<mesh_point, vec2i>
intersect_outside_circle_with_entries(const bezier_mesh &mesh,
                                      const Circle &first, const Circle &second,
                                      const Circle &container) {
  auto are_not_bary = [](const vec2f &bary) {
    if (bary.x < -1e-3 || bary.x > 1 + 1e3)
      return false;
    if (bary.y < -1e-3 || bary.y > 1 + 1e3)
      return false;

    return true;
  };
  auto &distances = container.distances;
  auto &radius = container.radius;
  auto [i0, i1, range_first, range_second] =
      intersect_with_entries(mesh, first.isoline, second.isoline);
  if (i0.face == -1 || !are_not_bary(i0.uv) || i1.face == -1 ||
      !are_not_bary(i1.uv))
    std::cout << "this should not happen" << std::endl;

  auto d0 = interpolate_distances(mesh, distances, i0);
  auto d1 = interpolate_distances(mesh, distances, i1);

  return (d1 > d0) ? std::make_pair(i1, vec2i{range_first.y, range_second.y})
                   : std::make_pair(i0, vec2i{range_first.x, range_second.x});
}
bool intersection_point(const bezier_mesh &mesh, const closed_curve &first,
                        const closed_curve &second, const int first_entry,
                        const int second_entry, vec3f &intersection_pos) {
  intersection_pos = zero3f;
  auto &strip0 = first.strip;
  auto &strip1 = second.strip;
  auto s0 = strip0.size();
  auto s1 = strip1.size();

  auto prev = strip0[(s0 + first_entry - 1) % s0];
  auto curr = strip0[first_entry];
  auto k = find_in_vec(mesh.adjacencies[curr], prev);
  auto start1 = eval_point_from_lerp(
      curr, k, 1 - first.lerps[(s0 + first_entry - 1) % s0]);
  auto end1 = mesh_point_from_isoline_entry(mesh.triangles, mesh.adjacencies,
                                            strip0, first.lerps, first_entry);
  prev = strip1[(s1 + second_entry - 1) % s1];
  curr = strip1[second_entry];
  k = find_in_vec(mesh.adjacencies[curr], prev);
  auto start2 = eval_point_from_lerp(
      curr, k, 1 - second.lerps[(s1 + second_entry - 1) % s1]);
  auto end2 = mesh_point_from_isoline_entry(mesh.triangles, mesh.adjacencies,
                                            strip1, second.lerps, second_entry);
  auto lerp = intersect_line_segments(mesh.triangles, mesh.positions, curr,
                                      start1, end1, start2, end2);
  if (lerp < 0 || lerp > 1)
    return false;
  auto bary0 = start2.uv;
  auto bary1 = end2.uv;
  auto bary = (1 - lerp) * vec3f{1 - bary0.x - bary0.y, bary0.x, bary0.y} +
              lerp * vec3f{1 - bary1.x - bary1.y, bary1.x, bary1.y};
  intersection_pos = interpolate_triangle(
      mesh.positions[mesh.triangles[curr].x],
      mesh.positions[mesh.triangles[curr].y],
      mesh.positions[mesh.triangles[curr].z], vec2f{bary.y, bary.z});
  return true;
}
std::tuple<geodesic_path, Circle, Circle>
find_segment_bisector(const bezier_mesh &mesh, const geodesic_path &path) {
  auto radius =
      path_length(path, mesh.triangles, mesh.positions, mesh.adjacencies);
  auto cs = create_circle(mesh, path.start, 1.5 * radius);
  auto ce = create_circle(mesh, path.end, 1.5 * radius);

  auto intersections = intersect(mesh, cs, ce);
  auto p = 1.0F;

  while (!validate_points({intersections.second}) && p > 0.6F) {
    p -= 0.1F;
    set_radius(mesh, &cs, radius * p);
    set_radius(mesh, &ce, radius * p);
    intersections = intersect(mesh, cs, ce);
  }

  if (!validate_points({intersections.second})) {
    return {geodesic_path(), Circle(), Circle()};
  }

  return {
      compute_geodesic_path(mesh, intersections.first, intersections.second),
      cs, ce};
}
Isoline find_segment_bisector_isolines(const bezier_mesh &mesh,
                                       const geodesic_path &path) {
  auto dist0 = get_geodesic_distances(mesh, path.start);
  auto dist1 = get_geodesic_distances(mesh, path.end);

  return create_isoline(mesh, dist0, dist1);
}
std::pair<Isoline, vector<float>>
find_segment_bisector_isolines(const bezier_mesh &mesh, const mesh_point &a,
                               const mesh_point &b) {
  auto dista = get_geodesic_distances(mesh, a);
  auto distb = get_geodesic_distances(mesh, b);

  return {create_isoline(mesh, dista, distb), dista};
}

mesh_point find_midpoint(const bezier_mesh &mesh, const geodesic_path &path) {

  return intersect(mesh, path, find_segment_bisector_isolines(mesh, path))
      .first;
}

mesh_point find_mirror_point(const bezier_mesh &mesh, const geodesic_path &path,
                             const mesh_point &point, const float &tol) {
  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;
  auto &adjacencies = mesh.adjacencies;

  auto ix = find_in_vec(path.strip, point.face);

  if (ix != -1) {
    auto p = eval_position(triangles, positions, point);
    auto s = eval_position(triangles, positions,
                           get_geodesic_path_point(mesh, path, ix));
    auto e = eval_position(triangles, positions,
                           get_geodesic_path_point(mesh, path, ix + 1));

    auto theta = length(cross(normalize(p - s), normalize(e - s)));

    if (theta > -tol && theta < tol) {
      return point;
    }
  }

  auto cp = create_circle(mesh, point, 0.0F);

  auto ds = get_distance(triangles[path.start.face], get_bary(path.start.uv),
                         cp.distances);
  auto de = get_distance(triangles[path.end.face], get_bary(path.end.uv),
                         cp.distances);

  auto radius = min(ds, de);

  set_radius(mesh, &cp, radius);

  auto [p0, p1] = intersect(mesh, path, cp);

  if (p0.face == -1) {
    return mesh_point();
  }

  auto cs = Circle(), ce = Circle();

  if (p1.face == -1) {
    auto ps = ds <= de ? path.start : path.end;
    cs = create_circle(mesh, ps, point);

    auto [p0, p1] = intersect(mesh, path, cs);
    if (p0.face == -1) {
      ce = create_circle(mesh,
                         further_point(triangles, positions, adjacencies, ps,
                                       path.start, path.end),
                         point);
    } else {
      ce = create_circle(
          mesh, further_point(triangles, positions, adjacencies, ps, p0, p1),
          point);
    }
  } else {
    cs = create_circle(mesh, p0, point);
    ce = create_circle(mesh, p1, point);
  }

  tie(p0, p1) = intersect(mesh, cs, ce);

  if (p0.face == -1) {
    return mesh_point();
  }

  return further_point(triangles, positions, adjacencies, point, p0, p1);
}

geodesic_path find_perpendicular(const bezier_mesh &mesh,
                                 const geodesic_path &path,
                                 const mesh_point &point, const float &tol) {
  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;
  auto &adjacencies = mesh.adjacencies;

  auto mirror = find_mirror_point(mesh, path, point, tol);

  if (is_same_point(triangles, positions, adjacencies, point, mirror)) {
    auto points = path_positions(path, mesh);
    auto p = eval_position(triangles, positions, point);
    auto ix = find_in_vec(path.strip, point.face);

    auto len = 0.0F;
    auto dist = 0.0F;

    for (auto i = 1; i < points.size(); i++) {
      if (ix == i - 1) {
        dist = len + length(p - points[i - 1]);
      }
      len += length(points[i] - points[i - 1]);
    }

    auto s = eval_position(triangles, positions,
                           get_geodesic_path_point(mesh, path, ix));
    auto e = eval_position(triangles, positions,
                           get_geodesic_path_point(mesh, path, ix + 1));
    auto dir = normalize(e - s);

    if (dist > len / 2) {
      dir = -dir;
    }

    // auto pp_start = mesh_point(), pp_end = mesh_point();

    dir = rotate_in_tangent_space(tid_normal(triangles, positions, point.face),
                                  dir, pi / 4);
    auto g = straightest_geodesic(mesh, point, dir, len / 3);
    auto center = g[g.size() - 1];

    auto circ = create_circle(mesh, center, point);

    auto [p0, p1] = intersect(mesh, path, circ);

    auto p2 = point.face == p0.face ? p1 : p0;

    auto circ2 = create_circle(mesh, point, p2);

    tie(p0, p1) = intersect(mesh, circ, circ2);

    return compute_geodesic_path(
        mesh, point,
        further_point(triangles, positions, adjacencies, p2, p0, p1));
  }

  return compute_geodesic_path(mesh, point, mirror);
}

std::tuple<geodesic_path, Circle, Circle, Circle>
find_angle_bisector(const bezier_mesh &mesh, const geodesic_path &first,
                    const geodesic_path &second, const float &scale_factor) {
  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;
  auto &adjacencies = mesh.adjacencies;

  auto center = closer_point(triangles, positions, adjacencies, second.start,
                             first.start, first.end);

  if (center.face == -1) {
    center = closer_point(triangles, positions, adjacencies, second.end,
                          first.start, first.end);

    if (center.face == -1) {
      return {};
    }
  }

  auto lf = path_length(first, triangles, positions, adjacencies);
  auto ls = path_length(second, triangles, positions, adjacencies);

  auto circle = create_circle(mesh, center, min(lf, ls) * scale_factor);

  auto [cf0, cf1] = intersect(mesh, first, circle);
  auto circle_f = create_circle(mesh, cf0, 0.0F);
  set_radius(mesh, &circle_f,
             get_distance(triangles[center.face], get_bary(center.uv),
                          circle_f.distances));

  auto [cs0, cs1] = intersect(mesh, second, circle);
  auto circle_s = create_circle(mesh, cs0, 0.0F);
  set_radius(mesh, &circle_s,
             get_distance(triangles[center.face], get_bary(center.uv),
                          circle_s.distances));

  auto [i0, i1] = intersect(mesh, circle_f, circle_s);

  return {compute_geodesic_path(
              mesh, center,
              further_point(triangles, positions, adjacencies, center, i0, i1)),
          circle, circle_f, circle_s};
}

geodesic_path find_tangent_to_circle(const bezier_mesh &mesh,
                                     const Circle &circle,
                                     const mesh_point &point,
                                     const float &tol) {
  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;
  auto &adjacencies = mesh.adjacencies;

  for (auto curve : circle.isoline) {
    auto ix = find_in_vec(curve.strip, point.face);

    if (ix != -1) {
      auto p = eval_position(triangles, positions, point);
      auto s = eval_position(
          triangles, positions,
          get_closed_curve_point(triangles, positions, adjacencies, curve, ix));
      auto e =
          eval_position(triangles, positions,
                        get_closed_curve_point(triangles, positions,
                                               adjacencies, curve, ix + 1));

      auto theta = length(cross(normalize(p - s), normalize(e - s)));

      if (theta > -tol && theta < tol) {
        return find_perpendicular(
            mesh, compute_geodesic_path(mesh, circle.center, point), point);
      }
    }
  }

  return geodesic_path();
}

std::tuple<Circle, geodesic_path, geodesic_path>
find_circle_3_points(const bezier_mesh &mesh, const mesh_point &a,
                     const mesh_point &b, const mesh_point &c) {
  auto [ab_bis, c0, c1] =
      find_segment_bisector(mesh, compute_geodesic_path(mesh, a, b));
  auto [bc_bis, c2, c3] =
      find_segment_bisector(mesh, compute_geodesic_path(mesh, b, c));

  if (!validate_paths({ab_bis, bc_bis})) {
    return {};
  }

  auto center = intersect(mesh, ab_bis, bc_bis);

  return {create_circle(mesh, center, a), ab_bis, bc_bis};
}

geodesic_path find_parallel(const bezier_mesh &mesh, const geodesic_path &path,
                            const mesh_point &point) {
  auto &triangles = mesh.triangles;

  auto p = path.start;
  auto l = compute_geodesic_path(mesh, p, point);
  auto perp = find_perpendicular(mesh, l, point);
  auto m = find_mirror_point(mesh, perp, p);

  if (m.face != -1) {
    auto circ_s = create_circle(mesh, path.start, path.end);
    auto circ_p = create_circle(mesh, point, path.end);
    auto circ_m = create_circle(mesh, m, circ_p.radius);

    set_radius(mesh, &circ_p, circ_s.radius);

    auto [i0, i1] = intersect(mesh, circ_p, circ_m);

    if (i0.face == -1) {
      return geodesic_path();
    }

    auto dist = get_geodesic_distances(mesh, path.end);
    auto pe = get_distance(triangles[i0.face], get_bary(i0.uv), dist) <=
                      get_distance(triangles[i1.face], get_bary(i1.uv), dist)
                  ? i0
                  : i1;

    return compute_geodesic_path(mesh, point, pe);
  }

  return geodesic_path();
}

geodesic_path generate_path(const bezier_mesh &mesh, const closed_curve &curve,
                            const mesh_point &point, float length) {
  auto points = closed_curve_positions(curve, mesh.triangles, mesh.positions,
                                       mesh.adjacencies);
  points.pop_back();

  auto size = points.size();

  auto offset = find_in_vec(curve.strip, point.face);

  if (offset == -1) {
    return geodesic_path();
  }

  auto p = eval_position(mesh.triangles, mesh.positions, point);

  auto next = offset;
  auto prev = (offset + size - 1) % size;

  auto path0 = geodesic_path();
  path0.start = point;
  path0.strip.push_back(curve.strip[next]);
  path0.lerps.push_back(curve.lerps[next]);

  auto len = distance(p, points[next]);

  while (len < length / 2.0F) {
    prev = next;
    next = (next + 1) % size;
    path0.strip.push_back(curve.strip[next]);
    path0.lerps.push_back(curve.lerps[next]);
    len += distance(points[prev], points[next]);
  }

  path0.lerps.pop_back();
  path0.end = eval_mesh_point(
      mesh.triangles, mesh.positions, path0.strip.back(),
      lerp(points[next], points[prev],
           (len - length / 2) / distance(points[prev], points[next])));

  next = offset;
  prev = (offset + size - 1) % size;

  auto path1 = geodesic_path();
  path1.start = point;
  path1.strip.push_back(curve.strip[next]);
  path1.lerps.push_back(1.0F - curve.lerps[prev]);

  len = distance(p, points[prev]);

  while (len < length / 2.0F) {
    next = (int)prev;
    prev = (prev + size - 1) % size;
    path1.strip.push_back(curve.strip[next]);
    path1.lerps.push_back(1.0F - curve.lerps[prev]);
    len += distance(points[prev], points[next]);
  }

  path1.lerps.pop_back();
  path1.end = eval_mesh_point(
      mesh.triangles, mesh.positions, path1.strip.back(),
      lerp(points[prev], points[next],
           (len - length / 2) / distance(points[prev], points[next])));

  return join_paths(reverse(path1), path0);
}

geodesic_path generate_path(const bezier_mesh &mesh, const closed_curve &curve,
                            const mesh_point &point, const vec3f &dir,
                            float length) {
  auto points = closed_curve_positions(curve, mesh.triangles, mesh.positions,
                                       mesh.adjacencies);
  points.pop_back();

  auto size = points.size();

  auto offset = find_in_vec(curve.strip, point.face);

  if (offset == -1) {
    return geodesic_path();
  }

  auto p = eval_position(mesh.triangles, mesh.positions, point);

  if (dot(dir, normalize(points[offset] - p)) > 0) {
    auto next = offset;
    auto prev = (offset + size - 1) % size;

    auto path = geodesic_path();
    path.start = point;
    path.strip.push_back(curve.strip[next]);
    path.lerps.push_back(curve.lerps[next]);

    auto len = distance(p, points[next]);

    while (len < length) {
      prev = next;
      next = (next + 1) % size;
      path.strip.push_back(curve.strip[next]);
      path.lerps.push_back(curve.lerps[next]);
      len += distance(points[prev], points[next]);
    }

    path.lerps.pop_back();
    path.end = eval_mesh_point(
        mesh.triangles, mesh.positions, path.strip.back(),
        lerp(points[next], points[prev],
             (len - length) / distance(points[prev], points[next])));

    return path;
  } else {
    auto next = offset;
    auto prev = (offset + size - 1) % size;

    auto path = geodesic_path();
    path.start = point;
    path.strip.push_back(curve.strip[next]);
    path.lerps.push_back(1.0F - curve.lerps[prev]);

    auto len = distance(p, points[prev]);

    while (len < length) {
      next = (int)prev;
      prev = (prev + size - 1) % size;
      path.strip.push_back(curve.strip[next]);
      path.lerps.push_back(1.0F - curve.lerps[prev]);
      len += distance(points[prev], points[next]);
    }

    path.lerps.pop_back();
    path.end = eval_mesh_point(
        mesh.triangles, mesh.positions, path.strip.back(),
        lerp(points[prev], points[next],
             (len - length) / distance(points[prev], points[next])));

    return path;
  }

  return geodesic_path();
}

geodesic_path generate_path(const bezier_mesh &mesh, const closed_curve &curve,
                            const mesh_point &point, const mesh_point &goal) {
  auto points = closed_curve_positions(curve, mesh.triangles, mesh.positions,
                                       mesh.adjacencies);
  points.pop_back();

  auto size = points.size();

  auto point_offset = find_in_vec(curve.strip, point.face);
  auto goal_offset = find_in_vec(curve.strip, goal.face);

  if (point_offset == -1 || goal_offset == -1) {
    return geodesic_path();
  }

  auto half = ceil(curve.strip.size() / 2.0F);

  if ((goal_offset + curve.strip.size() - point_offset) % curve.strip.size() >=
      half) {
    auto p = eval_position(mesh.triangles, mesh.positions, point);

    auto next = point_offset;
    auto prev = (point_offset + size - 1) % size;

    auto path1 = geodesic_path();
    path1.start = point;
    path1.end = goal;
    path1.strip.push_back(curve.strip[next]);
    path1.lerps.push_back(1.0F - curve.lerps[prev]);

    auto length = distance(p, points[prev]);

    while (next != goal_offset) {
      next = (int)prev;
      prev = (prev + size - 1) % size;
      path1.strip.push_back(curve.strip[next]);
      path1.lerps.push_back(1.0F - curve.lerps[prev]);
      length += distance(points[prev], points[next]);
    }

    path1.lerps.pop_back();
    length -= distance(points[prev],
                       eval_position(mesh.triangles, mesh.positions, goal));

    next = point_offset;
    prev = (point_offset + size - 1) % size;

    auto path0 = geodesic_path();
    path0.start = point;
    path0.strip.push_back(curve.strip[next]);
    path0.lerps.push_back(curve.lerps[next]);

    auto len = distance(p, points[next]);

    while (len < length) {
      prev = next;
      next = (next + 1) % size;
      path0.strip.push_back(curve.strip[next]);
      path0.lerps.push_back(curve.lerps[next]);
      len += distance(points[prev], points[next]);
    }

    path0.lerps.pop_back();
    path0.end = eval_mesh_point(
        mesh.triangles, mesh.positions, path0.strip.back(),
        lerp(points[next], points[prev],
             (len - length) / distance(points[prev], points[next])));

    return join_paths(reverse(path1), path0);
  } else {
    auto p = eval_position(mesh.triangles, mesh.positions, point);

    auto next = point_offset;
    auto prev = (point_offset + size - 1) % size;

    auto path0 = geodesic_path();
    path0.start = point;
    path0.end = goal;
    path0.strip.push_back(curve.strip[next]);
    path0.lerps.push_back(curve.lerps[next]);

    auto length = distance(p, points[next]);

    while (next != goal_offset) {
      prev = next;
      next = (next + 1) % size;
      path0.strip.push_back(curve.strip[next]);
      path0.lerps.push_back(curve.lerps[next]);
      length += distance(points[prev], points[next]);
    }

    path0.lerps.pop_back();
    length -= distance(points[next],
                       eval_position(mesh.triangles, mesh.positions, goal));

    next = point_offset;
    prev = (point_offset + size - 1) % size;

    auto path1 = geodesic_path();
    path1.start = point;
    path1.strip.push_back(curve.strip[next]);
    path1.lerps.push_back(1.0F - curve.lerps[prev]);

    auto len = distance(p, points[prev]);

    while (len < length) {
      next = (int)prev;
      prev = (prev + size - 1) % size;
      path1.strip.push_back(curve.strip[next]);
      path1.lerps.push_back(1.0F - curve.lerps[prev]);
      len += distance(points[prev], points[next]);
    }

    path1.lerps.pop_back();
    path1.end = eval_mesh_point(
        mesh.triangles, mesh.positions, path1.strip.back(),
        lerp(points[prev], points[next],
             (len - length) / distance(points[prev], points[next])));

    return join_paths(reverse(path1), path0);
  }

  return geodesic_path();
}

geodesic_path expected_segment_bisector(const bezier_mesh &mesh,
                                        const geodesic_path &path) {
  return expected_perpendicular(mesh, path, expected_midpoint(mesh, path));
}

mesh_point expected_midpoint(const bezier_mesh &mesh,
                             const geodesic_path &path) {
  return eval_path_midpoint(path, mesh.triangles, mesh.positions,
                            mesh.adjacencies);
}

mesh_point expected_mirror_point(const bezier_mesh &mesh,
                                 const geodesic_path &path,
                                 const mesh_point &point, float tol) {
  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;

  auto ix = find_in_vec(path.strip, point.face);

  // If the point is on the line, the mirror is the point itself
  if (ix != -1) {
    auto p = eval_position(triangles, positions, point);
    auto s = eval_position(triangles, positions,
                           get_geodesic_path_point(mesh, path, ix));
    auto e = eval_position(triangles, positions,
                           get_geodesic_path_point(mesh, path, ix + 1));

    auto theta = length(cross(normalize(p - s), normalize(e - s)));

    if (theta > -tol && theta < tol) {
      return point;
    }
  }

  auto perp = expected_perpendicular(mesh, path, point);

  if (!validate_paths({perp})) {
    return mesh_point();
  }

  return perp.end;
}

geodesic_path expected_perpendicular(const bezier_mesh &mesh,
                                     const geodesic_path &path,
                                     const mesh_point &point, const float &len,
                                     float tol) {
  // auto tangent = geodesic_path_tangent(mesh.triangles, mesh.positions,
  //                                      mesh.adjacencies, path, point);
  // auto n = triangle_normal(mesh.positions[mesh.triangles[point.face].x],
  //                          mesh.positions[mesh.triangles[point.face].y],
  //                          mesh.positions[mesh.triangles[point.face].z]);
  // auto len =
  //     path_length(path, mesh.triangles, mesh.positions, mesh.adjacencies);
  // auto perp = normalize(cross(tangent, n));

  // auto straightes = straightest_geodesic(mesh, point, perp, len);
  // auto reverse = straightest_geodesic(mesh, point, rot_vect(perp, n, pif),
  // len); auto path0 = compute_geodesic_path(mesh, straightes.back(), point);
  // auto path1 = compute_geodesic_path(mesh, point, reverse.back());
  // return join_paths(path0, path1);
  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;
  auto &adjacencies = mesh.adjacencies;

  auto pathp = path;

  auto sp = compute_geodesic_path(mesh, pathp.start, point);
  auto dir_sp = geodesic_path_direction(mesh, sp);
  auto dir_s = geodesic_path_direction(mesh, pathp);

  auto ep = compute_geodesic_path(mesh, pathp.end, point);
  auto dir_ep = zero3f;
  if (ep.strip.size() == 1) {
    auto e0 = get_edge(mesh.triangles, mesh.positions, mesh.adjacencies,
                       path.strip.rbegin()[1], path.strip.rbegin()[0]);
    dir_ep = normalize(
        lerp(mesh.positions[e0.x], mesh.positions[e0.y], path.lerps.back()) -
        eval_position(mesh.triangles, mesh.positions, path.end));
  } else
    dir_ep = geodesic_path_direction(mesh, ep);

  auto dir_e = geodesic_path_direction(mesh, pathp, true);

  if (dot(dir_sp, dir_s) <= 0) {
    do {
      pathp = join_paths(reverse(straightest_path(
                             triangles, positions, adjacencies, pathp.start,
                             -direction_2D_in_tangent_space(
                                 triangles, positions, pathp.start, dir_s),
                             0.1F)),
                         pathp);
      sp = compute_geodesic_path(mesh, pathp.start, point);
      dir_sp = geodesic_path_direction(mesh, sp);
      dir_s = geodesic_path_direction(mesh, pathp);
    } while (dot(dir_sp, dir_s) < 0);
  } else if (dot(dir_ep, dir_e) <= 0) {
    do {
      pathp = join_paths(
          pathp, straightest_path(triangles, positions, adjacencies, pathp.end,
                                  -direction_2D_in_tangent_space(
                                      triangles, positions, pathp.end, dir_e),
                                  0.1F));
      ep = compute_geodesic_path(mesh, pathp.end, point);
      dir_ep = geodesic_path_direction(mesh, ep);
      dir_e = geodesic_path_direction(mesh, pathp, true);
    } while (dot(dir_ep, dir_e) < 0);
  }

  // If point is in triangle in path.strip
  auto ixv = find_in_vec(pathp.strip, point.face);

  if (ixv != -1) {
    auto p = eval_position(triangles, positions, point);
    auto points = path_positions(pathp, mesh);
    auto dir0 = normalize(points[ixv + 1] - points[ixv]);
    auto dirp = p - points[ixv];

    // auto theta = length(cross(normalize(dirp), dir0));

    // point on line
    // if (theta > -tol && theta < tol) {
    auto len_of_bis = (len > 0) ? len : 3 * path_length(points);
    auto dir = geodesic_path_direction(mesh, pathp, pathp.strip[ixv]);
    auto dir_r =
        rotate_in_tangent_space(triangles, positions, point, dir, -M_PI / 2.0F);
    auto dir_l =
        rotate_in_tangent_space(triangles, positions, point, dir, M_PI / 2.0F);

    return join_paths(
        reverse(straightest_geodesic_path(mesh, point, dir_l, len_of_bis)),
        straightest_geodesic_path(mesh, point, dir_r, len_of_bis));
  }
  //   }

  //   auto t = dot(dirp, dir0) / distance(points[ixv], points[ixv + 1]);

  //   if (t < 0) {
  //     if (ixv - 1 < 0) {
  //       auto dir = -direction_2D_in_tangent_space(
  //           triangles, positions, pathp.start,
  //           geodesic_path_direction(mesh, pathp));
  //       auto prol = straightest_path(triangles, positions, adjacencies,
  //                                    pathp.start, dir, 0.1F);
  //       auto prol_points = path_positions(prol, mesh);

  //       auto dirp = p - prol_points[ixv + 1];
  //       auto dir0 = normalize(points[ixv + 1] - prol_points[ixv + 1]);
  //       auto t0 =
  //           dot(dirp, dir0) / distance(prol_points[ixv + 1], points[ixv +
  //           1]);

  //       if (t0 >= 0 && t0 <= 1) {
  //         auto p =
  //             eval_mesh_point(triangles, positions, pathp.strip[ixv],
  //                             lerp(prol_points[ixv + 1], points[ixv + 1],
  //                             t0));

  //         auto perp = compute_geodesic_path(mesh, point, p);
  //         auto ext_dir = -direction_2D_in_tangent_space(
  //             triangles, positions, p,
  //             geodesic_path_direction(mesh, perp, true));
  //         auto ext = straightest_path(
  //             triangles, positions, adjacencies, p, ext_dir,
  //             3 * path_length(perp, triangles, positions, adjacencies));

  //         return join_paths(perp, ext);
  //       }

  //       auto dirn = normalize(p - prol_points[ixv + 1]);
  //       auto prev = compute_geodesic_path(
  //           mesh, point, get_geodesic_path_point(mesh, prol, 2));
  //       dir0 = normalize(prol_points[ixv + 1] - prol_points[ixv + 2]);
  //       dirp = geodesic_path_direction(mesh, prev, true);

  //       auto dp0 = dot(dirp, dir0), dn0 = dot(dirn, dir0);

  //       if (dp0 * dn0 < 0) {
  //         auto p =
  //             eval_mesh_point(triangles, positions, prol.strip[ixv + 1],
  //                             lerp(prol_points[ixv + 2], prol_points[ixv +
  //                             1],
  //                                  dp0 / (dp0 - dn0)));

  //         auto perp = compute_geodesic_path(mesh, point, p);
  //         auto ext_dir = -direction_2D_in_tangent_space(
  //             triangles, positions, p,
  //             geodesic_path_direction(mesh, perp, true));
  //         auto ext = straightest_path(
  //             triangles, positions, adjacencies, p, ext_dir,
  //             3 * path_length(perp, triangles, positions, adjacencies));

  //         return join_paths(perp, ext);
  //       }

  //       return geodesic_path();
  //     } else {
  //       auto prev = compute_geodesic_path(mesh, point,
  //                                         eval_mesh_point(triangles,
  //                                         positions,
  //                                                         pathp.strip[ixv -
  //                                                         1], points[ixv -
  //                                                         1]));
  //       auto dir0 = normalize(points[ixv] - points[ixv - 1]);
  //       auto dirp = geodesic_path_direction(mesh, prev, true);
  //       auto dirn = normalize(p - points[ixv]);

  //       auto dp0 = dot(dirp, dir0), dn0 = dot(dirn, dir0);

  //       if (dp0 * dn0 < 0) {
  //         auto p = eval_mesh_point(
  //             triangles, positions, pathp.strip[ixv - 1],
  //             lerp(points[ixv - 1], points[ixv], dp0 / (dp0 - dn0)));

  //         auto perp = compute_geodesic_path(mesh, point, p);
  //         auto ext_dir = -direction_2D_in_tangent_space(
  //             triangles, positions, p,
  //             geodesic_path_direction(mesh, perp, true));
  //         auto ext = straightest_path(
  //             triangles, positions, adjacencies, p, ext_dir,
  //             3 * path_length(perp, triangles, positions, adjacencies));

  //         return join_paths(perp, ext);
  //       }
  //     }
  //   } else if (t > 1) {
  //     if (ixv + 2 >= points.size()) {
  //       auto dir = -direction_2D_in_tangent_space(
  //           triangles, positions, pathp.end,
  //           geodesic_path_direction(mesh, pathp, true));
  //       auto prol = straightest_path(triangles, positions, adjacencies,
  //                                    pathp.end, dir, 0.1F);
  //       auto prol_points = path_positions(prol, mesh);

  //       auto dirp = p - points[ixv];
  //       auto dir0 = normalize(prol_points[1] - points[ixv]);
  //       auto t0 = dot(dirp, dir0) / distance(prol_points[1], points[ixv]);

  //       if (t0 >= 0 && t0 <= 1) {
  //         auto p = eval_mesh_point(triangles, positions, pathp.strip[ixv],
  //                                  lerp(points[ixv], prol_points[1], t0));

  //         auto perp = compute_geodesic_path(mesh, point, p);
  //         auto ext_dir = -direction_2D_in_tangent_space(
  //             triangles, positions, p,
  //             geodesic_path_direction(mesh, perp, true));
  //         auto ext = straightest_path(
  //             triangles, positions, adjacencies, p, ext_dir,
  //             3 * path_length(perp, triangles, positions, adjacencies));

  //         return join_paths(perp, ext);
  //       }

  //       dirp = normalize(p - prol_points[1]);
  //       auto next = compute_geodesic_path(
  //           mesh, point, get_geodesic_path_point(mesh, prol, 2));
  //       auto dirn = geodesic_path_direction(mesh, next, true);
  //       dir0 = normalize(prol_points[ixv + 2] - prol_points[ixv + 1]);

  //       auto dp0 = dot(dirp, dir0), dn0 = dot(dirn, dir0);

  //       if (dp0 * dn0 < 0) {
  //         auto p = eval_mesh_point(
  //             triangles, positions, prol.strip[1],
  //             lerp(prol_points[1], prol_points[2], dp0 / (dp0 - dn0)));

  //         auto perp = compute_geodesic_path(mesh, point, p);
  //         auto ext_dir = -direction_2D_in_tangent_space(
  //             triangles, positions, p,
  //             geodesic_path_direction(mesh, perp, true));
  //         auto ext = straightest_path(
  //             triangles, positions, adjacencies, p, ext_dir,
  //             3 * path_length(perp, triangles, positions, adjacencies));

  //         return join_paths(perp, ext);
  //       }

  //       return geodesic_path();
  //     } else {
  //       auto next = compute_geodesic_path(mesh, point,
  //                                         eval_mesh_point(triangles,
  //                                         positions,
  //                                                         pathp.strip[ixv +
  //                                                         1], points[ixv +
  //                                                         2]));
  //       auto dirp = normalize(p - points[ixv + 1]);
  //       auto dirn = geodesic_path_direction(mesh, next, true);
  //       auto dir0 = normalize(points[ixv + 2] - points[ixv + 1]);

  //       auto dp0 = dot(dirp, dir0), dn0 = dot(dirn, dir0);

  //       if (dp0 * dn0 < 0) {
  //         auto p = eval_mesh_point(
  //             triangles, positions, pathp.strip[ixv + 1],
  //             lerp(points[ixv + 2], points[ixv + 1], dn0 / (dn0 - dp0)));

  //         auto perp = compute_geodesic_path(mesh, point, p);
  //         auto ext_dir = -direction_2D_in_tangent_space(
  //             triangles, positions, p,
  //             geodesic_path_direction(mesh, perp, true));
  //         auto ext = straightest_path(
  //             triangles, positions, adjacencies, p, ext_dir,
  //             3 * path_length(perp, triangles, positions, adjacencies));

  //         return join_paths(perp, ext);
  //       }
  //     }
  //   } else if (t >= 0 && t <= 1) {
  //     auto p = eval_mesh_point(triangles, positions, pathp.strip[ixv],
  //                              lerp(points[ixv], points[ixv + 1], t));

  //     auto perp = compute_geodesic_path(mesh, point, p);
  //     auto ext_dir = -direction_2D_in_tangent_space(
  //         triangles, positions, p, geodesic_path_direction(mesh, perp,
  //         true));
  //     auto ext = straightest_path(
  //         triangles, positions, adjacencies, p, ext_dir,
  //         3 * path_length(perp, triangles, positions, adjacencies));

  //     return join_paths(perp, ext);
  //   }
  // }

  // // Point outside of path strip
  // auto points = path_positions(pathp, mesh);
  // auto distances = get_geodesic_distances(mesh, point);
  // auto ix = 0;
  // auto min = get_distance(pathp.start, mesh, distances);

  // for (auto i = 1; i < points.size(); ++i) {
  //   auto dist = get_distance(
  //       eval_mesh_point(triangles, positions, pathp.strip[i - 1],
  //       points[i]), mesh, distances);

  //   if (dist < min) {
  //     min = dist;
  //     ix = i;
  //   }
  // }

  // auto prevp = compute_geodesic_path(
  //     mesh, point,
  //     eval_mesh_point(triangles, positions, pathp.strip[ix - 1],
  //     points[ix]));
  // auto prevn = compute_geodesic_path(
  //     mesh, point,
  //     eval_mesh_point(triangles, positions, pathp.strip[ix - 1],
  //     points[ix]));

  // for (auto i = 1; i < max(ix + 1, points.size() - ix); ++i) {
  //   if (ix - i >= 0) {
  //     auto nextp = compute_geodesic_path(mesh, point,
  //                                        eval_mesh_point(triangles,
  //                                        positions,
  //                                                        pathp.strip[ix -
  //                                                        i], points[ix -
  //                                                        i]));
  //     auto dirpp = geodesic_path_direction(mesh, prevp, true);
  //     auto dirnp = geodesic_path_direction(mesh, nextp, true);
  //     auto dir0 = normalize(points[ix - i + 1] - points[ix - i]);

  //     auto dpp0 = dot(dirpp, dir0), dnp0 = dot(dirnp, dir0);

  //     if (dpp0 * dnp0 < 0) {
  //       auto p = eval_mesh_point(
  //           triangles, positions, pathp.strip[ix - i],
  //           lerp(points[ix - i], points[ix - i + 1], dpp0 / (dpp0 -
  //           dnp0)));

  //       auto perp = compute_geodesic_path(mesh, point, p);
  //       auto ext_dir = -direction_2D_in_tangent_space(
  //           triangles, positions, p, geodesic_path_direction(mesh, perp,
  //           true));
  //       auto ext = straightest_path(
  //           triangles, positions, adjacencies, p, ext_dir,
  //           3 * path_length(perp, triangles, positions, adjacencies));

  //       return join_paths(perp, ext);
  //     }

  //     prevp = nextp;
  //   }
  //   if (ix + i < points.size()) {
  //     auto nextn = compute_geodesic_path(
  //         mesh, point,
  //         eval_mesh_point(triangles, positions, pathp.strip[ix + i - 1],
  //                         points[ix + i]));
  //     auto dirpn = geodesic_path_direction(mesh, prevn, true);
  //     auto dirnn = geodesic_path_direction(mesh, nextn, true);
  //     auto dir0 = normalize(points[ix + i] - points[ix + i - 1]);

  //     auto dpn0 = dot(dirpn, dir0), dnn0 = dot(dirnn, dir0);

  //     if (dpn0 * dnn0 < 0) {
  //       auto p = eval_mesh_point(
  //           triangles, positions, pathp.strip[ix + i - 1],
  //           lerp(points[ix + i - 1], points[ix + i], dpn0 / (dpn0 -
  //           dnn0)));

  //       auto perp = compute_geodesic_path(mesh, point, p);
  //       auto ext_dir = -direction_2D_in_tangent_space(
  //           triangles, positions, p, geodesic_path_direction(mesh, perp,
  //           true));
  //       auto ext = straightest_path(
  //           triangles, positions, adjacencies, p, ext_dir,
  //           3 * path_length(perp, triangles, positions, adjacencies));

  //       return join_paths(perp, ext);
  //     }

  //     prevn = nextn;
  //   }
  // }

  return geodesic_path();
}

geodesic_path expected_angle_bisector(const bezier_mesh &mesh,
                                      const geodesic_path &first,
                                      const geodesic_path &second) {
  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;
  auto &adjacencies = mesh.adjacencies;

  auto &point = second.start;

  auto len = (path_length(first, triangles, positions, adjacencies) +
              path_length(second, triangles, positions, adjacencies)) /
             2.0F;

  auto normal = tid_normal(mesh.triangles, mesh.positions, point.face);
  auto dir1 = geodesic_path_direction(mesh, first);
  auto dir2 = geodesic_path_direction(mesh, second);
  auto dir = normalize(cross(dir1, normal) + cross(normal, dir2)) *
             dot(normal, normalize(cross(dir2, dir1)));

  return straightest_path(
      triangles, positions, adjacencies, point,
      direction_2D_in_tangent_space(triangles, positions, point, dir), len);
}

geodesic_path expected_tangent_to_circle(const bezier_mesh &mesh,
                                         const Circle &circle,
                                         const mesh_point &point,
                                         const float &tol) {
  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;
  auto &adjacencies = mesh.adjacencies;

  for (auto curve : circle.isoline) {
    info("curve");
    auto ix = find_in_vec(curve.strip, point.face);

    if (ix != -1) {
      info("ix: $", ix);
      auto p = eval_position(triangles, positions, point);
      auto s = eval_position(
          triangles, positions,
          get_closed_curve_point(triangles, positions, adjacencies, curve, ix));
      auto e =
          eval_position(triangles, positions,
                        get_closed_curve_point(triangles, positions,
                                               adjacencies, curve, ix + 1));

      auto theta = length(cross(normalize(p - s), normalize(e - s)));

      info("theta: $", theta);

      if (theta > -tol && theta < tol) {
        return expected_perpendicular(
            mesh, compute_geodesic_path(mesh, circle.center, point), point);
      }
    }
  }

  return geodesic_path();
}

std::tuple<Circle, Isoline, Isoline>
expected_circle_3_points(const bezier_mesh &mesh, const mesh_point &a,
                         const mesh_point &b, const mesh_point &c) {
  auto ab_bis =
      find_segment_bisector_isolines(mesh, compute_geodesic_path(mesh, a, b));
  auto bc_bis =
      find_segment_bisector_isolines(mesh, compute_geodesic_path(mesh, b, c));

  auto intersections = intersect(mesh, ab_bis, bc_bis);
  auto dist = get_geodesic_distances(mesh, a);
  auto center = closer_point(mesh.triangles, mesh.positions, dist, a,
                             intersections.first, intersections.second);
  return {create_circle(mesh, center, a), ab_bis, bc_bis};
}
Circle circle_3_points(const bezier_mesh &mesh, const mesh_point &a,
                       const mesh_point &b, const mesh_point &c) {
  auto ab_bis =
      find_segment_bisector_isolines(mesh, compute_geodesic_path(mesh, a, b));
  auto bc_bis =
      find_segment_bisector_isolines(mesh, compute_geodesic_path(mesh, b, c));

  auto intersections = intersect(mesh, ab_bis, bc_bis);
  auto dist = get_geodesic_distances(mesh, a);
  auto center = closer_point(mesh.triangles, mesh.positions, dist, a,
                             intersections.first, intersections.second);
  auto result = create_circle(mesh, center, a);
  return result;
}

geodesic_path expected_parallel(const bezier_mesh &mesh,
                                const geodesic_path &path,
                                const mesh_point &point) {

  auto perp = expected_perpendicular(mesh, path, point);
  return expected_perpendicular(mesh, perp, point);
}

pair<int, float> vector_binary_search(const vector<float> &v, float t) {
  auto L = 0, R = (int)v.size(), entry = -1, m = 0;
  auto factor = 0.0;
  while (L < R) {
    m = (int)floor((L + R) / 2);
    if (t < v[m])
      R = m;
    else
      L = m + 1;
  }
  entry = R - 1;
  if (entry == v.size() - 1) {
    assert(t >= v[entry - 1] && t <= v[entry]);
    auto width = (v[entry] - v[entry - 1]);
    factor = (t - v[entry - 1]) / width;
  } else if (t >= v[entry] && t <= v[entry + 1]) {
    auto width = (v[entry + 1] - v[entry]);
    factor = (t - v[entry]) / width;
  } else
    assert(false);

  return {entry, factor};
}

pair<mesh_point, int>
sample_on_curve(const vector<vec3i> &triangles, const vector<vec3f> &positions,
                const vector<vec3i> &adjacencies, const geodesic_path &path,
                const vector<float> &path_parameter_t, const float &t) {
  // WHY DOUBLE IN THE INTERFACE?
  auto a = mesh_point{};
  auto entry = -1;
  if (path.strip.size() == 1) {
    entry = 0;
    auto p = eval_position(triangles, positions, path.start);
    auto q = eval_position(triangles, positions, path.end);
    auto start = (p + q) / 2;
    auto tid = path.end.face;
    auto [inside, bary] = point_in_triangle(triangles, positions, tid, start);

    assert(inside);

    a = {tid, bary};
  } else {
    auto i = vector_binary_search(path_parameter_t, t);
    auto p = zero3f, q = zero3f;
    auto tid = -1;
    entry = i.first;
    if (entry == 0) {
      tid = path.start.face;
      p = eval_position(triangles, positions, path.start);
      q = path_pos_from_entry(triangles, positions, adjacencies, path, 0);
    } else if (entry == path_parameter_t.size() - 1) {
      return std::make_pair(path.end, entry - 1);

    } else {
      p = path_pos_from_entry(triangles, positions, adjacencies, path,
                              i.first - 1);
      q = (entry == path.strip.size() - 1)
              ? eval_position(triangles, positions, path.end)
              : path_pos_from_entry(triangles, positions, adjacencies, path,
                                    i.first);
      tid = path.strip[i.first];
    }
    auto start = q - p;
    start *= i.second;
    start += p;

    auto [inside, bary] = point_in_triangle(triangles, positions, tid, start);
    if (!inside) {
      tid = path.strip[i.first - 1];
      std::tie(inside, bary) =
          point_in_triangle(triangles, positions, tid, start);
      entry = i.first - 1;
    }
    assert(inside);
    a = {tid, bary};
  }

  return std::make_pair(a, entry);
}
pair<geodesic_path, vector<float>>
cut_curve(const geodesic_path &path, const vector<float> &path_parameter_t,
          int entry, const mesh_point &sample, bool keep_the_first_portion) {
  assert(path.strip.size() == path.lerps.size() + 1);
  assert(path.strip.size() == path_parameter_t.size() - 1);
  assert(entry >= 0 && entry < path.strip.size());
  assert(sample.face == path.strip[entry]);
  // output vars
  auto cut = geodesic_path{};
  auto cut_t = vector<float>{};

  if (keep_the_first_portion) {
    cut.start = path.start;
    cut.end = sample;
    cut.strip.insert(cut.strip.end(), path.strip.begin(),
                     path.strip.begin() + entry + 1);
    cut.lerps.insert(cut.lerps.end(), path.lerps.begin(),
                     path.lerps.begin() + entry);

    cut_t.insert(cut_t.end(), path_parameter_t.begin(),
                 path_parameter_t.begin() + entry + 1);
    assert(cut_t.back() <= 0.5);
    cut_t.push_back(0.5);
    for (int i = 0; i < cut_t.size(); i++)
      cut_t[i] *= 2;
  } else {
    cut.start = sample;
    cut.end = path.end;
    cut.strip.insert(cut.strip.end(), path.strip.begin() + entry,
                     path.strip.end());
    cut.lerps.insert(cut.lerps.end(), path.lerps.begin() + entry,
                     path.lerps.end());
    cut_t.insert(cut_t.end(), path_parameter_t.begin() + entry,
                 path_parameter_t.end());
    assert(cut_t[0] < 0.5);
    cut_t[0] = 0; // we take the t corresponding to the prev sample and we
                  // put it to zero
    for (int i = 1; i < cut_t.size(); i++)
      cut_t[i] = (cut_t[i] - 0.5) * 2;
  }
  assert(cut.strip.size() == cut.lerps.size() + 1);
  assert(cut.strip.size() == cut_t.size() - 1);
  return std::make_pair(cut, cut_t);
}
std::tuple<vector<mesh_point>, Circle, Circle>
equilateral_triangle(const bezier_mesh &mesh, const mesh_point &a,
                     const mesh_point &b, const bool flipped) {
  auto len = path_length(compute_geodesic_path(mesh, a, b), mesh.triangles,
                         mesh.positions, mesh.adjacencies);
  auto c0 = create_circle(mesh, a, len);
  auto c1 = create_circle(mesh, b, len);

  auto [i0, i1] = intersect(mesh, c0, c1);
  if (flipped)
    return {{a, b, i1}, c0, c1};
  return {{a, b, i0}, c0, c1};
}
// https://www.petercollingridge.co.uk/tutorials/computational-geometry/circle-circle-intersections/
std::pair<vec2f, vec2f> intersect_circles(const vec2f &c0, const vec2f c1,
                                          const float &r0, const float &r1) {
  auto v = c1 - c0;
  auto d = length(v);
  v /= d;
  if (d > r0 + r1)
    return {};

  if (d < std::abs(r0 - r1))
    return {};

  auto a = (float)(pow(r0, 2) - pow(r1, 2) + pow(d, 2)) / (2 * d);
  auto h = (float)sqrt(pow(r0, 2) - pow(a, 2));
  auto p = c0 + a * v;
  return {{p.x + h * v.y, p.y - h * v.x}, {p.x - h * v.y, p.y + h * v.x}};
}
// https://cp-algorithms.com/geometry/circle-line-intersection.html
// eq of the line: ax+by+c=0
std::pair<vec2f, vec2f>
intersect_line_with_circle(const vec2f &o, const float &r, const float &a,
                           const float &b, const float &c) {
  auto den = (float)(pow(a, 2) + pow(b, 2));
  auto d0 = std::abs(c) / yocto::sqrt(den);
  if (d0 >= r)
    return {};
  auto d = yocto::sqrt(pow(r, 2) - pow(c, 2) / den);
  auto m = yocto::sqrt(pow(d, 2) / den);
  auto p = vec2f{-a * c / den, -b * c / den};
  return {{p.x + b * m, p.y - a * m}, {p.x - b * m, p.y + a * m}};
}

vector<mesh_point> equilateral_triangle_tangent_space(const bezier_mesh &mesh,
                                                      const Circle *circle,
                                                      const float teta) {
  auto r = circle->radius;
  auto o = circle->center;
  auto p = vec2f{-r * std::cos(teta), -r * std::sin(teta)};
  auto [b, c] = intersect_circles(vec2f{0, 0}, p, r, r);
  auto teta_b = yocto::atan2(b.y, b.x);
  auto teta_c = yocto::atan2(c.y, c.x);

  auto [is_vert, kv] = point_is_vert(o);
  mesh_point p0, p1, p2;
  vec3f n, e = zero3f;
  if (is_vert) {
    auto vid = mesh.triangles[o.face][kv];
    n = mesh.normals[vid];
    e = polar_basis(mesh.solver, mesh.positions, mesh.normals, vid);
  } else {
    e = polar_basis(mesh.triangles, mesh.positions, o.face);
    n = tid_normal(mesh.triangles, mesh.positions, o.face);
  }
  p0 = straightest_geodesic(mesh, o, rot_vect(e, n, teta), r).back();
  p1 = straightest_geodesic(mesh, o, rot_vect(e, n, teta_b), r).back();
  p2 = straightest_geodesic(mesh, o, rot_vect(e, n, teta_c), r).back();

  return {p0, p1, p2};
}
vector<mesh_point> square_in_tangent_space(const bezier_mesh &mesh,
                                           const Circle *circle,
                                           const float &teta) {
  auto r = circle->radius;
  auto o = circle->center;
  auto p = vec2f{r * std::cos(teta), r * std::sin(teta)};
  auto [i0, i1] = intersect_circles(vec2f{0, 0}, p, r, r);

  auto teta_Q0 = yocto::atan2(i0.y, i0.x);
  auto Q0 = vec2f{r * std::cos(teta_Q0), r * std::sin(teta_Q0)};

  auto teta_j = 0.f;
  if (cross(p, Q0) > 0) {
    teta_j = teta_Q0 - pif / 2;
  } else
    teta_j = teta_Q0 + pif / 2;

  auto [is_vert, kv] = point_is_vert(o);
  mesh_point p0, p1, p2, p3;
  vec3f n, e = zero3f;
  if (is_vert) {
    auto vid = mesh.triangles[o.face][kv];
    n = mesh.normals[vid];
    e = polar_basis(mesh.solver, mesh.positions, mesh.normals, vid);

  } else {
    e = polar_basis(mesh.triangles, mesh.positions, o.face);
    n = tid_normal(mesh.triangles, mesh.positions, o.face);
  }
  p0 = straightest_geodesic(mesh, o, rot_vect(e, n, teta_Q0), r).back();
  p1 = straightest_geodesic(mesh, o, rot_vect(e, n, teta_j), r).back();
  p2 = straightest_geodesic(mesh, o, rot_vect(e, n, teta_Q0 + pif), r).back();
  p3 = straightest_geodesic(mesh, o, rot_vect(e, n, teta_j + pif), r).back();

  return {p0, p1, p2, p3};
}
vector<mesh_point> rhombus_tangent_space(const bezier_mesh &mesh,
                                         const Circle *circle,
                                         const float &lambda,
                                         const float &teta) {
  auto r = circle->radius;
  auto o = circle->center;
  auto p = vec2f{r * std::cos(teta), r * std::sin(teta)};
  auto [i0, i1] = intersect_circles(vec2f{0, 0}, p, r, r);

  auto teta_Q0 = yocto::atan2(i0.y, i0.x);
  auto Q0 = vec2f{r * std::cos(teta_Q0), r * std::sin(teta_Q0)};

  auto teta_j = 0.f;
  if (cross(p, Q0) > 0) {
    teta_j = teta_Q0 - pif / 2;
  } else
    teta_j = teta_Q0 + pif / 2;

  auto [is_vert, kv] = point_is_vert(o);
  mesh_point p0, p1, p2, p3;
  vec3f n, e = zero3f;
  if (is_vert) {
    auto vid = mesh.triangles[o.face][kv];
    n = mesh.normals[vid];
    e = polar_basis(mesh.solver, mesh.positions, mesh.normals, vid);

  } else {
    e = polar_basis(mesh.triangles, mesh.positions, o.face);
    n = tid_normal(mesh.triangles, mesh.positions, o.face);
  }
  p0 =
      straightest_geodesic(mesh, o, rot_vect(e, n, teta_Q0), r * lambda).back();
  p1 = straightest_geodesic(mesh, o, rot_vect(e, n, teta_j), r).back();
  p2 = straightest_geodesic(mesh, o, rot_vect(e, n, teta_Q0 + pif), r * lambda)
           .back();
  p3 = straightest_geodesic(mesh, o, rot_vect(e, n, teta_j + pif), r).back();

  return {p0, p1, p2, p3};
}
vector<mesh_point> parallelogram_tangent_space(const bezier_mesh &mesh,
                                               const Circle *circle,
                                               const float &sigma,
                                               const float &lambda,
                                               const float &teta) {
  auto r = circle->radius;
  auto o = circle->center;
  auto p = vec2f{r * std::cos(teta), r * std::sin(teta)};
  auto [i0, i1] = intersect_circles(vec2f{0, 0}, p, r, sigma * r);
  auto teta_Q1 = yocto::atan2(i0.y, i0.x);
  auto Q1 = vec2f{r * std::cos(teta_Q1), r * std::sin(teta_Q1)};
  if (cross(p, Q1) < 0)
    teta_Q1 = yocto::atan2(i1.y, i1.x);

  auto [is_vert, kv] = point_is_vert(o);
  mesh_point p0, p1, p2, p3;
  vec3f n, e = zero3f;
  if (is_vert) {
    auto vid = mesh.triangles[o.face][kv];
    n = mesh.normals[vid];
    e = polar_basis(mesh.solver, mesh.positions, mesh.normals, vid);

  } else {
    e = polar_basis(mesh.triangles, mesh.positions, o.face);
    n = tid_normal(mesh.triangles, mesh.positions, o.face);
  }
  p0 = straightest_geodesic(mesh, o, rot_vect(e, n, teta), r * lambda).back();
  p1 = straightest_geodesic(mesh, o, rot_vect(e, n, teta_Q1), r).back();
  p2 = straightest_geodesic(mesh, o, rot_vect(e, n, teta + pif), r * lambda)
           .back();
  p3 = straightest_geodesic(mesh, o, rot_vect(e, n, teta_Q1 + pif), r).back();

  return {p0, p1, p2, p3};
}
vector<mesh_point> pentagon_tangent_space(const bezier_mesh &mesh,
                                          const Circle *circle,
                                          const float &teta) {
  auto r = circle->radius;
  auto o = circle->center;
  auto p = vec2f{r * std::cos(teta), r * std::sin(teta)};
  auto d0 = r / 2 * (yocto::sqrt(5) - 1);
  auto r1 = yocto::sqrt(pow(r, 2) + pow(d0, 2));
  auto [Q1, Q4] = intersect_circles(vec2f{0, 0}, p, r, r1);
  auto r2 = length(Q1 - p);
  auto [g0, g1] = intersect_circles(vec2f{0, 0}, Q1, r, r2);
  auto [h0, h1] = intersect_circles(vec2f{0, 0}, Q4, r, r2);
  auto Q2 = (length(g0 - p) < 1e-4) ? g1 : g0;
  auto Q3 = (length(h0 - p) < 1e-4) ? h1 : h0;
  auto teta1 = yocto::atan2(Q1.y, Q1.x);
  auto teta2 = yocto::atan2(Q2.y, Q2.x);
  auto teta3 = yocto::atan2(Q3.y, Q3.x);
  auto teta4 = yocto::atan2(Q4.y, Q4.x);

  auto [is_vert, kv] = point_is_vert(o);
  mesh_point p0, p1, p2, p3, p4;
  vec3f n, e = zero3f;
  if (is_vert) {
    auto vid = mesh.triangles[o.face][kv];
    n = mesh.normals[vid];
    e = polar_basis(mesh.solver, mesh.positions, mesh.normals, vid);

  } else {
    e = polar_basis(mesh.triangles, mesh.positions, o.face);
    n = tid_normal(mesh.triangles, mesh.positions, o.face);
  }
  p0 = straightest_geodesic(mesh, o, rot_vect(e, n, teta), r).back();
  p1 = straightest_geodesic(mesh, o, rot_vect(e, n, teta1), r).back();
  p2 = straightest_geodesic(mesh, o, rot_vect(e, n, teta2), r).back();
  p3 = straightest_geodesic(mesh, o, rot_vect(e, n, teta3), r).back();
  p4 = straightest_geodesic(mesh, o, rot_vect(e, n, teta4), r).back();

  return {p0, p1, p2, p3, p4};
}
vector<mesh_point> hexagon_tangent_space(const bezier_mesh &mesh,
                                         const Circle *circle,
                                         const float &teta) {
  auto r = circle->radius;
  auto o = circle->center;
  auto Q0 = vec2f{r * std::cos(teta), r * std::sin(teta)};
  auto [Q1, Q5] = intersect_circles(vec2f{0, 0}, Q0, r, r);
  auto [Q2, Q4] = intersect_circles(vec2f{0, 0}, Q1, r, r);
  auto teta1 = yocto::atan2(Q1.y, Q1.x);
  auto teta2 = yocto::atan2(Q2.y, Q2.x);
  auto [is_vert, kv] = point_is_vert(o);
  mesh_point p0, p1, p2, p3, p4, p5;
  vec3f n, e = zero3f;
  if (is_vert) {
    auto vid = mesh.triangles[o.face][kv];
    n = mesh.normals[vid];
    e = polar_basis(mesh.solver, mesh.positions, mesh.normals, vid);

  } else {
    e = polar_basis(mesh.triangles, mesh.positions, o.face);
    n = tid_normal(mesh.triangles, mesh.positions, o.face);
  }
  p0 = straightest_geodesic(mesh, o, rot_vect(e, n, teta), r).back();
  p1 = straightest_geodesic(mesh, o, rot_vect(e, n, teta1), r).back();
  p2 = straightest_geodesic(mesh, o, rot_vect(e, n, teta2), r).back();
  p3 = straightest_geodesic(mesh, o, rot_vect(e, n, teta + pif), r).back();
  p4 = straightest_geodesic(mesh, o, rot_vect(e, n, teta1 + pif), r).back();
  p5 = straightest_geodesic(mesh, o, rot_vect(e, n, teta2 + pif), r).back();

  return {p0, p1, p2, p3, p4, p5};
}

vector<mesh_point> octagon_tangent_space(const bezier_mesh &mesh,
                                         const Circle *circle) {
  auto theta = circle->theta;
  auto radius = circle->radius;
  auto center = circle->center;
  auto step = 2 * pif / (float)8;
  auto result = vector<mesh_point>(8);
  auto [is_vert, kv] = point_is_vert(center);
  vec3f n, e = zero3f;
  if (is_vert) {
    auto vid = mesh.triangles[center.face][kv];
    n = mesh.normals[vid];
    e = polar_basis(mesh.solver, mesh.positions, mesh.normals, vid);

  } else {
    e = polar_basis(mesh.triangles, mesh.positions, center.face);
    n = tid_normal(mesh.triangles, mesh.positions, center.face);
  }
  for (auto i = 0; i < 8; ++i) {
    result[i] = straightest_geodesic(mesh, center,
                                     rot_vect(e, n, theta + i * step), radius)
                    .back();
  }
  return result;
}
vector<mesh_point> decagon_tangent_space(const bezier_mesh &mesh,
                                         const Circle *circle) {
  auto theta = circle->theta;
  auto radius = circle->radius;
  auto center = circle->center;
  auto step = 2 * pif / (float)10;
  auto result = vector<mesh_point>(10);
  auto [is_vert, kv] = point_is_vert(center);
  vec3f n, e = zero3f;
  if (is_vert) {
    auto vid = mesh.triangles[center.face][kv];
    n = mesh.normals[vid];
    e = polar_basis(mesh.solver, mesh.positions, mesh.normals, vid);

  } else {
    e = polar_basis(mesh.triangles, mesh.positions, center.face);
    n = tid_normal(mesh.triangles, mesh.positions, center.face);
  }
  for (auto i = 0; i < 10; ++i) {
    result[i] = straightest_geodesic(mesh, center,
                                     rot_vect(e, n, theta + i * step), radius)
                    .back();
  }
  return result;
}
std::tuple<vector<mesh_point>, geodesic_path, Circle>
altitude_isoscele_triangle(const bezier_mesh &mesh, const mesh_point &a,
                           const mesh_point &b, const float &len,
                           const bool flipped) {
  auto path = compute_geodesic_path(mesh, a, b);
  auto iso = find_segment_bisector_isolines(mesh, path);
  auto midpoint = intersect(mesh, path, iso);
  auto bis = expected_perpendicular(mesh, path, midpoint.first);
  auto c0 = create_circle(mesh, midpoint.first, len);
  auto [i0, i1] = intersect(mesh, bis, c0.isoline);

  if (flipped)
    return {{a, b, i0}, bis, c0};
  return {{a, b, i1}, bis, c0};
}
std::tuple<vector<mesh_point>, Circle, Circle>
same_lengths_isoscele_triangle(const bezier_mesh &mesh, const mesh_point &a,
                               const mesh_point &b, const float &len,
                               const bool flipped) {

  auto c0 = create_circle(mesh, a, len);
  auto c1 = create_circle(mesh, b, len);
  auto [i0, i1] = intersect(mesh, c0, c1);
  if (flipped)
    return {{a, b, i1}, c0, c1};

  return {{a, b, i0}, c0, c1};
}
std::tuple<vector<mesh_point>, geodesic_path, Circle, Circle, Circle>
euclidean_rectangle(const bezier_mesh &mesh, const geodesic_path &base,
                    const float &height) {
  auto width =
      path_length(base, mesh.triangles, mesh.positions, mesh.adjacencies);
  auto e0 = get_edge(mesh.triangles, mesh.positions, mesh.adjacencies,
                     base.strip[0], base.strip[1]);
  auto d = lerp(mesh.positions[e0.x], mesh.positions[e0.y], base.lerps[0]) -
           eval_position(mesh.triangles, mesh.positions, base.start);
  d = rotate_in_tangent_space(mesh.triangles, mesh.positions, base.start,
                              normalize(d), M_PI / 2);
  auto h = straightest_geodesic_path(mesh, base.start, d, 3 * height);
  auto c0 = create_circle(mesh, base.start, height);
  auto c = intersect(mesh, h, c0);
  auto c1 = create_circle(mesh, c.first, width);
  auto c2 = create_circle(mesh, base.end, height);
  auto [d0, d1] = intersect(mesh, c1, c2);
  auto check0 = compute_geodesic_path(mesh, c.first, d0);
  auto check1 = compute_geodesic_path(mesh, d0, base.end);
  if (intersect(mesh, base, check0).face == -1 &&
      intersect(mesh, h, check1).face == -1)
    return {{base.start, c.first, d0, base.end}, h, c0, c1, c2};

  return {{base.start, c.first, d1, base.end}, h, c0, c1, c2};
}
std::tuple<vector<mesh_point>, geodesic_path, geodesic_path, Circle, Circle>
same_lengths_rectangle(const bezier_mesh &mesh, const geodesic_path &base,
                       const float &height) {
  auto e0 = get_edge(mesh.triangles, mesh.positions, mesh.adjacencies,
                     base.strip[0], base.strip[1]);
  auto dir = lerp(mesh.positions[e0.x], mesh.positions[e0.y], base.lerps[0]) -
             eval_position(mesh.triangles, mesh.positions, base.start);
  dir = rotate_in_tangent_space(mesh.triangles, mesh.positions, base.start,
                                normalize(dir), M_PI / 2);
  auto h0 = straightest_geodesic_path(mesh, base.start, dir, 2 * height);
  e0 = get_edge(mesh.triangles, mesh.positions, mesh.adjacencies,
                base.strip.rbegin()[1], base.strip.rbegin()[0]);
  dir = lerp(mesh.positions[e0.x], mesh.positions[e0.y], base.lerps.back()) -
        eval_position(mesh.triangles, mesh.positions, base.end);
  dir = rotate_in_tangent_space(mesh.triangles, mesh.positions, base.end,
                                normalize(dir), -M_PI / 2);
  auto h1 = straightest_geodesic_path(mesh, base.end, dir, 2 * height);
  auto c0 = create_circle(mesh, base.start, height);
  auto c1 = create_circle(mesh, base.end, height);
  auto c = intersect(mesh, h0, c0);
  auto d = intersect(mesh, h1, c1);
  return {{base.start, c.first, d.first, base.end}, h0, h1, c0, c1};
}
std::tuple<vector<mesh_point>, geodesic_path, geodesic_path, geodesic_path>
diagonal_rectangle(const bezier_mesh &mesh, const geodesic_path &base,
                   const geodesic_path &diagonal) {
  auto width =
      path_length(base, mesh.triangles, mesh.positions, mesh.adjacencies);
  auto e0 = get_edge(mesh.triangles, mesh.positions, mesh.adjacencies,
                     base.strip[0], base.strip[1]);
  auto base_dir =
      lerp(mesh.positions[e0.x], mesh.positions[e0.y], base.lerps[0]) -
      eval_position(mesh.triangles, mesh.positions, base.start);

  auto e1 = get_edge(mesh.triangles, mesh.positions, mesh.adjacencies,
                     diagonal.strip[0], diagonal.strip[1]);
  auto diag_dir =
      lerp(mesh.positions[e1.x], mesh.positions[e1.y], diagonal.lerps[0]) -
      eval_position(mesh.triangles, mesh.positions, diagonal.start);
  auto theta = angle(diag_dir, base_dir);

  auto e2 = get_edge(mesh.triangles, mesh.positions, mesh.adjacencies,
                     diagonal.strip.rbegin()[1], diagonal.strip.rbegin()[0]);

  auto diag_end_dir =
      lerp(mesh.positions[e2.x], mesh.positions[e2.y], diagonal.lerps.back()) -
      eval_position(mesh.triangles, mesh.positions, diagonal.end);

  auto ad_dir = rotate_in_tangent_space(
      mesh.triangles, mesh.positions, base.start, normalize(base_dir), pif / 2);
  auto h0 = straightest_geodesic_path(mesh, base.start, ad_dir, 1.5 * width);

  auto cd_dir = rotate_in_tangent_space(mesh.triangles, mesh.positions,
                                        diagonal.end, diag_end_dir, -theta);
  auto cb_dir =
      rotate_in_tangent_space(mesh.triangles, mesh.positions, diagonal.end,
                              diag_end_dir, pif / 2 - theta);

  // if (dot(cross(cd_dir, cb_dir),
  //         tid_normal(mesh.triangles, mesh.positions, diagonal.end.face)) >
  //         0)
  //   cb_dir = rotate_in_tangent_space(mesh.triangles, mesh.positions,
  //                                    diagonal.end, cb_dir, M_PI);

  auto ell = straightest_geodesic_path(mesh, diagonal.end, normalize(cd_dir),
                                       1.5 * width);
  auto h1 = straightest_geodesic_path(mesh, diagonal.end, normalize(cb_dir),
                                      1.5 * width);
  auto d = intersect(mesh, h0, ell);
  auto b = intersect(mesh, base, h1);

  return {{base.start, d, diagonal.end, b}, ell, h0, h1};
}
Isoline draw_ellipse(const bezier_mesh &mesh, const geodesic_path &major_axis,
                     const float &dist_from_mid_point) {
  auto d =
      path_length(major_axis, mesh.triangles, mesh.positions, mesh.adjacencies);
  auto f0 =
      compute_geodesic_distances(mesh.solver, mesh.triangles, mesh.positions,
                                 mesh.adjacencies, {major_axis.start});
  auto f1 =
      compute_geodesic_distances(mesh.solver, mesh.triangles, mesh.positions,
                                 mesh.adjacencies, {major_axis.end});
  auto f = vector<float>(f0.size());
  for (auto i = 0; i < f.size(); ++i) {
    f[i] = f0[i] + f1[i];
  }
  return create_isoline(mesh, f, (1 + dist_from_mid_point / d) * d);
  // return create_isoline(mesh, f, 2 * d);
}

vector<vector<mesh_point>> make_garland(const bezier_mesh &mesh, Circle *c,
                                        const vector<mesh_point> &vertices) {
  auto n = (int)vertices.size();
  auto theta = c->theta;
  auto result = vector<vector<mesh_point>>{};
  if (n == 3) {
    auto step = pif / 30;
    result.resize(20, vector<mesh_point>(3));
    result[0] = vertices;
    for (auto i = 1; i < 20; ++i)
      result[i] = equilateral_triangle_tangent_space(mesh, c, theta + i * step);
    c->garland_vertices = 3;
  } else if (n == 4) {
    auto step = pif / 10;
    result.resize(5, vector<mesh_point>(4));
    result[0] = vertices;
    for (auto i = 1; i < 5; ++i)
      result[i] = rhombus_tangent_space(mesh, c, 1.0, theta + i * step);
    c->garland_vertices = 4;
  } else if (n == 5) {
    auto step = pif / 15;
    result.resize(6, vector<mesh_point>(5));
    result[0] = vertices;
    for (auto i = 1; i < 6; ++i)
      result[i] = pentagon_tangent_space(mesh, c, theta + i * step);

    c->garland_vertices = 5;

  } else if (n == 6) {
    auto step = pif / 12;
    result.resize(4, vector<mesh_point>(6));
    result[0] = vertices;
    for (auto i = 1; i < 4; ++i)
      result[i] = hexagon_tangent_space(mesh, c, theta + i * step);

    c->garland_vertices = 6;
  }
  return result;
}
vector<float> subdivide_radius(const float &radius, const int k) {

  auto result = vector<float>(k);
  auto step = radius / (float)k;
  for (auto i = 0; i < k; ++i) {
    result[i] = radius - i * step;
  }
  return result;
}
vector<Circle> make_concentric_circles(const bezier_mesh &mesh, Circle *c) {
  auto radii = subdivide_radius(c->radius, c->levels);
  auto result = vector<Circle>(radii.size());
  result[0] = *c;
  for (auto i = 1; i < radii.size(); ++i) {
    auto curr_cir = *c;
    if (set_radius(mesh, &curr_cir, radii[i]))
      result[i] = curr_cir;
    else
      std::cerr << "Error: cannot set radius" << std::endl;
  }

  return result;
}
vector<vector<mesh_point>>
make_concentric_n_gon(const bezier_mesh &mesh, const Circle *c,
                      const float &sigma, const float &lambda,
                      const vector<mesh_point> &vertices) {
  if (c == nullptr)
    return {vertices};
  auto primit = c->primitive;
  if (primit == -1)
    return {vertices};
  auto radii = subdivide_radius(c->radius, c->levels);
  auto result = vector<vector<mesh_point>>(radii.size());
  result[0] = vertices;
  for (auto i = 1; i < radii.size(); ++i) {
    auto curr_cir = *c;
    curr_cir.radius = radii[i]; // we don't need to update the isolines
    switch (primit) {
    case tri:
      result[i] =
          equilateral_triangle_tangent_space(mesh, &curr_cir, curr_cir.theta);
      break;
    case square:
      result[i] = rhombus_tangent_space(mesh, &curr_cir, 1.0, curr_cir.theta);
      break;

    case rhomb:
      result[i] = rhombus_tangent_space(mesh, &curr_cir, sigma, curr_cir.theta);
      break;

    case rett:
      result[i] = parallelogram_tangent_space(mesh, &curr_cir, sigma, 1.0,
                                              curr_cir.theta);
      break;

    case parallelog:
      result[i] = parallelogram_tangent_space(mesh, &curr_cir, sigma, lambda,
                                              curr_cir.theta);
      break;
    case pent:
      result[i] = pentagon_tangent_space(mesh, &curr_cir, curr_cir.theta);
      break;

    case primitives::hex:
      result[i] = hexagon_tangent_space(mesh, &curr_cir, curr_cir.theta);
      break;

    case primitives::oct:
      result[i] = octagon_tangent_space(mesh, &curr_cir);
      break;

    case primitives::deca:
      result[i] = decagon_tangent_space(mesh, &curr_cir);
      break;
    }
  }

  return result;
}
// std::vector<closed_curve> cross(const bezier_mesh& mesh,const Circle* c)
// {
//   auto theta=c->theta;
//   auto radius=c->radius;
//}
std::tuple<vector<vector<mesh_point>>, vector<vector<vec3f>>>
spider_net(const bezier_mesh &mesh, const Circle *c, const bool just_vertices) {

  auto radius = c->radius;
  auto center = c->center;
  auto tetas = subdivide_angles(16);
  auto radii = subdivide_radius(c->radius, 3);
  auto vertices = vector<vector<mesh_point>>(3, vector<mesh_point>(8));
  auto circles = vector<vector<Circle>>(3, vector<Circle>(8));
  auto arcs = vector<vector<vec3f>>(24);
  if (just_vertices)
    return {
        make_concentric_n_gon(mesh, c, 0, 0, octagon_tangent_space(mesh, c)),
        {}};

  auto [is_vert, kv] = point_is_vert(center);
  vec3f n, e = zero3f;
  if (is_vert) {
    auto vid = mesh.triangles[center.face][kv];
    n = mesh.normals[vid];
    e = polar_basis(mesh.solver, mesh.positions, mesh.normals, vid);

  } else {
    e = polar_basis(mesh.triangles, mesh.positions, center.face);
    n = tid_normal(mesh.triangles, mesh.positions, center.face);
  }
  auto prev_gamma =
      straightest_geodesic(mesh, center, rot_vect(e, n, c->theta), radius);
  auto scale_factors = vector<float>{1, 1.25, 1.5};
  for (auto i = 0; i < 8; ++i) {
    auto gamma0 = prev_gamma;
    auto gamma1 = straightest_geodesic(
        mesh, center, rot_vect(e, n, c->theta + tetas[2 * i + 1]),
        1.75 * radius);
    auto gamma2 = straightest_geodesic(
        mesh, center, rot_vect(e, n, c->theta + tetas[(2 * i + 2) % 16]),
        radius);
    for (auto j = 0; j < 3; ++j) {
      auto curr_center = gamma1.back();
      auto curr_circle =
          create_circle(mesh, curr_center, radius * scale_factors[j]);
      circles[j][i] = curr_circle;
    }

    prev_gamma = gamma2;
  }

  for (auto i = 0; i < 3; ++i) {
    auto curr_circle = circles[i][0];
    auto prev_circle = circles[i][7];
    auto prev_vertex = mesh_point{};
    auto prev_entry = -1;
    for (auto j = 0; j < 8; ++j) {
      auto c_curr = curr_circle;
      auto c_prev = prev_circle;
      auto c_next = circles[i][(j + 1) % 8];
      if (j == 0) {
        vertices[i][j] = intersect_inside_circle(mesh, c_prev, c_curr, *c);
        prev_vertex = intersect_inside_circle(mesh, c_curr, c_next, *c);

        auto curr_verts = vector<mesh_point>{vertices[i][j], prev_vertex};
        auto crop_range = cropping_range(mesh, c_curr, curr_verts);
        auto pos = circle_positions(mesh.triangles, mesh.positions,
                                    mesh.adjacencies, c_curr);
        auto width_of_cropping = (crop_range.x < crop_range.y)
                                     ? crop_range.y - crop_range.x
                                     : crop_range.y + pos.size() - crop_range.x;

        arcs[8 * i + j] =
            (width_of_cropping < (float)pos.size() / (float)2)
                ? crop_circle(mesh, pos, crop_range, curr_verts)
                : crop_circle(mesh, pos, vec2i{crop_range.y, crop_range.x},
                              {curr_verts[1], curr_verts[0]});
      } else {
        vertices[i][j] = prev_vertex;
        prev_vertex = intersect_inside_circle(mesh, c_curr, c_next, *c);

        auto curr_verts = vector<mesh_point>{vertices[i][j], prev_vertex};
        auto crop_range = cropping_range(mesh, c_curr, curr_verts);
        auto pos = circle_positions(mesh.triangles, mesh.positions,
                                    mesh.adjacencies, c_curr);
        auto width_of_cropping = (crop_range.x < crop_range.y)
                                     ? crop_range.y - crop_range.x
                                     : crop_range.y + pos.size() - crop_range.x;

        arcs[8 * i + j] =
            (width_of_cropping < (float)pos.size() / (float)2)
                ? crop_circle(mesh, pos, crop_range, curr_verts)
                : crop_circle(mesh, pos, vec2i{crop_range.y, crop_range.x},
                              {curr_verts[1], curr_verts[0]});
      }
      curr_circle = c_next;
      prev_circle = c_curr;
    }
  }
  return {vertices, arcs};
}
vector<vector<bool>> croosed_by_circles(const vector<vec3i> &triangles,
                                        const vector<Circle> &circles) {
  auto tagged = vector<vector<bool>>(triangles.size(),
                                     vector<bool>(circles.size(), false));
  for (auto i = 0; i < circles.size(); ++i) {
    if (circles[i].isoline.size() != 1)
      continue;
    auto curve = circles[i].isoline[0];
    for (auto &tid : curve.strip) {
      tagged[tid][i] = true;
    }
  }
  return tagged;
}
int crossed_by_another_triangle(const vector<bool> &tagged,
                                const int curr_tag) {
  for (auto i = 0; i < tagged.size(); ++i) {
    if (i != curr_tag && tagged[i])
      return i;
  }

  return -1;
}
int find_entry_of_tid_in_circle(const bezier_mesh &mesh,
                                const closed_curve &curve, const int tid) {
  for (auto i = 0; i < curve.strip.size(); ++i) {
    if (curve.strip[i] == tid)
      return i;
  }

  return -1;
}
vector<vec3f> add_internal_arc_on_cross(
    const bezier_mesh &mesh, const vector<Circle> &circles,
    const vector<vector<bool>> &tagged, const mesh_point &startint_point,
    const int starting_entry, const int curr_tag, const bool forward = true) {

  auto &curr_circle = circles[curr_tag];
  auto &curr_curve = curr_circle.isoline[0];
  auto &strip = curr_curve.strip;
  auto &lerps = curr_curve.lerps;

  auto &curve0 = circles[0].isoline[0];
  auto &strip0 = curve0.strip;

  auto curr_entry = starting_entry;
  auto intersected_circle = -1;
  auto s = strip.size();
  auto o_s = strip0.size();
  auto result = vector<vec3f>{};
  if (forward) {
    while (intersected_circle == -1) {
      curr_entry = (curr_entry + 1) % s;
      intersected_circle =
          crossed_by_another_triangle(tagged[strip[curr_entry]], curr_tag);
    }
    auto &curve1 = circles[intersected_circle].isoline[0];
    auto &strip1 = curve1.strip;
    auto curr_s = strip1.size();
    auto first_pos = zero3f;
    auto second_entry =
        find_entry_of_tid_in_circle(mesh, curve1, strip[curr_entry]);
    while (!intersection_point(mesh, curr_curve, curve1, curr_entry,
                               second_entry, first_pos)) {
      curr_entry = (curr_entry + 1) % s;
      auto curr_tid = strip[curr_entry];
      second_entry = (strip1[(second_entry + 1) % curr_s] == curr_tid)
                         ? (second_entry + 1) % curr_s
                         : (curr_s - 1 + second_entry) % curr_s;
    }
    result.push_back(first_pos);

  } else {
    while (intersected_circle == -1) {
      curr_entry = (s - 1 + curr_entry) % s;
      intersected_circle =
          crossed_by_another_triangle(tagged[strip[curr_entry]], curr_tag);
    }
    auto &curve1 = circles[intersected_circle].isoline[0];
    auto &strip1 = curve1.strip;
    auto curr_s = strip1.size();
    auto first_pos = zero3f;
    auto second_entry =
        find_entry_of_tid_in_circle(mesh, curve1, strip[curr_entry]);
    while (!intersection_point(mesh, curr_curve, curve1, curr_entry,
                               second_entry, first_pos)) {
      curr_entry = (s - 1 + curr_entry) % s;
      auto curr_tid = strip[curr_entry];
      second_entry = (strip1[(second_entry + 1) % curr_s] == curr_tid)
                         ? (second_entry + 1) % curr_s
                         : (curr_s - 1 + second_entry) % curr_s;
    }
    result.push_back(first_pos);
  }

  if (forward) {
    while (intersected_circle != 0) {
      auto curr_tid = strip[curr_entry];
      auto next_tid = strip[(curr_entry + 1) % s];
      auto k = find_in_vec(mesh.adjacencies[curr_tid], next_tid);
      result.push_back(
          eval_position(mesh.triangles, mesh.positions,
                        eval_point_from_lerp(curr_tid, k, lerps[curr_entry])));
      curr_entry = (curr_entry + 1) % s;
      intersected_circle =
          crossed_by_another_triangle(tagged[strip[curr_entry]], curr_tag);
    }
    auto last_pos = zero3f;
    auto other_entry =
        find_entry_of_tid_in_circle(mesh, curve0, strip[curr_entry]);
    while (!intersection_point(mesh, curr_curve, curve0, curr_entry,
                               other_entry, last_pos)) {
      curr_entry = (curr_entry + 1) % s;
      auto curr_tid = strip[curr_entry];
      other_entry = (strip0[(other_entry + 1) % o_s] == curr_tid)
                        ? (other_entry + 1) % o_s
                        : (o_s - 1 + other_entry) % o_s;
    }
    result.push_back(last_pos);
  } else {
    while (intersected_circle != 0) {
      auto curr_tid = strip[curr_entry];
      auto prev_tid = strip[(s - 1 + curr_entry) % s];
      auto k = find_in_vec(mesh.adjacencies[curr_tid], prev_tid);
      result.push_back(
          eval_position(mesh.triangles, mesh.positions,
                        eval_point_from_lerp(
                            curr_tid, k, 1 - lerps[(s - 1 + curr_entry) % s])));
      curr_entry = (s - 1 + curr_entry) % s;
      intersected_circle =
          crossed_by_another_triangle(tagged[strip[curr_entry]], curr_tag);
    }
    auto last_pos = zero3f;
    auto other_entry =
        find_entry_of_tid_in_circle(mesh, curve0, strip[curr_entry]);
    while (!intersection_point(mesh, curr_curve, curve0, curr_entry,
                               other_entry, last_pos)) {
      curr_entry = (s - 1 + curr_entry) % s;
      auto curr_tid = strip[curr_entry];
      other_entry = (strip0[(other_entry + 1) % o_s] == curr_tid)
                        ? (other_entry + 1) % o_s
                        : (o_s - 1 + other_entry) % o_s;
    }
    result.push_back(last_pos);
  }

  return result;
}
vector<vec3f> add_external_arc_on_cross(const bezier_mesh &mesh,
                                        const vector<Circle> &circles,
                                        const vector<vector<bool>> &tagged,
                                        const mesh_point &starting_point,
                                        const int starting_entry) {

  auto &curr_circle = circles[0];
  auto &curr_curve = curr_circle.isoline[0];
  auto &strip = curr_curve.strip;
  auto &lerps = curr_curve.lerps;
  auto curr_entry = starting_entry;
  auto intersected_circle = -1;
  auto s = strip.size();
  auto result = vector<vec3f>{
      eval_position(mesh.triangles, mesh.positions, starting_point)};

  while (intersected_circle == -1) {
    curr_entry = (curr_entry + 1) % s;
    intersected_circle =
        crossed_by_another_triangle(tagged[strip[curr_entry]], 0);
    auto curr_tid = strip[curr_entry];
    auto next_tid = strip[(curr_entry + 1) % s];
    auto k = find_in_vec(mesh.adjacencies[curr_tid], next_tid);
    result.push_back(
        eval_position(mesh.triangles, mesh.positions,
                      eval_point_from_lerp(curr_tid, k, lerps[curr_entry])));
  }
  auto &curve1 = circles[intersected_circle].isoline[0];
  auto &strip1 = curve1.strip;
  auto curr_s = strip1.size();
  auto first_pos = zero3f;
  auto second_entry =
      find_entry_of_tid_in_circle(mesh, curve1, strip[curr_entry]);
  while (!intersection_point(mesh, curr_curve, curve1, curr_entry, second_entry,
                             first_pos)) {
    curr_entry = (curr_entry + 1) % s;
    auto curr_tid = strip[curr_entry];
    second_entry = (strip1[(second_entry + 1) % curr_s] == curr_tid)
                       ? (second_entry + 1) % curr_s
                       : (curr_s - 1 + second_entry) % curr_s;
  }
  result.push_back(first_pos);
  reverse(result.begin(), result.end());

  intersected_circle = -1;
  curr_entry = starting_entry;
  while (intersected_circle == -1) {
    curr_entry = (s - 1 + curr_entry) % s;
    auto curr_tid = strip[curr_entry];
    auto prev_tid = strip[(s - 1 + curr_entry) % s];
    auto k = find_in_vec(mesh.adjacencies[curr_tid], prev_tid);
    result.push_back(
        eval_position(mesh.triangles, mesh.positions,
                      eval_point_from_lerp(
                          curr_tid, k, 1 - lerps[(s - 1 + curr_entry) % s])));
    intersected_circle =
        crossed_by_another_triangle(tagged[strip[curr_entry]], 0);
  }
  auto &curve2 = circles[intersected_circle].isoline[0];
  auto &strip2 = curve2.strip;
  curr_s = strip2.size();
  auto last_pos = zero3f;
  second_entry = find_entry_of_tid_in_circle(mesh, curve2, strip[curr_entry]);
  while (!intersection_point(mesh, curr_curve, curve2, curr_entry, second_entry,
                             last_pos)) {
    curr_entry = (s - 1 + curr_entry) % s;
    auto curr_tid = strip[curr_entry];
    second_entry = (strip2[(second_entry + 1) % curr_s] == curr_tid)
                       ? (second_entry + 1) % curr_s
                       : (curr_s - 1 + second_entry) % curr_s;
  }
  result.push_back(last_pos);

  return result;
}
vector<vector<vec3f>> make_cross(const bezier_mesh &mesh, Circle *c,
                                 const bool no_arcs) {

  auto center = c->center;
  auto radius = c->radius;
  auto [is_vert, kv] = point_is_vert(center);
  auto result = vector<vector<vec3f>>(12);

  vec3f n, e = zero3f;
  if (is_vert) {
    auto vid = mesh.triangles[center.face][kv];
    n = mesh.normals[vid];
    e = polar_basis(mesh.solver, mesh.positions, mesh.normals, vid);

  } else {
    e = polar_basis(mesh.triangles, mesh.positions, center.face);
    n = tid_normal(mesh.triangles, mesh.positions, center.face);
  }

  if (no_arcs) {
    auto gamma0 =
        straightest_geodesic(mesh, center, rot_vect(e, n, c->theta), radius);
    auto gamma1 = straightest_geodesic(mesh, center,
                                       rot_vect(e, n, c->theta + pif), radius);
    auto sigma0 = straightest_geodesic(
        mesh, center, rot_vect(e, n, c->theta + pif / 2), radius);
    auto sigma1 = straightest_geodesic(
        mesh, center, rot_vect(e, n, c->theta + 1.5 * pif), radius);
    auto pos_gamma0 = path_positions(mesh, gamma0);
    auto pos_gamma1 = path_positions(mesh, gamma1);
    auto pos_sigma0 = path_positions(mesh, sigma0);
    auto pos_sigma1 = path_positions(mesh, sigma1);

    return {pos_gamma0, pos_gamma1, pos_sigma0, pos_sigma1};
  }
  auto gamma0 =
      straightest_geodesic(mesh, center, rot_vect(e, n, c->theta), 2 * radius);
  auto gamma1 = straightest_geodesic(
      mesh, center, rot_vect(e, n, c->theta + pif), 2 * radius);
  auto sigma0 = straightest_geodesic(
      mesh, center, rot_vect(e, n, c->theta + pif / 2), 2 * radius);
  auto sigma1 = straightest_geodesic(
      mesh, center, rot_vect(e, n, c->theta + 1.5 * pif), 2 * radius);
  auto c_sigma0 = create_circle(mesh, sigma0.back(), 1.75 * radius);
  auto c_sigma1 = create_circle(mesh, sigma1.back(), 1.75 * radius);
  auto c_gamma0 = create_circle(mesh, gamma0.back(), 1.75 * radius);
  auto c_gamma1 = create_circle(mesh, gamma1.back(), 1.75 * radius);
  auto tagged = croosed_by_circles(
      mesh.triangles, {*c, c_gamma0, c_sigma0, c_gamma1, c_sigma1});
  auto [p, i, r_gamma0, r_c] =
      intersect_with_entries(mesh, gamma0, c_gamma0.isoline);
  result[0] = add_internal_arc_on_cross(
      mesh, {*c, c_gamma0, c_sigma0, c_gamma1, c_sigma1}, tagged, p, r_c.x, 1);
  result[1] = add_internal_arc_on_cross(
      mesh, {*c, c_gamma0, c_sigma0, c_gamma1, c_sigma1}, tagged, p, r_c.x, 1,
      false);
  std::tie(p, i, r_gamma0, r_c) =
      intersect_with_entries(mesh, sigma0, c_sigma0.isoline);
  result[2] = add_internal_arc_on_cross(
      mesh, {*c, c_gamma0, c_sigma0, c_gamma1, c_sigma1}, tagged, p, r_c.x, 2);
  result[3] = add_internal_arc_on_cross(
      mesh, {*c, c_gamma0, c_sigma0, c_gamma1, c_sigma1}, tagged, p, r_c.x, 2,
      false);
  std::tie(p, i, r_gamma0, r_c) =
      intersect_with_entries(mesh, gamma1, c_gamma1.isoline);
  result[4] = add_internal_arc_on_cross(
      mesh, {*c, c_gamma0, c_sigma0, c_gamma1, c_sigma1}, tagged, p, r_c.x, 3);
  result[5] = add_internal_arc_on_cross(
      mesh, {*c, c_gamma0, c_sigma0, c_gamma1, c_sigma1}, tagged, p, r_c.x, 3,
      false);
  std::tie(p, i, r_gamma0, r_c) =
      intersect_with_entries(mesh, sigma1, c_sigma1.isoline);

  result[6] = add_internal_arc_on_cross(
      mesh, {*c, c_gamma0, c_sigma0, c_gamma1, c_sigma1}, tagged, p, r_c.x, 4);
  result[7] = add_internal_arc_on_cross(
      mesh, {*c, c_gamma0, c_sigma0, c_gamma1, c_sigma1}, tagged, p, r_c.x, 4,
      false);
  std::tie(p, i, r_gamma0, r_c) =
      intersect_with_entries(mesh, gamma0, c->isoline);
  result[8] = add_external_arc_on_cross(
      mesh, {*c, c_gamma0, c_sigma0, c_gamma1, c_sigma1}, tagged, p, r_c.x);
  std::tie(p, i, r_gamma0, r_c) =
      intersect_with_entries(mesh, sigma0, c->isoline);
  result[9] = add_external_arc_on_cross(
      mesh, {*c, c_gamma0, c_sigma0, c_gamma1, c_sigma1}, tagged, p, r_c.x);
  std::tie(p, i, r_gamma0, r_c) =
      intersect_with_entries(mesh, gamma1, c->isoline);
  result[10] = add_external_arc_on_cross(
      mesh, {*c, c_gamma0, c_sigma0, c_gamma1, c_sigma1}, tagged, p, r_c.x);
  std::tie(p, i, r_gamma0, r_c) =
      intersect_with_entries(mesh, sigma1, c->isoline);
  result[11] = add_external_arc_on_cross(
      mesh, {*c, c_gamma0, c_sigma0, c_gamma1, c_sigma1}, tagged, p, r_c.x);
  return result;
}
vector<vector<vec3f>> make_flower(const bezier_mesh &mesh, const Circle *circle,
                                  const int k, const bool no_arcs,
                                  const float &scale_factor,
                                  const bool &with_inner_circle) {

  vector<mesh_point> radial_geodesic(k);
  auto thetas = subdivide_angles(k);
  auto &center = circle->center;
  auto &radius = circle->radius;
  auto [is_vert, kv] = point_is_vert(center);

  vec3f n, e = zero3f;
  if (is_vert) {
    auto vid = mesh.triangles[center.face][kv];
    n = mesh.normals[vid];
    e = polar_basis(mesh.solver, mesh.positions, mesh.normals, vid);

  } else {
    e = polar_basis(mesh.triangles, mesh.positions, center.face);
    n = tid_normal(mesh.triangles, mesh.positions, center.face);
  }
  if (no_arcs) {
    vector<vector<vec3f>> result(k);
    auto len = radius * (1 + 1.3 * std::sin(pif / k));
    for (auto i = 0; i < k; ++i) {
      result[i] = path_positions(
          mesh,
          straightest_geodesic(mesh, center,
                               rot_vect(e, n, circle->theta + thetas[i]), len));
    }
    return result;
  }
  vector<vector<vec3f>> result{};
  if (with_inner_circle)
    result.resize(k + 1);
  else
    result.resize(k);
  vector<Circle> petals(k);
  for (auto i = 0; i < k; ++i) {
    radial_geodesic[i] =
        straightest_geodesic(mesh, center,
                             rot_vect(e, n, circle->theta + thetas[i]), radius)
            .back();
  }
  auto prev_distance = 0.f;
  for (auto j = 0; j < k; ++j) {
    auto curr = radial_geodesic[j];
    auto next = radial_geodesic[(j + 1) % k];
    auto prev = radial_geodesic[(k - 1 + j) % k];
    auto curr_dist = compute_pruned_geodesic_distances(
        mesh.solver, mesh.triangles, mesh.positions, mesh.adjacencies, mesh.v2t,
        curr, {next, prev});
    if (prev_distance == 0.f)
      prev_distance = interpolate_distances(mesh, curr_dist, prev);

    auto next_distance = interpolate_distances(mesh, curr_dist, next);

    auto r = std::max(prev_distance, next_distance);
    petals[j] = create_circle(mesh, curr, 0.65 * r);

    prev_distance = next_distance;
  }
  auto prev_point = mesh_point{};
  auto prev_range = zero2i;
  for (auto i = 0; i < k; ++i) {
    auto curr_cirle = petals[i];
    auto next_circle = petals[(i + 1) % k];
    // if (prev_point.face == -1) {
    auto prev_circle = petals[(k - 1 + i) % k];
    std::tie(prev_point, prev_range) = intersect_outside_circle_with_entries(
        mesh, curr_cirle, prev_circle, *circle);
    // }
    auto [next_point, next_range] = intersect_outside_circle_with_entries(
        mesh, curr_cirle, next_circle, *circle);
    result[i] = crop_outside_circle(mesh, curr_cirle, *circle, prev_point,
                                    prev_range.x, next_point, next_range.x);
    // prev_point = next_point;
    // prev_range = next_range;
    // circle_positions(mesh.triangles, mesh.positions,
    //                              mesh.adjacencies, curr_cirle);
  }
  if (with_inner_circle)
    result[k] =
        circle_positions(mesh.triangles, mesh.positions, mesh.adjacencies,
                         create_circle(mesh, center, scale_factor * radius));
  return result;
}
// std::tuple<vector<vector<mesh_point>>, vector<vector<vec3f>>>
// isol_spider_net(const bezier_mesh &mesh, const Circle *c,
//                 const bool just_vertices) {
//   auto are_not_bary = [](const vec2f &bary) {
//     if (bary.x < -1e-3 || bary.x > 1 + 1e3)
//       return false;
//     if (bary.y < -1e-3 || bary.y > 1 + 1e3)
//       return false;

//     return true;
//   };
//   auto radius = c->radius;
//   auto center = c->center;
//   auto tetas = subdivide_angles(16);
//   auto vertices =
//       make_concentric_n_gon(mesh, c, 0, 0, octagon_tangent_space(mesh, c));
//   auto arcs = vector<vector<vec3f>>(24);
//   if (just_vertices)
//     return {vertices, {}};
//   auto [is_vert, kv] = point_is_vert(center);
//   vec3f n, e = zero3f;
//   if (is_vert) {
//     auto vid = mesh.triangles[center.face][kv];
//     n = mesh.normals[vid];
//     e = polar_basis(mesh.solver, mesh.positions, mesh.normals, vid);

//   } else {
//     e = polar_basis(mesh.triangles, mesh.positions, center.face);
//     n = tid_normal(mesh.triangles, mesh.positions, center.face);
//   }
//   auto radii = subdivide_radius(c->radius, c->levels);
//   for (auto i = 0; i < 3; ++i) {
//     for (auto j = 0; j < 8; ++j) {
//       auto curr_verts =
//           vector<mesh_point>{vertices[i][j], vertices[i][(j + 1) % 8]};
//       auto curr_center =
//           find_center_for_spider_net(mesh, curr_verts[0], curr_verts[1], c);

//       if (curr_center.face == -1 || !are_not_bary(curr_center.uv))
//         std::cout << "this should not happen" << std::endl;
//       auto curr_circle = create_circle(mesh, curr_center, 0.75 * c->radius);

//       auto crop_range =
//           cropping_range_generic_points(mesh, curr_circle, curr_verts);
//       auto pos = circle_positions(mesh.triangles, mesh.positions,
//                                   mesh.adjacencies, curr_circle);
//       auto width_of_cropping = (crop_range.x < crop_range.y)
//                                    ? crop_range.y - crop_range.x
//                                    : crop_range.y + pos.size() -
//                                    crop_range.x;
//       arcs[8 * i + j] =
//           (width_of_cropping < pos.size() / 2)
//               ? crop_circle(mesh, pos, crop_range, curr_verts)
//               : crop_circle(mesh, pos, vec2i{crop_range.y, crop_range.x},
//                             curr_verts);
//     }
//   }

//   return {vertices, arcs};
// }

// 3-points circle approach

// for (auto i = 0; i < 8; ++i) {
//   // auto gamma0 = prev_gamma;
//   auto gamma1 = straightest_geodesic(
//       mesh, center, rot_vect(e, n, tetas[2 * i + 1]), radius);
//   // auto gamma2 = straightest_geodesic(
//   //     mesh, center, rot_vect(e, n, tetas[(2 * i + 2) % 16]), radius);
//   for (auto j = 0; j < 3; ++j) {
//     auto curr_verts =
//         vector<mesh_point>{vertices[j][i], vertices[j][(i + 1) % 8]};
//     auto middle = eval_point(mesh, gamma1, 0.8 * radii[j] / radius);
//     auto curr_circle =
//         circle_3_points(mesh, curr_verts[0], curr_verts[1], middle);
//     auto crop_range =
//         cropping_range_generic_points(mesh, curr_circle, curr_verts);
//     auto pos = circle_positions(mesh.triangles, mesh.positions,
//                                 mesh.adjacencies, curr_circle);
//     auto width_of_cropping = (crop_range.x < crop_range.y)
//                                  ? crop_range.y - crop_range.x
//                                  : crop_range.y + pos.size() -
//                                  crop_range.x;
//     arcs[3 * i + j] =
//         (width_of_cropping < pos.size() / 2)
//             ? crop_circle(mesh, pos, crop_range, curr_verts)
//             : crop_circle(mesh, pos, vec2i{crop_range.y, crop_range.x},
//                           curr_verts);
//   }
// auto [i0, j0] = intersect(mesh, gamma0, curr_circle.isoline);
// if (i0.face == -1) {
//   std::cerr << "vertex not found" << std::endl;
//   continue;
// } else
//   vertices[j][i] = i0;

// auto [i1, j1] = intersect(mesh, gamma2, curr_circle.isoline);

// if (i1.face == -1) {
//   std::cerr << "vertex not found" << std::endl;
//   continue;
// }

// auto crop_range = cropping_range(mesh, curr_circle, {i0, i1});
// auto pos = circle_positions(mesh.triangles, mesh.positions,
//                             mesh.adjacencies, curr_circle);
// auto width_of_cropping = (crop_range.x < crop_range.y)
//                              ? crop_range.y - crop_range.x
//                              : crop_range.y + pos.size() -
//                              crop_range.x;
// arcs[3 * i + j] =
//     (width_of_cropping < pos.size() / 2)
//         ? crop_circle(mesh, pos, crop_range, {i0, i1})
//         : crop_circle(mesh, pos, vec2i{crop_range.y, crop_range.x},
//                       {i1, i0});
// OLD CROSS
// auto [q0, entries_q0] =
//     intersect_inside_circle_with_entries(mesh, c_gamma0, c_sigma1, *c);
// if (entries_q0.x != 0) {
//   std::rotate(c_gamma0.isoline[0].strip.begin(),
//               c_gamma0.isoline[0].strip.begin() + entries_q0.x,
//               c_gamma0.isoline[0].strip.end());
//   std::rotate(c_gamma0.isoline[0].lerps.begin(),
//               c_gamma0.isoline[0].lerps.begin() + entries_q0.x,
//               c_gamma0.isoline[0].lerps.end());
// }
// if (entries_q0.y != 0) {
//   std::rotate(c_sigma1.isoline[0].strip.begin(),
//               c_sigma1.isoline[0].strip.begin() + entries_q0.y,
//               c_sigma1.isoline[0].strip.end());
//   std::rotate(c_sigma1.isoline[0].lerps.begin(),
//               c_sigma1.isoline[0].lerps.begin() + entries_q0.y,
//               c_sigma1.isoline[0].lerps.end());
// }
// auto [q2, entriesq2] =
//     intersect_inside_circle_with_entries(mesh, c_gamma1, c_sigma0, *c);
// if (entriesq2.x != 0) {
//   std::rotate(c_gamma1.isoline[0].strip.begin(),
//               c_gamma1.isoline[0].strip.begin() + entriesq2.x,
//               c_gamma1.isoline[0].strip.end());
//   std::rotate(c_gamma1.isoline[0].lerps.begin(),
//               c_gamma1.isoline[0].lerps.begin() + entriesq2.x,
//               c_gamma1.isoline[0].lerps.end());
// }
// if (entriesq2.y != 0) {
//   std::rotate(c_sigma0.isoline[0].strip.begin(),
//               c_sigma0.isoline[0].strip.begin() + entriesq2.y,
//               c_sigma0.isoline[0].strip.end());
//   std::rotate(c_sigma0.isoline[0].lerps.begin(),
//               c_sigma0.isoline[0].lerps.begin() + entriesq2.y,
//               c_sigma0.isoline[0].lerps.end());
// }
// auto [q1, entriesq1] =
//     intersect_inside_circle_with_entries(mesh, c_gamma1, c_sigma1, *c);

// auto [q3, entriesq3] =
//     intersect_inside_circle_with_entries(mesh, c_gamma0, c_sigma0, *c);
// auto [p0, p5, r05_c, r05_sig1] =
//     intersect_with_entries(mesh, c->isoline, c_sigma1.isoline);
// if (r05_c.x != 0) {
//   std::rotate(c->isoline[0].strip.begin(),
//               c->isoline[0].strip.begin() + r05_c.x,
//               c->isoline[0].strip.end());
//   std::rotate(c->isoline[0].lerps.begin(),
//               c->isoline[0].lerps.begin() + r05_c.x,
//               c->isoline[0].lerps.end());
// }
// if (r05_sig1.x < entriesq1.y && r05_sig1.x < r05_sig1.y) {
//   swap(p0, p5);
//   // swap(r05_c.x, r05_c.y);
//   // swap(r05_sig1.x, r05_sig1.y);

// } else if (r05_sig1.x > entriesq1.y && r05_sig1.x > r05_sig1.y) {
//   swap(p0, p5);
//   // swap(r05_c.x, r05_c.y);
//   // swap(r05_sig1.x, r05_sig1.y);
// }

// auto [p1, p4, r14_c, r14_sig0] =
//     intersect_with_entries(mesh, c->isoline, c_sigma0.isoline);

// if (r14_sig0.y > entriesq3.y && r14_sig0.y > r14_sig0.x) {
//   swap(p1, p4);
//   // swap(r14_c.x, r14_c.y);
//   // swap(r14_sig0.x, r14_sig0.y);
// } else if (r14_sig0.y < entriesq3.y && r14_sig0.y < r14_sig0.x) {
//   swap(p1, p4);
//   // swap(r14_c.x, r14_c.y);
//   // swap(r14_sig0.x, r14_sig0.y);
// }

// auto [p2, p7, r27_c, r27_ga1] =
//     intersect_with_entries(mesh, c->isoline, c_gamma1.isoline);
// if (r27_ga1.y > entriesq1.x && r27_ga1.y > r27_ga1.x) {
//   swap(p2, p7);
//   // swap(r27_c.x, r27_c.y);
//   // swap(r27_ga1.x, r27_ga1.y);
// } else if (r27_ga1.y < entriesq1.x && r27_ga1.y < r27_ga1.x) {
//   swap(p2, p7);
//   // swap(r27_c.x, r27_c.y);
//   // swap(r27_ga1.x, r27_ga1.y);
// }

// auto [p3, p6, r36_c, r36_ga0] =
//     intersect_with_entries(mesh, c->isoline, c_gamma0.isoline);

// if (r36_ga0.y > entriesq3.x && r36_ga0.y > r36_ga0.x) {
//   swap(p3, p6);
//   // swap(r36_c.x, r36_c.y);
//   // swap(r36_ga0.x, r36_ga0.y);
// } else if (r36_ga0.y < entriesq3.x && r36_ga0.y < r36_ga0.x) {
//   swap(p3, p6);
//   // swap(r36_c.x, r36_c.y);
//   // swap(r36_ga0.x, r36_ga0.y);
// }
// if (r14_c.x > r27_c.y)
//   c_ccw = false;
// if (r05_sig1.x < r05_sig1.y)
//   sig1_ccw = false;
// if (r14_sig0.y < r14_sig0.x)
//   sig0_ccw = false;
// if (r27_ga1.x < r27_ga1.y)
//   ga1_ccw = false;
// if (r36_ga0.y < r36_ga0.x)
//   ga0_ccw = false;
// auto pos =
//     circle_positions(mesh.triangles, mesh.positions, mesh.adjacencies, *c);
// auto ranges = vector<vec2i>{};
// auto samples = vector<vector<mesh_point>>{};
// auto last_entry = (int)pos.size() - 1;
// if (c_ccw) {
//   ranges = {{r27_c.y, r36_c.y},
//             {r05_c.y, r14_c.y},
//             {r36_c.x, r27_c.x},
//             {r14_c.x, last_entry}};
//   samples = {{p7, p6}, {p5, p4}, {p3, p2}, {p1, p0}};
// } else {
//   ranges = {{0, r14_c.x},
//             {r27_c.x, r36_c.x},
//             {r14_c.y, r05_c.y},
//             {r36_c.y, r27_c.y}};
//   samples = {{p0, p1}, {p2, p3}, {p4, p5}, {p6, p7}};
// }
// result = crop_circle(mesh, pos, ranges, samples);
// pos = circle_positions(mesh.triangles, mesh.positions, mesh.adjacencies,
//                        c_sigma1);
// last_entry = (int)pos.size() - 1;
// if (sig1_ccw) {
//   ranges = {{0, r05_sig1.y}, {r05_sig1.y, entriesq1.y}};
//   samples = {{q0, p5}, {p0, q1}};
// } else {
//   ranges = {{entriesq1.y, r05_sig1.x}, {r05_sig1.y, last_entry}};
//   samples = {{q1, p0}, {p5, q0}};
// }
// auto curr_arcs = crop_circle(mesh, pos, ranges, samples);
// result.insert(result.end(), curr_arcs.begin(), curr_arcs.end());
// pos = circle_positions(mesh.triangles, mesh.positions, mesh.adjacencies,
//                        c_sigma0);
// last_entry = (int)pos.size() - 1;
// if (sig0_ccw) {
//   ranges = {{0, r14_sig0.x}, {r14_sig0.y, entriesq3.y}};
//   samples = {{q2, p1}, {p4, q3}};
// } else {
//   ranges = {{entriesq3.y, r14_sig0.y}, {r14_sig0.x, last_entry}};
//   samples = {{q3, p4}, {p1, q2}};
// }
// curr_arcs = crop_circle(mesh, pos, ranges, samples);
// result.insert(result.end(), curr_arcs.begin(), curr_arcs.end());
// pos = circle_positions(mesh.triangles, mesh.positions, mesh.adjacencies,
//                        c_gamma0);
// last_entry = (int)pos.size() - 1;
// if (ga0_ccw) {
//   ranges = {{entriesq3.x, r36_ga0.x}, {r36_ga0.y, last_entry}};
//   samples = {{q3, p3}, {p6, q0}};
// } else {
//   ranges = {{0, r36_ga0.y}, {r36_ga0.x, entriesq3.x}};
//   samples = {{q0, p6}, {p3, q3}};
// }
// curr_arcs = crop_circle(mesh, pos, ranges, samples);
// result.insert(result.end(), curr_arcs.begin(), curr_arcs.end());
// pos = circle_positions(mesh.triangles, mesh.positions, mesh.adjacencies,
//                        c_gamma0);
// last_entry = (int)pos.size() - 1;

// if (ga1_ccw) {
//   ranges = {{entriesq1.x, r27_ga1.y}, {r27_ga1.x, last_entry}};
//   samples = {{q1, p7}, {p2, q2}};
// } else {
//   ranges = {{0, r27_ga1.x}, {r27_ga1.y, entriesq1.x}};
//   samples = {{q2, p2}, {p7, q1}};
// }
// curr_arcs = crop_circle(mesh, pos, ranges, samples);
// result.insert(result.end(), curr_arcs.begin(), curr_arcs.end());
