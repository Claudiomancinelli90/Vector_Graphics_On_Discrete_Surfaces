#include "VTP/geodesic_algorithm_exact.h"
#include "VTP/geodesic_mesh.h"
#include <cstring>

#include "drawing_functions.h"

using namespace logging;

vector<vec3f> path_positions(const geodesic_path &path,
                             const bezier_mesh &mesh) {
  if (!validate_paths({path})) {
    return vector<vec3f>();
  }

  auto result = vector<vec3f>(path.lerps.size() + 2);
  result[0] = eval_position(mesh.triangles, mesh.positions, path.start);
  for (auto i = 0; i < path.lerps.size(); i++) {
    auto e = get_edge(mesh.triangles, mesh.positions, mesh.adjacencies,
                      path.strip[i], path.strip[i + 1]);
    if (e == vec2i{-1, -1})
      continue;
    auto x = path.lerps[i];
    auto p = lerp(mesh.positions[e.x], mesh.positions[e.y], x);
    result[i + 1] = p;
  }
  result.back() = eval_position(mesh.triangles, mesh.positions, path.end);
  return result;
}

// -------------------------------------------------------------------------------------
mesh_point eval_mesh_point(const vector<vec3i> &triangles,
                           const vector<vec3f> &positions, const int &face,
                           const vec3f &point) {
  auto result = point_in_triangle(triangles, positions, face, point);

  if (result.first) {
    return mesh_point{face, result.second};
  }

  return mesh_point();
}

mesh_point eval_mesh_point(const vector<vec3i> &triangles,
                           const vector<vec3f> &positions,
                           const vector<int> &strip, const vec3f &point) {
  // point_in_triangle for every triangle i until <true, vec2f> -> mesh_point
  // {i, vec2f}
  for (auto it = strip.cbegin(); it != strip.cend(); ++it) {
    auto result = point_in_triangle(triangles, positions, *it, point);

    if (result.first) {
      return mesh_point{*it, result.second};
    }
  }

  return mesh_point();
}

// -------------------------------------------------------------------------------------
/*
vector<mesh_point> intersect_geodesics(App& app, const geodesic_path& first,
const geodesic_path& second) { vector<mesh_point> intersection;

  auto f = path_positions(first, app.mesh.triangles, app.mesh.positions,
app.mesh.adjacencies); auto s = path_positions(second, app.mesh.triangles,
app.mesh.positions, app.mesh.adjacencies);

  for (auto it_f = 0; it_f < first.strip.size(); ++it_f) {
    for (auto it_s = 0; it_s < second.strip.size(); ++it_s) {
      if (first.strip[it_f] == second.strip[it_s]) {
        auto point = intersect_segments(
          app.mesh.triangles, app.mesh.positions, first.strip,
          f[it_f], f[it_f + 1], s[it_s], s[it_s + 1]
        );

        if (point.face != -1) {
          intersection.push_back(point);

          app.point_selection.push_back(point);
          update_glpoints(app, app.point_selection);
        }
      }
    }
  }

  return intersection;
}

mesh_point intersect_geodesics(App& app, const geodesic_path& first, const int&
ix1, const geodesic_path& second, const int& ix2) { auto f =
path_positions(first, app.mesh.triangles, app.mesh.positions,
app.mesh.adjacencies); auto s = path_positions(second, app.mesh.triangles,
app.mesh.positions, app.mesh.adjacencies);

  return intersect_segments(app.mesh.triangles, app.mesh.positions, first.strip,
f[ix1], f[ix1 + 1], s[ix2], s[ix2 + 1]);
}
*/
// Change from vector<int> strip to int triangle (vector<int>{tid})
float max_of_field(const vector<float> &distances) {
  auto max_d = flt_min;
  for (auto i = 0; i < distances.size(); ++i) {
    if (distances[i] > max_d)
      max_d = distances[i];
  }
  return max_d;
}

mesh_point intersect_segments(const vector<vec3i> &triangles,
                              const vector<vec3f> &positions, const int &tid,
                              const vec3f &start1, const vec3f &end1,
                              const vec3f &start2, const vec3f &end2) {
  if (start1 - end1 == zero3f && start2 - end2 == zero3f) {
    return mesh_point();
  }

  vec2f uv;
  float dist;
  auto c = positions[triangles[tid].x];
  auto n = tid_normal(triangles, positions, tid);

  auto s1 = project_point(start1, c, n);
  auto e1 = project_point(end1, c, n);
  auto s2 = project_point(start2, c, n);
  auto e2 = project_point(end2, c, n);

  if (start1 - end1 == zero3f) {
    auto ray = ray3f{s2, e2 - s2, 0.0F, 1.0F};

    if (intersect_point(ray, s1, dist)) {
      return eval_mesh_point(triangles, positions, tid, ray_point(ray, dist));
      ;
    }
  } else if (start2 - end2 == zero3f) {
    auto ray = ray3f{s1, e1 - s1, 0.0F, 1.0F};

    if (intersect_point(ray, s2, dist)) {
      return eval_mesh_point(triangles, positions, tid, ray_point(ray, dist));
      ;
    }
  } else {
    auto ray = ray3f{s1, e1 - s1, 0.0F, 1.0F};

    if (intersect_line(ray, s2, e2, dist)) {
      return eval_mesh_point(triangles, positions, tid, ray_point(ray, dist));
    }
  }

  return mesh_point();
}

mesh_point intersect_segments(const vector<vec3i> &triangles,
                              const vector<vec3f> &positions, const int &tid,
                              const mesh_point &start1, const mesh_point &end1,
                              const mesh_point &start2,
                              const mesh_point &end2) {
  auto b = intersect_segments(triangles, positions, tid,
                              eval_position(triangles, positions, start1),
                              eval_position(triangles, positions, end1),
                              eval_position(triangles, positions, start2),
                              eval_position(triangles, positions, end2));

  return b;
}

// Intersections with straightest_geodesic approach
// void init_intersect_geodesic(const bezier_mesh &mesh,
//                              unordered_map<int, int> &faces,
//                              const mesh_point &from, vec3f &dir, vec3f &prev,
//                              queue<int> &prev_faces, int &tid, vec3f &bary,
//                              geodesic_path &path) {
//   auto &solver = mesh.solver;
//   auto &triangles = mesh.triangles;
//   auto &positions = mesh.positions;
//   auto &adjacencies = mesh.adjacencies;
//   auto &v2t = mesh.v2t;
//   auto &normals = mesh.normals;
//   auto &angles = mesh.angles;
//   auto &total_angles = mesh.total_angles;

//   auto [is_vert, kv] = bary_is_vert(bary);
//   auto [is_on_edge, ke] = bary_is_edge(bary);

//   if (is_vert) {
//     prev_faces.push(tid);

//     auto ccwise = bary[(kv + 2) % 3] > 0;
//     path.lerps.push_back(ccwise * 1.0F);
//     faces[tid] = path.lerps.size();

//     auto neighbours = v2t[triangles[from.face][kv]];

//     auto vid = triangles[from.face][kv];
//     parallel_transp(solver, angles, total_angles, triangles, positions,
//                     adjacencies, dir, normals, from.face, vid, T2V);
//     tid =
//         next_tid(solver, angles, positions, v2t, triangles, normals, vid,
//         dir);
//     kv = find(triangles[tid], vid);
//     bary = zero3f;
//     bary[kv] = 1;
//     parallel_transp(solver, angles, total_angles, triangles, positions,
//                     adjacencies, dir, normals, vid, tid, V2T);

//     auto offset = find_in_vec(neighbours, from.face);

//     for (int i = 1; i < neighbours.size(); ++i) {
//       auto n_tid =
//           neighbours[(ccwise ? offset + i : offset + neighbours.size() - i) %
//                      neighbours.size()];
//       path.strip.push_back(n_tid);

//       if (n_tid == tid) {
//         break;
//       }

//       path.lerps.push_back(ccwise * 1.0F);
//       faces[n_tid] = path.lerps.size();
//       prev_faces.push(n_tid);
//     }
//   } else if (is_on_edge) {
//     auto p0 = triangles[from.face][ke];
//     auto p1 = triangles[from.face][(ke + 1) % 3];
//     auto p2 = triangles[from.face][(ke + 2) % 3];
//     auto n = triangle_normal(positions[p0], positions[p1], positions[p2]);
//     auto edge = normalize(positions[p1] - positions[p0]);
//     if (dot(cross(edge, dir), n) > 0)
//       tid = from.face;
//     else {
//       prev_faces.push(tid);

//       tid = adjacencies[from.face][ke];

//       path.lerps.push_back(bary[(ke + 1) % 3]);
//       path.strip.push_back(tid);
//       faces[tid] = path.lerps.size();

//       bary = tri_bary_coords(triangles, positions, tid, prev);
//       parallel_transp(solver, angles, total_angles, triangles, positions,
//                       adjacencies, dir, normals, from.face, tid, T2T);
//     }
//   } else
//     tid = from.face;
// }

bool find_next_point(const bezier_mesh &mesh, unordered_map<int, int> &faces,
                     vec3f &dir, geodesic_path &path, vec3f &prev, vec3f &next,
                     queue<int> &prev_faces, int &tid, vec3f &bary,
                     vec3f &next_bary, int &next_tri, float &length) {

  // auto &solver = mesh.solver;
  // auto &triangles = mesh.triangles;
  // auto &positions = mesh.positions;
  // auto &adjacencies = mesh.adjacencies;
  // auto &v2t = mesh.v2t;
  // auto &normals = mesh.normals;
  // auto &angles = mesh.angles;
  // auto &total_angles = mesh.total_angles;

  // if (length < 0) {
  //   return false;
  // }

  // trace_in_triangles(mesh.positions, mesh.triangles, dir, bary, tid, next,
  //                    next_bary);

  // length -= distance(prev, next);

  // prev = next;
  // auto [V, k_v] = bary_is_vert(next_bary);
  // auto [E, k_e] = bary_is_edge(next_bary);

  // auto prev_face = tid;
  // prev_faces.push(prev_face);

  // if (V) {
  //   auto ccwise = next_bary[(k_v + 2) % 3] > 0;
  //   path.lerps.push_back(next_bary[ccwise ? k_v : (k_v + 1) % 3]);
  //   faces[prev_face] = path.lerps.size();

  //   auto neighbours = mesh.v2t[mesh.triangles[prev_face][k_v]];

  //   auto vid = mesh.triangles[tid][k_v];
  //   auto out = handle_vert(mesh.solver, mesh.triangles, mesh.positions,
  //                          mesh.normals, mesh.adjacencies, mesh.v2t,
  //                          mesh.angles, mesh.total_angles, vid, tid, dir);
  //   tid = out.first;
  //   dir = out.second;
  //   k_v = find(mesh.triangles[tid], vid);
  //   bary = zero3f;
  //   bary[k_v] = 1;

  //   auto offset = find_in_vec(neighbours, prev_face);

  //   for (int i = 1; i < neighbours.size(); ++i) {
  //     auto n_tid =
  //         neighbours[(ccwise ? offset + i : offset + neighbours.size() - i) %
  //                    neighbours.size()];
  //     path.strip.push_back(n_tid);

  //     if (n_tid == tid) {
  //       break;
  //     }

  //     path.lerps.push_back(ccwise * 1.0F);
  //     faces[n_tid] = path.lerps.size();
  //     prev_faces.push(n_tid);
  //   }
  // } else if (E) {
  //   path.lerps.push_back(next_bary[(k_e + 1) % 3]);
  //   faces[prev_face] = path.lerps.size();

  //   next_tri = mesh.adjacencies[tid][k_e];
  //   auto p0 = mesh.triangles[tid][k_e];
  //   auto offset0 = find(mesh.triangles[next_tri], p0);
  //   auto offset1 = (offset0 + 2) % 3;
  //   bary = zero3f;
  //   bary[offset0] = next_bary[k_e];
  //   bary[offset1] = next_bary[(k_e + 1) % 3];

  //   parallel_transp(mesh.solver, mesh.angles, mesh.total_angles,
  //   mesh.triangles,
  //                   mesh.positions, mesh.adjacencies, dir, mesh.normals, tid,
  //                   next_tri, T2T);
  //   tid = next_tri;

  //   path.strip.push_back(tid);
  // } else {
  //   return false;
  // }

  // return true;
}

geodesic_path find_intersecting_geodesic(const bezier_mesh &mesh,
                                         const vector<geodesic_path> &ends,
                                         const int &angle,
                                         const mesh_point &start,
                                         const vec3f &direction, const int &n,
                                         const float &max_length) {
  // auto length = max_length;
  // auto face = -1, next_tri = -1;
  // auto dir = direction;
  // auto next_bary = zero3f, next = zero3f;
  // auto prev = eval_position(mesh.triangles, mesh.positions, start);
  // auto bary = get_bary(start.uv);
  // auto prev_faces = std::queue<int>();

  // auto path = geodesic_path();
  // path.start = start;
  // path.strip.push_back(start.face);

  // auto faces = unordered_map<int, int>{};
  // auto faces2 = vector<unordered_map<int, int>>{};

  // init_intersect_geodesic(mesh, faces, start, dir, prev, prev_faces, face,
  // bary,
  //                         path);

  // auto intersection = mesh_point();

  // for (auto i = 0; i < ends.size(); ++i) {
  //   auto end_path = ends[i];
  //   auto curr = unordered_map<int, int>();

  //   auto start_index = 0;
  //   auto end_index = end_path.strip.size();

  //   if (i == angle) {
  //     start_index++;

  //     auto bary = get_bary(end_path.start.uv);
  //     auto [V, k_v] = bary_is_vert(bary);
  //     auto [E, k_e] = bary_is_edge(bary);

  //     if (V) {
  //       for (auto j = 0; j < end_path.lerps.size(); ++j) {
  //         if (end_path.lerps[j] == 1.0F || end_path.lerps[j] == 0.0F) {
  //           start_index++;
  //         } else {
  //           break;
  //         }
  //       }
  //     } else if (E) {
  //       auto end_dir = geodesic_path_direction(mesh, end_path);

  //       auto p0 = mesh.triangles[end_path.start.face][k_e];
  //       auto p1 = mesh.triangles[end_path.start.face][(k_e + 1) % 3];
  //       auto p2 = mesh.triangles[end_path.start.face][(k_e + 2) % 3];
  //       auto n = triangle_normal(mesh.positions[p0], mesh.positions[p1],
  //                                mesh.positions[p2]);
  //       auto edge = normalize(mesh.positions[p1] - mesh.positions[p0]);

  //       if (dot(cross(edge, end_dir), n) <= 0) {
  //         start_index++;
  //       }
  //     }
  //   } else if (i == (angle + n - 1) % n) {
  //     end_index--;

  //     auto bary = get_bary(end_path.end.uv);
  //     auto [V, k_v] = bary_is_vert(bary);
  //     auto [E, k_e] = bary_is_edge(bary);

  //     if (V) {
  //       for (auto j = end_path.lerps.size() - 1; j >= 0; --j) {
  //         if (end_path.lerps[j] == 1.0F || end_path.lerps[j] == 0.0F) {
  //           end_index--;
  //         } else {
  //           break;
  //         }
  //       }
  //     } else if (E) {
  //       auto end_dir = geodesic_path_direction(mesh, end_path, true);

  //       auto p0 = mesh.triangles[end_path.end.face][k_e];
  //       auto p1 = mesh.triangles[end_path.end.face][(k_e + 1) % 3];
  //       auto p2 = mesh.triangles[end_path.end.face][(k_e + 2) % 3];
  //       auto n = triangle_normal(mesh.positions[p0], mesh.positions[p1],
  //                                mesh.positions[p2]);
  //       auto edge = normalize(mesh.positions[p1] - mesh.positions[p0]);

  //       if (dot(cross(edge, end_dir), n) <= 0) {
  //         end_index--;
  //       }
  //     }
  //   }

  //   for (auto j = start_index; j < end_index; ++j) {
  //     curr[end_path.strip[j]] = j + 1;
  //   }

  //   faces2.push_back(curr);
  // }

  // while (find_next_point(mesh, faces, dir, path, prev, next, prev_faces,
  // face,
  //                        bary, next_bary, next_tri, length)) {
  //   for (int j = 0, size = prev_faces.size(); j < size; ++j) {
  //     auto prev_face = prev_faces.front();
  //     prev_faces.pop();

  //     for (auto i = 0; i < faces2.size(); ++i) {
  //       if (faces2[i].count(prev_face) > 0) {
  //         if (find_intersection(mesh, faces2[i][prev_face], ends[i],
  //                               faces[prev_face], path, prev_face,
  //                               intersection)) {
  //           path.end = intersection;

  //           path.strip.resize(path.strip.size() - prev_faces.size() - 1);
  //           path.lerps.resize(path.lerps.size() - prev_faces.size() - 1);

  //           return path;
  //         }
  //       }
  //     }
  //   }
  // }

  // return geodesic_path();
}

// pair<geodesic_path, geodesic_path>
// find_intersecting_geodesics(const bezier_mesh &mesh, const mesh_point
// &start1,
//                             const vec3f &direction1, const mesh_point
//                             &start2, const vec3f &direction2, const float
//                             &max_length) {
//   auto length1 = max_length;
//   auto path1 = geodesic_path{};
//   path1.start = start1;
//   path1.strip.push_back(start1.face);

//   auto faces1 = unordered_map<int, int>{};

//   auto face1 = -1, next_tri1 = -1;
//   auto dir1 = direction1;
//   auto next_bary1 = zero3f, next1 = zero3f;
//   auto prev1 = eval_position(mesh.triangles, mesh.positions, start1);
//   auto bary1 = get_bary(start1.uv);
//   auto prev_faces1 = std::queue<int>();

//   init_intersect_geodesic(mesh, faces1, start1, dir1, prev1, prev_faces1,
//   face1,
//                           bary1, path1);

//   auto length2 = max_length;
//   auto path2 = geodesic_path{};
//   path2.start = start2;
//   path2.strip.push_back(start2.face);

//   auto faces2 = unordered_map<int, int>{};

//   auto face2 = -1, next_tri2 = -1;
//   auto dir2 = direction2;
//   auto next_bary2 = zero3f, next2 = zero3f;
//   auto prev2 = eval_position(mesh.triangles, mesh.positions, start2);
//   auto bary2 = get_bary(start2.uv);
//   auto prev_faces2 = std::queue<int>();

//   init_intersect_geodesic(mesh, faces2, start2, dir2, prev2, prev_faces2,
//   face2,
//                           bary2, path2);

//   auto intersection = mesh_point();

//   auto running1 = true, running2 = true;

//   while (running1 || running2) {
//     if (running1) {
//       running1 =
//           find_next_point(mesh, faces1, dir1, path1, prev1, next1,
//           prev_faces1,
//                           face1, bary1, next_bary1, next_tri1, length1);
//     }

//     if (running2) {
//       running2 =
//           find_next_point(mesh, faces2, dir2, path2, prev2, next2,
//           prev_faces2,
//                           face2, bary2, next_bary2, next_tri2, length2);
//     }

//     // Path2 found path1
//     for (int j = 0, size = prev_faces2.size(); j < size; ++j) {
//       auto prev_face2 = prev_faces2.front();
//       prev_faces2.pop();

//       if (faces1.count(prev_face2) > 0) {
//         if (find_intersection(mesh, faces1[prev_face2], path1,
//                               faces2[prev_face2], path2, prev_face2,
//                               intersection)) {

//           path1.end = intersection;
//           path2.end = intersection;

//           path1.lerps.resize(faces1[prev_face2] - 1);
//           path1.strip.resize(faces1[prev_face2]);

//           path2.lerps.resize(faces2[prev_face2] - 1);
//           path2.strip.resize(faces2[prev_face2]);

//           return pair<geodesic_path, geodesic_path>{path1, path2};
//         }
//       }
//     }

//     // Path1 found path2
//     for (int j = 0, size = prev_faces1.size(); j < size; ++j) {
//       auto prev_face1 = prev_faces1.front();
//       prev_faces1.pop();

//       if (faces2.count(prev_face1) > 0) {
//         if (find_intersection(mesh, faces2[prev_face1], path2,
//                               faces1[prev_face1], path1, prev_face1,
//                               intersection)) {

//           path1.end = intersection;
//           path2.end = intersection;

//           path2.lerps.resize(faces2[prev_face1] - 1);
//           path2.strip.resize(faces2[prev_face1]);

//           path2.lerps.resize(faces1[prev_face1] - 1);
//           path2.strip.resize(faces1[prev_face1]);

//           return pair<geodesic_path, geodesic_path>{path1, path2};
//         }
//       }
//     }
//   }

//   return pair<geodesic_path, geodesic_path>();
// }

bool find_intersection(const bezier_mesh &mesh, const int &ix1,
                       const geodesic_path &path1, const int &ix2,
                       const geodesic_path &path2, const int &tid,
                       mesh_point &intersection) {
  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;
  auto &adjacencies = mesh.adjacencies;

  auto p1 = get_geodesic_path_point(mesh, path1, ix1);
  auto p2 = get_geodesic_path_point(mesh, path1, ix1 - 1);
  auto p3 = get_geodesic_path_point(mesh, path2, ix2);
  auto p4 = get_geodesic_path_point(mesh, path2, ix2 - 1);

  if (!validate_points({p1, p2, p3, p4})) {
    return false;
  }

  intersection = intersect_segments(triangles, positions, tid, p1, p2, p3, p4);

  if (intersection.face == -1) {
    return false;
  }

  return true;
}

geodesic_path reverse(const geodesic_path &path) {
  auto p = geodesic_path();

  p.start = path.end;
  p.end = path.start;
  p.strip = reverse_copy(path.strip);
  p.lerps = reverse_copy(path.lerps);

  for (auto &l : p.lerps) {
    l = 1.0F - l;
  }

  return p;
}

// -------------------------------------------------------------------------------------
float angle(const vector<vec3i> &triangles, const vector<vec3f> &positions,
            const mesh_point &p, const mesh_point &a, const mesh_point &b) {
  if (p.face < 0 || a.face < 0 || b.face < 0) {
    return 0.0F;
  }
  auto c = eval_position(triangles, positions, p);
  auto ca = normalize(eval_position(triangles, positions, a) - c);
  auto cb = normalize(eval_position(triangles, positions, b) - c);

  auto d = clamp(dot(ca, cb), -1.0F, 1.0F);

  return yocto::acos(d) * 360 / (2 * pi);
}

vec3f rotate_in_tangent_space(const vector<vec3i> &triangles,
                              const vector<vec3f> &positions,
                              const mesh_point &from, const vec3f &d,
                              const float &angle) {
  auto axis = tid_normal(triangles, positions, from.face);

  return rotate_in_tangent_space(axis, d, angle);
}

vec3f rotate_in_tangent_space(const vec3f &axis, const vec3f &d,
                              const float &angle) {
  return rot_vect(d, axis, angle);
}

vec3f direction_in_tangent_space(const vector<vec3i> &triangles,
                                 const vector<vec3f> &positions,
                                 const mesh_point &p, const vec3f &dir) {
  auto n = tid_normal(triangles, positions, p.face);
  auto d = normalize(dir);

  return project_vec(d, n);
}

vec2f direction_2D_in_tangent_space(const vector<vec3i> &triangles,
                                    const vector<vec3f> &positions,
                                    const int &tid, const vec3f &p,
                                    const vec3f &dir) {
  auto n = tid_normal(triangles, positions, tid);
  auto d = normalize(dir);

  auto t = triangles[tid];
  auto y_axis = normalize(positions[t.y] - positions[t.x]);
  auto x_axis = cross(y_axis, n);

  auto y = clamp(dot(y_axis, d), -1.0F, 1.0F);
  if (y < -1 || y > 1) {
    error("direction acos()");
  }
  auto x = std::sin(std::acos(y));

  return vec2f{dot(x_axis, d) < 0 ? -x : x, y};
}

vec2f direction_2D_in_tangent_space(const vector<vec3i> &triangles,
                                    const vector<vec3f> &positions,
                                    const mesh_point &p, const vec3f &dir) {
  return direction_2D_in_tangent_space(triangles, positions, p.face,
                                       eval_position(triangles, positions, p),
                                       dir);
}

vec2f mouse_direction_in_tangent_space(const ray3f &ray,
                                       const vector<vec3i> &triangles,
                                       const vector<vec3f> &positions,
                                       Window &win, const int &tid,
                                       const vec3f &p) {
  auto dist = 0.0f;
  auto n = tid_normal(triangles, positions, tid);

  if (dot(ray.d, n) >= 0) {
    return vec2f{0, 0};
  }

  auto t = triangles[tid];
  auto y_axis = normalize(positions[t.y] - positions[t.x]);
  auto x_axis = cross(y_axis, n);

  if (intersect_plane(ray, p, n, dist)) {
    auto intersection = ray_point(ray, dist);
    auto segment = normalize(intersection - p);

    auto y = dot(y_axis, segment);
    auto x = std::sin(std::acos(y));

    return vec2f{dot(x_axis, segment) < 0 ? -x : x, y};
  }

  return vec2f{0, 0};
}

vec2f mouse_direction_in_tangent_space(const ray3f &ray,
                                       const vector<vec3i> &triangles,
                                       const vector<vec3f> &positions,
                                       Window &win, const mesh_point &p) {
  return mouse_direction_in_tangent_space(
      ray, triangles, positions, win, p.face,
      eval_position(triangles, positions, p));
}

vec3f geodesic_path_direction(const bezier_mesh &mesh,
                              const geodesic_path &path, int tid) {
  auto &solver = mesh.solver;
  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;
  auto &adjacencies = mesh.adjacencies;
  auto &v2t = mesh.v2t;
  auto &normals = mesh.normals;
  auto &angles = mesh.angles;
  auto &total_angles = mesh.total_angles;

  auto a = zero3f, b = zero3f;

  auto points = path_positions(path, mesh);
  auto offset = find_in_vec(path.strip, tid);

  assert(offset != -1);

  return normalize(points[offset + 1] - points[offset]);
}

vec3f geodesic_path_direction(const bezier_mesh &mesh,
                              const geodesic_path &path, bool reverse) {
  auto &solver = mesh.solver;
  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;
  auto &adjacencies = mesh.adjacencies;
  auto &v2t = mesh.v2t;
  auto &normals = mesh.normals;
  auto &angles = mesh.angles;
  auto &total_angles = mesh.total_angles;

  auto a = zero3f, b = zero3f;

  auto points = path_positions(path, mesh);

  auto dir = zero3f;

  if (reverse) {
    auto [is_edge, edge] = point_is_edge(path.end);
    auto [is_vert, vert] = point_is_vert(path.end);

    if (is_vert) {
      auto ix = (int)points.size() - 2;
      auto vert0 = triangles[path.end.face][vert];
      auto p = get_geodesic_path_point(mesh, path, ix);
      tie(is_vert, vert) = point_is_vert(p);

      while (is_vert && triangles[p.face][vert] == vert0) {
        p = get_geodesic_path_point(mesh, path, --ix);
        if (!validate_points({p})) {
          return zero3f;
        }

        tie(is_vert, vert) = point_is_vert(p);
      }

      a = points[ix + 1];
      b = points[ix];
      dir = b - a;
      parallel_transp(solver, angles, total_angles, triangles, positions,
                      adjacencies, v2t, dir, normals, path.strip[ix], vert0,
                      T2V);
      parallel_transp(solver, angles, total_angles, triangles, positions,
                      adjacencies, v2t, dir, normals, vert0, path.end.face,
                      V2T);
    } else if (is_edge) {
      a = points[points.size() - 2];
      b = points[points.size() - 3];
      dir = b - a;
      parallel_transp(solver, angles, total_angles, triangles, positions,
                      adjacencies, v2t, dir, normals,
                      path.strip[path.strip.size() - 2],
                      path.strip[path.strip.size() - 1], T2T);
    } else {
      a = points[points.size() - 1];
      b = points[points.size() - 2];
      dir = b - a;
    }
  } else {
    auto [is_edge, edge] = point_is_edge(path.start);
    auto [is_vert, vert] = point_is_vert(path.start);

    if (is_vert) {
      auto ix = 1;
      auto vert0 = triangles[path.start.face][vert];
      auto p = get_geodesic_path_point(mesh, path, ix);
      tie(is_vert, vert) = point_is_vert(p);

      while (is_vert && triangles[p.face][vert] == vert0) {
        p = get_geodesic_path_point(mesh, path, ++ix);
        if (!validate_points({p})) {
          return zero3f;
        }

        tie(is_vert, vert) = point_is_vert(p);
      }

      a = points[ix - 1];
      b = points[ix];
      dir = b - a;
      parallel_transp(solver, angles, total_angles, triangles, positions,
                      adjacencies, v2t, dir, normals, path.strip[ix - 1], vert0,
                      T2V);
      parallel_transp(solver, angles, total_angles, triangles, positions,
                      adjacencies, v2t, dir, normals, vert0, path.start.face,
                      V2T);
    } else if (is_edge) {
      a = points[1];
      b = points[2];
      dir = b - a;
      parallel_transp(solver, angles, total_angles, triangles, positions,
                      adjacencies, v2t, dir, normals, path.strip[1],
                      path.strip[0], T2T);
    } else {
      a = points[0];
      b = points[1];
      dir = b - a;
    }
  }

  return normalize(dir);
}

// -------------------------------------------------------------------------------------
mesh_point get_geodesic_path_point(const bezier_mesh &mesh,
                                   const geodesic_path &path, float l) {
  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;
  auto &adjacencies = mesh.adjacencies;

  auto len = 0.0f;
  auto points = path_positions(path, mesh);
  auto path_len = path_length(points);

  for (auto i = 1; i < points.size(); i++) {
    len += length(points[i] - points[i - 1]) / path_len;

    if (len > l) {
      auto p = lerp(points[i], points[i - 1],
                    (len - l) * path_len / length(points[i] - points[i - 1]));
      return eval_mesh_point(triangles, positions, path.strip[i - 1], p);
    }
  }

  return mesh_point();
}

mesh_point get_geodesic_path_point(const bezier_mesh &mesh,
                                   const geodesic_path &path, int ix) {
  if (ix < 0 || ix > path.strip.size()) {
    return mesh_point();
  }

  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;
  auto &adjacencies = mesh.adjacencies;

  if (ix == 0) {
    return path.start;
  } else if (ix == path.strip.size()) {
    return path.end;
  }

  auto e = get_edge(triangles, positions, adjacencies, path.strip[ix - 1],
                    path.strip[ix]);

  if (e == vec2i{-1, -1}) {
    return mesh_point();
  }

  return eval_mesh_point(
      triangles, positions, path.strip[ix],
      lerp(positions[e.x], positions[e.y], path.lerps[ix - 1]));
}

mesh_point get_geodesic_path_point(const bezier_mesh &mesh,
                                   const geodesic_path &path, int ix, float l) {
  if (ix < 0 || ix > path.strip.size()) {
    return mesh_point();
  }

  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;
  auto &adjacencies = mesh.adjacencies;

  if (ix == 0) {
    return path.start;
  } else if (ix == path.strip.size()) {
    return path.end;
  }

  auto e0 = get_edge(triangles, positions, adjacencies, path.strip[ix - 1],
                     path.strip[ix]);
  auto p0 = lerp(positions[e0.x], positions[e0.y], path.lerps[ix - 1]);

  auto p1 = zero3f;

  if (ix + 1 == path.strip.size()) {
    p1 = eval_position(triangles, positions, path.end);
  } else {
    auto e1 = get_edge(triangles, positions, adjacencies, path.strip[ix],
                       path.strip[ix + 1]);
    p1 = lerp(positions[e1.x], positions[e1.y], path.lerps[ix]);
  }

  return eval_mesh_point(triangles, positions, path.strip[ix],
                         lerp(p0, p1, clamp(l, 0.0F, 1.0F)));
}

pair<mesh_point, mesh_point>
get_common_end_points(const vector<vec3i> &triangles,
                      const vector<vec3f> &positions, const geodesic_path &a,
                      const geodesic_path &b) {
  bool tmp1, tmp2;

  return get_common_end_points(triangles, positions, a, b, tmp1, tmp2);
}
int path_point_entry(const geodesic_path &path, const mesh_point &point) {
  for (auto i = 0; i < path.strip.size(); ++i) {
    if (path.strip[i] == point.face)
      return i;
  }
  return -1;
}

pair<mesh_point, mesh_point>
get_common_end_points(const vector<vec3i> &triangles,
                      const vector<vec3f> &positions, const geodesic_path &a,
                      const geodesic_path &b, bool &isEnd1, bool &isEnd2) {
  auto s1 = eval_position(triangles, positions, a.start);
  auto s2 = eval_position(triangles, positions, b.start);
  auto e1 = eval_position(triangles, positions, a.end);
  auto e2 = eval_position(triangles, positions, b.end);

  if (length(s1 - s2) < flt_eps) {
    return {a.start, b.start};
  } else if (length(s1 - e2) < flt_eps) {
    isEnd2 = true;
    return {a.start, b.end};
  } else if (length(e1 - s2) < flt_eps) {
    isEnd1 = true;
    return {a.end, b.start};
  } else if (length(e1 - e2) < flt_eps) {
    isEnd1 = true;
    isEnd2 = true;
    return {a.end, b.end};
  }

  return {mesh_point(), mesh_point()};
}

// -------------------------------------------------------------------------------------
vector<float> get_geodesic_distances(const bezier_mesh &mesh,
                                     const int &source) {
  return exact_geodesic_distance(mesh.triangles, mesh.positions, mesh.v2t,
                                 source);
}

vector<float> get_geodesic_distances(const bezier_mesh &mesh,
                                     const mesh_point &source) {
  return compute_geodesic_distances(mesh.solver, mesh.triangles, mesh.positions,
                                    mesh.adjacencies, {source});
  // return exact_geodesic_distance(mesh.triangles, mesh.positions, mesh.v2t,
  //                                source);
}
float interpolate_distances(const bezier_mesh &mesh,
                            const vector<float> &distances,
                            const mesh_point &point) {
  auto tid = point.face;
  auto bary = get_bary(point.uv);
  return bary.x * distances[mesh.triangles[tid].x] +
         bary.y * distances[mesh.triangles[tid].y] +
         bary.z * distances[mesh.triangles[tid].z];
}
vector<float> exact_geodesic_distance(const vector<vec3i> &triangles,
                                      const vector<vec3f> &positions,
                                      const vector<vector<int>> &v2t,
                                      const int &source) {
  int V = positions.size();
  int F = triangles.size();
  vector<double> points(3 * V);
  vector<uint> faces(3 * F);
  vector<float> f(V);

  for (int i = 0; i < V; ++i) {
    if (v2t[i].size() == 0)
      continue;
    points[3 * i] = positions[i].x;
    points[3 * i + 1] = positions[i].y;
    points[3 * i + 2] = positions[i].z;
  }
  for (int i = 0; i < F; ++i) {
    faces[3 * i] = triangles[i].x;
    faces[3 * i + 1] = triangles[i].y;
    faces[3 * i + 2] = triangles[i].z;
  }

  geodesic_VTP::Mesh mesh;
  mesh.initialize_mesh_data(points, faces);
  geodesic_VTP::GeodesicAlgorithmExact algorithm(&mesh);
  algorithm.propagate(source);
  vector<geodesic_VTP::Vertex> verts = mesh.vertices();

  for (int j = 0; j < V; ++j) {
    geodesic_VTP::Vertex v = verts[j];
    float value = (float)v.geodesic_distance();
    f[j] = value;
  }

  return f;
}

vector<float> exact_geodesic_distance(const vector<vec3i> &triangles,
                                      const vector<vec3f> &positions,
                                      const vector<vector<int>> &v2t,
                                      const mesh_point &source) {
  auto tid = source.face;
  auto [is_vert, offset] = point_is_vert(source);
  if (is_vert)
    return exact_geodesic_distance(triangles, positions, v2t,
                                   triangles[tid][offset]);
  else {
    int V = positions.size() + 1;
    int F = triangles.size();
    vector<double> points(3 * V);
    vector<uint> faces(3 * (F + 2));
    vector<float> f(V);
    auto pos = eval_position(triangles, positions, source);

    for (int i = 0; i < V; ++i) {
      if (v2t[i].size() == 0)
        continue;
      if (i != V - 1) {
        points[3 * i] = positions[i].x;
        points[3 * i + 1] = positions[i].y;
        points[3 * i + 2] = positions[i].z;
      } else {
        points[3 * i] = pos.x;
        points[3 * i + 1] = pos.y;
        points[3 * i + 2] = pos.z;
      }
    }

    for (int i = 0; i < F; ++i) {
      if (i != tid) {
        faces[3 * i] = triangles[i].x;
        faces[3 * i + 1] = triangles[i].y;
        faces[3 * i + 2] = triangles[i].z;
      } else {
        faces[3 * i] = triangles[i].x;
        faces[3 * i + 1] = triangles[i].y;
        faces[3 * i + 2] = V - 1;

        faces[3 * F] = triangles[i].y;
        faces[3 * F + 1] = triangles[i].z;
        faces[3 * F + 2] = V - 1;

        faces[3 * (F + 1)] = triangles[i].z;
        faces[3 * (F + 1) + 1] = triangles[i].x;
        faces[3 * (F + 1) + 2] = V - 1;
      }
    }

    geodesic_VTP::Mesh mesh;
    mesh.initialize_mesh_data(points, faces);
    geodesic_VTP::GeodesicAlgorithmExact algorithm(&mesh);
    algorithm.propagate(V - 1);
    vector<geodesic_VTP::Vertex> verts = mesh.vertices();

    for (int j = 0; j < V; ++j) {
      geodesic_VTP::Vertex v = verts[j];
      float value = (float)v.geodesic_distance();
      f[j] = value;
    }

    f.pop_back();
    return f;
  }
}

mesh_point select_closest(const mesh_point &point,
                          const vector<mesh_point> &list,
                          const vector<vec3i> &triangles,
                          const vector<vec3f> &positions,
                          const float &epsilon) {
  auto closest = mesh_point();

  if (point.face != -1) {
    auto min_dist = flt_max;

    for (auto v : list) {
      auto dist = distance(triangles, positions, point, v);

      if (dist < epsilon && dist < min_dist) {
        min_dist = dist;
        closest = v;
      }
    }
  }

  return closest;
}

mesh_point select_closest(const ray3f &ray, const Isoline &isoline,
                          const vector<vec3i> &triangles,
                          const vector<vec3f> &positions,
                          const vector<vec3i> &adjacencies,
                          const float &epsilon) {
  auto closest = mesh_point();

  auto min_dist = flt_max;

  for (auto curve : isoline) {
    auto points =
        closed_curve_positions(curve, triangles, positions, adjacencies);

    for (auto i = 0; i < points.size() - 1; ++i) {
      auto tid = curve.strip[i];
      auto n = tid_normal(triangles, positions, tid);
      auto dist = 0.0F;

      if (dot(ray.d, n) >= 0) {
        continue;
      }

      if (intersect_plane(ray, points[i], n, dist)) {
        auto intersection = ray_point(ray, dist);
        auto s = normalize(points[i + 1] - points[i]);
        auto a = intersection - points[i];
        auto b = intersection - points[i + 1];

        auto point = points[i] + s * dot(a, s);
        auto dist = length(intersection - point);

        if (dist < epsilon && dist < min_dist) {
          min_dist = dist;
          closest = eval_mesh_point(triangles, positions, curve.strip, point);
        }
      }
    }
  }

  return closest;
}

geodesic_path straightest_geodesic_path(const bezier_mesh &mesh,
                                        const mesh_point &from, const vec3f &v,
                                        const float &l) {
  auto &solver = mesh.solver;
  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;
  auto &adjacencies = mesh.adjacencies;
  auto &v2t = mesh.v2t;
  auto &normals = mesh.normals;
  auto &angles = mesh.angles;
  auto &total_angles = mesh.total_angles;

  auto vid = -1, tid = -1, next_tri = -1, prev_tid = -1;
  auto dir = v;
  auto next_bary = zero3f, next = zero3f;
  float len = 0.0;
  auto prev = eval_position(triangles, positions, from);
  auto bary = get_bary(from.uv);

  auto path = geodesic_path();
  path.start = from;
  path.strip.push_back(from.face);

  auto [is_vert, kv] = bary_is_vert(bary);
  auto [is_on_edge, ke] = bary_is_edge(bary);

  if (is_vert) {
    auto ccwise = bary[(kv + 2) % 3] > 0;
    auto neighbours = v2t[triangles[from.face][kv]];

    path.lerps.push_back(ccwise * 1.0F);

    vid = triangles[from.face][kv];
    parallel_transp(solver, angles, total_angles, triangles, positions,
                    adjacencies, v2t, dir, normals, from.face, vid, T2V);

    tid =
        next_tid(solver, angles, positions, v2t, triangles, normals, vid, dir);
    kv = find(triangles[tid], vid);
    bary = zero3f;
    bary[kv] = 1;
    parallel_transp(solver, angles, total_angles, triangles, positions,
                    adjacencies, v2t, dir, normals, vid, tid, V2T);

    auto offset = find_in_vec(neighbours, from.face);

    for (int i = 1; i < neighbours.size(); ++i) {
      auto n_tid =
          neighbours[(ccwise ? offset + i : offset + neighbours.size() - i) %
                     neighbours.size()];
      path.strip.push_back(n_tid);

      if (n_tid == tid) {
        break;
      }

      path.lerps.push_back(ccwise * 1.0F);
    }
  } else if (is_on_edge) {
    auto p0 = triangles[from.face][ke];
    auto p1 = triangles[from.face][(ke + 1) % 3];
    auto p2 = triangles[from.face][(ke + 2) % 3];
    auto n = triangle_normal(positions[p0], positions[p1], positions[p2]);
    auto edge = normalize(positions[p1] - positions[p0]);
    if (dot(cross(edge, dir), n) > 0)
      tid = from.face;
    else {
      tid = adjacencies[from.face][ke];

      path.lerps.push_back(bary[(ke + 1) % 3]);
      path.strip.push_back(tid);

      bary = tri_bary_coords(triangles, positions, tid, prev);
      parallel_transp(solver, angles, total_angles, triangles, positions,
                      adjacencies, v2t, dir, normals, from.face, tid, T2T);
    }
  } else
    tid = from.face;

  while (len < l) {
    trace_in_triangles(positions, triangles, dir, bary, tid, next, next_bary);

    len += length(next - prev);
    if (len < l) {
      prev = next;
      auto [V, k_v] = bary_is_vert(next_bary);
      auto [E, k_e] = bary_is_edge(next_bary);

      prev_tid = tid;

      if (V) {
        auto ccwise = next_bary[(k_v + 2) % 3] > 0;
        path.lerps.push_back(next_bary[ccwise ? k_v : (k_v + 1) % 3]);

        auto neighbours = v2t[triangles[tid][k_v]];

        vid = triangles[tid][k_v];
        auto out =
            handle_vert(solver, triangles, positions, normals, adjacencies, v2t,
                        angles, total_angles, vid, tid, dir);
        tid = out.first;
        dir = out.second;
        k_v = find(triangles[tid], vid);
        bary = zero3f;
        bary[k_v] = 1;

        auto offset = find_in_vec(neighbours, prev_tid);

        for (int i = 1; i < neighbours.size(); ++i) {
          auto n_tid = neighbours[(ccwise ? offset + i
                                          : offset + neighbours.size() - i) %
                                  neighbours.size()];
          path.strip.push_back(n_tid);

          if (n_tid == tid) {
            break;
          }

          path.lerps.push_back(ccwise * 1.0F);
        }
      } else if (E) {
        path.lerps.push_back(next_bary[(k_e + 1) % 3]);

        next_tri = adjacencies[tid][k_e];
        auto p0 = triangles[tid][k_e];
        auto offset0 = find(triangles[next_tri], p0);
        auto offset1 = (offset0 + 2) % 3;
        bary = zero3f;
        bary[offset0] = next_bary[k_e];
        bary[offset1] = next_bary[(k_e + 1) % 3];

        parallel_transp(solver, angles, total_angles, triangles, positions,
                        adjacencies, v2t, dir, normals, tid, next_tri, T2T);
        tid = next_tri;

        path.strip.push_back(tid);
      } else {
        assert(false);
      }
    }
  }

  auto factor = (len - l);
  auto w = normalize(prev - next);
  w *= factor;
  w += next;
  bary = tri_bary_coords(triangles, positions, tid, w);
  path.end = {tid, vec2f{bary.y, bary.z}};

  return path;
}

geodesic_path _straightest_geodesic_path(const bezier_mesh &mesh,
                                         const mesh_point &from, const vec3f &v,
                                         const float &l) {
  auto &solver = mesh.solver;
  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;
  auto &adjacencies = mesh.adjacencies;
  auto &v2t = mesh.v2t;
  auto &normals = mesh.normals;
  auto &angles = mesh.angles;
  auto &total_angles = mesh.total_angles;

  auto vid = -1, tid = -1, next_tri = -1, prev_tid = -1;
  auto dir = v;
  auto next_bary = zero3f, next = zero3f;
  float len = 0.0;
  auto prev = eval_position(triangles, positions, from);
  auto bary = get_bary(from.uv);

  auto path = geodesic_path();
  path.start = from;
  path.strip.push_back(from.face);

  auto [is_vert, kv] = bary_is_vert(bary);
  auto [is_on_edge, ke] = bary_is_edge(bary);

  if (is_vert) {
    auto ccwise = bary[(kv + 2) % 3] > 0;
    auto neighbours = v2t[triangles[from.face][kv]];

    path.lerps.push_back(ccwise * 1.0F);

    vid = triangles[from.face][kv];
    parallel_transp(solver, angles, total_angles, triangles, positions,
                    adjacencies, v2t, dir, normals, from.face, vid, T2V);

    tid =
        next_tid(solver, angles, positions, v2t, triangles, normals, vid, dir);
    kv = find(triangles[tid], vid);
    bary = zero3f;
    bary[kv] = 1;
    parallel_transp(solver, angles, total_angles, triangles, positions,
                    adjacencies, v2t, dir, normals, vid, tid, V2T);

    auto offset = find_in_vec(neighbours, from.face);

    for (int i = 1; i < neighbours.size(); ++i) {
      auto n_tid =
          neighbours[(ccwise ? offset + i : offset + neighbours.size() - i) %
                     neighbours.size()];
      path.strip.push_back(n_tid);

      if (n_tid == tid) {
        break;
      }

      path.lerps.push_back(ccwise * 1.0F);
    }
  } else if (is_on_edge) {
    auto p0 = triangles[from.face][ke];
    auto p1 = triangles[from.face][(ke + 1) % 3];
    auto p2 = triangles[from.face][(ke + 2) % 3];
    auto n = triangle_normal(positions[p0], positions[p1], positions[p2]);
    auto edge = normalize(positions[p1] - positions[p0]);
    if (dot(cross(edge, dir), n) > 0)
      tid = from.face;
    else {
      tid = adjacencies[from.face][ke];

      path.lerps.push_back(bary[(ke + 1) % 3]);
      path.strip.push_back(tid);

      bary = tri_bary_coords(triangles, positions, tid, prev);
      parallel_transp(solver, angles, total_angles, triangles, positions,
                      adjacencies, v2t, dir, normals, from.face, tid, T2T);
    }
  } else
    tid = from.face;

  while (len < l) {
    trace_in_triangles(positions, triangles, dir, bary, tid, next, next_bary);

    len += length(next - prev);
    if (len < l) {
      prev = next;
      auto [V, k_v] = bary_is_vert(next_bary);
      auto [E, k_e] = bary_is_edge(next_bary);

      prev_tid = tid;

      if (V) {
        auto ccwise = next_bary[(k_v + 2) % 3] > 0;
        path.lerps.push_back(next_bary[ccwise ? k_v : (k_v + 1) % 3]);

        auto neighbours = v2t[triangles[tid][k_v]];

        vid = triangles[tid][k_v];
        auto out =
            handle_vert(solver, triangles, positions, normals, adjacencies, v2t,
                        angles, total_angles, vid, tid, dir);
        tid = out.first;
        dir = out.second;
        k_v = find(triangles[tid], vid);
        bary = zero3f;
        bary[k_v] = 1;

        auto offset = find_in_vec(neighbours, prev_tid);

        for (int i = 1; i < neighbours.size(); ++i) {
          auto n_tid = neighbours[(ccwise ? offset + i
                                          : offset + neighbours.size() - i) %
                                  neighbours.size()];
          path.strip.push_back(n_tid);

          if (n_tid == tid) {
            break;
          }

          path.lerps.push_back(ccwise * 1.0F);
        }
      } else if (E) {
        path.lerps.push_back(next_bary[(k_e + 1) % 3]);

        next_tri = adjacencies[tid][k_e];
        auto p0 = triangles[tid][k_e];
        auto offset0 = find(triangles[next_tri], p0);
        auto offset1 = (offset0 + 2) % 3;
        bary = zero3f;
        bary[offset0] = next_bary[k_e];
        bary[offset1] = next_bary[(k_e + 1) % 3];

        parallel_transp(solver, angles, total_angles, triangles, positions,
                        adjacencies, mesh.v2t, dir, normals, tid, next_tri,
                        T2T);
        tid = next_tri;

        path.strip.push_back(tid);
      } else {
        assert(false);
      }
    }
  }

  auto factor = (len - l);
  auto w = normalize(prev - next);
  w *= factor;
  w += next;
  bary = tri_bary_coords(triangles, positions, tid, w);
  path.end = {tid, vec2f{bary.y, bary.z}};

  return path;
}

geodesic_path join_paths(const geodesic_path &a, const geodesic_path &b) {
  auto path = geodesic_path(a);

  if (path.strip.back() == b.strip.front()) {
    for (auto i = 1; i < b.strip.size(); ++i) {
      path.strip.push_back(b.strip[i]);
    }
    for (auto i = 0; i < b.lerps.size(); ++i) {
      path.lerps.push_back(b.lerps[i]);
    }

    path.end = b.end;
  }

  return path;
}