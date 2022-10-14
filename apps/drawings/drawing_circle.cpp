#include "drawing_circle.h"
#include "drawing_functions.h"
#include "straightedge_and_compass_constructions.h"

// void draw_isoline(unordered_map<string, Shader> &shaders,
//                   const Isoline &isoline, mat4f &view, mat4f &projection) {
//   bind_shader(shaders["points"]);
//   set_uniform(shaders["points"], Uniform("frame", identity4x4f),
//               Uniform("view", view), Uniform("projection", projection));

//   set_uniform(shaders["points"], Uniform("color", vec3f{0, 1, 0}));

//   for (auto curve : isoline) {
//     draw_shape(curve.shape);
//   }
// }

vector<vec3f> closed_curve_positions(const closed_curve &curve,
                                     const vector<vec3i> &triangles,
                                     const vector<vec3f> &positions,
                                     const vector<vec3i> &adjacencies,
                                     const vec2i &range) {

  auto pos = vector<vec3f>(curve.lerps.size() + 1);
  auto s = curve.strip.size();
  for (auto i = 0; i < s; i++) {
    auto e = get_edge(triangles, positions, adjacencies, curve.strip[i],
                      curve.strip[(i + 1) % s]);
    if (e == vec2i{-1, -1}) {
      std::cout << "point not found" << std::endl;
      continue;
    }
    auto x = curve.lerps[i];
    auto p = lerp(positions[e.x], positions[e.y], x);
    pos[i] = p;
  }
  pos.back() = pos.front();
  auto result = vector<vec3f>{};
  if (range.x < range.y) {
    result.insert(result.begin(), pos.begin() + range.x,
                  pos.begin() + range.y + 1);
  } else if (range.x > range.y) {
    result.insert(result.begin(), pos.begin() + range.y, pos.end());
    result.insert(result.end(), pos.begin(), pos.begin() + range.x);
  } else
    result = pos;

  // if (range.x == -1) {
  //   result.resize(curve.lerps.size() + 1);
  //   for (auto i = 0; i < curve.lerps.size(); i++) {
  //     auto e = get_edge(triangles, positions, adjacencies, curve.strip[i],
  //                       curve.strip[(i + 1) % curve.lerps.size()]);
  //     if (e == vec2i{-1, -1}) {
  //       std::cout << "point not found" << std::endl;
  //       continue;
  //     }
  //     auto x = curve.lerps[i];
  //     auto p = lerp(positions[e.x], positions[e.y], x);
  //     result[i] = p;
  //   }

  //   result.back() = result.front();

  // } else if (range.x < range.y) {
  //   result.resize(range.y - range.x);
  //   for (auto i = range.x; i < range.y; i++) {
  //     auto e = get_edge(triangles, positions, adjacencies, curve.strip[i],
  //                       curve.strip[(i + 1) % curve.lerps.size()]);
  //     if (e == vec2i{-1, -1}) {
  //       std::cout << "point not found in range x <range y" << std::endl;
  //       continue;
  //     }
  //     auto x = curve.lerps[i];
  //     auto p = lerp(positions[e.x], positions[e.y], x);
  //     result[i] = p;
  //   }
  // } else {
  //   auto s = (int)curve.lerps.size() - range.x + range.y;
  //   result.resize(s);
  //   for (auto i = range.x; i > range.y; --i) {
  //     auto e = get_edge(triangles, positions, adjacencies, curve.strip[i],
  //                       curve.strip[(i + 1) % curve.lerps.size()]);
  //     if (e == vec2i{-1, -1}) {
  //       std::cout << "point not found in range y <range x" << std::endl;
  //       continue;
  //     }
  //     auto x = curve.lerps[i];
  //     auto p = lerp(positions[e.x], positions[e.y], x);
  //     result[i] = p;
  //   }
  // }

  return result;
}

vector<vec3f> circle_positions(const vector<vec3i> &triangles,
                               const vector<vec3f> &positions,
                               const vector<vec3i> &adjacencies,
                               const Circle &c0) {
  auto pos = vector<vec3f>{};
  for (auto curve : c0.isoline) {
    auto curve_pos =
        closed_curve_positions(curve, triangles, positions, adjacencies);
    pos.insert(pos.end(), curve_pos.begin(), curve_pos.end());
  }

  return pos;
}

unordered_map<int, vec3f>
create_tids(const bezier_mesh &mesh, const vector<float> &distances0,
            float length0, const vector<float> &distances1, float length1) {
  auto tids = unordered_map<int, vec3f>();

  for (auto i = 0; i < mesh.triangles.size(); ++i) {
    auto t = mesh.triangles[i];
    auto d = vec3f{(distances0[t.x] * distances0[t.x] - length0 * length0) -
                       (distances1[t.x] * distances1[t.x] - length1 * length1),
                   (distances0[t.y] * distances0[t.y] - length0 * length0) -
                       (distances1[t.y] * distances1[t.y] - length1 * length1),
                   (distances0[t.z] * distances0[t.z] - length0 * length0) -
                       (distances1[t.z] * distances1[t.z] - length1 * length1)};

    /*
    auto d = vec3f{
      std::cos(distances0[t.x])/std::cos(length0) -
    std::cos(distances1[t.x])/std::cos(length1),
      std::cos(distances0[t.y])/std::cos(length0) -
    std::cos(distances1[t.y])/std::cos(length1),
      std::cos(distances0[t.z])/std::cos(length0) -
    std::cos(distances1[t.z])/std::cos(length1) };
    */

    /*
     auto d = vec3f{
       std::cosh(distances0[t.x])/std::cosh(length0) -
     std::cosh(distances1[t.x])/std::cosh(length1),
       std::cosh(distances0[t.y])/std::cosh(length0) -
     std::cosh(distances1[t.y])/std::cosh(length1),
       std::cosh(distances0[t.z])/std::cosh(length0) -
     std::cosh(distances1[t.z])/std::cosh(length1) };
     */

    if (d.x * d.y <= 0 || d.x * d.z <= 0) {
      tids[i] = d;
    }
  }

  return tids;
}

unordered_map<int, vec3f> create_tids(const bezier_mesh &mesh,
                                      const vector<float> &distances0,
                                      const vector<float> &distances1) {
  auto tids = unordered_map<int, vec3f>();

  for (auto i = 0; i < mesh.triangles.size(); ++i) {
    auto t = mesh.triangles[i];
    auto d = vec3f{distances0[t.x] - distances1[t.x],
                   distances0[t.y] - distances1[t.y],
                   distances0[t.z] - distances1[t.z]};

    if (d.x * d.y <= 0 || d.x * d.z <= 0) {
      tids[i] = d;
    }
  }

  return tids;
}

unordered_map<int, vec3f> create_tids(const bezier_mesh &mesh,
                                      const vector<float> &distances,
                                      const float &radius) {
  auto tids = unordered_map<int, vec3f>();

  for (auto i = 0; i < mesh.triangles.size(); ++i) {
    auto t = mesh.triangles[i];
    auto d = vec3f{distances[t.x] - radius, distances[t.y] - radius,
                   distances[t.z] - radius};

    if (d.x * d.y <= 0 || d.x * d.z <= 0) {
      tids[i] = d;
    }
  }

  return tids;
}
unordered_map<int, vec3f> create_tids(const bezier_mesh &mesh, Circle &circle,
                                      const float &radius) {
  auto tids = unordered_map<int, vec3f>();
  auto &distances = circle.distances;
  circle.tids.resize(mesh.triangles.size(), false);
  for (auto i = 0; i < mesh.triangles.size(); ++i) {
    auto t = mesh.triangles[i];
    auto d = vec3f{distances[t.x] - radius, distances[t.y] - radius,
                   distances[t.z] - radius};

    if (d.x * d.y <= 0 || d.x * d.z <= 0 || d.y * d.z <= 0) {
      tids[i] = d;
      circle.tids[i] = true;
    }
  }

  return tids;
}

std::tuple<int, float, int> first_tid_in_circle(const bezier_mesh &mesh,
                                                Circle &circle,
                                                const float &radius,
                                                const mesh_point &seed) {
  auto &distances = circle.distances;
  auto parsed = vector<bool>(mesh.triangles.size(), false);
  std::deque<int> Q;
  auto tid = (seed.face == -1) ? circle.center.face : seed.face;
  Q.push_back(tid);
  while (!Q.empty()) {
    auto curr = Q.back();
    auto t = mesh.triangles[curr];
    Q.pop_back();
    auto d = vec3f{distances[t.x] - radius, distances[t.y] - radius,
                   distances[t.z] - radius};
    if (d.x * d.y <= 0)
      return {curr, -d.x / (d.y - d.x), 0};
    if (d.x * d.z <= 0)
      return {curr, -d.z / (d.x - d.z), 2};
    if (d.z * d.y <= 0)
      return {curr, -d.y / (d.z - d.y), 1};

    parsed[curr] = true;
    for (auto adj : mesh.adjacencies[curr]) {
      if (!parsed[adj])
        Q.push_front(adj);
    }
  }

  return {-1, 0.f, -1};
}
vector<circle_tids> tids_in_circle(const bezier_mesh &mesh, Circle &circle,
                                   const float &radius) {
  auto &distances = circle.distances;
  auto result = vector<circle_tids>(mesh.triangles.size());
  for (auto i = 0; i < mesh.triangles.size(); ++i) {
    auto &t = mesh.triangles[i];
    auto d = vec3f{distances[t.x] - radius, distances[t.y] - radius,
                   distances[t.z] - radius};
    if (d.x * d.y <= 0 && result[mesh.adjacencies[i][0]].lerp < 0) {
      result[i].lerp = -d.x / (d.y - d.x);
      result[i].offset = 0;
    } else if (d.x * d.z <= 0 && result[mesh.adjacencies[i][2]].lerp < 0) {
      result[i].lerp = -d.z / (d.x - d.z);
      result[i].offset = 2;
    } else if (d.z * d.y <= 0 && result[mesh.adjacencies[i][1]].lerp < 0) {
      result[i].lerp = -d.y / (d.z - d.y);
      result[i].offset = 1;
    }
  }
  return result;
}
std::tuple<int, float, int> find_seed_in_circle(const bezier_mesh &mesh,
                                                const vector<float> &distances,
                                                const float &radius,
                                                const vector<bool> &parsed) {

  for (auto i = 0; i < mesh.triangles.size(); ++i) {
    if (parsed[i])
      continue;
    auto &t = mesh.triangles[i];
    auto d = vec3f{distances[t.x] - radius, distances[t.y] - radius,
                   distances[t.z] - radius};
    if (d.x * d.y <= 0 && !parsed[mesh.adjacencies[i][0]]) {
      auto lerp = -d.x / (d.y - d.x);
      if (lerp == 0 || lerp == 1)
        continue;
      return {i, lerp, 0};
    } else if (d.x * d.z <= 0 && !parsed[mesh.adjacencies[i][2]]) {
      auto lerp = -d.z / (d.x - d.z);
      if (lerp == 0 || lerp == 1)
        continue;
      return {i, lerp, 2};
    } else if (d.z * d.y <= 0 && !parsed[mesh.adjacencies[i][1]]) {
      auto lerp = -d.z / (d.y - d.z);
      if (lerp == 0 || lerp == 1)
        continue;
      return {i, lerp, 1};
    }
  }
  return {-1, 0, -1};
}
std::tuple<int, float, int> find_seed_in_circle(const bezier_mesh &mesh,
                                                const vector<float> &d0,
                                                const vector<float> &d1,
                                                const vector<bool> &parsed) {

  for (auto i = 0; i < mesh.triangles.size(); ++i) {
    if (parsed[i])
      continue;
    auto &t = mesh.triangles[i];
    auto d = vec3f{d0[t.x] - d1[t.x], d0[t.y] - d1[t.y], d0[t.z] - d1[t.z]};
    if (d.x * d.y <= 0 && !parsed[mesh.adjacencies[i][0]]) {
      return {i, -d.x / (d.y - d.x), 0};
    } else if (d.x * d.z <= 0 && !parsed[mesh.adjacencies[i][2]]) {
      return {i, -d.x / (d.z - d.x), 2};
    } else if (d.z * d.y <= 0 && !parsed[mesh.adjacencies[i][1]]) {
      return {i, -d.z / (d.y - d.z), 1};
    }
  }
  return {-1, 0, -1};
}
int find_next_tid_in_circle_from_vert(const bezier_mesh &mesh,
                                      const vector<float> &distances,
                                      const int vid, const float &radius) {
  auto star = mesh.v2t[vid];
  for (auto tid : star) {
    auto t = mesh.triangles[tid];
    auto d = vec3f{distances[t.x] - radius, distances[t.y] - radius,
                   distances[t.z] - radius};
  }
}
int choose_next_tid(const bezier_mesh &mesh, const vector<float> &distances,
                    const int curr_tid, const float &lerp, const int offset,
                    const float &radius) {
  auto next = -1;
  if (lerp > 1e-10 && lerp < 1 - 1e-10) {
    next = mesh.adjacencies[curr_tid][offset];
  } else {
    auto bary = zero3f;
    bary[offset] = clamp(1 - lerp, 0.f, 1.f);
    bary[(offset + 1) % 3] = clamp(lerp, 0.f, 1.f);
    auto point = mesh_point{curr_tid, vec2f{bary.y, bary.z}};
    auto [is_vert, kv] = point_is_vert(point);
    if (!is_vert)
      std::cout << "this point should be a vert" << std::endl;

    auto vid = mesh.triangles[curr_tid][kv];
  }
}
Isoline create_isoline(const bezier_mesh &mesh, const vector<float> &distances,
                       const float &radius) {
  Isoline iso = {};
  auto parsed = vector<bool>(mesh.triangles.size(), false);
  auto [seed, lerp, offset] =
      find_seed_in_circle(mesh, distances, radius, parsed);
  while (seed != -1) {
    auto curve = closed_curve{};
    while (!parsed[seed]) {
      if (lerp < 0 || lerp > 1) {
        std::cout << "point outside of trianle" << std::endl;
      }
      curve.strip.push_back(seed);
      curve.lerps.push_back(lerp);
      parsed[seed] = true;

      auto next = mesh.adjacencies[seed][offset];
      auto t = mesh.triangles[next];
      auto dist = vec3f{distances[t.x] - radius, distances[t.y] - radius,
                        distances[t.z] - radius};
      auto h = find_in_vec(mesh.adjacencies[next], seed);
      if (dist[h] * dist[(h + 2) % 3] <= 0) {
        seed = next;
        offset = (h + 2) % 3;
        lerp = -dist[(h + 2) % 3] / (dist[h] - dist[(h + 2) % 3]);
      } else if (dist[(h + 1) % 3] * dist[(h + 2) % 3] <= 0) {
        seed = next;
        offset = (h + 1) % 3;
        lerp = -dist[(h + 1) % 3] / (dist[(h + 2) % 3] - dist[(h + 1) % 3]);
      } else if (dist[h] * dist[(h + 1) % 3] <= 0) { // closing circle
        seed = next;
        offset = h;
        lerp = -dist[h] / (dist[(h + 1) % 3] - dist[h]);
        break;
      } else {
        std::cerr << "Error in computing tids on circle" << std::endl;
        break;
      }
      // lerp = clamp(lerp, 0.f, 1.f);
    }

    if (curve.strip.size() > 0) {
      iso.push_back(curve);
    }
    std::tie(seed, lerp, offset) =
        find_seed_in_circle(mesh, distances, radius, parsed);
  }
  return iso;
}
Isoline create_isoline(const bezier_mesh &mesh, const vector<float> &d0,
                       const vector<float> &d1) {
  Isoline iso = {};
  auto parsed = vector<bool>(mesh.triangles.size(), false);
  auto [seed, lerp, offset] = find_seed_in_circle(mesh, d0, d1, parsed);
  while (seed != -1) {
    auto curve = closed_curve{};
    while (!parsed[seed]) {
      curve.strip.push_back(seed);
      curve.lerps.push_back(lerp);
      parsed[seed] = true;
      auto next = mesh.adjacencies[seed][offset];
      auto t = mesh.triangles[next];
      auto dist =
          vec3f{d0[t.x] - d1[t.x], d0[t.y] - d1[t.y], d0[t.z] - d1[t.z]};
      auto h = find_in_vec(mesh.adjacencies[next], seed);
      if (dist[h] * dist[(h + 2) % 3] <= 0) {
        seed = next;
        offset = (h + 2) % 3;
        lerp = -dist[(h + 2) % 3] / (dist[h] - dist[(h + 2) % 3]);
      } else if (dist[(h + 1) % 3] * dist[(h + 2) % 3] <= 0) {
        seed = next;
        offset = (h + 1) % 3;
        lerp = -dist[(h + 1) % 3] / (dist[(h + 2) % 3] - dist[(h + 1) % 3]);
      } else if (dist[h] * dist[(h + 1) % 3] <= 0) { // closing circle
        seed = next;
        offset = h;
        lerp = -dist[h] / (dist[(h + 1) % 3] - dist[h]);
        break;
      } else {
        std::cerr << "Error in computing tids on circle" << std::endl;
        break;
      }
      lerp = clamp(lerp, 0.f, 1.f);
    }
    if (curve.strip.size() > 0)
      iso.push_back(curve);
    std::tie(seed, lerp, offset) = find_seed_in_circle(mesh, d0, d1, parsed);
  }
  return iso;
}
vector<bool> flood_circle(const bezier_mesh &mesh, const Circle &circle) {
  auto visited = vector<bool>(mesh.triangles.size(), false);
  for (auto &curve : circle.isoline) {
    for (auto tid : curve.strip)
      visited[tid] = true;
  }
  auto queue = deque<int>{};
  queue.push_back(circle.center.face);

  while (!queue.empty()) {
    int face = queue.back();
    queue.pop_back();
    for (int i = 0; i < 3; ++i) {
      int neighbor = mesh.adjacencies[face][i];
      if (neighbor == -1)
        continue;
      if (visited[neighbor])
        continue;

      queue.push_back(neighbor);
      visited[neighbor] = true;
    }
  }

  return visited;
}
mesh_point find_center_for_spider_net(const bezier_mesh &mesh,
                                      const mesh_point &a, const mesh_point &b,
                                      const Circle *c) {
  auto d0 = get_geodesic_distances(mesh, a);
  auto d1 = get_geodesic_distances(mesh, b);
  auto dc = c->distances;
  auto iso = create_isoline(mesh, d0, d1);
  auto curr_radius = 0.75 * c->radius;

  for (auto &curve : iso) {
    auto s = curve.strip.size();
    for (auto i = 0; i < s; ++i) {

      auto prev = curve.strip[(s - 1 + i) % s];
      auto curr = curve.strip[i];
      auto next = curve.strip[(i + 1) % s];

      auto prev_lerp = curve.lerps[(s - 1 + i) % s];
      auto curr_lerp = curve.lerps[i];

      auto h0 = find_in_vec(mesh.adjacencies[curr], prev);
      auto h1 = find_in_vec(mesh.adjacencies[curr], next);
      if (h0 == -1)
        std::cout << "This cannot happens" << std::endl;
      auto a0 = eval_point_from_lerp(curr, h0, 1 - prev_lerp);
      auto a1 = eval_point_from_lerp(curr, h1, curr_lerp);

      auto f0 = interpolate_distances(mesh, d0, a0);
      auto f1 = interpolate_distances(mesh, d0, a1);
      if (f0 < curr_radius && f1 > curr_radius &&
          interpolate_distances(mesh, dc, a1) > c->radius) {
        auto bary0 = a0.uv;
        auto bary1 = a1.uv;
        auto lambda = (curr_radius - f0) / (f1 - f0);
        auto bary =
            (1 - lambda) * vec3f{1 - bary0.x - bary0.y, bary0.x, bary0.y} +
            lambda * vec3f{1 - bary1.x - bary1.y, bary1.x, bary1.y};

        return {curr, {bary.y, bary.z}};
      } else if (f0 > curr_radius && f1 < curr_radius &&
                 interpolate_distances(mesh, dc, a0) > c->radius) {
        auto bary0 = a0.uv;
        auto bary1 = a1.uv;
        auto lambda = (curr_radius - f1) / (f0 - f1);
        auto bary =
            (1 - lambda) * vec3f{1 - bary1.x - bary1.y, bary1.x, bary1.y} +
            lambda * vec3f{1 - bary0.x - bary0.y, bary0.x, bary0.y};
        return {curr, {bary.y, bary.z}};
      }
    }
  }
  return mesh_point{};
  // auto distc = c->distances;
  // auto radius = c->radius;
  // // auto parsed = flood_circle(mesh, *c);
  // auto parsed = vector<bool>(mesh.triangles.size(), false);
  // std::deque<int> Q = {seed.face};
  // // auto [seed, lerp, offset] = find_seed_in_circle(mesh, d0, d1, parsed);
  // while (!Q.empty()) {
  //   auto curr = Q.back();
  //   Q.pop_back();
  //   parsed[curr] = true;
  //   auto lerp = 0.f;
  //   auto offset = -1;
  //   auto t = mesh.triangles[curr];
  //   auto d = vec3f{d0[t.x] - d1[t.x], d0[t.y] - d1[t.y], d0[t.z] - d1[t.z]};
  //   if (d.x * d.y <= 0 && !parsed[mesh.adjacencies[curr][0]]) {
  //     {
  //       lerp = -d.x / (d.y - d.x);
  //       offset = 0;
  //       Q.push_back(mesh.adjacencies[curr][0]);
  //     }
  //   } else if (d.x * d.z <= 0 && !parsed[mesh.adjacencies[curr][2]]) {
  //     lerp = -d.x / (d.z - d.x);
  //     offset = 2;

  //   } else if (d.z * d.y <= 0 && !parsed[mesh.adjacencies[curr][1]]) {
  //     lerp = -d.z / (d.y - d.z);
  //     offset = 1;
  //   }
  //   if (offset != -1) {
  //     auto bary = zero3f;
  //     bary[offset] = 1 - lerp;
  //     bary[(offset + 1) % 3] = lerp;
  //     auto curr_point = mesh_point{curr, {bary.y, bary.z}};
  //     auto dx = d0[mesh.triangles[curr][offset]];
  //     auto dy = d0[mesh.triangles[curr][(offset + 1) % 3]];
  //     auto value = (1 - lerp) * dx + lerp * dy;
  //     if (std::abs(value - radius) < 1e-3 &&
  //         interpolate_distances(mesh, distc, curr_point) > radius)
  //       return curr_point;
  //   }
  // }
  // return mesh_point{};
}
unordered_map<int, float> create_ordered_tids(const bezier_mesh &mesh,
                                              const float &radius,
                                              Circle &circle,
                                              const mesh_point &seed) {
  circle.tids.clear();
  auto [tid, d, k] = first_tid_in_circle(mesh, circle, radius, seed);
  unordered_map<int, float> tids_and_lerps;
  tids_and_lerps[tid] = d;
  circle.tids.push_back(tid);
  auto &distances = circle.distances;
  std::deque<vec2i> Q;
  Q.push_back({tid, k});
  auto parsed = vector<bool>(mesh.triangles.size(), false);
  parsed[tid] = true;
  while (!Q.empty()) {
    auto curr = Q.back();
    Q.pop_back();
    tid = mesh.adjacencies[curr.x][curr.y];
    if (parsed[tid])
      continue;
    auto t = mesh.triangles[tid];
    auto dist = vec3f{distances[t.x] - radius, distances[t.y] - radius,
                      distances[t.z] - radius};
    auto h = find_in_vec(mesh.adjacencies[tid], curr.x);

    if (dist[h] * dist[(h + 2) % 3] <= 0) {
      circle.tids.push_back(tid);
      Q.push_back({tid, (h + 2) % 3});
      tids_and_lerps[tid] = -dist[(h + 2) % 3] / (dist[h] - dist[(h + 2) % 3]);
      parsed[tid] = true;
    } else if (dist[(h + 1) % 3] * dist[(h + 2) % 3] <= 0) {
      circle.tids.push_back(tid);
      Q.push_back({tid, (h + 1) % 3});
      tids_and_lerps[tid] =
          -dist[(h + 1) % 3] / (dist[(h + 2) % 3] - dist[(h + 1) % 3]);
      parsed[tid] = true;
    } else {
      std::cerr << "Error tid with no neighbors" << std::endl;
    }
  }
  return tids_and_lerps;
}
unordered_map<int, vec3f> create_cropped_tids(const bezier_mesh &mesh,
                                              Circle &circle, const int tid0,
                                              const int tid1,
                                              const bool reverse) {

  auto tids = unordered_map<int, vec3f>();
  auto &distances = circle.distances;
  auto &map = circle.tids;
  auto arrived = false;
  auto adj = mesh.adjacencies[tid0];
  auto choices = vector<int>{};
  for (auto i = 0; i < 3; ++i) {
    if (!map[adj[i]])
      continue;
    choices.push_back(adj[i]);
  }
  auto curr_tid = (!reverse) ? choices[0] : choices[1];
  map[tid0] = false;
  while (!arrived) {
    adj = mesh.adjacencies[curr_tid];
    for (auto i = 0; i < 3; ++i) {
      if (!map[adj[i]])
        continue;
      map[curr_tid] = false;
      curr_tid = adj[i];
      break;
    }
    if (curr_tid == tid1)
      arrived = true;
  }
  map[tid0] = true;
  for (auto i = 0; i < mesh.triangles.size(); ++i) {
    if (!map[i])
      continue;
    auto t = mesh.triangles[i];
    auto d =
        vec3f{distances[t.x] - circle.radius, distances[t.y] - circle.radius,
              distances[t.z] - circle.radius};

    if (d.x * d.y <= 0 || d.x * d.z <= 0 || d.y * d.z <= 0) {
      tids[i] = d;
    }
  }

  return tids;
}

closed_curve create_closed_curve(const bezier_mesh &mesh, const Circle &circle,
                                 const unordered_map<int, float> &tids) {
  if (tids.size() == 0) {
    return closed_curve();
  }

  auto curve = closed_curve();
  auto s = (int)circle.tids.size();

  for (auto i = 0; i < s; ++i) {
    auto tid = circle.tids[i];
    curve.strip.push_back(tid);
    curve.lerps.push_back(tids.at(tid));
  }

  return curve;
}

// -------------------------------------------------------------------------------------
mesh_point get_closed_curve_point(const vector<vec3i> &triangles,
                                  const vector<vec3f> &positions,
                                  const vector<vec3i> &adjacencies,
                                  const closed_curve &curve, int ix) {
  if (ix < 0 || ix > curve.lerps.size() + 1) {
    return mesh_point();
  }

  if (ix == curve.lerps.size() + 1) {
    ix = 0;
  }

  auto e = get_edge(triangles, positions, adjacencies, curve.strip[ix],
                    curve.strip[(ix + 1) % curve.lerps.size()]);

  if (e == vec2i{-1, -1}) {
    return mesh_point();
  }

  return eval_mesh_point(triangles, positions, curve.strip[ix],
                         lerp(positions[e.x], positions[e.y], curve.lerps[ix]));
}

bool set_radius(const bezier_mesh &mesh, Circle *circle, const float &radius,
                const mesh_point &seed) {
  // for (auto curve : circle.isoline) {
  //   delete_shape(curve.shape);
  // }
  if (radius == 0.f)
    return true;

  // auto tids =
  //     // create_ordered_tids(mesh, radius, circle, seed);
  //     create_tids(mesh, circle, radius);

  // if (tids.size() > 0) {
  //   auto curve = create_closed_curve(mesh, tids);

  //   if (curve.strip.size() > 0) {
  //     circle.isoline.push_back(curve);
  //   }
  // }
  circle->isoline = create_isoline(mesh, circle->distances, radius);

  if (circle->isoline.size() == 0) {
    return false;
  }

  circle->radius = radius;

  return true;
}
bool set_center(const bezier_mesh &mesh, Circle *circle,
                const mesh_point &center) {

  circle->center = center;
  circle->distances = compute_geodesic_distances(
      mesh.solver, mesh.triangles, mesh.positions, mesh.adjacencies, {center});

  return set_radius(mesh, circle, circle->radius);
}
Circle create_circle(const bezier_mesh &mesh, const mesh_point &center,
                     const float &radius) {
  auto circle = Circle();
  circle.center = center;
  circle.distances = get_geodesic_distances(mesh, center);

  if (!set_radius(mesh, &circle, radius)) {
    return Circle();
  }

  return circle;
}
Circle create_circle(const bezier_mesh &mesh, const mesh_point &center,
                     const float &radius, const vector<float> &distances) {
  auto circle = Circle();
  circle.center = center;
  circle.distances = distances;

  if (!set_radius(mesh, &circle, radius)) {
    return Circle();
  }

  return circle;
}

Circle create_circle(const bezier_mesh &mesh, const mesh_point &center,
                     const mesh_point &point) {
  auto circle = Circle();
  circle.center = center;
  circle.distances = get_geodesic_distances(mesh, center);

  auto radius = get_distance(mesh.triangles[point.face], get_bary(point.uv),
                             circle.distances);

  if (!set_radius(mesh, &circle, radius)) {
    return Circle();
  }

  return circle;
}

// void map_tids(const bezier_mesh &mesh, Circle &circle) {

//   auto initialized = false;
//   for (auto &curve : circle.isoline) {
//     for (auto &tid : curve.strip) {
//       auto path =
//           compute_geodesic_path(mesh, circle.center, {tid, vec2f{0.33,
//           0.33}});
//       auto v = tangent_path_direction(mesh, path);
//       if (!initialized) {
//         circle.e = v;
//         circle.tids_mapping[tid] = 0.f;
//         initialized = true;

//       } else {
//         auto teta = angle(circle.e, v);
//         if (cross(circle.e, v) < 0)
//           teta = 2 * pif - teta;

//         circle.tids_mapping[tid] = teta;
//       }
//     }
//   }
// }
mesh_point closest_point_on_circle(const bezier_mesh &mesh,
                                   const Circle *circle,
                                   const mesh_point &point) {
  auto curve = circle->isoline[0];
  auto min_dist = flt_max;
  auto closest = mesh_point{};
  auto point_pos = eval_position(mesh.triangles, mesh.positions, point);
  auto curve_pos = closed_curve_positions(curve, mesh.triangles, mesh.positions,
                                          mesh.adjacencies);
  for (auto i = 0; i < curve_pos.size(); i++) {
    auto p = curve_pos[i];
    auto curr_dist = length(point_pos - p);
    if (curr_dist < min_dist) {
      closest = get_closed_curve_point(mesh.triangles, mesh.positions,
                                       mesh.adjacencies, curve, i);
      min_dist = curr_dist;
    }
  }

  return closest;
}
vec2i cropping_range(const bezier_mesh &mesh, const Circle &circle,
                     const vector<mesh_point> &range) {
  auto entries = vec2i{-1, -1};
  if (circle.isoline.size() != 1)
    return entries;
  auto &strip = circle.isoline[0].strip;

  for (auto i = 0; i < strip.size(); ++i) {
    if (strip[i] == range[0].face) {
      entries.x = i;

    } else if (strip[i] == range[1].face) {
      entries.y = i;
    }
    if (entries.y != -1 && entries.x != -1)
      return entries;
  }

  if (entries.x != -1) {
    if (entries.y == -1 && range[0].face == range[1].face)
      return {entries.x, entries.x};
    else if (entries.y != -1)
      return entries;
  }

  std::cout << "tids not found" << std::endl;
  return {-1, -1};
}
vec2i cropping_range_generic_points(const bezier_mesh &mesh,
                                    const Circle &circle,
                                    vector<mesh_point> &range) {

  auto entries = vec2i{-1, -1};
  auto posx = eval_position(mesh.triangles, mesh.positions, range[0]);
  auto posy = eval_position(mesh.triangles, mesh.positions, range[1]);
  if (circle.isoline.size() != 1)
    return entries;
  auto &strip = circle.isoline[0].strip;

  for (auto i = 0; i < strip.size(); ++i) {
    auto [insidex, baryx] =
        point_in_triangle(mesh.triangles, mesh.positions, strip[i], posx);
    auto [insidey, baryy] =
        point_in_triangle(mesh.triangles, mesh.positions, strip[i], posy);
    if (insidex) {
      range[0] = {strip[i], baryx};

      entries.x = i;

    } else if (insidey) {
      range[1] = {strip[i], baryy};
      entries.y = i;
    }
    if (entries.y != -1 && entries.x != -1)
      return entries;
  }

  if (entries.x != -1) {
    if (entries.y == -1 && range[0].face == range[1].face)
      return {entries.x, entries.x};
    else if (entries.y != -1)
      return entries;
  }

  std::cout << "tids not found" << std::endl;
  return {-1, -1};
}
vector<vec3f> crop_circle(const vector<vec3f> &pos, const vec2i &range) {

  auto result = vector<vec3f>{};
  if (range.x < range.y) {
    result.insert(result.begin(), pos.begin() + range.x,
                  pos.begin() + range.y + 1);
  } else if (range.x > range.y) {
    result.insert(result.begin(), pos.begin() + range.x, pos.end());
    result.insert(result.end(), pos.begin(), pos.begin() + range.y + 1);
  } else if (range.x != -1) {
    result = {pos[range.x], pos[range.x + 1]};
  } else
    result = pos;

  return result;
}
vector<vec3f> crop_circle(const bezier_mesh &mesh, const vector<vec3f> &pos,
                          const vec2i &range,
                          const vector<mesh_point> &extreme) {

  auto result = vector<vec3f>{};
  if (range.x < range.y) {
    result.insert(result.end(), pos.begin() + range.x,
                  pos.begin() + range.y + 1);
    result[0] = eval_position(mesh.triangles, mesh.positions, extreme[0]);
    result.back() = eval_position(mesh.triangles, mesh.positions, extreme[1]);
  } else if (range.x > range.y) {
    result.insert(result.end(), pos.begin() + range.x, pos.end());
    result.insert(result.end(), pos.begin(), pos.begin() + range.y + 1);
    result[0] = eval_position(mesh.triangles, mesh.positions, extreme[0]);
    result.back() = eval_position(mesh.triangles, mesh.positions, extreme[1]);
  } else if (range.x != -1) {
    result = {pos[range.x], pos[range.x + 1]};
  } else
    result = pos;

  return result;
}
vector<vector<vec3f>> crop_circle(const bezier_mesh &mesh,
                                  const vector<vec3f> &pos,
                                  const vector<vec2i> &ranges,
                                  const vector<vector<mesh_point>> &extrema) {

  auto result = vector<vector<vec3f>>(ranges.size());
  for (auto i = 0; i < ranges.size(); ++i) {
    auto curr_pos = crop_circle(mesh, pos, ranges[i], extrema[i]);
    result[i] = curr_pos;
  }

  return result;
}
vector<vec3f> crop_outside_circle(const bezier_mesh &mesh,
                                  const Circle &curr_circle,
                                  const Circle &container,
                                  const mesh_point &start,
                                  const int start_entry, const mesh_point &end,
                                  const int end_entry) {
  auto pos = circle_positions(mesh.triangles, mesh.positions, mesh.adjacencies,
                              curr_circle);
  if (start_entry < end_entry) {
    auto entry = (start_entry + end_entry) / 2;
    auto tid = curr_circle.isoline[0].strip[entry];
    auto offset = find_in_vec(
        mesh.adjacencies[tid],
        curr_circle.isoline[0]
            .strip[(entry + 1) % curr_circle.isoline[0].strip.size()]);
    auto lerp = curr_circle.isoline[0].lerps[entry];
    auto middle = eval_point_from_lerp(tid, offset, lerp);
    if (interpolate_distances(mesh, container.distances, middle) >
        container.radius)
      return crop_circle(mesh, pos, vec2i{start_entry, end_entry},
                         {start, end});
    else
      return crop_circle(mesh, pos, vec2i{end_entry, start_entry},
                         {end, start});
  } else {
    auto entry = (start_entry + end_entry) / 2;
    auto tid = curr_circle.isoline[0].strip[entry];
    auto offset = find_in_vec(
        mesh.adjacencies[tid],
        curr_circle.isoline[0]
            .strip[(entry + 1) % curr_circle.isoline[0].strip.size()]);
    auto lerp = curr_circle.isoline[0].lerps[entry];
    auto middle = eval_point_from_lerp(tid, offset, lerp);
    if (interpolate_distances(mesh, container.distances, middle) >
        container.radius)
      return crop_circle(mesh, pos, vec2i{end_entry, start_entry},
                         {end, start});

    else
      return crop_circle(mesh, pos, vec2i{start_entry, end_entry},
                         {start, end});
  }
}