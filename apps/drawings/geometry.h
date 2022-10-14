#pragma once
#include <stdio.h>
#include <yocto/yocto_geometry.h>
#include <yocto/yocto_mesh.h>
#include <yocto/yocto_shape.h>

#include <iostream>

#include "Eigen/Dense"
#include "Eigen/Sparse"
//#include "strip.h"
#include "struct.h"
using namespace yocto;

vec3f project_vec(const vec3f &v, const vec3f &n);

template <typename T> inline int find(const T &vec, int x) {
  for (int i = 0; i < size(vec); i++)
    if (vec[i] == x)
      return i;
  return -1;
}

inline vec3f get_bary(const vec2f &uv) {
  return vec3f{1 - uv.x - uv.y, uv.x, uv.y};
}

int next_tid(const geodesic_solver &solver, const vector<vector<float>> &angles,
             const vector<vec3f> &positions, const vector<vector<int>> &v2t,
             const vector<vec3i> &triangles, const vector<vec3f> &normals,
             const int from, const vec3f &v);

enum transport_mode { V2V, V2T, T2V, T2T };
// Utility
Eigen::VectorXd eigen_wrapper(const vector<float> &f);

vec3f polar_basis(const vector<vec3i> &triangles,
                  const vector<vec3f> &positions, int tid);

vec3f polar_basis(const geodesic_solver &solver, const vector<vec3f> &positions,
                  const vector<vec3f> &normals, int vid);

float angle_in_tangent_space(const vector<vec3i> &triangles,
                             const vector<vec3f> &positions,
                             const vector<vector<int>> &v2t, const vec3f &v,
                             const int vid, const vec3f &n);
vec3f rot_vect(const vec3f &p, const vec3f &axis, const float angle);
vec2f rot_vect(const vec2f &p, const float theta);
void trace_in_triangles(const vector<vec3f> &positions,
                        const vector<vec3i> &triangles, const vec3f &dir,
                        const vec3f &bary, const int pid, vec3f &sample_pos,
                        vec3f &sample_bary);

float intersect_line_segments(const vector<vec3i> &triangles,
                              const vector<vec3f> &positions, const int tid,
                              mesh_point &start1, mesh_point &end1,
                              mesh_point &start2, mesh_point &end2);
pair<int, vec3f> handle_vert(const geodesic_solver &solver,
                             const vector<vec3i> &triangles,
                             const vector<vec3f> &positions,
                             const vector<vec3f> &normals,
                             const vector<vec3i> &adjacencies,
                             const vector<vector<int>> &v2p_adjacencies,
                             const vector<vector<float>> &angles,
                             const vector<float> &total_angles, const int vid,
                             const int tid, const vec3f dir);

inline vec3f tid_normal(const vector<vec3i> &triangles,
                        const vector<vec3f> &positions, const int tid) {
  auto p0 = positions[triangles[tid].x];
  auto p1 = positions[triangles[tid].y];
  auto p2 = positions[triangles[tid].z];

  return normalize(cross(p1 - p0, p2 - p0));
}
inline vec3f tid_centroid(const vector<vec3i> &triangles,
                          const vector<vec3f> &positions, const int tid) {
  vec3f p0 = positions[triangles[tid].x];
  vec3f p1 = positions[triangles[tid].y];
  vec3f p2 = positions[triangles[tid].z];

  return (p0 + p1 + p2) / 3.0;
}
vector<vec3f> straight_line(const vec3f &p0, const vec3f &p1);
vector<vec3f> compute_gradient(const geodesic_solver &solver,
                               const vector<vec3f> &positions,
                               const vector<vec3f> &normals,
                               const Eigen::SparseMatrix<double> &G,
                               const Eigen::VectorXd &f, bool normalized);
std::tuple<Eigen::SparseMatrix<double, 1>, Eigen::SparseMatrix<double, 1>,
           vector<Eigen::MatrixXd>, vector<vector<vec2f>>,
           vector<Eigen::Matrix2d>, vector<vector<vector<pair<int, float>>>>,
           vector<pair<vec3f, vec3f>>, float>
init_discrete_diff_op_2(
    const geodesic_solver &solver, const vector<vector<float>> &angles,
    const vector<float> &total_angles, const vector<vec3i> &triangles,
    const vector<vec3f> &positions, const vector<vec3i> &adjacencies,
    const vector<vector<int>> &v2t, const vector<vec3f> &normals);
// compute geodesic between start and end
geodesic_path compute_geodesic_path(const bezier_mesh &mesh,
                                    const mesh_point &start,
                                    const mesh_point &end);
vector<float> compute_pruned_geodesic_distances(
    const geodesic_solver &solver, const vector<vec3i> &triangles,
    const vector<vec3f> &positions, const vector<vec3i> &adjacencies,
    const vector<vector<int>> &v2t, const mesh_point &source,
    const vector<mesh_point> &targets);
mesh_point eval_point(const bezier_mesh &mesh, const vector<mesh_point> &path,
                      const float &t);
vec3f tranport_vector(
    const geodesic_solver &solver, const vector<vector<float>> &angles,
    const vector<float> &total_angles, const vector<vec3i> &triangles,
    const vector<vec3f> &positions, const vector<vec3i> &adjacencies,
    const vector<vector<int>> &v2t, const vec3f &v,
    const vector<vec3f> &normals, const mesh_point &from, const mesh_point &to);

vector<mesh_point> straightest_geodesic(
    const geodesic_solver &solver, const vector<vec3i> &triangles,
    const vector<vec3f> &positions, const vector<vec3f> &normals,
    const vector<vec3i> &adjacencies,
    const vector<vector<int>> &v2p_adjacencies,
    const vector<vector<float>> &angles, const vector<float> &total_angles,
    const mesh_point &from, const vec3f &v, const float &l);
vector<vec3f> path_positions(const bezier_mesh &mesh,
                             const vector<mesh_point> &path);
vec3f geodesic_path_tangent(const vector<vec3i> &triangles,
                            const vector<vec3f> &positions,
                            const vector<vec3i> &adjacencies,
                            const geodesic_path &path, const mesh_point &point);
vector<vec3f> trace_polyline_from_samples(const vector<mesh_point> &samples,
                                          const vector<vec3i> &triangles,
                                          const vector<vec3f> &positions);

Eigen::SparseMatrix<double, 1> init_riemannian_gradient_matrix(
    const geodesic_solver &solver, const vector<vector<float>> &angles,
    const vector<vec3f> &positions, const vector<vec3f> &normals);

void parallel_transp(
    const geodesic_solver &solver, const vector<vector<float>> &angles,
    const vector<float> &total_angles, const vector<vec3i> &triangles,
    const vector<vec3f> &positions, const vector<vec3i> &adjacencies,
    const vector<vector<int>> &v2t, vec3f &v, const vector<vec3f> &normals,
    const int &from, const int &to, const int &mode);

inline void parallel_transport(const bezier_mesh &mesh, vec3f &v,
                               const int from, const int to, const int mode) {
  return parallel_transp(mesh.solver, mesh.angles, mesh.total_angles,
                         mesh.triangles, mesh.positions, mesh.adjacencies,
                         mesh.v2t, v, mesh.normals, from, to, mode);
}
void p_transp_along_path(const bezier_mesh &mesh, const geodesic_path &path,
                         vec3f v);

vec3f tri_bary_coords(const vec3f &v0, const vec3f &v1, const vec3f &v2,
                      const vec3f &p);

inline vec3f tri_bary_coords(const vector<vec3i> &triangles,
                             const vector<vec3f> &positions, const int pid,
                             const vec3f &p) {
  auto px = positions[triangles[pid].x];
  auto py = positions[triangles[pid].y];
  auto pz = positions[triangles[pid].z];
  return tri_bary_coords(px, py, pz, p);
}
inline bool point_are_too_close(bezier_mesh &mesh, const mesh_point &a,
                                const mesh_point &b) {
  auto pos_a = eval_position(mesh.triangles, mesh.positions, a);
  auto pos_b = eval_position(mesh.triangles, mesh.positions, b);
  return (length(pos_b - pos_a) < 1e-6);
}
vector<float> subdivide_angles(const int number_of_subdivision);
void extended_solver(bezier_mesh &mesh, const int k);
// Strip
// struct strip_arena {
//   vector<float> field = {};
//   vector<int> parents = {};
//   vector<bool> in_queue = {};
// };

// void init_arena(strip_arena &arena, size_t size) {
//   arena.parents = vector<int>(size, -1);
//   arena.field = vector<float>(size, flt_max);
//   arena.in_queue = vector<bool>(size, false);
// }

template <typename Update, typename Stop, typename Exit>
void search_strip(vector<float> &field, vector<bool> &in_queue,
                  const dual_geodesic_solver &solver,
                  const vector<vec3i> &triangles,
                  const vector<vec3f> &positions, int start, int end,
                  Update &&update, Stop &&stop, Exit &&exit) {
  auto destination_pos =
      eval_position(triangles, positions, {end, {1.0f / 3, 1.0f / 3}});

  auto estimate_dist = [&](int face) {
    auto p = eval_position(triangles, positions, {face, {1.0f / 3, 1.0f / 3}});
    return length(p - destination_pos);
  };
  field[start] = estimate_dist(start);

  // Cumulative weights of elements in queue. Used to keep track of the
  // average weight of the queue.
  double cumulative_weight = 0.0;

  // setup queue
  auto queue = std::deque<int>{};
  in_queue[start] = true;
  cumulative_weight += field[start];
  queue.push_back(start);

  while (!queue.empty()) {
    auto node = queue.front();
    auto average_weight = (float)(cumulative_weight / queue.size());

    // Large Label Last (see comment at the beginning)
    for (auto tries = 0; tries < queue.size() + 1; tries++) {
      if (field[node] <= average_weight)
        break;
      queue.pop_front();
      queue.push_back(node);
      node = queue.front();
    }

    // Remove node from queue.
    queue.pop_front();
    in_queue[node] = false;
    cumulative_weight -= field[node];

    // Check early exit condition.
    if (exit(node))
      break;
    if (stop(node))
      continue;

    for (auto i = 0; i < (int)solver.graph[node].size(); i++) {
      auto neighbor = solver.graph[node][i].node;
      if (neighbor == -1)
        continue;

      // Distance of neighbor through this node
      auto new_distance = field[node];
      new_distance += solver.graph[node][i].length;
      new_distance += estimate_dist(neighbor);
      new_distance -= estimate_dist(node);

      auto old_distance = field[neighbor];
      if (new_distance >= old_distance)
        continue;

      if (in_queue[neighbor]) {
        // If neighbor already in queue, don't add it.
        // Just update cumulative weight.
        cumulative_weight += new_distance - old_distance;
      } else {
        // If neighbor not in queue, add node to queue using Small Label
        // First (see comment at the beginning).
        if (queue.empty() || (new_distance < field[queue.front()]))
          queue.push_front(neighbor);
        else
          queue.push_back(neighbor);

        // Update queue information.
        in_queue[neighbor] = true;
        cumulative_weight += new_distance;
      }

      // Update distance of neighbor.
      field[neighbor] = new_distance;
      if (update(node, neighbor, new_distance))
        return;
    }
  }
}

// vector<int> compute_strip(const bezier_mesh &mesh, strip_arena &arena,
//                           int start, int end) {
//   if (start == end)
//     return {start};

//   // initialize once for all and sparsely cleanup at the end of every solve
//   // auto parents  = vector<int>(mesh.dual_solver.graph.size(), -1);
//   // auto field    = vector<float>(mesh.dual_solver.graph.size(), flt_max);
//   // auto in_queue = vector<bool>(mesh.dual_solver.graph.size(), false);
//   auto visited = vector<int>{start};

//   auto sources = vector<int>{start};
//   auto update = [&arena, &visited, end](int node, int neighbor,
//                                         float new_distance) {
//     arena.parents[neighbor] = node;
//     visited.push_back(neighbor);
//     return neighbor == end;
//   };
//   auto stop = [](int node) { return false; };
//   auto exit = [](int node) { return false; };

//   search_strip(arena.field, arena.in_queue, mesh.dual_solver, mesh.triangles,
//                mesh.positions, start, end, update, stop, exit);
//   // visit_dual_graph(field, mesh.dual_solver, mesh.triangles,
//   mesh.positions,
//   //     in_queue, start, end, update, visited);

//   // extract_strip
//   auto strip = vector<int>{};
//   auto node = end;
//   assert(arena.parents[end] != -1);
//   strip.reserve((int)sqrt(arena.parents.size()));
//   while (node != -1) {
//     assert(find_in_vector(strip, node) != 1);
//     strip.push_back(node);
//     node = arena.parents[node];
//   }

//   // cleanup buffers
//   for (auto &v : visited) {
//     arena.parents[v] = -1;
//     arena.field[v] = flt_max;
//     arena.in_queue[v] = false;
//   }
//   // assert(check_strip(mesh.adjacencies, strip));
//   return strip;
// }
