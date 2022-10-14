//
//  karcher.cpp
//  glfw
//
//  Created by Claudio Mancinelli on 21/07/2020.
//

#include "geometry.h"

#include <deque>

// utility (geometry)
// project a vector v onto a plane having n as normal
vec3f project_vec(const vec3f &v, const vec3f &n) {
  auto proj = n * dot(v, n);

  return v - proj;
}

using namespace std;
mesh_point point_from_vert(const vector<vec3i> &triangles,
                           const vector<vector<int>> &v2t, const int vid,
                           const int tid = -1) {
  auto tr = -1;
  if (tid < 0)
    tr = v2t[vid][0];
  else {
    auto entry = find(v2t[vid], tid);
    tr = v2t[vid][entry];
  }

  auto k = find(triangles[tr], vid);
  if (k == -1)
    std::cerr << "Error in conveting vert to point" << std::endl;
  auto bary = zero3f;
  bary[k] = 1.0;
  return {tr, {bary.y, bary.z}};
}
vec3f rot_vect(const vec3f &p, const vec3f &axis, const float angle) {
  auto M = rotation_frame(axis, angle);
  auto v = p;
  return transform_vector(M, v);
}
vec2f rot_vect(const vec2f &p, const float theta) {
  auto M = mat2f{{yocto::cos(theta), -yocto::sin(theta)},
                 {yocto::sin(theta), yocto::cos(theta)}};
  auto v = M * p;
  return v;
}

float nbr_avg_edge_length(const geodesic_solver &G, int vid) {
  auto nbr = G.graph[vid];
  auto avg = 0.f;
  auto s = (int)nbr.size();
  for (int i = 0; i < s; ++i) {
    avg += nbr[i].length;
  }

  return (s != 0) ? avg / s : avg;
}
// get the basis of the tangent space of a vertex
vec3f polar_basis(const geodesic_solver &solver, const vector<vec3f> &positions,
                  const vector<vec3f> &normals, int vid) {
  int vid0 = solver.graph[vid][0].node;
  vec3f v = positions[vid0] - positions[vid];
  vec3f e = project_vec(v, normals[vid]);
  return normalize(e);
}
vec3f polar_basis(const vector<vec3i> &triangles,
                  const vector<vec3f> &positions, int tid) {
  auto c = tid_centroid(triangles, positions, tid);
  vec3f v = positions[triangles[tid].x];
  return normalize(v - c);
}
// Compute polar coordinates
float angle_in_tangent_space(const geodesic_solver &solver,
                             const vector<vec3f> &positions, const vec3f &v,
                             const int vid, const vec3f &n) {
  float teta;
  int vid0 = solver.graph[vid][0].node;
  vec3f e0 = positions[vid0] - positions[vid];
  vec3f e = normalize(project_vec(e0, n));

  teta = angle(v, e);
  if (dot(cross(e, v), n) < 0)
    teta *= -1;

  return teta;
}
float angle_in_tangent_space(const vector<vec3i> &triangles,
                             const vector<vec3f> &positions,
                             const vector<vector<int>> &v2t, const vec3f &v,
                             const int vid, const vec3f &n) {
  float teta;
  auto tid = v2t[vid][0];
  auto k = find(triangles[tid], vid);
  int vid0 = triangles[tid][(k + 1) % 3];
  vec3f e0 = positions[vid0] - positions[vid];
  vec3f e = normalize(project_vec(e0, n));

  teta = angle(v, e);
  if (dot(cross(e, v), n) < 0)
    teta *= -1;

  return teta;
}

float angle_in_tangent_space(const vector<vec3i> &triangles,
                             const vector<vec3f> &positions, const int tid,
                             const vec3f &v) {
  auto teta = 0.f;

  auto e = polar_basis(triangles, positions, tid);
  auto n = tid_normal(triangles, positions, tid);
  teta = angle(v, e);
  if (dot(cross(e, v), n) < 0)
    teta = 2 * M_PI - teta;

  return teta;
}
inline int node_is_adjacent(const geodesic_solver &solver, int vid, int node) {
  auto nbr = solver.graph[vid];
  for (auto i = 0; i < nbr.size(); ++i) {
    if (nbr[i].node == node) {
      return i;
    }
  }
  return -1;
}
float angle_in_tangent_space(const geodesic_solver &solver,
                             const vector<vector<float>> &angles,
                             const vector<float> &total_angles,
                             const vector<vec3i> &triangles,
                             const vector<vec3f> &positions, const int tid,
                             const int vid, const float &lerp) {
  auto tr = triangles[tid];
  auto k = find(tr, vid);
  if (k == -1)
    std::cerr << "the vertex does not belong to the triangle" << std::endl;

  auto v0 = positions[tr[(k + 1) % 3]], v1 = positions[tr[(k + 2) % 3]];
  auto pos = (1 - lerp) * v0 + lerp * v1;
  auto [is_in_tri, b2f] = point_in_triangle(triangles, positions, tid, pos);
  if (!is_in_tri)
    std::cerr << "the point is not in the triangle" << std::endl;
  auto entry = node_is_adjacent(solver, vid, tr[(k + 1) % 3]);
  if (entry == -1)
    std::cerr << "the vertex should be a neighbor of vid" << std::endl;
  auto scale_factor = 2 * pif / total_angles[vid];
  auto theta0 = angles[vid][entry];
  auto theta = angle(pos - positions[vid], v0 - positions[vid]) * scale_factor;
  return theta + theta0;
}

bool checking_bary(const vec3f &bary, int &entry) {
  int count = 0;
  entry = -1;
  float treshold = flt_max;
  for (int i = 0; i < 3; ++i) {
    if (bary[i] < 0) {
      ++count;
      if (yocto::abs(bary[i]) < treshold) {
        entry = i;
        treshold = yocto::abs(bary[i]);
      }
    }
  }
  if (count > 1)
    return false;
  else if (yocto::abs(bary[entry] <= 1e-1))
    return true;

  return false;
}
inline bool are_barycentric_coordinates(const vec3f &bary,
                                        const float tol = 1e-2) {
  if (bary.x >= -tol && bary.x <= 1 + tol && bary.y >= -tol &&
      bary.y <= 1 + tol && bary.z >= -tol && bary.x <= 1 + tol)
    return true;
  return false;
}

vec3f tri_bary_coords(const vec3f &v0, const vec3f &v1, const vec3f &v2,
                      const vec3f &p) {
  vec3f wgts = vec3f{0.0, 0.0, 0.0};
  vec3f u = v1 - v0, v = v2 - v0, w = p - v0;
  float d00 = dot(u, u), d01 = dot(u, v), d11 = dot(v, v), d20 = dot(w, u),
        d21 = dot(w, v), d = d00 * d11 - d01 * d01;

  if (d == 0)
    return zero3f;

  wgts[2] = (d00 * d21 - d01 * d20) / d;
  assert(!isnan(wgts[2]));
  wgts[1] = (d11 * d20 - d01 * d21) / d;
  assert(!isnan(wgts[1]));
  wgts[0] = 1.0 - wgts[1] - wgts[2];
  assert(!isnan(wgts[0]));

  return wgts;
}

pair<bool, vec2f> point_in_unfold_triangle(const vec2f &pos,
                                           const unfold_triangle &tr,
                                           float tol) {
  // http://www.r-5.org/files/books/computers/algo-list/realtime-3d/Christer_Ericson-Real-Time_Collision_Detection-EN.pdf
  // pag.48
  auto bary = vec3f{0, 0, 0};
  auto v0 = tr[0];
  auto v1 = tr[1];
  auto v2 = tr[2];

  auto u = v1 - v0, v = v2 - v0, w = pos - v0;
  auto d00 = dot(u, u), d01 = dot(u, v), d11 = dot(v, v), d20 = dot(w, u),
       d21 = dot(w, v), d = d00 * d11 - d01 * d01;

  if (d == 0)
    return {false, zero2f};

  bary[2] = (d00 * d21 - d01 * d20) / d;
  assert(!isnan(bary[2]));
  bary[1] = (d11 * d20 - d01 * d21) / d;
  assert(!isnan(bary[1]));
  bary[0] = 1.0 - bary[1] - bary[2];
  assert(!isnan(bary[0]));

  for (auto i = 0; i < 3; ++i) {
    if (bary[i] < -tol || bary[i] > 1.0 + tol)
      return {false, zero2f};
  }
  auto uv = vec2f{bary.y, bary.z};
  uv = clamp(uv, 0.f, 1.f);
  return {true, uv};
}

// returns the index of the triangles in the star of from which v is pointing
// to
int next_tid(const geodesic_solver &solver, const vector<vector<float>> &angles,
             const vector<vec3f> &positions, const vector<vector<int>> &v2t,
             const vector<vec3i> &triangles, const vector<vec3f> &normals,
             const int from, const vec3f &v) {
  auto teta = angle_in_tangent_space(solver, positions, v, from, normals[from]);
  if (teta < 0)
    teta += 2 * M_PI;
  auto nbr = angles[from];
  int s = nbr.size();
  if (teta == 0)
    return v2t[from][0];
  for (int i = 0; i < s; ++i) {
    if (nbr[i] < teta)
      continue;

    if (i % 2 == 0) {
      return v2t[from][(i - 2) / 2];
    } else {
      return v2t[from][(i - 1) / 2];
    }
  }
  return v2t[from].back();
}
// https://rootllama.wordpress.com/2014/06/20/ray-line-segment-intersection-test-in-2d/
float intersect_line_segments(const vec2f &start1, const vec2f &end1,
                              const vec2f &start2, const vec2f &end2) {
  if (end1 == start2)
    return 0;
  if (end2 == start1)
    return 1;
  if (start2 == start1)
    return 0;
  if (end2 == end1)
    return 1;
  auto d = end1 - start1;
  auto v1 = start1 - start2;
  auto v2 = end2 - start2;
  auto v3 = vec2f{-d.y, d.x};
  auto det = dot(v2, v3);
  if (det == 0) // lines are parallel
    return -1;
  return dot(v1, v3) / det;
}
float intersect_line_segments(const vector<vec3i> &triangles,
                              const vector<vec3f> &positions, const int tid,
                              mesh_point &start1, mesh_point &end1,
                              mesh_point &start2, mesh_point &end2) {
  auto flat_tid = init_flat_triangle(positions, triangles[tid]);
  if (start1.face != tid) {
    auto [inside, bary] = point_in_triangle(
        triangles, positions, tid, eval_position(triangles, positions, start1));
    if (!inside) {
      std::cerr << "This point should be inside the triangle" << std::endl;
      return -1;
    }
    start1 = {tid, bary};
  }
  if (end1.face != tid) {
    auto [inside, bary] = point_in_triangle(
        triangles, positions, tid, eval_position(triangles, positions, end1));
    if (!inside) {
      std::cerr << "This point should be inside the triangle" << std::endl;
      return -1;
    }
    end1 = {tid, bary};
  }
  if (start2.face != tid) {
    auto [inside, bary] = point_in_triangle(
        triangles, positions, tid, eval_position(triangles, positions, start2));
    if (!inside) {
      std::cerr << "This point should be inside the triangle" << std::endl;
      return -1;
    }
    start2 = {tid, bary};
  }
  if (end2.face != tid) {
    auto [inside, bary] = point_in_triangle(
        triangles, positions, tid, eval_position(triangles, positions, end2));
    if (!inside) {
      std::cerr << "This point should be inside the triangle" << std::endl;
      return -1;
    }
    end2 = {tid, bary};
  }
  auto s1 =
      interpolate_triangle(flat_tid[0], flat_tid[1], flat_tid[2], start1.uv);
  auto s2 =
      interpolate_triangle(flat_tid[0], flat_tid[1], flat_tid[2], start2.uv);
  auto e1 =
      interpolate_triangle(flat_tid[0], flat_tid[1], flat_tid[2], end1.uv);
  auto e2 =
      interpolate_triangle(flat_tid[0], flat_tid[1], flat_tid[2], end2.uv);
  return intersect_line_segments(s1, e1, s2, e2);
}
// get the k-ring of vid
// note: we consider the connectivity of the graph, to get the usual k- ring
// uncomment the line below
vector<int> k_ring(const geodesic_solver &solver, const int vid, const int k,
                   bool mesh_connectivity = true) {
  vector<int> ring;
  vector<int> active_set = {vid};
  for (int i = 0; i < k; ++i) {
    vector<int> next_active_set;
    for (int j = 0; j < active_set.size(); ++j) {
      auto nbr = solver.graph[active_set[j]];
      for (int h = 0; h < nbr.size(); ++h) {
        if (h % 2 && mesh_connectivity)
          continue;
        int curr = nbr[h].node;
        if (find(ring, curr) == -1 && curr != vid) {
          next_active_set.push_back(curr);
          ring.push_back(curr);
        }
      }
    }
    active_set = next_active_set;
  }

  return ring;
}
vector<int> one_ring(const vector<vec3i> &triangles,
                     const vector<vector<int>> &v2t, const int vid,
                     const bool extended = false) {
  auto ring = vector<int>{};
  for (auto tid : v2t[vid]) {
    auto k = find(triangles[tid], vid);
    ring.push_back(triangles[tid][(k + 1) % 3]);
    if (extended) {
      auto eid =
          vec2i{triangles[tid][(k + 1) % 3], triangles[tid][(k + 2) % 3]};
      ring.push_back(opposite_vertex(triangles[tid], eid));
    }
  }

  return ring;
}
vector<int> k_ring(const vector<vec3i> &triangles,
                   const vector<vector<int>> &v2t, const int vid, const int k) {
  vector<int> ring;
  vector<int> active_set = {vid};
  for (int i = 0; i < k; ++i) {
    vector<int> next_active_set;
    for (int j = 0; j < active_set.size(); ++j) {
      auto nbr = one_ring(triangles, v2t, active_set[j]);
      for (int h = 0; h < nbr.size(); ++h) {
        int curr = nbr[h];
        if (find(ring, curr) == -1 && curr != vid) {
          next_active_set.push_back(curr);
          ring.push_back(curr);
        }
      }
    }
    active_set = next_active_set;
  }

  return ring;
}
vec3f trace_segment_vert(const vector<vec3f> &verts, const vec3f n,
                         const vec3f bary, const vec3f baryV, const vec3f &dir,
                         const int offset) {
  auto right = verts[(offset + 1) % 3] - verts[offset];
  auto left = verts[(offset + 2) % 3] - verts[offset];
  auto sample_bary = zero3f;
  if (dot(cross(right, dir), n) > 0) {
    if (dot(cross(dir, left), n) > 0) {
      auto factor = bary[offset] / baryV[offset];
      sample_bary[offset] = 0;
      sample_bary[(offset + 1) % 3] =
          bary[(offset + 1) % 3] - baryV[(offset + 1) % 3] * factor;
      sample_bary[(offset + 2) % 3] =
          bary[(offset + 2) % 3] - baryV[(offset + 2) % 3] * factor;
    } else
      sample_bary[(offset + 2) % 3] = 1;
  } else
    sample_bary[(offset + 1) % 3] = 1;

  return sample_bary;
}

vec3f trace_segment_edge(const vector<vec3f> &verts, const vec3f n,
                         const vec3f bary, const vec3f baryV, const vec3f &dir,
                         const int offset, const vec3f &sample_coords) {
  auto sample_bary = zero3f;
  auto right = verts[(offset + 1) % 3] - sample_coords;
  auto left = verts[offset] - sample_coords;
  auto front = verts[(offset + 2) % 3] - sample_coords;
  if (dot(cross(right, dir), n) > 0) {
    if (dot(cross(dir, front), n) > 0) {
      auto factor = bary[offset] / baryV[offset];
      sample_bary[offset] = 0;
      sample_bary[(offset + 1) % 3] =
          bary[(offset + 1) % 3] - baryV[(offset + 1) % 3] * factor;
      sample_bary[(offset + 2) % 3] =
          bary[(offset + 2) % 3] - baryV[(offset + 2) % 3] * factor;

    } else {
      if (dot(cross(dir, left), n) > 0) {
        auto factor = bary[(offset + 1) % 3] / baryV[(offset + 1) % 3];
        sample_bary[(offset + 1) % 3] = 0;
        sample_bary[offset] = bary[offset] - baryV[offset] * factor;
        sample_bary[(offset + 2) % 3] =
            bary[(offset + 2) % 3] - baryV[(offset + 2) % 3] * factor;

      } else {
        if (dot(left, dir) > 0) {
          sample_bary[(offset)] = 1;
        } else {
          sample_bary[(offset + 1) % 3] = 1;
        }
      }
    }
  } else {
    if (dot(right, dir) > 0) {
      sample_bary[(offset + 1) % 3] = 1;
    } else {
      sample_bary[(offset)] = 1;
    }
  }

  return sample_bary;
}
vec3f trace_segment_tri(const vector<vec3f> &verts, const vec3f n,
                        const vec3f bary, const vec3f baryV, const vec3f &dir,
                        const vec3f &sample_coords) {
  auto sample_bary = zero3f;
  vec3f w0 = verts[0] - sample_coords;
  vec3f w1 = verts[1] - sample_coords;
  vec3f w2 = verts[2] - sample_coords;
  if (dot(cross(w0, dir), n) > 0 && dot(cross(dir, w1), n) > 0) {
    sample_bary = vec3f{bary[0] - bary[2] * baryV[0] / baryV[2],
                        bary[1] - bary[2] * baryV[1] / baryV[2], 0};

  } else if (dot(cross(w1, dir), n) > 0 && dot(cross(dir, w2), n) > 0) {
    sample_bary = vec3f{0, bary[1] - bary[0] * baryV[1] / baryV[0],
                        bary[2] - bary[0] * baryV[2] / baryV[0]};
  } else {
    sample_bary = vec3f{bary[0] - bary[1] * baryV[0] / baryV[1], 0,
                        bary[2] - bary[1] * baryV[2] / baryV[1]};
  }

  return sample_bary;
}
// Identify the intersection of the polyline inside triangle pid
// tracing: https://cims.nyu.edu/gcl/papers/campen2016bms.pdf
void trace_in_triangles(const vector<vec3f> &positions,
                        const vector<vec3i> &triangles, const vec3f &dir,
                        const vec3f &bary, const int pid, vec3f &sample_pos,
                        vec3f &sample_bary) {
  vec3f baryM = zero3f, baryV = zero3f;
  vec3f v0 = positions[triangles[pid].x];
  vec3f v1 = positions[triangles[pid].y];
  vec3f v2 = positions[triangles[pid].z];
  vector<vec3f> verts = {v0, v1, v2};
  vec3f n = triangle_normal(v0, v1, v2);
  vec3f sample_coords = bary.x * v0 + bary.y * v1 + bary.z * v2;
  vec3f M = sample_coords + dir;

  baryM = tri_bary_coords(v0, v1, v2, M);
  for (int i = 0; i < 3; ++i) {
    baryV[i] = baryM[i] - bary[i];
  }
  auto [is_vertex, k_vert] = bary_is_vert(bary);
  auto [is_on_edge, k_edge] = bary_is_edge(bary);
  if (is_vertex) {
    sample_bary = trace_segment_vert(verts, n, bary, baryV, dir, k_vert);
    sample_pos = sample_bary.x * verts[0] + sample_bary.y * verts[1] +
                 sample_bary.z * verts[2];
  } else if (is_on_edge) {
    sample_bary =
        trace_segment_edge(verts, n, bary, baryV, dir, k_edge, sample_coords);
    sample_pos = sample_bary.x * verts[0] + sample_bary.y * verts[1] +
                 sample_bary.z * verts[2];
  } else {
    sample_bary = trace_segment_tri(verts, n, bary, baryV, dir, sample_coords);
    sample_pos = sample_bary.x * verts[0] + sample_bary.y * verts[1] +
                 sample_bary.z * verts[2];
  }
}

float rescaled_angle(const geodesic_solver &solver,
                     const vector<vector<float>> &angles,
                     const vector<vec3i> &triangles,
                     const vector<vec3f> &positions,
                     const vector<vec3i> &adjacencies, const int tid,
                     const int vid, const float &lerp) {
  auto tr = triangles[tid];
  auto k = find(tr, vid);
  assert(k != -1);
  auto v0 = positions[tr[(k + 1) % 3]], v1 = positions[tr[(k + 2) % 3]];
  auto pos = (1 - lerp) * v0 + lerp * v1;
  auto v = pos - positions[vid];
  auto w = v0 - positions[vid];
  auto u = v1 - positions[vid];
  auto theta = angle(v, w);
  auto phi3D = angle(w, u);
  auto s = angles[vid].size();
  auto entry = node_is_adjacent(solver, vid, tr[(k + 1) % 3]);
  auto theta0 = angles[vid][entry];
  auto phi2D =
      (entry + 2 == s) ? 2 * pif - theta0 : angles[vid][entry + 2] - theta0;
  auto scale_factor = phi3D / phi2D;

  return theta0 + theta * scale_factor;
}
bool useless_arc(const geodesic_path &arc) {
  for (auto i = 1; i < arc.lerps.size() - 1; ++i) {
    if (1 - arc.lerps[i] < 1e-2 || arc.lerps[i] < 1e-2)
      return true;
  }
  return false;
}

// parallel transport from the tangent space of "from" to the tangent space of
// "to"
void parallel_transp(
    const geodesic_solver &solver, const vector<vector<float>> &angles,
    const vector<float> &total_angles, const vector<vec3i> &triangles,
    const vector<vec3f> &positions, const vector<vec3i> &adjacencies,
    const vector<vector<int>> &v2t, vec3f &v, const vector<vec3f> &normals,
    const int &from, const int &to, const int &mode) {
  switch (mode) {
  case V2V: {
    float teta =
        angle_in_tangent_space(solver, positions, v, from, normals[from]);
    auto nbr_from = one_ring(triangles, v2t, from, true);
    auto nbr_to = one_ring(triangles, v2t, to, true);
    float phi_ij = -1;
    float phi_ji = -1;

    for (int i = 0; i < nbr_from.size(); ++i) {
      if (nbr_from[i] == to) {
        phi_ij = angles[from][i];

        break;
      }
    }

    for (int j = 0; j < nbr_to.size(); ++j) {
      if (nbr_to[j] == from) {
        phi_ji = angles[to][j];
        break;
      }
    }
    assert(phi_ij != -1);
    assert(phi_ji != -1);

    vec3f e0 = polar_basis(solver, positions, normals, to);
    float rotation = teta + phi_ji + M_PI - phi_ij;

    v = rot_vect(e0, normals[to], rotation);

  } break;

  case V2T: {
    float teta =
        angle_in_tangent_space(solver, positions, v, from, normals[from]);
    vec3i tri = triangles[to];
    vec3f p0 = positions[tri.x];
    vec3f p1 = positions[tri.y];
    vec3f p2 = positions[tri.z];
    vec3f normal = triangle_normal(p0, p1, p2);
    vec3f centroid = (p0 + p1 + p2) / 3.0;
    vec3f e = normalize(p0 - centroid);

    vec3f coords = positions[from] - centroid;
    float phi_ji = angle(e, coords);
    if (dot(cross(e, coords), normal) < 0)
      phi_ji = 2 * M_PI - phi_ji;
    int offset = find(tri, from);
    assert(offset != -1);
    int vid1 = tri[(offset + 1) % 3];
    int vid2 = tri[(offset + 2) % 3];
    float factor = 2 * M_PI / total_angles[from];
    auto nbr_from = one_ring(triangles, v2t, from, true);
    float phi_ij = -1;
    coords *= -1;
    if (nbr_from[0] == vid2) {
      vec3f edge = positions[vid2] - positions[from];
      float curr_angle = angle(edge, coords);
      curr_angle *= factor;
      curr_angle = 2 * M_PI - curr_angle;
      phi_ij = curr_angle;
    } else {
      for (int i = 0; i < nbr_from.size(); ++i) {
        if (nbr_from[i] == vid1) {
          phi_ij = angles[from][i];
          break;
        }
      }

      vec3f edge = positions[vid1] - positions[from];
      float curr_angle = angle(edge, coords);
      curr_angle *= factor;
      phi_ij += curr_angle;
    }

    float rot = teta + phi_ji + M_PI - phi_ij;

    e *= length(v);
    v = rot_vect(e, normal, rot);

  }

  break;

  case T2V: {
    vec3i tri = triangles[from];
    vec3f p0 = positions[tri.x];
    vec3f p1 = positions[tri.y];
    vec3f p2 = positions[tri.z];
    vec3f n = triangle_normal(p0, p1, p2);
    vec3f centroid = (p0 + p1 + p2) / 3.0;
    vec3f e = normalize(p0 - centroid);
    float teta = angle(e, v);

    if (dot(cross(e, v), n) < 0)
      teta = 2 * M_PI - teta;
    int offset = find(tri, to);
    assert(offset != -1);
    int vid1 = tri[(offset + 1) % 3];

    vec3f vert = positions[tri[offset]];
    vec3f v1 = positions[vid1] - vert;

    vec3f coords = vert - centroid;
    float phi_ij = angle(e, coords);
    if (dot(cross(e, coords), n) < 0)
      phi_ij = 2 * M_PI - phi_ij;

    coords *= -1;
    float phi_ji = angle(v1, coords);
    float factor = 2 * M_PI / total_angles[to];
    phi_ji *= factor;
    auto nbr = one_ring(triangles, v2t, to, true);
    for (int i = 0; i < nbr.size(); ++i) {
      if (nbr[i] == vid1) {
        float phi = angles[to][i];
        phi_ji += phi;
        break;
      }
    }

    float rot = teta + phi_ji + M_PI - phi_ij;
    vec3f e0 = polar_basis(solver, positions, normals, to);
    e0 *= length(v);
    v = rot_vect(e0, normals[to], rot);

  } break;

  case T2T: {
    auto flat_from = init_flat_triangle(positions, triangles[from]);
    auto k = find(adjacencies[from], to);
    assert(k != -1);
    auto flat_to =
        unfold_face(triangles, positions, adjacencies, flat_from, from, k);
    auto bary = vec2f{0.333, 0.333};
    auto c0 =
        interpolate_triangle(flat_from[0], flat_from[1], flat_from[2], bary);
    auto c1 = interpolate_triangle(flat_to[0], flat_to[1], flat_to[2], bary);
    auto e0 = flat_from[0] - c0;
    auto e1 = flat_to[0] - c1;

    auto w = c1 - c0;
    auto phi_ij = angle(e0, w);
    if (cross(e0, w) < 0)
      phi_ij = 2 * M_PI - phi_ij;
    w *= -1;
    auto phi_ji = angle(e1, w);
    if (cross(e1, w) < 0)
      phi_ji = 2 * M_PI - phi_ji;

    auto n = tid_normal(triangles, positions, from);
    auto e = polar_basis(triangles, positions, from);
    float teta = angle(e, v);
    if (dot(cross(e, v), n) < 0)
      teta = 2 * M_PI - teta;

    auto e_to = polar_basis(triangles, positions, to);
    auto n_to = tid_normal(triangles, positions, to);
    float rot = teta + phi_ji + M_PI - phi_ij;
    e_to *= length(v);
    v = rot_vect(e_to, n_to, rot);

  }

  break;
  }
}

vec3f transp_vec(const geodesic_solver &solver,
                 const vector<vector<float>> &angles,
                 const vector<float> &total_angles,
                 const vector<vec3i> &triangles, const vector<vec3f> &positions,
                 const vector<vec3i> &adjacencies,
                 const vector<vector<int>> &v2t, const vec3f &v,
                 const vector<vec3f> &normals, const int &from, const int &to,
                 const int &mode) {
  vec3f w = v;
  parallel_transp(solver, angles, total_angles, triangles, positions,
                  adjacencies, v2t, w, normals, from, to, mode);
  return w;
}
vector<int> strip_to_point(const geodesic_solver &solver,
                           const vector<vec3i> &triangles,
                           const vector<vec3f> &positions,
                           const vector<vec3i> &adjacencies,
                           const vector<vector<int>> &v2t, int parent,
                           const mesh_point &p) {
  vector<int> strip = {p.face};
  auto [is_vert, k] = bary_is_vert(get_bary(p.uv));
  if (is_vert) {
    auto vid = triangles[p.face][k];
    auto entry = node_is_adjacent(solver, vid, parent);
    assert(entry >= 0);
    if (entry % 2) {
      auto first = (entry - 1) / 2;
      auto tid = opposite_face(triangles, adjacencies, v2t[vid][first], vid);
      strip.push_back(tid);
    }
    return strip;

  } else {
    auto h = find(triangles[p.face], parent);

    if (h == -1) {
      for (auto i = 0; i < 3; ++i) {
        auto adj = adjacencies[p.face][i];
        h = find(triangles[adj], parent);
        if (h != -1) {
          strip.push_back(adj);
          return strip;
        }
      }
    }
  }
  return strip;
}
// parallel transport a vector from the tangen space of "from" to the tangent
// space of "to"
vec3f tranport_vector(const geodesic_solver &solver,
                      const vector<vector<float>> &angles,
                      const vector<float> &total_angles,
                      const vector<vec3i> &triangles,
                      const vector<vec3f> &positions,
                      const vector<vec3i> &adjacencies,
                      const vector<vector<int>> &v2t, const vec3f &v,
                      const vector<vec3f> &normals, const mesh_point &from,
                      const mesh_point &to) {
  auto dir = v;
  auto parents = point_to_point_geodesic_path(solver, triangles, positions,
                                              adjacencies, to, from);
  // handle degenerate cases
  if (parents.size() == 0) {
    auto [is_vert, k] = bary_is_vert(get_bary(from.uv));
    if (is_vert) {
      auto vid = triangles[from.face][k];
      // parallel_transp(solver, angles, total_angles, triangles, positions,
      //                 adjacencies, dir, normals, from.face, vid, T2V);
      auto strip = strip_to_point(solver, triangles, positions, adjacencies,
                                  v2t, vid, to);
      parallel_transp(solver, angles, total_angles, triangles, positions,
                      adjacencies, v2t, dir, normals, vid, strip.back(), V2T);
      if (strip.size())
        return dir;
      else {
        parallel_transp(solver, angles, total_angles, triangles, positions,
                        adjacencies, v2t, dir, normals, strip.back(), strip[0],
                        T2T);
        return dir;
      }
    }
  }
  // bring dir in the tangent space of parents[0]
  auto [from_is_vert, kf] = bary_is_vert(get_bary(from.uv));
  if (from_is_vert) {
    // parallel_transp(solver, angles, total_angles, triangles, positions,
    //                 adjacencies, dir, normals, from.face,
    //                 triangles[from.face][kf], T2V);
    parallel_transp(solver, angles, total_angles, triangles, positions,
                    adjacencies, v2t, dir, normals, triangles[from.face][kf],
                    parents[0], V2V);
  } else {
    auto strip = strip_to_point(solver, triangles, positions, adjacencies, v2t,
                                parents[0], from);
    for (int i = 0; i < strip.size() - 1; ++i) {
      parallel_transp(solver, angles, total_angles, triangles, positions,
                      adjacencies, v2t, dir, normals, strip[i], strip[i + 1],
                      T2T);
    }
    parallel_transp(solver, angles, total_angles, triangles, positions,
                    adjacencies, v2t, dir, normals, strip.back(), parents[0],
                    T2V);
  }

  for (int i = 0; i < parents.size() - 1; ++i) {
    parallel_transp(solver, angles, total_angles, triangles, positions,
                    adjacencies, v2t, dir, normals, parents[i], parents[i + 1],
                    V2V);
  }
  auto [to_is_vert, kt] = bary_is_vert(get_bary(to.uv));
  if (to_is_vert) {
    parallel_transp(solver, angles, total_angles, triangles, positions,
                    adjacencies, v2t, dir, normals, to.face,
                    triangles[to.face][kt], T2V);
    parallel_transp(solver, angles, total_angles, triangles, positions,
                    adjacencies, v2t, dir, normals, triangles[to.face][kt],
                    parents.back(), V2V);
    return dir;
  } else {
    auto strip = strip_to_point(solver, triangles, positions, adjacencies, v2t,
                                parents.back(), to);
    parallel_transp(solver, angles, total_angles, triangles, positions,
                    adjacencies, v2t, dir, normals, parents.back(),
                    strip.back(), V2T);
    if (strip.size())
      return dir;
    else {
      parallel_transp(solver, angles, total_angles, triangles, positions,
                      adjacencies, v2t, dir, normals, strip.back(), strip[0],
                      T2T);
      return dir;
    }
  }
}

// utility (gradient matrix)
Eigen::VectorXd eigen_wrapper(const vector<float> &f) {
  Eigen::VectorXd F(f.size());
  for (int i = 0; i < f.size(); ++i) {
    F(i) = f[i];
  }
  return F;
}
Eigen::MatrixXd rhs(int s) {
  Eigen::MatrixXd E(s, s + 1);
  Eigen::MatrixXd X = Eigen::MatrixXd::Constant(s, 1, -1);
  E.topLeftCorner(s, 1) = X;
  Eigen::MatrixXd I(s, s);
  I.setIdentity();
  E.topRightCorner(s, s) = I;
  return E;
}
void fill_riemannian_gradient_entries(vector<Eigen::Triplet<double>> &entries,
                                      const vector<int> &ring,
                                      const Eigen::VectorXd &c,
                                      const Eigen::VectorXd &a0,
                                      const Eigen::VectorXd &a1, const int n) {
  int vid = ring[0];
  int s = ring.size();
  double c0_squared = pow(c[0], 2);
  double c1_squared = pow(c[1], 2);
  Eigen::Matrix2d g_inv;
  double det = 1 + c0_squared + c1_squared;
  g_inv << 1 + c1_squared, -c[0] * c[1], -c[0] * c[1], 1 + c0_squared;
  g_inv /= det;
  typedef Eigen::Triplet<double> T;
  for (int i = 0; i < s; ++i) {
    int entry = ring[i];
    entries.push_back(T(vid, entry, g_inv(0, 0) * a0(i) + g_inv(0, 1) * a1(i)));
    entries.push_back(
        T(n + vid, entry, g_inv(1, 0) * a0(i) + g_inv(1, 1) * a1(i)));
  }
}
float max_nbr_edge_length(const vector<vec3i> &triangles,
                          const vector<vec3f> &positions,
                          const vector<vector<int>> &v2t, int vid) {
  auto nbr = vector<float>{};
  auto vert = positions[vid];
  for (auto tid : v2t[vid]) {
    auto k = find(triangles[tid], vid);
    nbr.push_back(length(positions[triangles[tid][(k + 1) % 3]] - vert));
  }
  auto lambda = flt_min;
  auto s = (int)nbr.size();
  for (int i = 0; i < s; ++i) {
    lambda = std::max(lambda, nbr[i]);
  }

  return lambda;
}
vector<float> subdivide_angles(const int number_of_subdivision) {
  auto tetas = vector<float>(number_of_subdivision);
  auto step = 2 * M_PI / number_of_subdivision;
  auto phi = 0.f;
  for (auto i = 0; i < number_of_subdivision; ++i) {
    tetas[i] = phi;
    phi += step;
  }
  return tetas;
}
std::tuple<vector<mesh_point>, float, vector<float>> uniform_stencil(
    const vector<vec3i> &triangles, const vector<vec3f> &positions,
    const geodesic_solver &solver, const vector<vec3f> &normals,
    const vector<vec3i> &adjacencies, const vector<vector<int>> &v2t,
    const vector<vector<float>> &angles, const vector<float> &total_angles,
    const int vid, const int number_of_samples) {
  vector<vec2f> directions(number_of_samples);
  vector<int> tris(number_of_samples);
  vector<mesh_point> samples(number_of_samples);
  // vector<mesh_point> start(number_of_samples);
  // auto len = nbr_avg_edge_length(solver, vid);
  auto len = max_nbr_edge_length(triangles, positions, v2t, vid);
  auto tetas = subdivide_angles(number_of_samples);
  auto start = point_from_vert(triangles, v2t, vid);
  auto e = polar_basis(solver, positions, normals, vid);
  // auto e = polar_basis(triangles, v2t, positions, normals, vid);
  for (auto i = 0; i < number_of_samples; ++i) {
    // auto path = straightest_path(triangles, positions, adjacencies, start[i],
    //                              directions[i], len);
    auto curr_dir = rot_vect(e, normals[vid], tetas[i]);
    auto path =
        straightest_geodesic(solver, triangles, positions, normals, adjacencies,
                             v2t, angles, total_angles, start, curr_dir, len);
    samples[i] = path.back();
  }
  return {samples, len, tetas};
}
void laplacian_entries(vector<Eigen::Triplet<double>> &entries,
                       const Eigen::Matrix2d &g, const Eigen::Matrix2d &g_inv,
                       const vector<vector<pair<int, float>>> &ring,
                       const Eigen::VectorXd &c, const Eigen::MatrixXd &a) {
  int vid = ring[0][0].first;
  typedef Eigen::Triplet<double> T;
  Eigen::VectorXd b;
  double c0_squared = pow(c[0], 2);
  double c1_squared = pow(c[1], 2);

  double det = 1 + c0_squared + c1_squared;

  double g11u = 2 * c[0] * c[2], g12u = c[0] * c[3] + c[1] * c[2],
         g22_u = 2 * c[1] * c[3], g11_v = 2 * c[0] * c[3],
         g12_v = c[0] * c[4] + c[1] * c[3], g22_v = 2 * c[1] * c[4],
         g_u = g11u + g22_u, g_v = g11_v + g22_v,
         g_invu11 = (det * g22_u - g(1, 1) * g_u) / pow(det, 2),
         g_invu12 = -(det * g12u - g(0, 1) * g_u) / pow(det, 2),
         g_invv21 = -(det * g12_v - g(0, 1) * g_v) / pow(det, 2),
         g_invv12 = g_invv21,
         g_invv22 = (det * g11_v - g(0, 0) * g_v) / pow(det, 2);

  double coeff0 = (g_u * g_inv(0, 0) + g_v * g_inv(1, 0)) / (2 * det) +
                  g_invu11 + g_invv21,
         coeff1 = (g_u * g_inv(0, 1) + g_v * g_inv(1, 1)) / (2 * det) +
                  g_invu12 + g_invv22,
         coeff2 = g_inv(0, 0), coeff3 = 2 * g_inv(0, 1), coeff4 = g_inv(1, 1);

  b = coeff0 * a.row(0) + coeff1 * a.row(1) + coeff2 * a.row(2) +
      coeff3 * a.row(3) + coeff4 * a.row(4);
  for (int i = 0; i < b.size(); ++i) {
    for (auto j = 0; j < ring[i].size(); ++j) {
      int entry = ring[i][j].first;

      entries.push_back(T(vid, entry, b(i) * ring[i][j].second));
    }
  }
}
void fill_riemannian_gradient_entries(
    vector<Eigen::Triplet<double>> &entries,
    const vector<vector<pair<int, float>>> &ring, const Eigen::VectorXd &c,
    const Eigen::VectorXd &a0, const Eigen::VectorXd &a1, const int n) {
  int vid = ring[0][0].first;
  int s = (int)ring.size();
  double c0_squared = pow(c[0], 2);
  double c1_squared = pow(c[1], 2);
  Eigen::Matrix2d g_inv;
  double det = 1 + c0_squared + c1_squared;
  g_inv << 1 + c1_squared, -c[0] * c[1], -c[0] * c[1], 1 + c0_squared;
  g_inv /= det;
  typedef Eigen::Triplet<double> T;
  for (int i = 0; i < s; ++i) {
    for (auto j = 0; j < ring[i].size(); ++j) {
      int entry = ring[i][j].first;
      auto w = ring[i][j].second;
      entries.push_back(
          T(vid, entry, w * (g_inv(0, 0) * a0(i) + g_inv(0, 1) * a1(i))));
      entries.push_back(
          T(n + vid, entry, w * (g_inv(1, 0) * a0(i) + g_inv(1, 1) * a1(i))));
    }
  }
}
// we follow the order 00 01 10 11
vector<vec2f> Christoffel_symbol(const Eigen::Matrix2d &g_inv,
                                 const Eigen::VectorXd &c) {
  vector<vec2f> Christoffel(4);
  double g11u = 2 * c[0] * c[2], g12u = c[0] * c[3] + c[1] * c[2], g21u = g12u,
         g22u = 2 * c[1] * c[3], g11v = 2 * c[0] * c[3],
         g12v = c[0] * c[4] + c[1] * c[3], g21v = g12v, g22v = 2 * c[1] * c[4];

  Christoffel[0].x = 0.5 * (g_inv(0, 0) * g11u - g_inv(0, 1) * g11v);
  Christoffel[0].y =
      0.5 * (g_inv(1, 0) * g11u + g_inv(1, 1) * (2 * g12u - g11v));

  Christoffel[1].x = 0.5 * (g_inv(0, 0) * g11v + g_inv(0, 1) * g22u);
  Christoffel[1].y = 0.5 * (g_inv(1, 0) * g11v + g_inv(1, 1) * g22u);

  Christoffel[2] = Christoffel[1];

  Christoffel[3].x = 0.5 * (g_inv(0, 0) * g22u + g_inv(0, 1) * g22v);
  Christoffel[3].y =
      0.5 * (g_inv(1, 0) * (2 * g21v - g22u) + g_inv(1, 1) * g22v);

  return Christoffel;
}
// Gradient matrix
// Note: this construction allows the estimation of the gradient of a scalar
// field F through the matrix-dot-vector multiplication Grad*F, although such
// estimation it is not so accurate near sharp features.
// https://link.springer.com/content/pdf/10.1007/s40304-013-0018-2.pdf
std::tuple<Eigen::SparseMatrix<double, 1>, Eigen::SparseMatrix<double, 1>,
           vector<Eigen::MatrixXd>, vector<vector<vec2f>>,
           vector<Eigen::Matrix2d>, vector<vector<vector<pair<int, float>>>>,
           vector<pair<vec3f, vec3f>>, float>
init_discrete_diff_op_2(
    const geodesic_solver &solver, const vector<vector<float>> &angles,
    const vector<float> &total_angles, const vector<vec3i> &triangles,
    const vector<vec3f> &positions, const vector<vec3i> &adjacencies,
    const vector<vector<int>> &v2t, const vector<vec3f> &normals) {
  typedef Eigen::Triplet<double> T;
  vector<T> L_entries;
  vector<T> G_entries;
  int V = (int)positions.size();
  Eigen::SparseMatrix<double, 1> Grad;
  Eigen::SparseMatrix<double, 1> Lap;
  vector<Eigen::MatrixXd> A_coeff(V);
  vector<vector<vec2f>> Christoffel(V);
  vector<Eigen::Matrix2d> g_invs(V);
  vector<pair<vec3f, vec3f>> basis;
  vector<vector<vector<pair<int, float>>>> contributes_map(V);
  vector<float> gaussian_curv(positions.size());
  float min_curv = flt_min;
  int invertible = 0;

  for (int i = 0; i < V; ++i) {
    auto [nbr, len, tetas] =
        uniform_stencil(triangles, positions, solver, normals, adjacencies, v2t,
                        angles, total_angles, i, 36);
    // auto nbr = solver.graph[i];
    vec3f vert = positions[i];
    vec3f n = normals[i];
    int s = (int)nbr.size();
    Eigen::MatrixXd Q(s, 5);
    Eigen::VectorXd h(s);
    auto teta = 0.f;
    auto pos = zero2f;
    auto d = 0.f;
    auto coords = zero3f;
    vector<vector<pair<int, float>>> a_map(s + 1);
    a_map[0].push_back({i, 1});
    // // auto len = nbr_avg_edge_length(solver, i);

    for (int j = 0; j < s; ++j) {
      // int curr = nbr[j].node;

      // if (j % 2 == 1) {

      // auto tid = v2t[i][(j - 1) / 2];
      // auto lerp = 0.f;
      // auto endpoints = zero2i;
      // std::tie(lerp, endpoints, len) =
      //     flat_opposite_sample(triangles, positions, adjacencies, i, tid);
      // a_map[j + 1].push_back({endpoints.x, 1 - lerp});
      // a_map[j + 1].push_back({endpoints.y, lerp});
      // coords = positions[endpoints.x] * (1 - lerp) +
      //          positions[endpoints.y] * lerp - vert;
      //   auto bary_end = get_bary(samples[(j - 1) / 2].uv);
      //   for (auto h = 0; h < 3; ++h) {
      //     a_map[j + 1].push_back(
      //         {triangles[samples[(j - 1) / 2].face][h], bary_end[h]});
      //   }
      //   coords =
      //       eval_position(triangles, positions, samples[(j - 1) / 2]) -
      //       vert;
      //   auto teta_prev = angles[i][j - 1];
      //   auto teta_next = (j == s - 1) ? 2 * pif : angles[i][j + 1];
      //   auto curr_teta = (teta_prev + teta_next) / 2;
      //   pos = vec2f{len * yocto::cos(curr_teta), len *
      //   yocto::sin(curr_teta)};

      // } else {

      //   a_map[j + 1].push_back({nbr[j].node, 1});
      //   teta = angles[i][j];
      //   d = nbr[j].length;
      //   pos = vec2f{d * std::cos(teta), d * std::sin(teta)};
      //   coords = positions[curr] - vert;
      // }
      pos = vec2f{len * yocto::cos(tetas[j]), len * yocto::sin(tetas[j])};
      // pos = vec2f{nbr[j].length * yocto::cos(angles[i][j]),
      //             nbr[j].length * yocto::sin(angles[i][j])};
      // coords = positions[nbr[j].node] - vert;
      coords = eval_position(triangles, positions, nbr[j]) - vert;

      // auto point = point_from_vert(triangles, v2t, nbr[j].node);
      auto bary_end = get_bary(nbr[j].uv);
      for (auto h = 0; h < 3; ++h) {
        a_map[j + 1].push_back({triangles[nbr[j].face][h], bary_end[h]});
      }
      Q(j, 0) = pos[0];
      Q(j, 1) = pos[1];
      Q(j, 2) = pow(pos[0], 2) / 2;
      Q(j, 3) = pos[0] * pos[1];
      Q(j, 4) = pow(pos[1], 2) / 2;

      h(j) = dot(coords, n);
    }

    Eigen::MatrixXd Qt = Eigen::Transpose<Eigen::MatrixXd>(Q);

    Eigen::MatrixXd A = Qt * Q;
    Eigen::MatrixXd E = rhs(s);
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> dec(A);
    Eigen::VectorXd c(5);
    Eigen::MatrixXd a(5, s + 1);

    if (dec.isInvertible()) {
      Eigen::MatrixXd inv = A.inverse();
      c = inv * Qt * h;
      a = inv * Qt * E;
      ++invertible;

    } else {
      Eigen::MatrixXd Rhsc = Qt * h;
      Eigen::MatrixXd Rhsa = Qt * E;
      c = dec.solve(Rhsc);
      a = dec.solve(Rhsa);
    }
    // auto [c, a, a_map] =
    //     fitting_quadric(solver, angles, triangles, positions, adjacencies,
    //                     total_angles, v2t, normals, i, 36);
    // adapt_to_regular_stencil(a);
    A_coeff[i] = a;
    Eigen::Matrix2d g;
    Eigen::Matrix2d g_inv;

    double c0_squared = pow(c[0], 2);
    double c1_squared = pow(c[1], 2);
    g << 1 + c0_squared, c[0] * c[1], c[0] * c[1], 1 + c1_squared;

    double det = 1 + c0_squared + c1_squared;
    g_inv << 1 + c1_squared, -c[0] * c[1], -c[0] * c[1], 1 + c0_squared;
    g_inv /= det;
    // auto curv_det = 1 + pow(2 * c[0], 2) + pow(2 * c[1], 2);
    // max_curv =
    //     std::max(max_curv, (4*c[2] * c[4] - pow(2 * c[3], 2)) /
    //     pow(curv_det, 2));
    // max_curv = std::max(max_curv, compute_curvature(det, n, c));
    // min_curv = std::min(min_curv, (float)compute_curvature(det, n, c));
    laplacian_entries(L_entries, g, g_inv, a_map, c, a);
    // gaussian_curv[i] = compute_curvature(det, n, c);
    fill_riemannian_gradient_entries(G_entries, a_map, c, a.row(0), a.row(1),
                                     V);
    Christoffel[i] = Christoffel_symbol(g, c);
    contributes_map[i] = a_map;
    g_invs[i] = g_inv;
    // basis[i] = {vec3f{1, 0, (float)c[0]}, vec3f{0, 1, (float)c[1]}};
  }
  // auto perc = percentile(gaussian_curv);
  // for (auto i = 0; i < gaussian_curv.size(); ++i) {
  //   if (perc[i] < 15)
  //     continue;
  //   min_curv = gaussian_curv[i];
  //   break;
  // }

  Lap.resize(V, V);
  Grad.resize(2 * V, V);
  Lap.setFromTriplets(L_entries.begin(), L_entries.end());
  Grad.setFromTriplets(G_entries.begin(), G_entries.end());
  std::cout << invertible / V * 100 << std::endl;

  // log_info("invertible / V * 100: $", invertible / V * 100);
  return {Grad,  Lap,     A_coeff, Christoffel, g_invs, contributes_map,
          basis, min_curv};
}

vec3f polar_to_cartesian(const geodesic_solver &solver,
                         const vector<vec3f> &positions,
                         const vector<vec3f> &normals, const double x,
                         const double y, const int vid) {
  vec3f g = zero3f;
  vec2f sol = vec2f{(float)x, (float)y};
  double phi = yocto::atan2(y, x);

  float mag = length(sol);
  vec3f e = polar_basis(solver, positions, normals, vid);
  g = rot_vect(e, normals[vid], phi);
  g *= mag;

  return g;
}
vec3f assembling_gradient(const geodesic_solver &solver,
                          const vector<vec3f> &positions,
                          const vector<vec3f> &normals,
                          const float g00_contribute,
                          const float g01_contribute,
                          const float g10_contribute,
                          const float g11_contribute, const int vid) {
  vec3f g = zero3f;
  auto x = g00_contribute + g10_contribute;
  auto y = g10_contribute + g11_contribute;
  vec2f sol = vec2f{(float)x, (float)y};
  double phi = yocto::atan2(y, x);

  float mag = length(sol);
  vec3f e = polar_basis(solver, positions, normals, vid);
  g = rot_vect(e, normals[vid], phi);
  g *= mag;

  return g;
}
// Compute the gradient of a scalar field f
// Note: by default we consider -grad(f);
vector<vec3f> compute_gradient(const geodesic_solver &solver,
                               const vector<vec3f> &positions,
                               const vector<vec3f> &normals,
                               const Eigen::SparseMatrix<double> &G,
                               const Eigen::VectorXd &f, bool normalized) {
  int n;
  n = positions.size();
  vector<vec3f> g(n);

  Eigen::VectorXd Grad = G * f;

  for (int i = 0; i < n; ++i) {
    g[i] = -polar_to_cartesian(solver, positions, normals, Grad(i), Grad(n + i),
                               i);
    if (normalized)
      g[i] = normalize(g[i]);
  }
  return g;
}

int next_tid_extended_graph(const vector<float> &total_angles,
                            const vector<vec3f> &positions,
                            const vector<vector<int>> &v2t,
                            const vector<vec3i> &triangles,
                            const vector<vec3f> &normals, const int from,
                            const vec3f &v) {
  auto star = v2t[from];
  auto teta =
      angle_in_tangent_space(triangles, positions, v2t, v, from, normals[from]);
  auto curr_angle = 0.f;
  auto scale_factor = 2 * pif / total_angles[from];
  auto vert_pos = positions[from];
  if (teta == 0)
    return star[0];
  if (teta < 0)
    teta += 2 * pif;
  for (int i = 0; i < star.size(); ++i) {
    auto tid = star[i];
    auto offset = find(triangles[tid], from);
    auto vid0 = triangles[tid][(offset + 1) % 3];
    auto vid1 = triangles[tid][(offset + 2) % 3];
    curr_angle +=
        angle(positions[vid0] - vert_pos, positions[vid1] - vert_pos) *
        scale_factor;

    if (curr_angle > teta)
      return tid;
  }

  return star.back();
}

void p_transp_along_path(const bezier_mesh &mesh, const geodesic_path &path,
                         vec3f v) {

  if (path.strip.size() == 1 && path.start.face != path.end.face)
    parallel_transport(mesh, v, path.start.face, path.end.face, T2T);
  else {
    for (auto i = 0; i < path.strip.size() - 1; ++i) {
      parallel_transport(mesh, v, path.strip[i], path.strip[i + 1], T2T);
    }
  }
}
pair<int, vec3f> handle_vert(const geodesic_solver &solver,
                             const vector<vec3i> &triangles,
                             const vector<vec3f> &positions,
                             const vector<vec3f> &normals,
                             const vector<vec3i> &adjacencies,
                             const vector<vector<int>> &v2p_adjacencies,
                             const vector<vector<float>> &angles,
                             const vector<float> &total_angles, const int vid,
                             const int tid, const vec3f dir) {
  auto v = dir;
  parallel_transp(solver, angles, total_angles, triangles, positions,
                  adjacencies, v2p_adjacencies, v, normals, tid, vid, T2V);
  auto next = next_tid_extended_graph(total_angles, positions, v2p_adjacencies,
                                      triangles, normals, vid, v);
  // next_tid(solver, angles, positions, v2p_adjacencies, triangles,
  //                      normals, vid, v);

  parallel_transp(solver, angles, total_angles, triangles, positions,
                  adjacencies, v2p_adjacencies, v, normals, vid, next, V2T);
  return std::make_pair(next, v);
}
// we assume that v is defined in the tangent space of from.face
vector<mesh_point> straightest_geodesic(
    const geodesic_solver &solver, const vector<vec3i> &triangles,
    const vector<vec3f> &positions, const vector<vec3f> &normals,
    const vector<vec3i> &adjacencies,
    const vector<vector<int>> &v2p_adjacencies,
    const vector<vector<float>> &angles, const vector<float> &total_angles,
    const mesh_point &from, const vec3f &v, const float &l) {
  auto vid = -1, tid = -1, next_tri = -1;
  auto dir = v;
  float len = 0.0;
  auto next_bary = zero3f, next = zero3f;
  auto prev = eval_position(triangles, positions, from);
  auto samples = vector<mesh_point>{from};
  auto bary = get_bary(from.uv);
  auto [is_vert, kv] = bary_is_vert(bary);
  auto [is_on_edge, ke] = bary_is_edge(bary);
  if (is_vert) {
    vid = triangles[from.face][kv];
    // parallel_transp(solver, angles, total_angles, triangles, positions,
    //                 adjacencies, dir, normals, from.face, vid, T2V);

    tid = next_tid_extended_graph(total_angles, positions, v2p_adjacencies,
                                  triangles, normals, vid, v);
    kv = find(triangles[tid], vid);
    bary = zero3f;
    bary[kv] = 1;
    parallel_transp(solver, angles, total_angles, triangles, positions,
                    adjacencies, v2p_adjacencies, dir, normals, vid, tid, V2T);
  } else if (is_on_edge) {
    auto p0 = triangles[from.face][ke];
    auto p1 = triangles[from.face][(ke + 1) % 3];
    auto p2 = triangles[from.face][(ke + 2) % 3];
    auto n = triangle_normal(positions[p0], positions[p1], positions[p2]);
    auto edge = normalize(positions[p1] - positions[p0]);
    if (dot(cross(edge, v), n) > 0)
      tid = from.face;
    else {
      tid = adjacencies[from.face][ke];
      bary = tri_bary_coords(triangles, positions, tid, prev);
      parallel_transp(solver, angles, total_angles, triangles, positions,
                      adjacencies, v2p_adjacencies, dir, normals, from.face,
                      tid, T2T);
    }

  } else
    tid = from.face;

  while (len < l) {
    trace_in_triangles(positions, triangles, dir, bary, tid, next, next_bary);
    samples.push_back({tid, vec2f{next_bary.y, next_bary.z}});
    len += length(next - prev);
    if (len < l) {
      prev = next;
      auto [V, k_v] = bary_is_vert(next_bary);
      auto [E, k_e] = bary_is_edge(next_bary);
      if (V) {
        vid = triangles[tid][k_v];
        auto out =
            handle_vert(solver, triangles, positions, normals, adjacencies,
                        v2p_adjacencies, angles, total_angles, vid, tid, dir);
        tid = out.first;
        dir = out.second;
        k_v = find(triangles[tid], vid);
        bary = zero3f;
        bary[k_v] = 1;
      } else if (E) {
        next_tri = adjacencies[tid][k_e];
        auto p0 = triangles[tid][k_e];
        auto offset0 = find(triangles[next_tri], p0);
        auto offset1 = (offset0 + 2) % 3;
        bary = zero3f;
        bary[offset0] = next_bary[k_e];
        bary[offset1] = next_bary[(k_e + 1) % 3];

        parallel_transp(solver, angles, total_angles, triangles, positions,
                        adjacencies, v2p_adjacencies, dir, normals, tid,
                        next_tri, T2T);
        tid = next_tri;
      } else
        assert(false);
    }
  }

  auto factor = (len - l);
  auto w = normalize(prev - next);
  w *= factor;
  w += next;
  bary = tri_bary_coords(triangles, positions, tid, w);
  samples.pop_back();
  samples.push_back({tid, vec2f{bary.y, bary.z}});

  return samples;
}

vector<vec3f> trace_polyline_from_samples(const vector<mesh_point> &samples,
                                          const vector<vec3i> &triangles,
                                          const vector<vec3f> &positions) {
  vector<vec3f> poly_line;
  for (int i = 0; i < samples.size(); ++i) {
    int tid = samples[i].face;
    int p0 = triangles[tid].x;
    int p1 = triangles[tid].y;
    int p2 = triangles[tid].z;
    vec2f bary = samples[i].uv;
    float third = 1 - bary.x - bary.y;
    vec3f pos =
        positions[p0] * third + positions[p1] * bary.x + positions[p2] * bary.y;

    poly_line.push_back(pos);
  }
  return poly_line;
}

// geodesic_path compute_geodesic_path(const bezier_mesh &mesh,
//                                     const mesh_point &start,
//                                     const mesh_point &end) {
//   // profile_function();
//   auto path = geodesic_path{};
//   if (start.face == end.face) {
//     path.start = start;
//     path.end = end;
//     path.strip = {start.face};
//     return path;
//   }
//   auto strip = vector<int>{};
//   strip = get_strip(mesh.solver, mesh.triangles, mesh.positions,
//                     mesh.adjacencies, mesh.v2t, mesh.angles, end, start);

//   path = shortest_path(mesh.triangles, mesh.positions, mesh.adjacencies,
//   start,
//                        end, strip);
//   return path;
// }

pair<int, float> binary_search(const vector<float> &v, float t) {
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
vec3f geodesic_path_tangent(const vector<vec3i> &triangles,
                            const vector<vec3f> &positions,
                            const vector<vec3i> &adjacencies,
                            const geodesic_path &path,
                            const mesh_point &point) {
  auto a = mesh_point{};
  auto dir = zero3f;
  auto entry = -1;
  auto point_pos = eval_position(triangles, positions, point);
  auto path_pos = path_positions(path, triangles, positions, adjacencies);
  if (path.strip.size() == 1) {
    dir = eval_position(triangles, positions, point) - point_pos;
  } else {
    for (auto i = 0; i < path.strip.size(); ++i) {
      if (path.strip[i] == point.face) {
        entry = i;
        break;
      }
    }
    if (entry == -1)
      return zero3f;
    if (entry == 0) {
      auto x = path.lerps[0];
      auto e = get_edge(triangles, positions, adjacencies, path.strip[0],
                        path.strip[1]);
      auto p1 = lerp(positions[e.x], positions[e.y], x);
      dir = path_pos[1] - point_pos;
      // dir = p1 - point_pos;
    } else if (entry == path.strip.size() - 1) {
      auto x = path.lerps.back();
      auto e = get_edge(triangles, positions, adjacencies,
                        path.strip.rbegin()[1], path.strip.rbegin()[0]);
      auto p1 = lerp(positions[e.x], positions[e.y], x);
      // dir = p1 - point_pos;
      dir = path_pos.back() - point_pos;
    } else {
      auto x = path.lerps[entry];
      auto e = get_edge(triangles, positions, adjacencies, path.strip[entry],
                        path.strip[entry + 1]);
      auto p1 = lerp(positions[e.x], positions[e.y], x);
      // dir = p1 - point_pos;
      dir = path_pos[entry + 1] - point_pos;
    }
  }

  return normalize(dir);
}
std::tuple<vector<int>, mesh_point, mesh_point>
handle_short_strips(const vector<vec3i> &triangles,
                    const vector<vec3f> &positions, const vector<int> &strip,
                    const mesh_point &start, const mesh_point &end) {
  if (strip.size() == 1) {
    return {strip, start, end};
  } else if (strip.size() == 2) {
    auto [inside, b2f] =
        point_in_triangle(triangles, positions, start.face,
                          eval_position(triangles, positions, end));
    if (inside) {
      auto new_end = mesh_point{start.face, b2f};
      return {{start.face}, start, new_end};
    }
    std::tie(inside, b2f) =
        point_in_triangle(triangles, positions, end.face,
                          eval_position(triangles, positions, start));

    if (inside) {
      auto new_start = mesh_point{end.face, b2f};
      return {{end.face}, new_start, end};
    }

    return {strip, start, end};
  }
  return {{-1}, {}, {}};
}
vec3f flip_bary_to_adjacent_tri(const vector<vec3i> &adjacencies,
                                const int tid0, const int tid1,
                                const vec3f &bary) {
  if (tid0 == tid1)
    return bary;
  auto new_bary = zero3f;
  auto k1 = find(adjacencies[tid1], tid0);
  auto k0 = find(adjacencies[tid0], tid1);
  if (k1 == -1) {
    std::cout << "Error, faces are not adjacent" << std::endl;
    return zero3f;
  }
  new_bary[k1] = bary[(k0 + 1) % 3];
  new_bary[(k1 + 1) % 3] = bary[k0];
  new_bary[(k1 + 2) % 3] = bary[(k0 + 2) % 3];

  return new_bary;
}
std::tuple<vector<int>, mesh_point, mesh_point>
cleaned_strip(const vector<vec3i> &triangles, const vector<vec3f> &positions,
              const vector<vec3i> &adjacencies, const vector<int> &strip,
              const mesh_point &start, const mesh_point &end) {
  vector<int> cleaned = strip;

  auto start_entry = 0, end_entry = (int)strip.size() - 1;
  auto b3f = zero3f;
  auto new_start = start;
  auto new_end = end;
  auto [is_vert, kv] = point_is_vert(end);
  auto [is_edge, ke] = point_is_edge(end);
  if (strip.size() <= 2)
    return handle_short_strips(triangles, positions, strip, start, end);
  // Erasing from the bottom
  if (is_vert) {
    auto vid = triangles[end.face][kv];
    auto curr_tid = strip[end_entry - 1];
    kv = find(triangles[curr_tid], vid);
    while (kv != -1) {
      cleaned.pop_back();
      --end_entry;
      if (end_entry == 1)
        break;
      // see comment below
      auto curr_tid = strip[end_entry - 1];
      kv = find(triangles[curr_tid], vid);
    }
    kv = find(triangles[cleaned.back()], vid);
    assert(kv != -1);
    b3f[kv] = 1;
    new_end = mesh_point{cleaned.back(), vec2f{b3f.y, b3f.z}}; // updating end
  } else if (is_edge) {
    if (end.face != strip.back()) {
      assert(adjacencies[end.face][ke] == strip.back());

      if (end.face == strip[end_entry - 1])
        cleaned.pop_back();
    } else if (adjacencies[end.face][ke] == strip[end_entry - 1])
      cleaned.pop_back();

    b3f = flip_bary_to_adjacent_tri(adjacencies, end.face, cleaned.back(),
                                    get_bary(end.uv));

    new_end = mesh_point{cleaned.back(), vec2f{b3f.y, b3f.z}}; // updating end
  }
  std::tie(is_vert, kv) = point_is_vert(start);
  std::tie(is_edge, ke) = point_is_vert(start);

  if (is_vert) {
    auto vid = triangles[start.face][kv];
    auto curr_tid = strip[start_entry + 1];
    kv = find(triangles[curr_tid], vid);
    while (kv != -1) {
      cleaned.erase(cleaned.begin());
      ++start_entry;
      if (start_entry > end_entry - 1)
        break;
      auto curr_tid = strip[start_entry + 1];
      kv = find(triangles[curr_tid], vid);
    }
    kv = find(triangles[cleaned[0]], vid);
    assert(kv != -1);
    b3f = zero3f;
    b3f[kv] = 1;
    new_start = mesh_point{cleaned[0], vec2f{b3f.y, b3f.z}}; // udpdating start

  } else if (is_edge) {
    if (start.face != strip[0]) {
      assert(adjacencies[start.face][ke] == strip[0]);
      if (start.face == strip[1])
        cleaned.erase(cleaned.begin());
    } else if (adjacencies[start.face][ke] == strip[1]) {
      cleaned.erase(cleaned.begin());
    }
    b3f = flip_bary_to_adjacent_tri(adjacencies, start.face, cleaned[0],
                                    get_bary(start.uv));
    new_start = {cleaned[0], vec2f{b3f.y, b3f.z}}; // updating start
  }
  return {cleaned, new_start, new_end};
}
vector<int> compute_strip_tlv(const bezier_mesh &mesh, int start, int end) {
  if (start == end)
    return {start};

  thread_local static auto parents = vector<int>{};
  thread_local static auto field = vector<float>{};
  thread_local static auto in_queue = vector<bool>{};

  if (parents.size() != mesh.dual_solver.graph.size()) {
    parents.assign(mesh.dual_solver.graph.size(), -1);
    field.assign(mesh.dual_solver.graph.size(), flt_max);
    in_queue.assign(mesh.dual_solver.graph.size(), false);
  }

  // initialize once for all and sparsely cleanup at the end of every solve
  auto visited = vector<int>{start};
  auto sources = vector<int>{start};
  auto update = [&visited, end](int node, int neighbor, float new_distance) {
    parents[neighbor] = node;
    visited.push_back(neighbor);
    return neighbor == end;
  };
  auto stop = [](int node) { return false; };
  auto exit = [](int node) { return false; };

  search_strip(field, in_queue, mesh.dual_solver, mesh.triangles,
               mesh.positions, start, end, update, stop, exit);

  // extract_strip
  auto strip = vector<int>{};
  auto node = end;
  strip.reserve((int)sqrt(parents.size()));
  while (node != -1) {
    assert(find(strip, node) != 1);
    strip.push_back(node);
    node = parents[node];
  }

  // cleanup buffers
  for (auto &v : visited) {
    parents[v] = -1;
    field[v] = flt_max;
    in_queue[v] = false;
  }
  // assert(check_strip(mesh.adjacencies, strip));
  return strip;
}

geodesic_path compute_geodesic_path(const bezier_mesh &mesh,
                                    const mesh_point &start,
                                    const mesh_point &end) {
  // profile_function();

  vector<int> parents;
  auto strip = compute_strip_tlv(mesh, end.face, start.face);
  // get_strip(mesh.solver, mesh.triangles, mesh.positions,
  // mesh.adjacencies,
  //           mesh.v2t, mesh.angles, end, start);
  auto path = geodesic_path{};
  auto [cleaned, new_start, new_end] = cleaned_strip(
      mesh.triangles, mesh.positions, mesh.adjacencies, strip, start, end);

  if (new_start.face == new_end.face) {
    path.start = new_start;
    path.end = new_end;
    path.strip = {new_start.face};
    return path;
  }

  path = shortest_path(mesh.triangles, mesh.positions, mesh.adjacencies,
                       new_start, new_end, cleaned);
  return path;
}
pair<float, int> optimal_sample(const vector<vec3i> &triangles,
                                const geodesic_path &path, const int vid) {
  auto lerp = path.lerps[0];
  auto tid = path.strip[0];
  auto entry = 0;
  while (lerp == 0 || lerp == 1) {
    ++entry;
    if (path.lerps.size() == entry)
      return {lerp, tid};
    if (find(triangles[path.strip[entry]], vid) != -1) {
      lerp = path.lerps[entry];
      tid = path.strip[entry];
    } else
      return {lerp, tid};
  }

  return {lerp, tid};
}
void extended_solver(bezier_mesh &mesh, const int k) {
  auto old_solver = make_geodesic_solver(mesh.triangles, mesh.positions,
                                         mesh.adjacencies, mesh.v2t);
  mesh.angles = compute_angles(mesh.triangles, mesh.positions, mesh.adjacencies,
                               mesh.v2t, mesh.total_angles, true);
  auto avg_valence = 0;
  mesh.solver.graph.resize(mesh.positions.size());
  // mesh.angles.resize(mesh.positions.size());

  for (auto i = 0; i < mesh.positions.size(); ++i) {

    auto neighborhood = k_ring(mesh.triangles, mesh.v2t, i, k);
    auto valence = neighborhood.size();
    // auto new_angles = vector<pair<float, int>>(neighborhood.size());
    auto lengths = unordered_map<int, float>{};
    auto source = point_from_vert(mesh.triangles, mesh.v2t, i);
    for (auto j = 0; j < neighborhood.size(); ++j) {
      auto curr_point =
          point_from_vert(mesh.triangles, mesh.v2t, neighborhood[j]);
      auto path = compute_geodesic_path(mesh, source, curr_point);
      if (path.lerps.size() > 0) {
        auto [lerp, tid] = optimal_sample(mesh.triangles, path, i);
        // new_angles[j] =
        //     std::make_pair(angle_in_tangent_space(
        //                        old_solver, old_angles, mesh.total_angles,
        //                        mesh.triangles, mesh.positions, tid, i, lerp),
        //                    neighborhood[j]);
        lengths[neighborhood[j]] =
            path_length(path, mesh.triangles, mesh.positions, mesh.adjacencies);
      } else {
        auto entry = node_is_adjacent(old_solver, i, neighborhood[j]);
        if (entry == -1) {
          std::cout << "Error, the vertices should be adjacent" << std::endl;
          return;
        }

        // new_angles[j] = std::make_pair(old_angles[i][entry],
        // neighborhood[j]);
        lengths[neighborhood[j]] = old_solver.graph[i][entry].length;
      }
    }
    avg_valence += valence;
    // sort(new_angles.begin(), new_angles.end());
    // mesh.angles[i].resize(new_angles.size());
    mesh.solver.graph[i].resize(valence);
    for (auto j = 0; j < valence; ++j) {
      mesh.solver.graph[i][j] = {neighborhood[j], lengths.at(neighborhood[j])};
    }
  }
  // std::cout << "avg valence is" << std::endl;
  // std::cout << avg_valence / mesh.positions.size() << std::endl;
}
std::pair<vector<vec3f>, vector<float>>
path_parameters_and_positions(const bezier_mesh &mesh,
                              const vector<mesh_point> &path) {
  auto len = 0.0f;
  auto parameter_t = vector<float>(path.size(), 0);
  auto positions = vector<vec3f>(path.size());
  for (auto i = 0; i < path.size(); i++) {
    positions[i] = eval_position(mesh.triangles, mesh.positions, path[i]);
    if (i)
      len += length(positions[i] - positions[i - 1]);
    parameter_t[i] = len;
  }
  for (auto &t : parameter_t)
    t /= len;
  return {positions, parameter_t};
}
vector<vec3f> path_positions(const bezier_mesh &mesh,
                             const vector<mesh_point> &path) {

  auto positions = vector<vec3f>(path.size());
  for (auto i = 0; i < path.size(); i++) {
    positions[i] = eval_position(mesh.triangles, mesh.positions, path[i]);
  }
  return positions;
}
mesh_point eval_point(const bezier_mesh &mesh, const vector<mesh_point> &path,
                      const float &t) {
  auto [positions, parameters] = path_parameters_and_positions(mesh, path);
  if (path.size() == 1)
    return path[0];

  auto [entry, factor] = binary_search(parameters, t);
  auto p = zero3f, q = zero3f;
  auto tid = -1;
  if (entry == 0) {
    tid = path[0].face;
    p = eval_position(mesh.triangles, mesh.positions, path[0]);
    q = eval_position(mesh.triangles, mesh.positions, path[1]);
  } else if (entry == parameters.size() - 1) {
    return path.back();

  } else {
    p = eval_position(mesh.triangles, mesh.positions, path[entry - 1]);
    q = eval_position(mesh.triangles, mesh.positions, path[entry]);
    tid = path[entry].face;
  }
  auto start = q - p;
  start *= factor;
  start += p;

  auto [inside, bary] =
      point_in_triangle(mesh.triangles, mesh.positions, tid, start);
  if (!inside) {
    tid = path[entry - 1].face;
    std::tie(inside, bary) =
        point_in_triangle(mesh.triangles, mesh.positions, tid, start);
    entry -= 1;
  }
  assert(inside);
  return {tid, bary};
}
// extend geodesic computations
// utility (Compute distances)
template <typename Update, typename Stop, typename Exit>
void visit_graph(vector<float> &field, const geodesic_solver &solver,
                 const vector<int> &sources, Update &&update, Stop &&stop,
                 Exit &&exit) {
  /*
     This algortithm uses the heuristic Small Label Fisrt and Large Label Last
     https://en.wikipedia.org/wiki/Shortest_Path_Faster_Algorithm

     Large Label Last (LLL): When extracting nodes from the queue, pick the
     front one. If it weights more than the average weight of the queue, put
     on the back and check the next node. Continue this way.
     Sometimes average_weight is less than every value due to floating point
     errors (doesn't happen with double precision).

     Small Label First (SLF): When adding a new node to queue, instead of
     always pushing it to the end of the queue, if it weights less than the
     front node of the queue, it is put on front. Otherwise the node is put at
     the end of the queue.
  */

  auto in_queue = vector<bool>(solver.graph.size(), false);
  // Cumulative weights of elements in queue. Used to keep track of the
  // average weight of the queue.
  double cumulative_weight = 0.0;

  // setup queue
  auto queue = std::deque<int>();
  for (auto source : sources) {
    in_queue[source] = true;
    cumulative_weight += field[source];
    queue.push_back(source);
  }

  while (!queue.empty()) {
    auto node = queue.front();
    auto average_weight = (float)cumulative_weight / queue.size();

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
    if (exit(node, field[node]))
      continue;
    if (stop(node))
      continue;

    for (auto i = 0; i < (int)solver.graph[node].size(); i++) {
      // Distance of neighbor through this node
      auto new_distance = field[node] + solver.graph[node][i].length;
      auto neighbor = solver.graph[node][i].node;

      auto old_distance = field[neighbor];
      if (new_distance >= old_distance)
        continue;

      // Binomial Coefficient
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
      update(node, neighbor, new_distance);
    }
  }
}
vector<pair<int, float>>
nodes_around_mesh_point(const vector<vec3i> &triangles,
                        const vector<vec3f> &positions,
                        const vector<vec3i> &adjacencies, const mesh_point &p) {
  auto nodes = vector<pair<int, float>>{};
  auto [is_vertex, offset] = bary_is_vert(get_bary(p.uv));

  if (is_vertex) {
    auto vid = triangles[p.face][offset];
    nodes.push_back({vid, 0});
  } else {
    auto pid = p.face;
    auto pos = eval_position(triangles, positions, p);

    for (int i = 0; i < 3; ++i) {
      int p0 = triangles[pid][i], p1 = triangles[pid][(i + 1) % 3];
      float d = length(positions[p0] - pos);
      nodes.push_back(std::make_pair(p0, d));

      int CW_pid = adjacencies[pid][i];
      int opp = opposite_vertex(triangles, adjacencies, pid, i);
      vector<int> strip = {CW_pid, pid};
      float l =
          length_by_flattening(triangles, positions, adjacencies, p, strip);

      nodes.push_back(std::make_pair(opp, l));

      int opp_pid = opposite_face(triangles, adjacencies, CW_pid, p0);
      strip = {opp_pid, CW_pid, pid};
      int k = find(adjacencies[CW_pid], opp_pid);
      int q = opposite_vertex(triangles, adjacencies, CW_pid, k);
      d = length_by_flattening(triangles, positions, adjacencies, p, strip);
      nodes.push_back(std::make_pair(q, d));

      opp_pid = opposite_face(triangles, adjacencies, CW_pid, p1);
      strip = {opp_pid, CW_pid, pid};

      k = find(adjacencies[CW_pid], opp_pid);
      q = opposite_vertex(triangles, adjacencies, CW_pid, k);
      d = length_by_flattening(triangles, positions, adjacencies, p, strip);
      nodes.push_back(std::make_pair(q, d));
    }
  }

  return nodes;
}
vector<float>
solve_with_targets(const geodesic_solver &solver,
                   const vector<pair<int, float>> &sources_and_dist,
                   const vector<pair<int, float>> &targets) {
  auto update = [](int node, int neighbor, float new_distance) {};
  auto stop = [](int node) { return false; };
  auto max_distance = flt_min;
  auto exit_verts = vector<int>{};
  for (auto i = 0; i < targets.size(); ++i) {
    auto it = find(exit_verts.begin(), exit_verts.end(), targets[i].first);
    if (it == exit_verts.end())
      exit_verts.push_back(targets[i].first);
  }
  auto exit = [&exit_verts, &max_distance](int node,
                                           const float &curr_distance) {
    auto it = find(exit_verts.begin(), exit_verts.end(), node);
    if (it != exit_verts.end()) {
      max_distance = yocto::max(curr_distance, max_distance);
      exit_verts.erase(it);
    }

    if (exit_verts.empty() && curr_distance > max_distance + max_distance / 5) {
      return true;
    }
    return false;
  };

  auto distances = vector<float>(solver.graph.size(), flt_max);
  auto sources_id = vector<int>(sources_and_dist.size());
  for (auto i = 0; i < sources_and_dist.size(); ++i) {
    sources_id[i] = sources_and_dist[i].first;
    distances[sources_and_dist[i].first] = sources_and_dist[i].second;
  }

  visit_graph(distances, solver, sources_id, update, stop, exit);
  return distances;
}
static vector<pair<int, float>>
check_surrounding_nodes(vector<pair<int, float>> &nodes) {
  sort(nodes.begin(), nodes.end());
  auto new_nodes = vector<pair<int, float>>{};
  for (auto i = 1; i < nodes.size(); ++i) {
    auto prev = nodes[i - 1];
    auto curr = nodes[i];
    if (prev.first == curr.first) {
      auto d0 = prev.second, d1 = curr.second;
      if (d0 <= d1)
        new_nodes.push_back(prev);
      else
        new_nodes.push_back(curr);
      ++i;
    } else {
      new_nodes.push_back(prev);
    }
  }
  if (nodes.back().first != new_nodes.back().first)
    new_nodes.push_back(nodes.back());

  nodes = new_nodes;
  return nodes;
}
vector<float> compute_pruned_geodesic_distances(
    const geodesic_solver &solver, const vector<vec3i> &triangles,
    const vector<vec3f> &positions, const vector<vec3i> &adjacencies,
    const vector<vector<int>> &v2t, const mesh_point &source,
    const vector<mesh_point> &targets) {
  auto target_nodes = vector<pair<int, float>>{};

  auto source_nodes =
      nodes_around_mesh_point(triangles, positions, adjacencies, source);

  for (auto i = 0; i < targets.size(); ++i) {
    auto curr_nodes =
        nodes_around_mesh_point(triangles, positions, adjacencies, targets[i]);
    if (curr_nodes.size() > 1)
      check_surrounding_nodes(curr_nodes);
    target_nodes.insert(target_nodes.end(), curr_nodes.begin(),
                        curr_nodes.end());
  }
  if (source_nodes.size() > 1)
    check_surrounding_nodes(source_nodes);

  return solve_with_targets(solver, source_nodes, target_nodes);
}
vector<vec3f> straight_line(const vec3f &p0, const vec3f &p1) {
  float delta = 1 / 100;
  auto result = vector<vec3f>(101);
  for (auto i = 0; i <= 100; ++i) {
    result[i] = p0 + i * delta * (p1 - p0);
  }
  result.back() = p1;

  return result;
}