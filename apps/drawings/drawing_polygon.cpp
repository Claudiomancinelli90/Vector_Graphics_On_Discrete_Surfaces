#include <realtime/ext/glad/glad.h>

#include "drawing_functions.h"
#include "drawing_polygon.h"
#include "geometry.h"

inline float max(const vector<float> &vec) {
  auto m = vec.front();

  for (auto val : vec) {
    if (val > m) {
      m = val;
    }
  }

  return m;
}

float get_angle(const bezier_mesh &mesh, Polygon &polygon, const int &ix) {
  auto a = polygon.edges[ix], b = polygon.angle_bisectors[ix];

  auto v = polygon.vertices[ix];

  auto pa = geodesic_path_direction(mesh, a);
  auto pb = geodesic_path_direction(mesh, b);

  return 2 * angle(pa, pb) * 360 / (2 * pi);
}

// TODO: checks on index
void update_angle(const bezier_mesh &mesh, Polygon &polygon, const int &ix) {
  if (polygon.n_vertices != polygon.n) {
    return;
  }

  polygon.angles[ix] = get_angle(mesh, polygon, ix);
}

// TODO: checks on closed polygon
void update_angles(const bezier_mesh &mesh, Polygon &polygon) {
  for (auto i = 0; i < polygon.n_vertices; ++i) {
    update_angle(mesh, polygon, i);
  }
}

Polygon create_polygon(int n) {
  auto polygon = Polygon();

  polygon.n = n;
  polygon.n_vertices = 0;
  polygon.vertices = vector<mesh_point>(polygon.n);
  polygon.edges = vector<geodesic_path>(polygon.n);
  polygon.angle_bisectors = vector<geodesic_path>(polygon.n);
  polygon.angles = vector<float>(polygon.n);
  polygon.edges_length = vector<float>(polygon.n);

  polygon.shapes = {
      {"vertices", vector<Shape>(polygon.n)},
      {"edges", vector<Shape>(polygon.n)},
      {"bisectors", vector<Shape>(polygon.n)},
  };

  unordered_set<int> selected_shapes_ids = unordered_set<int>();

  return polygon;
}

void reset_polygon(Polygon &polygon) {
  polygon.n_vertices = 0;

  polygon.vertices.assign(polygon.n, mesh_point());
  polygon.edges.assign(polygon.n, geodesic_path());
  polygon.angle_bisectors.assign(polygon.n, geodesic_path());
  polygon.angles.assign(polygon.n, 0.0F);
  polygon.edges_length.assign(polygon.n, 0.0F);

  polygon.shapes["vertices"].assign(polygon.n, Shape());
  polygon.shapes["edges"].assign(polygon.n, Shape());
  polygon.shapes["bisectors"].assign(polygon.n, Shape());

  polygon.selected_shapes_ids.clear();
  polygon.selected_vertex = -1;
  polygon.selected_edge = -1;
}

void change_polygon(Polygon &polygon, int n, const bezier_mesh &mesh) {
  polygon.n = n;
  polygon.n_vertices = min(polygon.n_vertices, polygon.n);

  polygon.vertices.resize(polygon.n, mesh_point());
  polygon.edges.assign(polygon.n, geodesic_path());
  polygon.angle_bisectors.assign(polygon.n, geodesic_path());
  polygon.angles.assign(polygon.n, 0.0F);
  polygon.edges_length.assign(polygon.n, 0.0F);

  polygon.shapes["vertices"].resize(polygon.n, Shape());
  polygon.shapes["edges"].assign(polygon.n, Shape());
  polygon.shapes["bisectors"].assign(polygon.n, Shape());

  polygon.selected_shapes_ids.clear();
  polygon.selected_vertex = -1;
  polygon.selected_edge = -1;

  update_edges(mesh, polygon);
  update_angle_bisectors(mesh, polygon);
  update_angles(mesh, polygon);
}

void draw_polygon(unordered_map<string, Shader> &shaders, Polygon &polygon,
                  mat4f &view, mat4f &projection, bool bisectors) {
  bind_shader(shaders["points"]);
  set_uniform(shaders["points"], Uniform("frame", identity4x4f),
              Uniform("view", view), Uniform("projection", projection));

  for (auto shape : polygon.shapes["vertices"]) {
    if (!polygon.selected_shapes_ids.count(shape.id)) {
      set_uniform(shaders["points"], Uniform("color", vec3f{0, 0, 0}));
    } else {
      set_uniform(shaders["points"], Uniform("color", vec3f{1, 0, 0}));
    }

    draw_shape(shape);
  }

  if (bisectors) {
    for (auto shape : polygon.shapes["bisectors"]) {
      if (!polygon.selected_shapes_ids.count(shape.id)) {
        set_uniform(shaders["points"], Uniform("color", vec3f{0, 0, 0}));
      } else {
        set_uniform(shaders["points"], Uniform("color", vec3f{1, 0, 0}));
      }

      draw_shape(shape);
    }
  }

  for (auto shape : polygon.shapes["edges"]) {
    if (!polygon.selected_shapes_ids.count(shape.id)) {
      set_uniform(shaders["points"], Uniform("color", vec3f{0, 0, 0}));
    } else {
      set_uniform(shaders["points"], Uniform("color", vec3f{1, 0, 0}));
    }

    draw_shape(shape);
  }
}

// -------------------------------------------------------------------------------------
bool is_angle_valid(const bezier_mesh &mesh, const mesh_point &point,
                    const geodesic_path &edge) {
  auto normal = tid_normal(mesh.triangles, mesh.positions, edge.start.face);

  auto new_edge = compute_geodesic_path(mesh, point, edge.start);

  auto dir1 = geodesic_path_direction(mesh, new_edge, true);
  auto dir2 = geodesic_path_direction(mesh, edge);

  auto bis_dir = normalize(cross(dir1, normal) + cross(normal, dir2)) *
                 dot(normal, normalize(cross(dir2, dir1)));

  return dot(bis_dir, normalize(dir1 + dir2)) > 0 &&
         angle(bis_dir, dir2) <= M_PI / 2;
}

bool is_angle_valid(const bezier_mesh &mesh, const geodesic_path &edge,
                    const mesh_point &point) {
  auto normal = tid_normal(mesh.triangles, mesh.positions, edge.end.face);

  auto new_edge = compute_geodesic_path(mesh, edge.end, point);

  auto dir1 = geodesic_path_direction(mesh, edge, true);
  auto dir2 = geodesic_path_direction(mesh, new_edge);

  auto bis_dir = normalize(cross(dir1, normal) + cross(normal, dir2)) *
                 dot(normal, normalize(cross(dir2, dir1)));

  return dot(bis_dir, normalize(dir1 + dir2)) > 0 &&
         angle(bis_dir, dir2) <= M_PI / 2;
}

bool is_angle_valid(const bezier_mesh &mesh, const mesh_point &a,
                    const mesh_point &b, const mesh_point &c) {
  return is_angle_valid(mesh, compute_geodesic_path(mesh, a, b), c);
}

bool is_angle_valid(const bezier_mesh &mesh, const geodesic_path &a,
                    const geodesic_path &b) {
  auto normal = tid_normal(mesh.triangles, mesh.positions, a.end.face);

  auto dir1 = geodesic_path_direction(mesh, a, true);
  auto dir2 = geodesic_path_direction(mesh, b);

  auto bis_dir = normalize(cross(dir1, normal) + cross(normal, dir2)) *
                 dot(normal, normalize(cross(dir2, dir1)));

  return dot(bis_dir, normalize(dir1 + dir2)) > 0 &&
         angle(bis_dir, dir2) <= M_PI / 2;
}

void add_vertex(const bezier_mesh &mesh, Polygon &polygon,
                const mesh_point &point) {
  if (polygon.n_vertices >= polygon.n) {
    return;
  }

  auto ix = polygon.n_vertices;

  if (ix > 1) {
    if (!is_angle_valid(mesh, polygon.edges[ix - 2], point) ||
        !is_angle_valid(mesh, polygon.vertices[ix - 1], point,
                        polygon.vertices[0]) ||
        !is_angle_valid(mesh, point, polygon.vertices[0],
                        polygon.vertices[1])) {
      return;
    }
  }

  polygon.n_vertices++;

  polygon.vertices[ix] = point;
  auto shape = make_points_shape(
      vector<vec3f>{eval_position(mesh.triangles, mesh.positions, point)});
  polygon.shapes["vertices"][ix] = shape;

  if (polygon.n_vertices > 1) {
    update_edge(mesh, polygon, (ix + polygon.n - 1) % polygon.n);
  }

  // Close the polygon
  if ((ix + 1) % polygon.n < polygon.n_vertices) {
    update_edge(mesh, polygon, ix);
    update_angle_bisectors(mesh, polygon);
    update_angles(mesh, polygon);
  }
}

void set_vertex(const bezier_mesh &mesh, Polygon &polygon, const int &ix,
                const mesh_point &p) {
  if (ix >= polygon.n_vertices)
    return;

  auto p_ix = (ix + polygon.n - 1) % polygon.n;
  auto pp_ix = (ix + polygon.n - 2) % polygon.n;
  auto n_ix = (ix + 1) % polygon.n;
  auto nn_ix = (ix + 2) % polygon.n;

  if (n_ix < polygon.n_vertices) {
    if (p_ix < polygon.n_vertices &&
            !is_angle_valid(mesh, polygon.vertices[p_ix], p,
                            polygon.vertices[n_ix]) ||
        nn_ix < polygon.n_vertices &&
            !is_angle_valid(mesh, p, polygon.vertices[n_ix],
                            polygon.vertices[nn_ix])) {
      return;
    }
  }
  if (pp_ix < polygon.n_vertices &&
      !is_angle_valid(mesh, polygon.vertices[pp_ix], polygon.vertices[p_ix],
                      p)) {
    return;
  }

  polygon.vertices[ix] = p;
  auto shape = make_points_shape(vector<vec3f>{
      eval_position(mesh.triangles, mesh.positions, polygon.vertices[ix])});
  if (polygon.selected_vertex == ix) {
    polygon.selected_shapes_ids.erase(
        polygon.shapes["vertices"][polygon.selected_vertex].id);
    polygon.selected_shapes_ids.insert(shape.id);
  }
  delete_shape(polygon.shapes["vertices"][ix]);
  polygon.shapes["vertices"][ix] = shape;
}

void update_vertices(const vector<vec3i> &triangles,
                     const vector<vec3f> &positions, Polygon &polygon) {
  for (auto i = 0; i < polygon.n; ++i) {
    update_vertex(triangles, positions, polygon, i);
  }
}

void update_vertex(const vector<vec3i> &triangles,
                   const vector<vec3f> &positions, Polygon &polygon,
                   const int &ix) {
  if (ix >= polygon.n_vertices)
    return;

  polygon.vertices[ix] = polygon.edges[ix].start;
  auto shape = make_points_shape(
      vector<vec3f>{eval_position(triangles, positions, polygon.vertices[ix])});
  if (polygon.selected_vertex == ix) {
    polygon.selected_shapes_ids.erase(
        polygon.shapes["vertices"][polygon.selected_vertex].id);
    polygon.selected_shapes_ids.insert(shape.id);
  }
  delete_shape(polygon.shapes["vertices"][ix]);
  polygon.shapes["vertices"][ix] = shape;
}

void set_edge(const bezier_mesh &mesh, Polygon &polygon, const int &ix,
              const geodesic_path &path) {
  if (ix > polygon.n_vertices - 1) {
    return;
  }

  auto p_ix = (ix + polygon.n - 1) % polygon.n;
  auto n_ix = (ix + 1) % polygon.n;

  if (polygon.n_vertices == polygon.n) {
    if (n_ix < polygon.n_vertices &&
            !is_angle_valid(mesh, path, polygon.edges[n_ix]) ||
        p_ix < polygon.n_vertices &&
            !is_angle_valid(mesh, polygon.edges[p_ix], path)) {
      return;
    }
  } else {
    if (n_ix < polygon.n_vertices - 1 &&
            !is_angle_valid(mesh, path, polygon.edges[n_ix]) ||
        p_ix < polygon.n_vertices - 1 &&
            !is_angle_valid(mesh, polygon.edges[p_ix], path)) {
      return;
    }
  }

  polygon.edges[ix] = path;
  auto points = path_positions(polygon.edges[ix], mesh);
  auto shape = make_polyline_shape(points, {});
  if (polygon.selected_edge == ix) {
    polygon.selected_shapes_ids.erase(
        polygon.shapes["edges"][polygon.selected_edge].id);
    polygon.selected_shapes_ids.insert(shape.id);
  }
  delete_shape(polygon.shapes["edges"][ix]);
  polygon.shapes["edges"][ix] = shape;

  polygon.edges_length[ix] = path_length(points);
}

void update_edges(const bezier_mesh &mesh, Polygon &polygon) {
  for (auto i = 0; i < polygon.n; ++i) {
    update_edge(mesh, polygon, i);
  }
}

void update_edge(const bezier_mesh &mesh, Polygon &polygon, const int &ix) {
  auto next = (ix + 1) % polygon.n;

  if (ix > polygon.n_vertices - 1 || next > polygon.n_vertices - 1) {
    return;
  }

  polygon.edges[ix] =
      compute_geodesic_path(mesh, polygon.vertices[ix], polygon.vertices[next]);
  auto points = path_positions(polygon.edges[ix], mesh);
  auto shape = make_polyline_shape(points, {});
  if (polygon.selected_edge == ix) {
    polygon.selected_shapes_ids.erase(
        polygon.shapes["edges"][polygon.selected_edge].id);
    polygon.selected_shapes_ids.insert(shape.id);
  }
  delete_shape(polygon.shapes["edges"][ix]);
  polygon.shapes["edges"][ix] = shape;

  polygon.edges_length[ix] = path_length(points);
}

void set_angle(const bezier_mesh &mesh, Polygon &polygon,
               const float &next_angle, const int &ix) {
  if (next_angle < MIN_ANGLE || next_angle > MAX_ANGLE) {
    return;
  }

  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;
  auto &adjacencies = mesh.adjacencies;

  auto &vertices = polygon.vertices;
  auto &n = polygon.n;

  auto prev = (ix + n - 1) % n;

  auto theta = (next_angle - polygon.angles[ix]) * 2 * pi / 360;

  auto dprev = direction_in_tangent_space(
      triangles, positions, vertices[ix],
      rotate_in_tangent_space(
          triangles, positions, vertices[ix],
          geodesic_path_direction(mesh, polygon.edges[prev], true), theta / 2));
  auto dnext = direction_in_tangent_space(
      triangles, positions, vertices[ix],
      rotate_in_tangent_space(
          triangles, positions, vertices[ix],
          geodesic_path_direction(mesh, polygon.edges[ix], false), -theta / 2));

  auto a = reverse(straightest_geodesic_path(mesh, vertices[ix], dprev,
                                             polygon.edges_length[prev]));
  auto b = straightest_geodesic_path(mesh, vertices[ix], dnext,
                                     polygon.edges_length[ix]);

  if ((ix + 2) % n < polygon.n_vertices &&
          !is_angle_valid(mesh, b, vertices[(ix + 2) % n]) ||
      (ix + 3) % n < polygon.n_vertices &&
          !is_angle_valid(mesh, b.end, vertices[(ix + 2) % n],
                          vertices[(ix + 3) % n]) ||
      (ix + n - 2) % n < polygon.n_vertices &&
          !is_angle_valid(mesh, vertices[(ix + n - 2) % n], a) ||
      (ix + n - 3) % n < polygon.n_vertices &&
          !is_angle_valid(mesh, vertices[(ix + n - 3) % n],
                          vertices[(ix + n - 2) % n], a.start)) {
    return;
  }

  set_vertex(mesh, polygon, prev, a.start);
  set_vertex(mesh, polygon, (ix + 1) % n, b.end);
  set_edge(mesh, polygon, prev, a);
  set_edge(mesh, polygon, ix, b);
  update_edge(mesh, polygon, (ix + n - 2) % n);
  update_edge(mesh, polygon, (ix + 1) % n);
  update_angle_bisectors(mesh, polygon);
  update_angles(mesh, polygon);
}

geodesic_path get_angle_bisector(const bezier_mesh &mesh,
                                 const Polygon &polygon, const int &ix) {

  // auto v = polygon.vertices[ix];
  // auto normal = tid_normal(mesh.triangles, mesh.positions, v.face);

  // auto dir1 = geodesic_path_direction(
  //     mesh, polygon.edges[(ix + polygon.n - 1) % polygon.n], true);
  // auto dir2 = geodesic_path_direction(mesh, polygon.edges[ix]);

  // auto dir = normalize(cross(dir1, normal) + cross(normal, dir2)) *
  //            dot(normal, normalize(cross(dir2, dir1)));

  // auto bis = find_intersecting_geodesic(mesh, polygon.edges, ix, v, dir,
  //                                       polygon.n_vertices,
  //                                       max(polygon.edges_length) * 2);

  // if (bis.start.face == -1 || bis.end.face == -1) {
  //   auto next = zero3f, next_bary = zero3f;
  //   trace_in_triangles(mesh.positions, mesh.triangles, dir, get_bary(v.uv),
  //                      v.face, next, next_bary);

  //   bis.start = v;
  //   bis.end = eval_mesh_point(mesh.triangles, mesh.positions, v.face, next);
  //   bis.lerps.resize(0);
  //   bis.strip.resize(0);
  //   bis.strip.push_back(v.face);
  // }

  // return bis;
}

void update_angle_bisectors(const bezier_mesh &mesh, Polygon &polygon) {
  // if (polygon.n_vertices != polygon.n) {
  //   return;
  // }

  //   for (auto i = 0; i < polygon.n_vertices; ++i) {
  //     auto bisector = get_angle_bisector(mesh, polygon, i);

  //     polygon.angle_bisectors[i] = bisector;
  //     auto points = path_positions(bisector, mesh);
  //     auto shape = make_polyline_shape(points, {});
  //     delete_shape(polygon.shapes["bisectors"][i]);
  //     polygon.shapes["bisectors"][i] = shape;
  //   }
}

void extend_edge(const bezier_mesh &mesh, Polygon &polygon, const int &ix,
                 const float &d) {
  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;
  auto &adjacencies = mesh.adjacencies;

  auto edge = polygon.edges[ix];

  if (d <= 0) {
    return;
  }

  auto dir = direction_in_tangent_space(triangles, positions, edge.start,
                                        -geodesic_path_direction(mesh, edge));
  auto new_edge =
      reverse(straightest_geodesic_path(mesh, edge.start, dir, d / 2));

  for (auto i = 1; i < edge.strip.size(); ++i) {
    new_edge.strip.push_back(edge.strip[i]);
    new_edge.lerps.push_back(edge.lerps[i - 1]);
  }

  dir = direction_in_tangent_space(triangles, positions, edge.end,
                                   -geodesic_path_direction(mesh, edge, true));
  auto path = straightest_geodesic_path(mesh, edge.end, dir, d / 2);

  for (auto i = 1; i < path.strip.size(); ++i) {
    new_edge.strip.push_back(path.strip[i]);
    new_edge.lerps.push_back(path.lerps[i - 1]);
  }

  new_edge.end = path.end;

  if (!is_angle_valid(mesh, new_edge, polygon.vertices[(ix + 2) % polygon.n])) {
    return;
  }
  if (!is_angle_valid(mesh, polygon.vertices[(ix + polygon.n - 1) % polygon.n],
                      new_edge)) {
    return;
  }

  if (!is_angle_valid(mesh, new_edge.end,
                      polygon.edges[(ix + 2) % polygon.n])) {
    return;
  }
  if (!is_angle_valid(mesh, polygon.edges[(ix + polygon.n - 2) % polygon.n],
                      new_edge.start)) {
    return;
  }

  set_vertex(mesh, polygon, ix, new_edge.start);
  set_vertex(mesh, polygon, (ix + 1) % polygon.n, new_edge.end);
  set_edge(mesh, polygon, ix, new_edge);
  update_edge(mesh, polygon, (ix + 1) % polygon.n);
  update_edge(mesh, polygon, (ix + polygon.n - 1) % polygon.n);

  update_angle_bisectors(mesh, polygon);
  update_angles(mesh, polygon);
}

void extend_edge(const bezier_mesh &mesh, Polygon &polygon, const int &ix,
                 const float &d, bool from_end) {
  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;
  auto &adjacencies = mesh.adjacencies;

  auto edge = polygon.edges[ix];
  auto curr_length = polygon.edges_length[ix];

  if (d <= 0) {
    return;
  }

  if (from_end) {
    auto dir =
        direction_in_tangent_space(triangles, positions, edge.end,
                                   -geodesic_path_direction(mesh, edge, true));
    auto path = straightest_geodesic_path(mesh, edge.end, dir, d);

    edge.end = path.end;

    for (auto i = 1; i < path.strip.size(); ++i) {
      edge.strip.push_back(path.strip[i]);
      edge.lerps.push_back(path.lerps[i - 1]);
    }

    if (!is_angle_valid(mesh, edge, polygon.vertices[(ix + 2) % polygon.n])) {
      return;
    }
    if (!is_angle_valid(mesh, edge.end, polygon.vertices[(ix + 2) % polygon.n],
                        polygon.vertices[(ix + 3) % polygon.n])) {
      return;
    }

    set_vertex(mesh, polygon, (ix + 1) % polygon.n, edge.end);
    set_edge(mesh, polygon, ix, edge);
    update_edge(mesh, polygon, (ix + 1) % polygon.n);
  } else {
    auto dir = direction_in_tangent_space(triangles, positions, edge.start,
                                          -geodesic_path_direction(mesh, edge));

    auto path = reverse(straightest_geodesic_path(mesh, edge.start, dir, d));

    path.end = edge.end;

    for (auto i = 1; i < edge.strip.size(); ++i) {
      path.strip.push_back(edge.strip[i]);
      path.lerps.push_back(edge.lerps[i - 1]);
    }

    if (!is_angle_valid(
            mesh, polygon.vertices[(ix + polygon.n - 1) % polygon.n], path)) {
      return;
    }
    if (!is_angle_valid(
            mesh, polygon.vertices[(ix + polygon.n - 2) % polygon.n],
            polygon.vertices[(ix + polygon.n - 1) % polygon.n], path.start)) {
      return;
    }

    set_vertex(mesh, polygon, ix, path.start);
    set_edge(mesh, polygon, ix, path);
    update_edge(mesh, polygon, (ix + polygon.n - 1) % polygon.n);
  }

  update_angle_bisectors(mesh, polygon);
  update_angles(mesh, polygon);
}

template <typename T>
inline void resize_vector(vector<T> &vec, int first, int last) {
  if (first < 0 || last > vec.size()) {
    return;
  }

  vector<T> new_vec(vec.begin() + first, vec.begin() + last);

  vec = new_vec;
}

void retract_edge(const bezier_mesh &mesh, Polygon &polygon, const int &ix,
                  const float &d) {
  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;
  auto &adjacencies = mesh.adjacencies;

  auto edge = polygon.edges[ix];
  auto curr_length = polygon.edges_length[ix];

  auto length = curr_length - d <= 0 ? curr_length - 0.01F : d;

  if (length <= 0) {
    return;
  }

  auto points = path_positions(edge, mesh);

  auto start = 0, end = (int)points.size();

  auto l = length / 2;

  for (auto i = points.size() - 1; i > 0; --i) {
    auto dist = distance(points[i - 1], points[i]);
    l -= dist;

    if (l <= 0) {
      end = i;
      edge.end =
          eval_mesh_point(triangles, positions, edge.strip[end - 1],
                          lerp(points[i], points[i - 1], (dist + l) / dist));

      break;
    }
  }

  l = length / 2;

  for (auto i = 1; i < points.size(); ++i) {
    auto dist = distance(points[i - 1], points[i]);
    l -= dist;

    if (l <= 0) {
      start = i - 1;
      edge.start =
          eval_mesh_point(triangles, positions, edge.strip[start],
                          lerp(points[i - 1], points[i], (dist + l) / dist));

      break;
    }
  }

  resize_vector(edge.strip, start, end);
  resize_vector(edge.lerps, start, end - 1);

  if (!is_angle_valid(mesh, edge, polygon.vertices[(ix + 2) % polygon.n])) {
    return;
  }
  if (!is_angle_valid(mesh, polygon.vertices[(ix + polygon.n - 1) % polygon.n],
                      edge)) {
    return;
  }

  if (!is_angle_valid(mesh, edge.end, polygon.edges[(ix + 2) % polygon.n])) {
    return;
  }
  if (!is_angle_valid(mesh, polygon.edges[(ix + polygon.n - 2) % polygon.n],
                      edge.start)) {
    return;
  }

  set_vertex(mesh, polygon, ix, edge.start);
  set_vertex(mesh, polygon, (ix + 1) % polygon.n, edge.end);
  set_edge(mesh, polygon, ix, edge);
  update_edge(mesh, polygon, (ix + 1) % polygon.n);
  update_edge(mesh, polygon, (ix + polygon.n - 1) % polygon.n);

  update_angle_bisectors(mesh, polygon);
  update_angles(mesh, polygon);
}

void retract_edge(const bezier_mesh &mesh, Polygon &polygon, const int &ix,
                  const float &d, bool from_end) {
  auto &triangles = mesh.triangles;
  auto &positions = mesh.positions;
  auto &adjacencies = mesh.adjacencies;

  auto edge = polygon.edges[ix];
  auto curr_length = polygon.edges_length[ix];

  auto length = d;

  if (curr_length - length <= 0) {
    length = curr_length - 0.01F;
  }

  if (length <= 0) {
    return;
  }

  auto points = path_positions(edge, mesh);

  if (from_end) {
    for (auto i = points.size() - 1; i > 0; --i) {
      auto dist = distance(points[i - 1], points[i]);
      length -= dist;

      if (length <= 0) {
        auto p = lerp(points[i], points[i - 1], (dist + length) / dist);

        resize_vector(edge.strip, 0, i);
        resize_vector(edge.lerps, 0, i - 1);
        edge.end = eval_mesh_point(triangles, positions, edge.strip.back(), p);

        break;
      }
    }

    if (!is_angle_valid(mesh, edge, polygon.vertices[(ix + 2) % polygon.n])) {
      return;
    }
    if (!is_angle_valid(mesh, edge.end, polygon.vertices[(ix + 2) % polygon.n],
                        polygon.vertices[(ix + 3) % polygon.n])) {
      return;
    }

    set_vertex(mesh, polygon, (ix + 1) % polygon.n, edge.end);
    set_edge(mesh, polygon, ix, edge);
    update_edge(mesh, polygon, (ix + 1) % polygon.n);
  } else {
    for (auto i = 1; i < points.size(); ++i) {
      auto dist = distance(points[i - 1], points[i]);
      length -= dist;

      if (length <= 0) {
        auto p = lerp(points[i - 1], points[i], (dist + length) / dist);

        resize_vector(edge.strip, i - 1, edge.strip.size());
        resize_vector(edge.lerps, i - 1, edge.lerps.size());
        edge.start =
            eval_mesh_point(triangles, positions, edge.strip.front(), p);

        break;
      }
    }

    if (!is_angle_valid(
            mesh, polygon.vertices[(ix + polygon.n - 1) % polygon.n], edge)) {
      return;
    }
    if (!is_angle_valid(
            mesh, polygon.vertices[(ix + polygon.n - 2) % polygon.n],
            polygon.vertices[(ix + polygon.n - 1) % polygon.n], edge.start)) {
      return;
    }

    set_vertex(mesh, polygon, ix, edge.start);
    set_edge(mesh, polygon, ix, edge);
    update_edge(mesh, polygon, (ix + polygon.n - 1) % polygon.n);
  }

  update_angle_bisectors(mesh, polygon);
  update_angles(mesh, polygon);
}

mesh_point select_vertex(const mesh_point &point,
                         const vector<vec3i> &triangles,
                         const vector<vec3f> &positions, Polygon &polygon) {
  auto closest = mesh_point();

  if (polygon.selected_vertex >= 0) {
    polygon.selected_shapes_ids.erase(
        polygon.shapes["vertices"][polygon.selected_vertex].id);
  }

  polygon.selected_vertex = -1;

  if (polygon.n_vertices <= 0) {
    return closest;
  }

  if (point.face != -1) {
    auto epsilon = flt_eps * 5e4F;
    auto min_dist = flt_max;

    for (auto i = 0; i < polygon.n_vertices; ++i) {
      auto v = polygon.vertices[i];
      auto dist = distance(triangles, positions, point, v);

      if (dist < epsilon && dist < min_dist) {
        min_dist = dist;
        closest = v;
        polygon.selected_shapes_ids.insert(polygon.shapes["vertices"][i].id);
        polygon.selected_vertex = i;
      }
    }
  }

  return closest;
}

mesh_point select_vertex(Polygon &polygon, int ix) {
  auto point = mesh_point();

  if (polygon.selected_vertex >= 0) {
    polygon.selected_shapes_ids.erase(
        polygon.shapes["vertices"][polygon.selected_vertex].id);
  }

  polygon.selected_vertex = -1;

  if (polygon.n_vertices <= 0 || ix < 0 || ix >= polygon.n_vertices) {
    return point;
  }

  point = polygon.vertices[ix];
  polygon.selected_shapes_ids.insert(polygon.shapes["vertices"][ix].id);
  polygon.selected_vertex = ix;

  return point;
}

geodesic_path select_edge(Polygon &polygon, int ix) {
  auto path = geodesic_path();

  if (polygon.selected_edge >= 0) {
    polygon.selected_shapes_ids.erase(
        polygon.shapes["edges"][polygon.selected_edge].id);
  }

  polygon.selected_edge = -1;

  if (polygon.n_vertices < 2 || ix < 0 || ix >= polygon.n_vertices ||
      polygon.n_vertices < polygon.n && ix > polygon.n_vertices - 2) {
    return path;
  }

  path = polygon.edges[ix];
  polygon.selected_shapes_ids.insert(polygon.shapes["edges"][ix].id);
  polygon.selected_edge = ix;

  return path;
}