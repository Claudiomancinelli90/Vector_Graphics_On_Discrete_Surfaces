#include "splineio.h"

#include "logging.h"
#include <yocto/yocto_commonio.h>

// -----------------------------------------------------------------------------
// JSON SUPPORT
// -----------------------------------------------------------------------------

// support for json conversions

#define HEAVY 1
#if HEAVY
void init_mesh(bezier_mesh &mesh, bool with_opposite) {
  mesh.v2t =
      vertex_to_triangles(mesh.triangles, mesh.positions, mesh.adjacencies);

  auto t0 = logging::time_begin("Computing graph");
  // mesh.solver = make_geodesic_solver(mesh.triangles, mesh.positions,
  //                                    mesh.adjacencies, mesh.v2t);

  // mesh.angles = compute_angles(mesh.triangles, mesh.positions,
  // mesh.adjacencies,
  //                              mesh.v2t, mesh.total_angles, true);
  extended_solver(mesh, 3);
  logging::time_end(t0);
  // sort(curvature.begin(), curvature.end());
  // std::cout << pif / yocto::sqrt(curvature.back()) << std::endl;

  // mesh.avg_edge_length = (float)igl::avg_edge_length(V, F);
  // std::tie(mesh.Grad, mesh.Lap) = init_differential_operators(
  //     mesh.solver, mesh.angles, mesh.positions, mesh.normals);
  // std::tie(mesh.Grad, mesh.Lap, mesh.a, mesh.christoffel) =
  //     init_discrete_diff_op(mesh.solver, mesh.angles, mesh.positions,
  //                           mesh.normals);
  auto max_curv = 0.f;
  // std::tie(mesh.Grad, mesh.Lap, mesh.a, mesh.christoffel, mesh.g_inv,
  // mesh.ring,
  //          mesh.e, max_curv) =
  //     init_discrete_diff_op_2(mesh.solver, mesh.angles, mesh.total_angles,
  //                             mesh.triangles, mesh.positions,
  //                             mesh.adjacencies, mesh.v2t, mesh.normals);
}
#endif

#include "stl_reader.h"

void load_mesh_stl(bezier_mesh &mesh, const string &filename) {
  std::vector<float> coords, normals;
  std::vector<unsigned int> tris, solids;

  stl_reader::ReadStlFile(filename.c_str(), coords, normals, tris, solids);
  const size_t numTris = tris.size() / 3;
  for (size_t itri = 0; itri < numTris; ++itri) {
    for (size_t icorner = 0; icorner < 3; ++icorner) {
      float *c = &coords[3 * tris[3 * itri + icorner]];
    }

    float *n = &normals[3 * itri];
  }
  mesh.positions.assign((vec3f *)coords.data(),
                        (vec3f *)(&coords[0] + coords.size()));
  for (auto &p : mesh.positions) {
    std::swap(p.y, p.z);
    std::swap(p.x, p.z);
  }
  mesh.triangles.assign((vec3i *)tris.data(),
                        (vec3i *)(&tris[0] + tris.size()));
  mesh.normals.assign((vec3f *)normals.data(),
                      (vec3f *)(&normals[0] + normals.size()));
}

bool load_mesh(const string &filename, bezier_mesh &mesh, string &error) {
  mesh = bezier_mesh{};

  vector<int> points;
  vector<vec2i> lines;
  vector<vec3i> triangles;
  vector<vec4i> quads;
  vector<vec4i> quadspos;
  vector<vec4i> quadsnorm;
  vector<vec4i> quadstexcoord;
  vector<vec3f> positions;
  vector<vec3f> normals;
  vector<vec4f> colors;
  vector<float> radius;

#if 0
  mesh.positions = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
  mesh.triangles = {{0, 1, 2}, {0, 2, 3}};
  mesh.normals   = {{0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}};
  mesh.texcoords = {{0, 1}, {1, 1}, {1, 0}, {0, 0}};
#else

  auto ext = path_extension(filename);
  if (ext == ".stl") {
    load_mesh_stl(mesh, filename);
  } else {
    if (!load_shape(filename, points, lines, mesh.triangles, quads, quadspos,
                    quadsnorm, quadstexcoord, mesh.positions, normals,
                    mesh.texcoords, colors, radius, error, false)) {
      return false;
    }
  }
  if (quads.size()) {
    mesh.triangles = quads_to_triangles(quads);
  }
  printf("%s: mesh has %ld  triangle\n", __FUNCTION__, mesh.triangles.size());
#endif

  // bumped_sphere(0.0001f, mesh.positions);
  // Normalize positions in the cube [-1, 1]^3
  auto bbox = invalidb3f;
  for (auto &p : mesh.positions)
    bbox = merge(bbox, p);
  auto center = (bbox.max + bbox.min) / 2;
  auto scale = 1.0f / max(bbox.max - bbox.min);
  for (auto &p : mesh.positions)
    p = (p - center) * scale;

  mesh.adjacencies = face_adjacencies(mesh.triangles);
#if 0
  close_pole(mesh.positions, mesh.triangles, mesh.adjacencies);
#endif
  mesh.normals = compute_normals(mesh.triangles, mesh.positions);

  mesh.dual_solver = make_dual_geodesic_solver(mesh.triangles, mesh.positions,
                                               mesh.adjacencies);
  // mesh.e2v = make_edge_map(mesh.triangles);
#if HEAVY
  init_mesh(mesh, true);
#endif
  return true;
}
