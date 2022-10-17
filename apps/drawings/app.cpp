#include "app.h"

#include <yocto/yocto_color.h>
#include <yocto/yocto_commonio.h>
#include <yocto/yocto_modelio.h>
#include <yocto/yocto_shape.h>

#include "logging.h"

void init_bvh(App_base &app) {
  app.bvh = make_triangles_bvh(app.mesh.triangles, app.mesh.positions, {});
}

shade_camera _make_lookat_camera(const vec3f &from, const vec3f &to,
                                 const vec3f &up = {0, 1, 0}) {
  auto camera = shade_camera{};
  camera.frame = lookat_frame(from, to, {0, 1, 0});
  camera.focus = length(from - to);
  return camera;
}

shade_camera _make_framing_camera(const vector<vec3f> &positions) {
  auto direction = vec3f{0, 1, 2};
  auto box = bbox3f{};
  for (auto &p : positions) {
    expand(box, p);
  }
  auto box_center = center(box);
  auto box_size = max(size(box));
  return _make_lookat_camera(direction * box_size + box_center, box_center);
}

void init_camera(App_base &app, const vec3f &from, const vec3f &to) {
  *app.ogl_camera = _make_framing_camera(app.mesh.positions);
  app.camera_focus = app.ogl_camera->focus;
}
void set_points_shape(ogl_shape *shape, const vector<vec3f> &positions) {
  auto sphere = make_uvsphere({16, 16}, 1, {1, 1});

  set_vertex_buffer(shape, sphere.positions, 0);
  set_index_buffer(shape, quads_to_triangles(sphere.quads));

  set_vertex_buffer(shape, positions, 1);
  set_instance_buffer(shape, 1, true);
}
void set_points_shape(ogl_shape *shape, const bezier_mesh &mesh,
                      const vector<mesh_point> &points) {
  auto pos = vector<vec3f>(points.size());
  for (int i = 0; i < points.size(); i++) {
    pos[i] = eval_position(mesh.triangles, mesh.positions, points[i]);
  }
  set_points_shape(shape, pos);
}

void set_polyline_shape(ogl_shape *shape, const vector<vec3f> &positions) {
  if (positions.empty())
    return;
  auto cylinder = make_uvcylinder({16, 1, 1}, {1, 1});
  for (auto &p : cylinder.positions) {
    p.z = p.z * 0.5 + 0.5;
  }

  set_vertex_buffer(shape, cylinder.positions, 0);
  set_vertex_buffer(shape, cylinder.normals, 1);
  set_index_buffer(shape, quads_to_triangles(cylinder.quads));

  auto froms = vector<vec3f>();
  auto tos = vector<vec3f>();
  auto colors = vector<vec3f>();
  froms.reserve(positions.size() - 1);
  tos.reserve(positions.size() - 1);
  // colors.reserve(positions.size() - 1);
  for (int i = 0; i < positions.size() - 1; i++) {
    if (positions[i] == positions[i + 1])
      continue;
    froms.push_back(positions[i]);
    tos.push_back(positions[i + 1]);
    // colors.push_back({sinf(i) * 0.5f + 0.5f, cosf(i) * 0.5f + 0.5f, 0});
  }

  // TODO(giacomo): solve rendering bug with degenerate polyline
  if (froms.empty()) {
    shape->num_instances = 0;
    set_vertex_buffer(shape, {}, 0);
    set_vertex_buffer(shape, {}, 1);
    set_vertex_buffer(shape, {}, 2);
    set_vertex_buffer(shape, {}, 3);
    set_index_buffer(shape, vector<vec3i>{});
  } else {
    set_vertex_buffer(shape, froms, 2);
    set_instance_buffer(shape, 2, true);
    set_vertex_buffer(shape, tos, 3);
    set_instance_buffer(shape, 3, true);
  }
}
void set_mesh_shape(ogl_shape *shape, const vector<vec3i> &triangles,
                    const vector<vec3f> &positions,
                    const vector<vec3f> &normals) {
  set_vertex_buffer(shape, positions, 0);
  set_vertex_buffer(shape, normals, 1);
  set_index_buffer(shape, triangles);
}

void _set_polyline_shape(ogl_shape *shape, const vector<vec3f> &positions,
                         const vector<vec3f> &normals) {
  set_vertex_buffer(shape, positions, 0);
  if (normals.size()) {
    set_vertex_buffer(shape, normals, 1);
  }
  shape->elements = ogl_element_type::lines;
}
void update_path_shape(shade_shape *shape, const bezier_mesh &mesh,
                       const vector<vec3f> &positions, float radius) {
  // if (thin) {
  //   set_positions(shape, positions);
  //   shape->shape->elements = ogl_element_type::line_strip;
  //   set_instances(shape, {});
  //   return;
  // }

  auto frames = vector<frame3f>();
  frames.reserve(positions.size() - 1);
  //  froms.reserve(positions.size() - 1);
  //  tos.reserve(positions.size() - 1);
  for (int i = 0; i < positions.size() - 1; i++) {
    auto from = positions[i];
    auto to = positions[i + 1];
    if (from == to)
      continue;

    auto frame = frame_fromz(from, normalize(to - from));
    frame.z *= length(to - from);
    frames.push_back(frame);
  }

  auto cylinder = make_uvcylinder({8, 1, 1}, {radius, 1});
  for (auto &p : cylinder.positions) {
    p.z = p.z * 0.5 + 0.5;
  }

  set_quads(shape, cylinder.quads);
  set_positions(shape, cylinder.positions);
  set_normals(shape, cylinder.normals);
  set_texcoords(shape, cylinder.texcoords);
  set_colors(shape, {});
  set_instances(shape, frames);
}
void update_path_shape(const vector<Added_Path *> &paths,
                       const bezier_mesh &mesh, const vector<vec3f> &positions,
                       const float &radius, const int entry) {

  auto &shape = paths[entry]->instance->shape;
  auto frames = vector<frame3f>();
  frames.reserve(positions.size() - 1);
  for (int i = 0; i < positions.size() - 1; i++) {
    auto from = positions[i];
    auto to = positions[i + 1];
    if (from == to)
      continue;

    auto frame = frame_fromz(from, normalize(to - from));
    frame.z *= length(to - from);
    frames.push_back(frame);
  }

  auto cylinder = make_uvcylinder({8, 1, 1}, {radius, 1});
  for (auto &p : cylinder.positions) {
    p.z = p.z * 0.5 + 0.5;
  }

  set_quads(shape, cylinder.quads);
  set_positions(shape, cylinder.positions);
  set_normals(shape, cylinder.normals);
  set_texcoords(shape, cylinder.texcoords);
  set_colors(shape, {});
  set_instances(shape, frames);
  paths[entry]->positions = positions;
  paths[entry]->radius = radius;
}
void update_path_shape(const vector<Added_Path *> &paths,
                       const bezier_mesh &mesh, const float &radius,
                       const vector<int> &entries) {
  for (auto i = 0; i < entries.size(); ++i) {
    auto positions = paths[entries[i]]->positions;
    update_path_shape(paths, mesh, positions, radius, entries[i]);
  }
}
void update_path_shape(const vector<Added_Path *> &paths,
                       const bezier_mesh &mesh, const vec3f &color,
                       const int entry) {
  // shade_material *material = {};
  // material->emission = vec3f{0, 0, 0};
  // material->metallic = 0.f;
  // material->roughness = 0.4;
  // material->specular = 1;
  // material->color = color;
  paths[entry]->instance->material->color = color;
}
void update_path_shape(const vector<Added_Path *> &paths,
                       const bezier_mesh &mesh, const vec3f &color,
                       const vector<int> &entries) {

  for (auto entry : entries)
    paths[entry]->instance->material->color = color;
}
void update_path_shape(shade_shape *shape, const bezier_mesh &mesh,
                       const vector<vec3f> &positions, float radius,
                       const float &treshold) {
  // if (thin) {
  //   set_positions(shape, positions);
  //   shape->shape->elements = ogl_element_type::line_strip;
  //   set_instances(shape, {});
  //   return;
  // }

  auto frames = vector<frame3f>();
  frames.reserve(positions.size() - 1);
  //  froms.reserve(positions.size() - 1);
  //  tos.reserve(positions.size() - 1);
  for (int i = 0; i < positions.size() - 1; i++) {
    auto from = positions[i];
    auto to = positions[i + 1];
    if (from == to || length(to - from) >= treshold)
      continue;

    auto frame = frame_fromz(from, normalize(to - from));
    frame.z *= length(to - from);
    frames.push_back(frame);
  }

  auto cylinder = make_uvcylinder({8, 1, 1}, {radius, 1});
  for (auto &p : cylinder.positions) {
    p.z = p.z * 0.5 + 0.5;
  }

  set_quads(shape, cylinder.quads);
  set_positions(shape, cylinder.positions);
  set_normals(shape, cylinder.normals);
  set_texcoords(shape, cylinder.texcoords);
  set_colors(shape, {});
  set_instances(shape, frames);
}
void update_path_shape(shade_shape *shape, const bezier_mesh &mesh,
                       const geodesic_path &path, float radius) {
  auto positions =
      path_positions(path, mesh.triangles, mesh.positions, mesh.adjacencies);
  update_path_shape(shape, mesh, positions, radius);
}

Added_Path *add_path_shape(App_base &app, const geodesic_path &path,
                           float radius, const vec3f &color) {
  auto added_path = app.added_paths.emplace_back(new Added_Path{});
  auto pos = path_positions(path, app.mesh.triangles, app.mesh.positions,
                            app.mesh.adjacencies);
  added_path->positions = pos;
  added_path->color = color;
  added_path->radius = radius;

  auto shape = add_shape(app.scene);
  auto material = add_material(app.scene, {0, 0, 0}, color, 1, 0, 0.4);
  update_path_shape(shape, app.mesh, path, radius);
  added_path->instance =
      add_instance(app.scene, identity3x4f, shape, material, false);

  return added_path;
}
Added_Path *add_path_shape(App_base &app, const vector<vec3f> &positions,
                           float radius, const vec3f &color,
                           const float &threshold) {
  auto added_path = app.added_paths.emplace_back(new Added_Path{});
  added_path->color = color;
  added_path->radius = radius;
  added_path->positions = positions;
  auto shape = add_shape(app.scene);
  auto material = add_material(app.scene, {0, 0, 0}, color, 1, 0, 0.4);
  update_path_shape(shape, app.mesh, positions, radius, threshold);
  added_path->instance =
      add_instance(app.scene, identity3x4f, shape, material, false);

  return added_path;
}

Added_Points *add_points_shape(App_base &app, const vector<mesh_point> &points,
                               float radius, const vec3f &color) {
  auto added_points = app.added_points.emplace_back(new Added_Points{});
  added_points->points = points;
  added_points->color = color;
  added_points->radius = radius;

  auto shape = add_shape(app.scene);
  auto material = add_material(app.scene, {0, 0, 0}, color, 1, 0, 0.4);
  update_points_shape(shape, app.mesh, points, radius);
  added_points->instance =
      add_instance(app.scene, identity3x4f, shape, material, false);
  return added_points;
}
Added_Points *add_points_shape(App_base &app, const vector<vec3f> &points,
                               float radius, const vec3f &color) {
  auto added_points = app.added_points.emplace_back(new Added_Points{});
  // added_points->points = points; TODO(giacomo): fix this
  added_points->color = color;
  added_points->radius = radius;

  auto shape = add_shape(app.scene);
  auto material = add_material(app.scene, {0, 0, 0}, color, 1, 0, 0.4);
  update_points_shape(shape, points, radius);
  added_points->instance =
      add_instance(app.scene, identity3x4f, shape, material, false);
  return added_points;
}

void update_points_shape(shade_shape *shape, const vector<vec3f> &positions,
                         float radius) {
  auto frames = vector<frame3f>(positions.size(), identity3x4f);
  frames.reserve(positions.size() - 1);
  for (int i = 0; i < positions.size(); i++) {
    frames[i].o = positions[i];
  }

  auto sphere = make_sphere(32, radius);

  set_quads(shape, sphere.quads);
  set_positions(shape, sphere.positions);
  set_normals(shape, sphere.normals);
  set_texcoords(shape, sphere.texcoords);
  set_colors(shape, {});
  set_instances(shape, frames);
}

void update_points_shape(shade_shape *shape, const bezier_mesh &mesh,
                         const vector<mesh_point> &points, float radius) {
  auto positions = vector<vec3f>(points.size());
  for (int i = 0; i < positions.size(); i++) {
    positions[i] = eval_position(mesh.triangles, mesh.positions, points[i]);
  }
  update_points_shape(shape, positions, radius);
}
void update_glvector_field(App_base &app, const vector<vec3f> &vector_field,
                           float scale, const string &name) {
  delete_shape(app.shapes[name]);
  auto &triangles = app.mesh.triangles;
  auto &positions = app.mesh.positions;
  if (vector_field.size() == triangles.size()) {
    app.shapes[name] =
        make_vector_field_shape(vector_field, triangles, positions, scale);
  } else {
    app.shapes[name] = make_vector_field_shape(vector_field, positions, scale);
  }
}

void update_glpoints(App_base &app, const vector<vec3f> &positions,
                     const string &name) {
  delete_shape(app.shapes[name]);
  app.shapes[name] = make_points_shape(positions);
}

void update_glpoints(App_base &app, const vector<mesh_point> &points,
                     const string &name) {
  delete_shape(app.shapes[name]);
  if (points.empty())
    return;
  auto positions = vector<vec3f>(points.size());
  auto colors = vector<vec3f>(points.size(), {1, 1, 1});
  for (int i = 0; i < positions.size(); i++) {
    positions[i] =
        eval_position(app.mesh.triangles, app.mesh.positions, points[i]);
  }
  colors[0] = {0, 0.7, 0};
  if (points.size() >= 2)
    colors[1] = {0.7, 0, 0};
  app.shapes[name] = make_points_shape(positions);
  add_vertex_attribute(app.shapes[name], colors);
}

// void update_glpatch(App_base &app, const vector<int> &faces,
//                     const string &name) {
//   auto positions = vector<vec3f>();
//   for (int i = 0; i < faces.size(); ++i) {
//     auto t = app.mesh.triangles[faces[i]];
//     append(positions, app.mesh.positions[t.x]);
//     append(positions, app.mesh.positions[t.y]);
//     append(positions, app.mesh.positions[t.z]);
//   }
//   delete_shape(app.shapes[name]);
//   init_shape(app.shapes[name]);
//   add_vertex_attribute(app.shapes[name], positions);
//   app.shapes[name].type = Shape::type::triangles;
//   app.shapes[name].is_strip = false;
// }

void update_glpolyline(App_base &app, const vector<vec3f> &vertices,
                       const string &name) {
  delete_shape(app.shapes[name]);
  app.shapes[name] = make_polyline_shape(vertices, {});
}

void update_glcolors(App_base &app, const vector<vec3f> &colors) {
  set_vertex_attribute(app.shapes["mesh"], 2, colors);
}

void update_glcolors(App_base &app, const vector<float> &f) {
  auto colors = vector<vec3f>(f.size());
  for (int i = 0; i < f.size(); i++) {
    colors[i] = vec3f{f[i], f[i], f[i]};
  }
  update_glcolors(app, colors);
}

vector<vec3f> make_normals(const vector<vec3i> &triangles,
                           const vector<vec3f> &positions) {
  auto normals = vector<vec3f>{positions.size()};
  for (auto &normal : normals)
    normal = zero3f;
  for (auto &t : triangles) {
    auto normal =
        cross(positions[t.y] - positions[t.x], positions[t.z] - positions[t.x]);
    normals[t.x] += normal;
    normals[t.y] += normal;
    normals[t.z] += normal;
  }
  for (auto &normal : normals)
    normal = normalize(normal);
  return normals;
}
ray3f camera_ray(const App_base &app, vec2f mouse) {
  auto camera_ray = [](const frame3f &frame, float lens, const vec2f &film,
                       const vec2f &image_uv) {
    auto e = zero3f;
    auto q =
        vec3f{film.x * (0.5f - image_uv.x), film.y * (image_uv.y - 0.5f), lens};
    auto q1 = -q;
    auto d = normalize(q1 - e);
    auto ray = ray3f{transform_point(frame, e), transform_direction(frame, d)};
    return ray;
  };

  mouse += 1;
  mouse /= 2;
  mouse.y = 1 - mouse.y;
  auto &camera = *app.ogl_camera;
  return camera_ray(camera.frame, camera.lens,
                    {camera.film, camera.film / camera.aspect}, mouse);
}

vec2f screenspace_from_worldspace(App_base &app, const vec3f &position) {
  auto [x, y, z] = position;
  auto uv4f = app.matrices.projection_view * vec4f{x, y, z, 1};
  return vec2f{uv4f.x / uv4f.w, uv4f.y / uv4f.w};
};
void init_app(App_base &app, string filename) {
  // init shape
  vector<int> points;
  vector<vec2i> lines;
  vector<vec3i> triangles;
  vector<vec4i> quads;
  vector<vec4i> quadspos;
  vector<vec4i> quadsnorm;
  vector<vec4i> quadstexcoord;
  vector<vec3f> positions;
  vector<vec3f> normals;
  vector<vec2f> texcoords;
  vector<vec4f> colors;
  vector<float> radius;

  auto ioerror = string{};
  if (!load_shape(filename, points, lines, app.mesh.triangles, quads, quadspos,
                  quadsnorm, quadstexcoord, app.mesh.positions, normals,
                  texcoords, colors, radius, ioerror, false))
    print_fatal(ioerror);

  bbox3f bbox;
  for (auto &p : app.mesh.positions)
    expand(bbox, p);
  vec3f center = (bbox.max + bbox.min) * 0.5f;
  float scale = 1.0f / max(bbox.max - bbox.min);
  for (auto &p : app.mesh.positions)
    p = (p - center) * scale;
  init_bvh(app);
  init_camera(app);
}

mesh_point intersect_mesh(const App_base &app, vec2f mouse) {
  auto ray = camera_ray(app, mouse);
  auto isec = intersect_triangles_bvh(app.bvh, app.mesh.triangles,
                                      app.mesh.positions, ray);

  if (isec.hit) {
    return mesh_point{isec.element, isec.uv};
  } else {
    return {-1, {0, 0}};
  }
}
bool load_program(ogl_program *program, const string &vertex_filename,
                  const string &fragment_filename) {
  auto error = ""s;
  auto vertex_source = ""s;
  auto fragment_source = ""s;

  if (!load_text(vertex_filename, vertex_source, error)) {
    printf("error loading vertex shader (%s): \n%s\n", vertex_filename.c_str(),
           error.c_str());
    return false;
  }
  if (!load_text(fragment_filename, fragment_source, error)) {
    printf("error loading fragment shader (%s): \n%s\n",
           fragment_filename.c_str(), error.c_str());
    return false;
  }

  auto error_buf = ""s;
  if (!set_program(program, vertex_source, fragment_source, error, error_buf)) {
    printf("\nerror: %s\n", error.c_str());
    printf("    %s\n", error_buf.c_str());
    return false;
  }
  return true;
}

void init_gpu(App_base &app, bool envlight) {
  string error;
  init_ogl(error);

  app.scene = new shade_scene{};

  app.ogl_camera = add_camera(app.scene);
  // app.scene->cameras.push_back(&app.camera);
  app.lines_material = add_material(app.scene, {0, 0, 0}, {1, 0, 0}, 1, 0, 0.4);

  init_camera(app);

  // Init opengl mesh
  auto &mesh = app.mesh;
  // auto  colors     = bake_ambient_occlusion(app, 1024);
  app.mesh_shape = add_shape(app.scene, {}, {}, mesh.triangles, {},
                             mesh.positions, mesh.normals, {}, {});
  app.mesh_material =
      add_material(app.scene, {0, 0, 0}, {0.9, 0.9, 0.9}, 0.04, 0, 0.5);
  // set_opacity(app.mesh_material, 0.2);
  // app.mesh_material = add_material(
  //     app.scene, {0, 0, 0}, {0.9, 0.724, 0.27}, 0.04, 0, 0.4);
  add_instance(app.scene, identity3x4f, app.mesh_shape, app.mesh_material);

  app.m_shade_params.hide_environment = true;
  app.m_shade_params.exposure = -0.5;
  app.m_shade_params.background = {1, 1, 1, 1};
  app.m_shade_params.lighting =
      envlight ? shade_lighting_type::envlight : shade_lighting_type::eyelight;

  init_scene(app.scene, true);

  // setup IBL
  if (envlight) {
    auto img = image<vec4f>{};
    load_image("data/uffizi.hdr", img, error);
    auto texture = new shade_texture{};
    set_texture(texture, img, true, true, true);
    auto environment = add_environment(app.scene);
    set_emission(environment, {1, 1, 1}, texture);
    init_environments(app.scene);
  }

  // Init shaders.
  auto base = string(SHADERS_PATH); // defined in parent CMakeLists.txt

  load_program(&app.ogl_shaders["mesh"], base + "mesh.vert",
               base + "mesh.frag");
  load_program(&app.ogl_shaders["lines"], base + "points.vert",
               base + "points.frag");

  if (envlight) {
    load_program(&app.ogl_shaders["points"], base + "sphere.vert",
                 base + "sphere-envlight.frag");
    load_program(&app.ogl_shaders["polyline"], base + "polyline.vert",
                 base + "polyline-envlight.frag");
  } else {
    load_program(&app.ogl_shaders["points"], base + "sphere.vert",
                 base + "sphere-eyelight.frag");
    load_program(&app.ogl_shaders["polyline"], base + "polyline.vert",
                 base + "polyline-eyelight.frag");
  }
  load_program(&app.ogl_shaders["flat"], base + "flat.vert",
               base + "flat.frag");

  app.shaders["points"] =
      gpu::make_shader_from_file(base + "points.vert", base + "points.frag");
  app.shaders["mesh"] =
      gpu::make_shader_from_file(base + "mesh.vert", base + "mesh.frag");
  // init edge shape
  auto surface_offset = 0.00002f;
  auto positions = mesh.positions;
  for (int i = 0; i < mesh.positions.size(); i++) {
    positions[i] += mesh.normals[i] * surface_offset;
  }
  auto edges = vector<vec2i>();
  edges.reserve(mesh.positions.size() * 3);
  for (auto &t : mesh.triangles) {
    if (t.x < t.y)
      edges.push_back({t.x, t.y});
    if (t.y < t.z)
      edges.push_back({t.y, t.z});
    if (t.z < t.x)
      edges.push_back({t.z, t.x});
  }
  set_vertex_buffer(&app.edges_shape, positions, 0);
  set_index_buffer(&app.edges_shape, edges);
}

void delete_app(App_base &app) {
  for (auto &[name, shape] : app.shapes) {
    delete_shape(shape);
  }
  for (auto &[name, shader] : app.shaders) {
    delete_shader(shader);
  }
}

void set_vertices_number(App_base &app) {
  change_polygon(app.polygon, app.n, app.mesh);
  app.angles_eq.assign(app.n, false);
  app.next_angles.assign(app.n, 0.0F);
  app.next_lengths.assign(app.n, 0.0F);
  app.move_vertices.assign(app.n, {0, 0});
}

void reset_polygon(App_base &app) {
  reset_polygon(app.polygon);
  app.angles_eq.assign(app.n, false);
  app.next_angles.assign(app.n, 0.0F);
  app.next_lengths.assign(app.n, 0.0F);
  app.move_vertices.assign(app.n, {0, 0});
}

void clear_mesh(App_base &app) { reset_polygon(app); }
pair<vector<vec3f>, vector<vec3f>>
points_pos_and_colors(const bezier_mesh &mesh,
                      const vector<Added_Points *> &points) {
  auto pos = vector<vec3f>{};
  auto colors = vector<vec3f>{};
  for (auto point : points) {
    auto curr_pos = vector<vec3f>(point->points.size());
    auto curr_color = vector<vec3f>(point->points.size(), point->color);
    for (auto i = 0; i < point->points.size(); ++i) {
      curr_pos[i] =
          eval_position(mesh.triangles, mesh.positions, point->points[i]);
    }
    pos.insert(pos.end(), curr_pos.begin(), curr_pos.end());
    colors.insert(colors.end(), curr_color.begin(), curr_color.end());
  }
  return {pos, colors};
}
// void export_scene(App& app, const string& filename, const bool export_edges,
//     const vector<vec3f>& spline_colors) {
//   auto final_pos     = app.mesh.positions;
//   auto final_normals = app.mesh.normals;
//   auto final_tri     = app.mesh.triangles;
//   auto final_colors = vector<vec3f>(final_pos.size(),
//   app.mesh_material->color); auto size         =
//   (int)app.mesh.positions.size(); auto point_pos    = vector<vec3f>{};

//   auto spline_c = spline_colors;
//   if (spline_c.size() != app.splines().size()) {
//     spline_c = vector<vec3f>(app.splines().size(), vec3f{1, 0, 0});
//     // std::cout << "wrong number of splines" << std::endl;
//     printf("number of splines is %d while number of colors is %d",
//         (int)app.splines().size(), (int)spline_c.size());
//   }

//   for (auto i = 0; i < app.splines().size(); ++i) {
//     auto& spline = app.splines()[i];
//     for (auto curve : spline.curves) {
//       auto quads   = vector<vec4i>{};
//       auto pos     = vector<vec3f>{};
//       auto normals = vector<vec3f>{};
//       auto tex     = vector<vec2f>{};
//       if (app.enable_jumps) {
//         auto len = path_length(curve.positions);
//         for (auto j = 0; j < curve.sampling_rate.size(); ++j) {
//           if (curve.sampling_rate[j] > app.scale_factor * len) continue;
//           auto j0       = curve.offsets[j].x;
//           auto j1       = curve.offsets[j].y;
//           auto curr_pos = vector<vec3f>{};
//           for (auto h = j0; h < j1; ++h) {
//             curr_pos.push_back(curve.positions[h]);
//           }
//           if (curr_pos.size() <= 1) continue;
//           polyline_to_cylinders(
//               quads, pos, normals, tex, curr_pos, 4, 0.003f *
//               app.curve_size);

//           auto tri = quads_to_triangles(quads);
//           merge_triangles(final_tri, tri, size);
//           final_pos.insert(final_pos.end(), pos.begin(), pos.end());
//           final_normals.insert(
//               final_normals.end(), normals.begin(), normals.end());
//           auto curr_colors = vector<vec3f>(pos.size(), spline_c[i]);
//           final_colors.insert(
//               final_colors.end(), curr_colors.begin(), curr_colors.end());
//           size += (int)pos.size();
//         }
//       } else {
//         polyline_to_cylinders(quads, pos, normals, tex, curve.positions, 12,
//             0.003f * app.curve_size);

//         auto tri = quads_to_triangles(quads);
//         merge_triangles(final_tri, tri, size);
//         final_pos.insert(final_pos.end(), pos.begin(), pos.end());
//         final_normals.insert(
//             final_normals.end(), normals.begin(), normals.end());
//         auto curr_colors = vector<vec3f>(pos.size(), spline_c[i]);
//         final_colors.insert(
//             final_colors.end(), curr_colors.begin(), curr_colors.end());
//         size += (int)pos.size();
//       }
//     }
//     for (auto j = 0; j < spline.control_points.size() - 1; ++j) {
//       // auto path = compute_geodesic_path(
//       //     app.mesh, spline.control_points[j], spline.control_points[j +
//       //     1]);
//       // auto quads   = vector<vec4i>{};
//       // auto pos     = vector<vec3f>{};
//       // auto normals = vector<vec3f>{};
//       // auto tex     = vector<vec2f>{};

//       // polyline_to_cylinders(
//       //     quads, pos, normals, tex, path_positions(app.mesh, path), 8,
//       //     0.0020);

//       // auto tri = quads_to_triangles(quads);
//       // merge_triangles(final_tri, tri, size);
//       // final_pos.insert(final_pos.end(), pos.begin(), pos.end());
//       // final_normals.insert(final_normals.end(), normals.begin(),
//       // normals.end()); auto curr_colors = vector<vec3f>(pos.size(), {0, 0,
//       // 1}); final_colors.insert(
//       //     final_colors.end(), curr_colors.begin(), curr_colors.end());
//       // size += (int)pos.size();
//       if (i > 0 && j > 0) continue;
//       point_pos.push_back(eval_position(app.mesh, spline.control_points[j]));
//     }
//     point_pos.push_back(eval_position(app.mesh,
//     spline.control_points.back()));
//   }

//   // for (auto& path : app.added_paths) {
//   //   auto quads   = vector<vec4i>{};
//   //   auto pos     = vector<vec3f>{};
//   //   auto normals = vector<vec3f>{};
//   //   auto tex     = vector<vec2f>{};

//   //   polyline_to_cylinders(
//   //       quads, pos, normals, tex, path->positions, 12, path->radius);
//   //   auto tri = quads_to_triangles(quads);
//   //   merge_triangles(final_tri, tri, size);
//   //   final_pos.insert(final_pos.end(), pos.begin(), pos.end());
//   //   final_normals.insert(final_normals.end(), normals.begin(),
//   //   normals.end()); auto curr_colors = vector<vec3f>(
//   //       pos.size(), path->instance->material->color);
//   //   final_colors.insert(
//   //       final_colors.end(), curr_colors.begin(), curr_colors.end());
//   //   size += (int)pos.size();
//   // }
//   // for (auto points : app.added_points) {
//   //   auto quads   = vector<vec4i>{};
//   //   auto pos     = vector<vec3f>{};
//   //   auto normals = vector<vec3f>{};
//   //   auto tex     = vector<vec2f>{};
//   //   point_pos.resize(points->points.size());
//   //   for (auto j = 0; j < points->points.size(); ++j) {
//   //     point_pos[j] = eval_position(app.mesh, points->points[j]);
//   //   }

//   //   points_to_spheres(quads, pos, normals, tex, point_pos, 4,
//   //   points->radius); for (auto& n : normals) n *= -1; auto tri =
//   //   quads_to_triangles(quads); merge_triangles(final_tri, tri, size);
//   //   final_pos.insert(final_pos.end(), pos.begin(), pos.end());
//   //   final_normals.insert(final_normals.end(), normals.begin(),
//   //   normals.end()); auto curr_colors = vector<vec3f>(
//   //       pos.size(), points->instance->material->color);
//   //   final_colors.insert(
//   //       final_colors.end(), curr_colors.begin(), curr_colors.end());
//   //   size += (int)pos.size();
//   // }

//   auto quads   = vector<vec4i>{};
//   auto pos     = vector<vec3f>{};
//   auto normals = vector<vec3f>{};
//   auto tex     = vector<vec2f>{};

//   points_to_spheres(quads, pos, normals, tex, point_pos, 8, 0.0090);
//   auto point_color = vector<vec3f>(pos.size(), vec3f{0, 0, 0});
//   for (auto& n : normals) n *= -1;
//   auto tri = quads_to_triangles(quads);
//   merge_triangles(final_tri, tri, size);
//   final_pos.insert(final_pos.end(), pos.begin(), pos.end());
//   final_normals.insert(final_normals.end(), normals.begin(), normals.end());
//   final_colors.insert(
//       final_colors.end(), point_color.begin(), point_color.end());

//   // auto colors8 = vector<vec3b>(final_colors.size());
//   // for (auto idx = 0; idx < final_colors.size(); idx++)
//   //   colors8[idx] = float_to_byte(final_colors[idx]);

//   string err = "";
//   // save_mesh(filename, final_tri, final_pos, final_normals, tex, colors8,
//   // err,
//   //           true);
//   save_mesh(filename, final_tri, final_pos, final_normals, tex, final_colors,
//       err, true);
// }
void export_scene(App_base &app, const vec3f &mesh_color,
                  const string &filename, const bool export_edges,
                  const bool export_just_the_curves) {
  auto final_pos = app.mesh.positions;
  auto final_normals = app.mesh.normals;
  auto final_tri = app.mesh.triangles;
  auto final_colors = vector<vec3f>(final_pos.size(), mesh_color);
  auto size = (int)app.mesh.positions.size();
  auto path_pos = vector<vec3f>{};
  auto path_normals = vector<vec3f>{};
  auto path_tri = vector<vec3i>{};
  auto path_colors = vector<vec3f>{};
  auto path_size = 0;

  auto point_pos = vector<vec3f>{};
  if (export_edges) {
    for (auto tr : app.mesh.triangles) {
      for (int k = 0; k < 3; ++k) {
        auto a = tr[k];
        auto b = tr[(k + 1) % 3];
        if (a > b)
          continue;
        auto quads = vector<vec4i>{};
        auto pos = vector<vec3f>{};
        auto normals = vector<vec3f>{};
        auto tex = vector<vec2f>{};
        polyline_to_cylinders(quads, pos, normals, tex,
                              {app.mesh.positions[a], app.mesh.positions[b]}, 8,
                              0.0004);
        auto tri = quads_to_triangles(quads);
        merge_triangles(final_tri, tri, size);
        final_pos.insert(final_pos.end(), pos.begin(), pos.end());
        final_normals.insert(final_normals.end(), normals.begin(),
                             normals.end());
        auto curr_colors = vector<vec3f>(pos.size(), {0, 0, 0});
        final_colors.insert(final_colors.end(), curr_colors.begin(),
                            curr_colors.end());
        size += (int)pos.size();
      }
    }
  }
  for (auto i = 0; i < app.added_paths.size(); ++i) {
    auto path = app.added_paths[i];
    auto quads = vector<vec4i>{};
    auto pos = vector<vec3f>{};
    auto normals = vector<vec3f>{};
    auto tex = vector<vec2f>{};
    polyline_to_cylinders(quads, pos, normals, tex, path->positions, 4, 0.0025);
    auto tri = quads_to_triangles(quads);
    merge_triangles(path_tri, tri, path_size);
    path_pos.insert(path_pos.end(), pos.begin(), pos.end());
    path_normals.insert(path_normals.end(), normals.begin(), normals.end());
    auto curr_colors =
        vector<vec3f>(pos.size(), path->instance->material->color);
    path_colors.insert(path_colors.end(), curr_colors.begin(),
                       curr_colors.end());
    path_size += (int)pos.size();
  }
  for (auto points : app.added_points) {
    auto quads = vector<vec4i>{};
    auto pos = vector<vec3f>{};
    auto normals = vector<vec3f>{};
    auto tex = vector<vec2f>{};
    point_pos.resize(points->points.size());
    for (auto j = 0; j < points->points.size(); ++j) {
      point_pos[j] = eval_position(app.mesh.triangles, app.mesh.positions,
                                   points->points[j]);
    }

    points_to_spheres(quads, pos, normals, tex, point_pos, 4, points->radius);
    for (auto &n : normals)
      n *= -1;
    auto tri = quads_to_triangles(quads);
    merge_triangles(path_tri, tri, path_size);
    path_pos.insert(path_pos.end(), pos.begin(), pos.end());
    path_normals.insert(path_normals.end(), normals.begin(), normals.end());
    auto curr_colors =
        vector<vec3f>(pos.size(), points->instance->material->color);
    path_colors.insert(path_colors.end(), curr_colors.begin(),
                       curr_colors.end());
    path_size += (int)pos.size();
  }
  // auto colors8 = vector<vec3b>(final_colors.size());
  // for (auto idx = 0; idx < final_colors.size(); idx++)
  //   colors8[idx] = float_to_byte(final_colors[idx]);
  auto tex = vector<vec2f>{};
  string err = "";
  string curve_name = "curve_";
  if (!export_just_the_curves)
    save_mesh(filename, final_tri, final_pos, final_normals, tex, final_colors,
              err, true);
  save_mesh(curve_name.append(filename), path_tri, path_pos, path_normals, tex,
            path_colors, err, true);
  // save_mesh(filename, final_tri, final_pos, final_normals, tex,
  // final_colors,
  //           err, true);
}
/*
void add_point(App_base& app, const mesh_point& point) {
  if (app.scc_points.size() >= 3) { return; }

  app.scc_points.push_back(point);

  if (app.scc_points.size() == 2) {
    app.scc_path = compute_geodesic_path(app.mesh, app.scc_points[0],
app.scc_points[1]);
  }
}*/
// #include <yocto/yocto_sceneio.h>

// #include "scene_exporter.h"
// void export_yocto_scene(const App_base &app, const string &name) {
//   auto error = ""s;
//   auto output = name;

//   // make a directory if needed
//   auto dirname = name;
//   auto shapes_path = path_join(dirname, "shapes");
//   auto textures_path = path_join(dirname, "textures");
//   if (!make_directory(dirname, error))
//     print_fatal(error);
//   if (!make_directory(shapes_path, error))
//     print_fatal(error);
//   if (!make_directory(textures_path, error))
//     print_fatal(error);

//   auto scene = new sceneio_scene{};
//   scene->name = "name";
//   auto camera = add_camera(scene);

//   camera->frame = app.scene->cameras[0]->frame;
//   camera->lens = app.scene->cameras[0]->lens;
//   camera->aspect = app.scene->cameras[0]->aspect;
//   camera->film = app.scene->cameras[0]->film;
//   camera->aperture = app.scene->cameras[0]->aperture;
//   camera->focus = app.scene->cameras[0]->focus;

//   // add mesh
//   {
//     auto shape = add_shape(scene);
//     shape->name = "mesh";
//     shape->triangles = app.mesh.triangles;
//     shape->positions = app.mesh.positions;
//     shape->normals = app.mesh.normals;

//     auto material = add_material(scene);
//     material->color = {0.5, 0.5, 0.9};
//     material->metallic = 0.04;
//     material->roughness = 0.4;
//     material->opacity = 0.3;

//     auto instance = add_instance(scene);
//     instance->shape = shape;
//     instance->material = material;

//     auto success = save_shape(
//         path_join(shapes_path, "mesh.obj"), {}, {}, app.mesh.triangles, {},
//         {},
//         {}, {}, app.mesh.positions, app.mesh.normals, {}, {}, {}, error);
//     if (!success) {
//       printf("error saving mesh: %s \n", error.c_str());
//     }
//   }

//   { // add curves
//     auto shape = add_shape(scene);
//     shape->name = "curve";
//     for (auto &path : app.added_paths) {
//       auto lines = vector<vec2i>(path->positions.size() - 1);
//       for (int k = 0; k < lines.size(); k++) {
//         lines[k] = {k, k + 1};
//         lines[k] += shape->positions.size();
//         append(shape->lines, lines);
//       }
//       append(shape->positions, curve.positions);
//     }
//   }
//   shape->radius = vector<float>(shape->positions.size(), 0.0015);

//   auto material = add_material(scene);
//   material->color = {0.8, 0.1, 0.1};
//   auto instance = add_instance(scene);
//   instance->shape = shape;
//   instance->material = material;
//   auto success =
//       save_shape(path_join(shapes_path, "splines.obj"), {}, shape->lines,
//       {},
//                  {}, {}, {}, {}, shape->positions, {}, {}, {}, {}, error);
//   if (!success) {
//     printf("error saving splines: %s \n", error.c_str());
//   }
// }

// { // add paths
//   for (int i = 0; i < app.added_paths.size(); i++) {
//     auto &path = app.added_paths[i]->path;
//     auto &color = app.added_paths[i]->color;
//     auto &radius = app.added_paths[i]->radius;
//     auto shape = add_shape(scene);
//     shape->positions = path_positions(app.mesh, path);
//     shape->lines.resize(shape->positions.size() - 1);
//     for (int i = 0; i < shape->lines.size(); i++) {
//       shape->lines[i] = {i, i + 1};
//     }
//     shape->radius = vector<float>(shape->positions.size(), radius);

//     auto material = add_material(scene);
//     material->color = color;
//     auto instance = add_instance(scene);
//     instance->shape = shape;
//     instance->material = material;
//     auto success =
//         save_shape(path_join(shapes_path, std::to_string(i) + "path.obj"),
//         {},
//                    shape->lines, {}, {}, {}, {}, {}, shape->positions, {},
//                    {},
//                    {}, {}, error);
//     if (!success) {
//       printf("error saving path: %s \n", error.c_str());
//     }
//   }
// }

// { // add points
//   for (int i = 0; i < app.added_points.size(); i++) {
//     auto &points = app.added_points[i]->points;
//     auto &color = app.added_points[i]->color;
//     auto &radius = app.added_points[i]->radius;
//     auto sphere = make_sphere(32, radius);

//     auto material = add_material(scene);
//     material->color = color;

//     auto shape = add_shape(scene);
//     shape->positions = sphere.positions;
//     shape->triangles = quads_to_triangles(sphere.quads);
//     shape->normals = sphere.normals;

//     for (int i = 0; i < points.size(); i++) {
//       auto instance = add_instance(scene);
//       instance->shape = shape;
//       instance->material = material;
//       instance->frame.o = eval_position(app.mesh, points[i]);
//     }

//     auto success = save_shape(
//         path_join(shapes_path, std::to_string(i) + "point.obj"),
//         shape->points,
//         {}, {}, {}, {}, {}, {}, shape->positions, {}, {}, {}, {}, error);
//     if (!success) {
//       printf("error saving point: %s \n", error.c_str());
//     }
//   }
// }

// auto environment_tex = add_texture(scene);
// load_image("data/env.png", environment_tex->hdr, error);
// environment_tex->name = "env";
// auto environment = add_environment(scene);
// environment->emission = {0, 0, 0};
// environment->emission_tex = environment_tex;

// add_instance(scene, "arealight1",
//              lookat_frame({-1, 1, 1}, {0, 0.1, 0}, {0, 1, 0}, true),
//              add_shape(scene, "arealight1", make_rect({1, 1}, {0.2, 0.2})),
//              add_emission_material(scene, "arealight1", {10, 10, 10},
//              nullptr));
// add_instance(scene, "arealight2",
//              lookat_frame({1, 1, 0.5}, {0, 0.1, 0}, {0, 1, 0}, true),
//              add_shape(scene, "arealight2", make_rect({1, 1}, {0.2, 0.2})),
//              add_emission_material(scene, "arealight2", {10, 10, 10},
//              nullptr));
// add_instance(scene, "arealight3",
//              lookat_frame({0, 1, -1}, {0, 0.1, 0}, {0, 1, 0}, true),
//              add_shape(scene, "arealight3", make_rect({1, 1}, {0.2, 0.2})),
//              add_emission_material(scene, "arealight3", {10, 10, 10},
//              nullptr));

// auto floor_material = add_material(scene, "floor_material");
// floor_material->color = {1, 1, 1};
// auto floor_frame = identity3x4f;
// std::swap(floor_frame.z, floor_frame.y);
// floor_frame.o.y = -0.5;

// add_instance(scene, "floor", floor_frame,
//              add_shape(scene, "floor", make_rect({1, 1}, {1000, 1000})),
//              floor_material);

// save_scene(path_join(output, "scene.json"), scene, error);
// }