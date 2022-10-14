#pragma once

#include <deque>
#include <vector>

#include <realtime/gpu.h>
#include <unordered_set>
#include <yocto/yocto_bvh.h>
#include <yocto/yocto_mesh.h>
#include <yocto_gui/yocto_imgui.h>
#include <yocto_gui/yocto_window.h>

#include "drawing_circle.h"
#include "drawing_polygon.h"
#include "straightedge_and_compass_constructions.h"
#include "struct.h"

using namespace yocto;
// using namespace gpu;
// using namespace window;
// using namespace std;

#define SURFACE_OFFSET 0.00002f

enum app_state { APP_NONE, MACROS, POLYGON, STRAIGHTEDGE, EDITING };
struct Gui_Input {
  bool translating = false;
  bool rotating = false;
  bool scaling = false;
  bool duplicating = false;
  bool deleting = false;
  bool cropping = false;
};
struct App_base {
  vector<Added_Path *> added_paths = {};
  vector<Added_Points *> added_points = {};
  Circle *selected_circle = nullptr;
  int result_entry = -1;
  vector<int> selected_circle_entries = {};
  vector<int> overlapping_circles = {};
  string curr_primitive_name = "None Selected";
  int curr_primitive = 0;
  float lambda = 0.75;
  float sigma = 0.75;
  float scale_factor_circle_for_angle_bisector = 0.6666;
  bool reverse_cropping = false;
  bool use_slider_for_circle = false;
  double scale_factor_radius = -1;
  float max_dist = -1;
  float scale_factor_inner_circle_radius = 0.5;
  float max_radius = 0;
  float length_of_straightest = 1.f;
  string scene_name = "scene_name.ply";
  Added_Points *selected_point = nullptr;
  Added_Points *result_point = nullptr;
  int solution = 0;
  int type_of_isoscele = -1;
  int concentric_levels = 1;
  int petals = 10;
  int type_of_equilateral = 0;
  int type_of_triangle = 0;
  int type_of_rectangle = 0;
  bool use_exp_map_construction = true;
  bool garland = false;
  bool concentric = false;
  bool flip_triangle = false;
  bool straighthest = false;
  bool edit_inner_circle = false;
  bool with_inner_circle = false;
  bool export_just_the_paths = false;
  int curr_path = 0;
  int curr_circle = 0;
  int type_of_intersection = 0;
  int curve_size = 20;
  bezier_mesh mesh;
  vec3f curr_color = zero3f;
  mat4f projection_view = identity4x4f;
  bool envlight = false;
  bool show_edges = false;
  bool show_gradient = false;
  bool show_scalar_field = false;
  float field_frequency = 500;
  bool show_strip = true;
  vec2f screenspace_tangent = {0, 0};
  shade_scene *scene = nullptr;
  shade_material *lines_material = nullptr;
  shade_material *mesh_material = nullptr;
  shade_shape *mesh_shape = nullptr;
  shade_camera *ogl_camera = {};
  shade_params shade_params{};
  float camera_focus;
  shape_bvh bvh;
  bool started = false;
  gui_widget *widget = new gui_widget{};
  ogl_shape edges_shape = {};
  std::unordered_map<string, ogl_shape> ogl_shapes;
  std::unordered_map<string, ogl_program> ogl_shaders;
  // Data stored on the gpu for rendering.
  std::unordered_map<string, Shape> shapes;
  std::unordered_map<string, Shader> shaders;
  struct {
    mat4f view = identity4x4f;
    mat4f projection = identity4x4f;
    mat4f projection_view = identity4x4f;
  } matrices;
  string filename = "data/mesh.obj";
  string error = "";
  vec2i window_size = {};
  int n = 3;
  Polygon polygon = create_polygon(n);
  deque<bool> angles_eq = deque<bool>(n, false);
  vector<float> next_angles = vector<float>(n, 0.0F);
  vector<float> next_lengths = vector<float>(n, 0.0F);
  vector<pair<int, int>> move_vertices = vector<pair<int, int>>(n, {0, 0});
  pair<mesh_point, mesh_point> intersections = pair<mesh_point, mesh_point>();
  Isoline isoline = Isoline();
  bool move_selected = false;
  bool hover_selected = false;

  SCC scc = SCC();
  struct Editing_State {
    Gui_Input input = {};
  };
  app_state state = app_state::APP_NONE;
  Editing_State e_state = {};
  const Gui_Input &input() const { return e_state.input; }
  Gui_Input &input() { return e_state.input; }
  Circle circle_compute;
  Circle circle_exact;
};

void init_bvh(App_base &app);

shade_camera _make_framing_camera(const vector<vec3f> &positions);

vector<vec3f> make_normals(const vector<vec3i> &triangles,
                           const vector<vec3f> &positions);

ray3f camera_ray(const App_base &app, vec2f mouse);

vec2f screenspace_from_worldspace(App_base &app, const vec3f &position);

mesh_point intersect_mesh(const App_base &app, vec2f mouse);

void init_gpu(App_base &app, bool envlight);

void init_camera(App_base &app, const vec3f &from = vec3f{0, 0.5, 1.5},
                 const vec3f &to = {0, 0, 0});

void delete_app(App_base &app);

bool load_program(ogl_program *program, const string &vertex_filename,
                  const string &fragment_filename);
void set_points_shape(ogl_shape *shape, const vector<vec3f> &positions);
void set_points_shape(ogl_shape *shape, const bezier_mesh &mesh,
                      const vector<mesh_point> &points);
void set_mesh_shape(ogl_shape *shape, const vector<vec3i> &triangles,
                    const vector<vec3f> &positions,
                    const vector<vec3f> &normals);
void set_polyline_shape(ogl_shape *shape, const vector<vec3f> &positions);

inline void update_camera_info(App_base &app, const gui_input &input) {
  auto &camera = *app.ogl_camera;
  auto viewport = input.framebuffer_viewport;
  camera.aspect = (float)viewport.z / (float)viewport.w;

  auto camera_yfov =
      (camera.aspect >= 0)
          ? (2 * yocto::atan(camera.film / (camera.aspect * 2 * camera.lens)))
          : (2 * yocto::atan(camera.film / (2 * camera.lens)));

  app.matrices.view = frame_to_mat(inverse(camera.frame));
  app.matrices.projection = perspective_mat(
      camera_yfov, camera.aspect, app.shade_params.near, app.shade_params.far);
  app.matrices.projection_view = app.matrices.projection * app.matrices.view;
}
Added_Path *add_path_shape(App_base &app, const geodesic_path &path,
                           float radius, const vec3f &color);
Added_Path *add_path_shape(App_base &app, const vector<vec3f> &positions,
                           float radius, const vec3f &color,
                           const float &threshold = flt_max);
Added_Points *add_points_shape(App_base &app, const vector<mesh_point> &points,
                               float radius, const vec3f &color);
Added_Points *add_points_shape(App_base &app, const vector<vec3f> &points,
                               float radius, const vec3f &color);
void update_path_shape(shade_shape *shape, const bezier_mesh &mesh,
                       const geodesic_path &path, float radius);
void update_path_shape(const vector<Added_Path *> &paths,
                       const bezier_mesh &mesh, const vec3f &color,
                       const int entry);
void update_path_shape(const vector<Added_Path *> &paths,
                       const bezier_mesh &mesh, const vec3f &color,
                       const vector<int> &entries);
void update_path_shape(const vector<Added_Path *> &paths,
                       const bezier_mesh &mesh, const float &radius,
                       const vector<int> &entries);
void update_path_shape(shade_shape *shape, const bezier_mesh &mesh,
                       const vector<vec3f> &positions, float radius);
void update_path_shape(const vector<Added_Path *> &paths,
                       const bezier_mesh &mesh, const vector<vec3f> &positions,
                       const float &radius, const int entry);
void update_points_shape(shade_shape *shape, const vector<vec3f> &positions,
                         float radius);
void update_points_shape(shade_shape *shape, const bezier_mesh &mesh,
                         const vector<mesh_point> &points, float radius);
void update_glvector_field(App_base &app, const vector<vec3f> &vector_field,
                           float scale = 0.01,
                           const string &name = "vector_field");

void update_glpoints(App_base &app, const vector<vec3f> &positions,
                     const string &name = "selected_points");

void update_glpoints(App_base &app, const vector<mesh_point> &points,
                     const string &name = "selected_points");

void update_glpatch(App_base &app, const vector<int> &faces,
                    const string &name = "patch");

void update_glpolyline(App_base &app, const vector<vec3f> &vertices,
                       const string &name = "polyline");

void update_glcolors(App_base &app, const vector<vec3f> &colors);

void update_glcolors(App_base &app, const vector<float> &f);

void init_app(App_base &app, string filename);

void update_glpath(App_base &app);

void set_vertices_number(App_base &app);

void reset_polygon(App_base &app);

void clear_mesh(App_base &app);

void export_scene(App_base &app, const vec3f &mesh_color,
                  const string &filename, const bool export_edges,
                  const bool export_just_the_curves = false);