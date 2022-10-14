#pragma once

#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <string>

#include <yocto/yocto_mesh.h>
#include <yocto/yocto_math.h>
#include <realtime/gpu.h>

#include "struct.h"

using std::vector, std::unordered_map, std::unordered_set, std::string;
using yocto::mesh_point, yocto::geodesic_path;
using namespace gpu;

inline const auto MIN_ANGLE = 1.0F;
inline const auto MAX_ANGLE = 179.0F;

struct Polygon {
  int                   n;
  int                   n_vertices = 0;
  vector<mesh_point>    vertices = {};
  vector<geodesic_path> edges = {};
  vector<geodesic_path> angle_bisectors = {};
  vector<float>         angles = {};
  vector<float>         edges_length = {};

  unordered_map<string, vector<Shape>>  shapes = {};

  unordered_set<int>  selected_shapes_ids = {};
  int                 selected_vertex     = -1;
  int                 selected_edge       = -1;
};

// -------------------------------------------------------------------------------------
Polygon create_polygon(int n);

void reset_polygon(Polygon& polygon);

void change_polygon(Polygon& polygon, int n, const bezier_mesh& mesh);

void draw_polygon(unordered_map<string, Shader>& shaders, Polygon &polygon, mat4f& view, mat4f& projection, bool bisectors);

// -------------------------------------------------------------------------------------
bool is_angle_valid(const bezier_mesh& mesh, const mesh_point& point, const geodesic_path& edge);

bool is_angle_valid(const bezier_mesh& mesh, const geodesic_path& edge, const mesh_point& point);

bool is_angle_valid(const bezier_mesh& mesh, const mesh_point& a, const mesh_point& b, const mesh_point& c);

void add_vertex(const bezier_mesh& mesh, Polygon& polygon, const mesh_point& point);

void set_vertex(const bezier_mesh& mesh, Polygon& polygon, const int& ix, const mesh_point& p);

void update_vertices(const vector<vec3i>& triangles, const vector<vec3f>& positions, Polygon& polygon) ;

void update_vertex(const vector<vec3i>& triangles, const vector<vec3f>& positions, Polygon& polygon, const int& ix);

void set_edge(const bezier_mesh& mesh, Polygon& polygon, const int& ix, const geodesic_path& path);

void update_edges(const bezier_mesh& mesh, Polygon& polygon);

void update_edge(const bezier_mesh& mesh, Polygon& polygon, const int& ix);

void set_triangle_angles(const bezier_mesh& mesh, Polygon& polygon, const float& next_angle, const int& ix);

// TODO: check if angle can be > 180, compute angle between edges and not between edge and bisector
float get_angle(const bezier_mesh& mesh, Polygon& polygon, const int& ix);

// TODO: checks on index
void update_angle(const bezier_mesh& mesh, Polygon& polygon,
    const int& ix);

// TODO: checks on closed polygon
void update_angles(const bezier_mesh& mesh, Polygon& polygon);

geodesic_path get_angle_bisector(const bezier_mesh& mesh, const Polygon& polygon, const int& ix);

// TODO: change when not found
void update_angle_bisectors(const bezier_mesh& mesh, Polygon& polygon);

void set_edge_length(const bezier_mesh& mesh, Polygon& polygon, const int& ix, const float& length);

void set_angle(const bezier_mesh& mesh, Polygon& polygon, const float& next_angle, const int& ix);

void extend_edge(const bezier_mesh& mesh, Polygon& polygon, const int& ix, const float& length, bool from_end);

void extend_edge(const bezier_mesh& mesh, Polygon& polygon, const int& ix, const float& length);

void retract_edge(const bezier_mesh& mesh, Polygon& polygon, const int& ix, const float& length, bool from_end);

void retract_edge(const bezier_mesh& mesh, Polygon& polygon, const int& ix, const float& d);

mesh_point select_vertex(const mesh_point& point, const vector<vec3i>& triangles, const vector<vec3f>& positions, Polygon& polygon);

mesh_point select_vertex(Polygon& polygon, int ix);

geodesic_path select_edge(Polygon& polygon, int ix);