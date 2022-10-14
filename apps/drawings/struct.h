#pragma once

#include <yocto/yocto_geometry.h>
#include <yocto/yocto_mesh.h>
#include <yocto/yocto_shape.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <deque>
using namespace yocto;

struct bezier_mesh {
  vector<vec3i> triangles = {};
  vector<vec3i> adjacencies = {};
  vector<vec3f> positions = {};
  vector<vec3f> normals = {};
  vector<vec2f> texcoords = {};
  // Additional data for experimental algorithms
  geodesic_solver solver = {};
  vector<vector<int>> v2t = {};
  vector<vector<float>> angles = {};
  vector<float> total_angles = {};
  dual_geodesic_solver dual_solver = {};
  Eigen::SparseMatrix<double, 1> Grad;
  Eigen::SparseMatrix<double> Lap;
  vector<Eigen::Matrix2d> g_inv;
  vector<Eigen::MatrixXd> a;
  vector<pair<vec3f, vec3f>> e;
  vector<vector<vector<pair<int, float>>>> ring;
  // vector<vector<vec2f>> christoffel;
  vector<vector<vec2f>> christoffel;
  vector<Eigen::ColPivHouseholderQR<Eigen::MatrixXf>> M;
  vector<Eigen::MatrixXf> RHS;
  vector<float> K;
  float avg_edge_length = 0.f;
  edge_map e2v = {};
};