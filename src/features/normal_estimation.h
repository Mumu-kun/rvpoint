#pragma once

#include "core/point_types.h"
#include "search/fast_3d_spatial_grid.h"
#include <cstddef>
#include <vector>

namespace rvpoint {

/**
 * @brief Surface Normal and Curvature Estimation.
 *
 * Estimates 3D surface normals by computing local neighborhood covariance matrices
 * and solving for the smallest principal eigenvector via Jacobi rotations.
 *
 * Implements a stateful, zero-vtable Functor pattern owning neighbor/distance scratch
 * buffers and accelerated by uniform spatial hashing (ADR-0010, ADR-0011).
 */
class NormalEstimation {
public:
  explicit NormalEstimation(int k = 10, float radius = 0.5f,
                            float vp_x = 0.0f, float vp_y = 0.0f, float vp_z = 0.0f,
                            Backend backend = Backend::Auto);

  void set_k_search(int k) noexcept { k_ = k; }
  int k_search() const noexcept { return k_; }

  void set_radius_search(float r) noexcept { radius_ = r; }
  float radius_search() const noexcept { return radius_; }

  void set_viewpoint(float vx, float vy, float vz) noexcept {
    vp_x_ = vx; vp_y_ = vy; vp_z_ = vz;
  }

  void set_eigen_iterations(int iters) noexcept { eigen_iters_ = iters; }
  int eigen_iterations() const noexcept { return eigen_iters_; }

  void set_backend(Backend b) noexcept { backend_ = b; }
  Backend backend() const noexcept { return backend_; }

  NormalEstimation& threads(int num_threads) noexcept { num_threads_ = num_threads; return *this; }
  void set_num_threads(int num_threads) noexcept { num_threads_ = num_threads; }
  int num_threads() const noexcept { return num_threads_; }

  void reserve(std::size_t max_points);

  /**
   * @brief Compute surface normals into output PointCloud (where x=nx, y=ny, z=nz).
   */
  void operator()(const PointCloudView& in, PointCloud& normals, int k, float radius,
                  float vpx, float vpy, float vpz);
  void operator()(const PointCloudView& in, PointCloud& normals) {
    (*this)(in, normals, k_, radius_, vp_x_, vp_y_, vp_z_);
  }
  void operator()(const PointCloud& in, PointCloud& normals, int k, float radius,
                  float vpx, float vpy, float vpz) {
    (*this)(in.view(), normals, k, radius, vpx, vpy, vpz);
  }

  // Store-piped external search grid overloads
  void operator()(const PointCloudView& in, const Fast3DSpatialGrid& grid, PointCloud& normals,
                  int k, float radius, float vpx, float vpy, float vpz);
  void operator()(const PointCloudView& in, const Fast3DSpatialGrid& grid, PointCloud& normals) {
    (*this)(in, grid, normals, k_, radius_, vp_x_, vp_y_, vp_z_);
  }
  void operator()(const PointCloud& in, PointCloud& normals) {
    (*this)(in.view(), normals, k_, radius_, vp_x_, vp_y_, vp_z_);
  }

  void apply(const PointCloudView& in, PointCloud& normals, int k, float radius,
             float vpx, float vpy, float vpz) {
    (*this)(in, normals, k, radius, vpx, vpy, vpz);
  }
  void apply(const PointCloudView& in, PointCloud& normals) {
    (*this)(in, normals, k_, radius_, vp_x_, vp_y_, vp_z_);
  }
  void apply(const PointCloud& in, PointCloud& normals, int k, float radius,
             float vpx, float vpy, float vpz) {
    (*this)(in.view(), normals, k, radius, vpx, vpy, vpz);
  }
  void apply(const PointCloud& in, PointCloud& normals) {
    (*this)(in.view(), normals, k_, radius_, vp_x_, vp_y_, vp_z_);
  }

  /**
   * @brief Compute surface normals writing into raw contiguous float arrays.
   */
  void compute(const PointCloudView& in, float* nx, float* ny, float* nz,
               int k, float radius, float vpx, float vpy, float vpz);
  void compute(const PointCloudView& in, const Fast3DSpatialGrid& grid, float* nx, float* ny, float* nz,
               int k, float radius, float vpx, float vpy, float vpz);

private:
  int k_ = 10;
  float radius_ = 0.5f;
  float vp_x_ = 0.0f;
  float vp_y_ = 0.0f;
  float vp_z_ = 0.0f;
  int eigen_iters_ = 4;
  int num_threads_ = 0;
  Backend backend_ = Backend::Auto;

  // Instance-owned scratch workspaces and spatial acceleration
  Fast3DSpatialGrid grid_;
  std::vector<int> neighbors_;
  std::vector<float> dists2_;
  std::vector<std::vector<int>> thread_neighbors_;
  std::vector<std::vector<float>> thread_dists2_;
};

} // namespace rvpoint
