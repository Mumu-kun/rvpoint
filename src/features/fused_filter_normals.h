#pragma once

#include "core/point_types.h"
#include "search/fast_3d_spatial_grid.h"
#include <cstddef>
#include <vector>

namespace rvpoint {

/**
 * @brief Fused Outlier Filter and Surface Normal Estimation.
 *
 * Performs Radius Outlier Removal (ROR) and Cardano 3D surface normal
 * estimation in a single unified spatial grid traversal pass, eliminating
 * duplicate neighbor queries and intermediate spatial structure builds.
 *
 * Follows ADR-0010 (zero heap hot-path) and ADR-0011 (non-virtual functor).
 */
class FusedFilterNormals {
public:
  explicit FusedFilterNormals(float search_radius = 0.5f, int min_neighbors = 5,
                              bool compute_normals = true, Backend backend = Backend::Auto);

  void set_radius(float r) noexcept { search_radius_ = r; }
  float radius() const noexcept { return search_radius_; }

  void set_min_neighbors(int k) noexcept { min_neighbors_ = k; }
  int min_neighbors() const noexcept { return min_neighbors_; }

  void set_compute_normals(bool enable) noexcept { compute_normals_ = enable; }
  bool compute_normals() const noexcept { return compute_normals_; }

  void set_backend(Backend b) noexcept { backend_ = b; }
  Backend backend() const noexcept { return backend_; }

  void reserve(std::size_t max_points);

  /**
   * @brief Execute fused filtering and normal estimation.
   *
   * @param in Input point cloud view.
   * @param filtered Output point cloud storing surviving points.
   * @param normals Optional output point cloud storing estimated surface normals (x=nx, y=ny, z=nz).
   * @return Number of surviving points.
   */
  std::size_t operator()(const PointCloudView& in, PointCloud& filtered, PointCloud* normals,
                         float radius, int min_neighbors, bool compute_normals);

  std::size_t operator()(const PointCloudView& in, PointCloud& filtered, PointCloud* normals = nullptr) {
    return (*this)(in, filtered, normals, search_radius_, min_neighbors_, compute_normals_);
  }

  std::size_t apply(const PointCloudView& in, PointCloud& filtered, PointCloud* normals = nullptr) {
    return (*this)(in, filtered, normals);
  }

private:
  float search_radius_ = 0.5f;
  int min_neighbors_ = 5;
  bool compute_normals_ = true;
  Backend backend_ = Backend::Auto;

  // Instance-owned scratch workspaces (ADR-0010)
  Fast3DSpatialGrid grid_;
  std::vector<int> neighbors_;
  std::vector<uint8_t> keep_mask_;
};

} // namespace rvpoint

