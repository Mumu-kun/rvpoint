#pragma once

#include "core/point_types.h"
#include "search/fast_3d_spatial_grid.h"
#include <cstddef>

namespace rvpoint {

/**
 * @brief Radius Outlier Removal (ROR) filter.
 *
 * Removes points that have fewer than min_neighbors within a Euclidean ball of radius search_radius.
 * Accelerated using an internal Fast3DSpatialGrid with self-cell fastpath for zero-heap real-time execution.
 */
class RadiusOutlierRemoval {
public:
  explicit RadiusOutlierRemoval(float search_radius = 0.5f, int min_neighbors = 5, Backend backend = Backend::Auto);

  void set_radius(float r) noexcept { search_radius_ = r; }
  float radius() const noexcept { return search_radius_; }

  void set_min_neighbors(int k) noexcept { min_neighbors_ = k; }
  int min_neighbors() const noexcept { return min_neighbors_; }

  void set_backend(Backend b) noexcept { backend_ = b; }
  Backend backend() const noexcept { return backend_; }

  void reserve(std::size_t max_points);

  std::size_t operator()(const PointCloudView& in, PointCloud& out, float search_radius, int min_neighbors);
  std::size_t operator()(const PointCloudView& in, PointCloud& out);
  std::size_t operator()(const PointCloud& in, PointCloud& out, float search_radius, int min_neighbors) {
    return (*this)(in.view(), out, search_radius, min_neighbors);

  // Store-piped external search grid overloads
  std::size_t operator()(const PointCloudView& in, const Fast3DSpatialGrid& grid, PointCloud& out, float search_radius, int min_neighbors);
  std::size_t operator()(const PointCloudView& in, const Fast3DSpatialGrid& grid, PointCloud& out) {
    return (*this)(in, grid, out, search_radius_, min_neighbors_);
  }
  std::size_t operator()(const PointCloud& in, PointCloud& out) {
    return (*this)(in.view(), out);
  }

  std::size_t apply(const PointCloudView& in, PointCloud& out, float search_radius, int min_neighbors) {
    return (*this)(in, out, search_radius, min_neighbors);
  }

  std::size_t apply(const PointCloudView& in, PointCloud& out) {
    return (*this)(in, out);
  }

  std::size_t apply(const PointCloud& in, PointCloud& out, float search_radius, int min_neighbors) {
    return (*this)(in.view(), out, search_radius, min_neighbors);
  }

  std::size_t apply(const PointCloud& in, PointCloud& out) {
    return (*this)(in.view(), out);
  }

private:
  float search_radius_ = 0.5f;
  int min_neighbors_ = 5;
  Backend backend_ = Backend::Auto;
  Fast3DSpatialGrid grid_;
};

} // namespace rvpoint

