#pragma once

#include "core/point_types.h"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace rvpoint {

/**
 * @brief Voxel Grid Downsampling Filter.
 *
 * Reduces point density by partitioning 3D space into regular voxels and
 * replacing all points in each voxel with their 3D centroid.
 *
 * Implements a stateful, zero-vtable Functor pattern owning all sort and reduction
 * scratch buffers. Calling reserve(max_points) guarantees zero heap allocations
 * on the per-frame hot path (ADR-0010, ADR-0011).
 */
class VoxelGrid {
public:
  explicit VoxelGrid(float leaf_size = 0.05f, Backend backend = Backend::Auto);

  void set_leaf_size(float leaf_size) noexcept { leaf_size_ = leaf_size; }
  float leaf_size() const noexcept { return leaf_size_; }

  void set_backend(Backend b) noexcept { backend_ = b; }
  Backend backend() const noexcept { return backend_; }

  /**
   * @brief Warmup allocation for zero-heap steady-state execution.
   */
  void reserve(std::size_t max_points);

  /**
   * @brief Downsample input cloud into output PointCloud (SoA).
   */
  std::size_t operator()(const PointCloudView& in, PointCloud& out, float leaf_size);
  std::size_t operator()(const PointCloudView& in, PointCloud& out) {
    return (*this)(in, out, leaf_size_);
  }
  std::size_t operator()(const PointCloud& in, PointCloud& out, float leaf_size) {
    return (*this)(in.view(), out, leaf_size);
  }
  std::size_t operator()(const PointCloud& in, PointCloud& out) {
    return (*this)(in.view(), out, leaf_size_);
  }

  std::size_t apply(const PointCloudView& in, PointCloud& out, float leaf_size) {
    return (*this)(in, out, leaf_size);
  }
  std::size_t apply(const PointCloudView& in, PointCloud& out) {
    return (*this)(in, out, leaf_size_);
  }
  std::size_t apply(const PointCloud& in, PointCloud& out, float leaf_size) {
    return (*this)(in.view(), out, leaf_size);
  }
  std::size_t apply(const PointCloud& in, PointCloud& out) {
    return (*this)(in.view(), out, leaf_size_);
  }

  /**
   * @brief Backward-compatible filter method writing directly into an AoS buffer.
   */
  std::size_t filter(const PointCloudView& in, PointXYZ* out, float leaf_size);

private:
  float leaf_size_ = 0.05f;
  Backend backend_ = Backend::Auto;

  // Instance-owned scratch buffers
  std::vector<int32_t> keys_;
  std::vector<uint32_t> order_;
  std::vector<uint32_t> tmp_k_;
  std::vector<uint32_t> tmp_v_;
  std::vector<std::size_t> chunk_split_;

  struct ThreadScratch {
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;

    void reserve(std::size_t cap) {
      x.reserve(cap);
      y.reserve(cap);
      z.reserve(cap);
    }
    void clear() noexcept {
      x.clear();
      y.clear();
      z.clear();
    }
    std::size_t size() const noexcept { return x.size(); }
    void push_back(float px, float py, float pz) {
      x.push_back(px);
      y.push_back(py);
      z.push_back(pz);
    }
  };
  std::vector<ThreadScratch> thread_scratch_;

  std::size_t filter_rvv(const PointCloudView& in, PointCloud& out, float leaf_size);
  std::size_t filter_scalar(const PointCloudView& in, PointCloud& out, float leaf_size);

  std::size_t filter_rvv_aos(const PointCloudView& in, PointXYZ* out, float leaf_size);
  std::size_t filter_scalar_aos(const PointCloudView& in, PointXYZ* out, float leaf_size);
};

} // namespace rvpoint



