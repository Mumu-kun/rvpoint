#pragma once

#include "core/point_types.h"
#include <cstddef>
#include <vector>

namespace rvpoint {

/**
 * @brief Statistical Outlier Removal (SOR) Filter.
 *
 * Removes noise points whose mean k-NN distance exceeds global mean + alpha * std_dev.
 * Implements a stateful, zero-vtable Functor pattern owning per-point distance scratch
 * buffers (ADR-0010, ADR-0011).
 */
class StatisticalOutlierRemoval {
public:
  explicit StatisticalOutlierRemoval(int k = 20, float alpha = 1.0f, Backend backend = Backend::Auto);

  void set_mean_k(int k) noexcept { k_ = k; }
  int mean_k() const noexcept { return k_; }

  void set_std_threshold(float alpha) noexcept { alpha_ = alpha; }
  float std_threshold() const noexcept { return alpha_; }

  void set_backend(Backend b) noexcept { backend_ = b; }
  Backend backend() const noexcept { return backend_; }

  void reserve(std::size_t max_points);

  std::size_t operator()(const PointCloudView& in, PointCloud& out, int k, float alpha);
  std::size_t operator()(const PointCloudView& in, PointCloud& out) {
    return (*this)(in, out, k_, alpha_);
  }
  std::size_t operator()(const PointCloud& in, PointCloud& out, int k, float alpha) {
    return (*this)(in.view(), out, k, alpha);
  }
  std::size_t operator()(const PointCloud& in, PointCloud& out) {
    return (*this)(in.view(), out, k_, alpha_);
  }

  std::size_t apply(const PointCloudView& in, PointCloud& out, int k, float alpha) {
    return (*this)(in, out, k, alpha);
  }
  std::size_t apply(const PointCloudView& in, PointCloud& out) {
    return (*this)(in, out, k_, alpha_);
  }
  std::size_t apply(const PointCloud& in, PointCloud& out, int k, float alpha) {
    return (*this)(in.view(), out, k, alpha);
  }
  std::size_t apply(const PointCloud& in, PointCloud& out) {
    return (*this)(in.view(), out, k_, alpha_);
  }

  std::size_t filter(const PointCloudView& in, PointXYZ* out, int k, float alpha);

private:
  int k_ = 20;
  float alpha_ = 1.0f;
  Backend backend_ = Backend::Auto;

  std::vector<float> mean_dists_;
  std::vector<float> dists_;
  std::vector<float> dists_scratch_;

  std::size_t filter_rvv(const PointCloudView& in, PointCloud& out, int k, float alpha);
  std::size_t filter_scalar(const PointCloudView& in, PointCloud& out, int k, float alpha);

  std::size_t filter_rvv_aos(const PointCloudView& in, PointXYZ* out, int k, float alpha);
  std::size_t filter_scalar_aos(const PointCloudView& in, PointXYZ* out, int k, float alpha);
};

} // namespace rvpoint



