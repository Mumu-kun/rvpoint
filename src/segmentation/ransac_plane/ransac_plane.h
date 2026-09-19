#pragma once

#include "core/point_types.h"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace rvpoint {

/**
 * @brief RANSAC Plane Segmentation.
 *
 * Robustly fits a 3D planar model ax + by + cz + d = 0 to a point cloud and
 * separates planar inliers from obstacle outliers.
 *
 * Implements a stateful, zero-vtable Functor pattern owning PRNG state and
 * extraction scratch buffers (ADR-0010, ADR-0011).
 */
class RansacPlane {
public:
  explicit RansacPlane(float dist_thresh = 0.05f, int max_iters = 100, Backend backend = Backend::Auto);

  void set_distance_threshold(float thresh) noexcept { dist_thresh_ = thresh; }
  float distance_threshold() const noexcept { return dist_thresh_; }

  void set_max_iterations(int iters) noexcept { max_iters_ = iters; }
  int max_iterations() const noexcept { return max_iters_; }

  void set_collinear_threshold(float thresh) noexcept { collinear_thresh_ = thresh; }
  float collinear_threshold() const noexcept { return collinear_thresh_; }

  void set_probability(float p) noexcept { probability_ = p; }
  float probability() const noexcept { return probability_; }

  void set_seed(uint32_t s) noexcept { seed_ = s; }
  uint32_t seed() const noexcept { return seed_; }

  void set_ground_normal_prior(float nx, float ny, float nz, float min_dot = 0.707f) noexcept {
    prior_nx_ = nx;
    prior_ny_ = ny;
    prior_nz_ = nz;
    min_ground_dot_ = min_dot;
    has_prior_ = true;
  }
  void clear_ground_normal_prior() noexcept { has_prior_ = false; }
  bool has_ground_normal_prior() const noexcept { return has_prior_; }

  void set_use_sample_screening(bool enable) noexcept { use_sample_screening_ = enable; }
  bool use_sample_screening() const noexcept { return use_sample_screening_; }

  void set_use_covariance_refinement(bool enable) noexcept { use_cov_refinement_ = enable; }
  bool use_covariance_refinement() const noexcept { return use_cov_refinement_; }

  void set_backend(Backend b) noexcept { backend_ = b; }
  Backend backend() const noexcept { return backend_; }

  void reserve(std::size_t max_points);

  /**
   * @brief Fit plane model to input cloud.
   * @return Number of inliers found.
   */
  int operator()(const PointCloudView& in, PlaneModel& model, float dist_thresh, int max_iters);
  int operator()(const PointCloudView& in, PlaneModel& model) {
    return (*this)(in, model, dist_thresh_, max_iters_);
  }
  int operator()(const PointCloud& in, PlaneModel& model, float dist_thresh, int max_iters) {
    return (*this)(in.view(), model, dist_thresh, max_iters);
  }
  int operator()(const PointCloud& in, PlaneModel& model) {
    return (*this)(in.view(), model, dist_thresh_, max_iters_);
  }

  // Explicit named fit methods
  int fit(const PointCloudView& in, PlaneModel& model, float dist_thresh, int max_iters) {
    return (*this)(in, model, dist_thresh, max_iters);
  }
  int fit(const PointCloudView& in, PlaneModel& model) {
    return (*this)(in, model, dist_thresh_, max_iters_);
  }
  int fit(const PointCloud& in, PlaneModel& model, float dist_thresh, int max_iters) {
    return (*this)(in.view(), model, dist_thresh, max_iters);
  }
  int fit(const PointCloud& in, PlaneModel& model) {
    return (*this)(in.view(), model, dist_thresh_, max_iters_);
  }

  /**
   * @brief Composite operator: Fit plane model AND extract inlier (ground) and outlier (obstacle) clouds.
   * Enables single-operator direct pipeline node plugging.
   */
  void operator()(const PointCloudView& in, PlaneModel& model,
                  PointCloud& inliers, PointCloud& outliers,
                  float dist_thresh, int max_iters) {
    model.inliers = (*this)(in, model, dist_thresh, max_iters);
    this->extract(in, model, dist_thresh, inliers, outliers);
  }
  void operator()(const PointCloudView& in, PlaneModel& model,
                  PointCloud& inliers, PointCloud& outliers) {
    (*this)(in, model, inliers, outliers, dist_thresh_, max_iters_);
  }
  void operator()(const PointCloud& in, PlaneModel& model,
                  PointCloud& inliers, PointCloud& outliers,
                  float dist_thresh, int max_iters) {
    (*this)(in.view(), model, inliers, outliers, dist_thresh, max_iters);
  }
  void operator()(const PointCloud& in, PlaneModel& model,
                  PointCloud& inliers, PointCloud& outliers) {
    (*this)(in.view(), model, inliers, outliers, dist_thresh_, max_iters_);
  }

  int apply(const PointCloudView& in, PlaneModel& model, float dist_thresh, int max_iters) {
    return (*this)(in, model, dist_thresh, max_iters);
  }
  int apply(const PointCloudView& in, PlaneModel& model) {
    return (*this)(in, model, dist_thresh_, max_iters_);
  }

  /**
   * @brief Extract inliers and outliers into SoA PointClouds.
   * Null pointers can be passed to skip inlier or outlier extraction without allocation.
   */
  void extract(const PointCloudView& in, const PlaneModel& model, float dist_thresh,
               PointCloud* inliers, PointCloud* outliers);
  void extract(const PointCloudView& in, const PlaneModel& model, float dist_thresh,
               PointCloud& inliers, PointCloud& outliers) {
    extract(in, model, dist_thresh, &inliers, &outliers);
  }
  void extract(const PointCloudView& in, const PlaneModel& model,
               PointCloud& inliers, PointCloud& outliers) {
    extract(in, model, dist_thresh_, &inliers, &outliers);
  }
  void extract(const PointCloud& in, const PlaneModel& model, float dist_thresh,
               PointCloud& inliers, PointCloud& outliers) {
    extract(in.view(), model, dist_thresh, &inliers, &outliers);
  }
  void extract(const PointCloud& in, const PlaneModel& model,
               PointCloud& inliers, PointCloud& outliers) {
    extract(in.view(), model, dist_thresh_, &inliers, &outliers);
  }

  std::size_t extract_inliers(const PointCloudView& in, const PlaneModel& model,
                              float dist_thresh, PointCloud& inliers);
  std::size_t extract_outliers(const PointCloudView& in, const PlaneModel& model,
                               float dist_thresh, PointCloud& outliers);

  // Backward-compatible AoS extraction methods
  std::size_t extract_inliers(const PointCloudView& in, const float* model,
                              float dist_thresh, PointXYZ* inliers);
  std::size_t extract_outliers(const PointCloudView& in, const float* model,
                               float dist_thresh, PointXYZ* outliers);
  void extract_inliers_outliers(const PointCloudView& in, const float* model,
                                float dist_thresh, PointXYZ* inliers, PointXYZ* outliers,
                                std::size_t& n_inliers, std::size_t& n_outliers);

private:
  float dist_thresh_ = 0.05f;
  int max_iters_ = 100;
  float collinear_thresh_ = 1e-6f;
  float probability_ = 0.99f;
  uint32_t seed_ = 0;
  uint32_t rng_state_ = 0x12345678u;
  Backend backend_ = Backend::Auto;

  bool has_prior_ = false;
  float prior_nx_ = 0.0f;
  float prior_ny_ = 0.0f;
  float prior_nz_ = 1.0f;
  float min_ground_dot_ = 0.707f;
  bool use_sample_screening_ = false;
  bool use_cov_refinement_ = false;

  // Instance-owned scratch buffers (ADR-0010 zero-heap hot-path)
  std::vector<int> inlier_indices_scratch_;
  std::vector<uint8_t> inlier_mask_scratch_;
  std::vector<float> sample_x_;
  std::vector<float> sample_y_;
  std::vector<float> sample_z_;

  uint32_t next_rand() noexcept {
    rng_state_ ^= rng_state_ << 13;
    rng_state_ ^= rng_state_ >> 17;
    rng_state_ ^= rng_state_ << 5;
    return rng_state_;
  }

  int fit_rvv(const PointCloudView& in, PlaneModel& model, float dist_thresh, int max_iters);
  int fit_scalar(const PointCloudView& in, PlaneModel& model, float dist_thresh, int max_iters);

  void extract_rvv(const PointCloudView& in, const PlaneModel& model, float dist_thresh,
                   PointCloud* inliers, PointCloud* outliers);
  void extract_scalar(const PointCloudView& in, const PlaneModel& model, float dist_thresh,
                      PointCloud* inliers, PointCloud* outliers);
};

} // namespace rvpoint


