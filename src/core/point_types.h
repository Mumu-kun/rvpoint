#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include <cstring>
#include <cmath>
#include <utility>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvpoint {

/**
 * @brief Backend execution engine selection.
 */
enum class Backend {
  Auto,   ///< Automatically detect: RVV 1.0 if hardware/compiler supported, else Scalar
  RVV,    ///< Explicit RISC-V Vector 1.0 intrinsics execution
  Scalar  ///< Explicit portable C++ scalar execution
};

/**
 * @brief Standard 3D Point structure.
 */
struct PointXYZ {
  float x, y, z;
};

struct PointXYZI {
  float x, y, z;
  float intensity;
};

struct PointXYZRGB {
  float x, y, z;
  uint8_t r, g, b;
};

struct Normal {
  float normal_x, normal_y, normal_z;
  float curvature;
};

struct PointNormal {
  float x, y, z;
  float normal_x, normal_y, normal_z;
  float curvature;
};

/**
 * @brief Non-owning read-only view of a point cloud in Structure-of-Arrays (SoA) layout.
 *
 * Provides view-only accessors over contiguous coordinate arrays without memory ownership.
 */
struct PointCloudView {
  const float *x = nullptr;
  const float *y = nullptr;
  const float *z = nullptr;
  std::size_t n = 0;

  constexpr PointCloudView() noexcept = default;
  constexpr PointCloudView(const float *px, const float *py, const float *pz, std::size_t count) noexcept
      : x(px), y(py), z(pz), n(count) {}

  constexpr std::size_t size() const noexcept { return n; }
  constexpr bool empty() const noexcept { return n == 0; }

  inline PointXYZ operator[](std::size_t i) const noexcept {
    return {x[i], y[i], z[i]};
  }
};

/**
 * @brief Backward compatibility alias for PointCloudView.
 */
using PointCloudSoA = PointCloudView;

/**
 * @brief Standard owning point cloud container storing contiguous X, Y, and Z coordinate buffers.
 *
 * Implements value semantics and RAII memory management. Provides zero-copy view projection.
 */
class PointCloud {
public:
  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> z;
  std::size_t n = 0;

  PointCloud() = default;
  explicit PointCloud(std::size_t capacity) {
    reserve(capacity);
  }

  void reserve(std::size_t cap) {
    x.reserve(cap);
    y.reserve(cap);
    z.reserve(cap);
  }

  void resize(std::size_t count) {
    if (x.capacity() < count) {
      reserve(count);
    }
    x.resize(count);
    y.resize(count);
    z.resize(count);
    n = count;
  }

  void clear() noexcept {
    x.clear();
    y.clear();
    z.clear();
    n = 0;
  }

  void push_back(float px, float py, float pz) {
    x.push_back(px);
    y.push_back(py);
    z.push_back(pz);
    n++;
  }

  void push_back(const PointXYZ& pt) {
    push_back(pt.x, pt.y, pt.z);
  }

  std::size_t size() const noexcept { return n; }
  std::size_t capacity() const noexcept { return x.capacity(); }
  bool empty() const noexcept { return n == 0; }

  PointXYZ operator[](std::size_t i) const noexcept {
    return {x[i], y[i], z[i]};
  }

  PointCloudView view() const noexcept {
    return PointCloudView(x.data(), y.data(), z.data(), n);
  }

  void copy_from(const PointCloudView& src) {
    resize(src.n);
    if (src.n > 0) {
      std::memcpy(x.data(), src.x, src.n * sizeof(float));
      std::memcpy(y.data(), src.y, src.n * sizeof(float));
      std::memcpy(z.data(), src.z, src.n * sizeof(float));
    }
  }

  /**
   * @brief Scale all coordinates uniformly in-place.
   * @param factor Uniform scale factor (e.g. 0.001f for mm -> meters).
   */
  void scale(float factor) noexcept {
    scale(factor, factor, factor);
  }

  /**
   * @brief Scale coordinates along X, Y, and Z axes in-place.
   * @param sx Scale factor for X
   * @param sy Scale factor for Y
   * @param sz Scale factor for Z
   */
  void scale(float sx, float sy, float sz) noexcept {
#if defined(__riscv_vector)
    std::size_t i = 0;
    while (i < n) {
      std::size_t vl = __riscv_vsetvl_e32m8(n - i);
      vfloat32m8_t vx = __riscv_vle32_v_f32m8(x.data() + i, vl);
      vfloat32m8_t vy = __riscv_vle32_v_f32m8(y.data() + i, vl);
      vfloat32m8_t vz = __riscv_vle32_v_f32m8(z.data() + i, vl);
      vx = __riscv_vfmul_vf_f32m8(vx, sx, vl);
      vy = __riscv_vfmul_vf_f32m8(vy, sy, vl);
      vz = __riscv_vfmul_vf_f32m8(vz, sz, vl);
      __riscv_vse32_v_f32m8(x.data() + i, vx, vl);
      __riscv_vse32_v_f32m8(y.data() + i, vy, vl);
      __riscv_vse32_v_f32m8(z.data() + i, vz, vl);
      i += vl;
    }
#else
    for (std::size_t i = 0; i < n; ++i) {
      x[i] *= sx;
      y[i] *= sy;
      z[i] *= sz;
    }
#endif
  }

  /**
   * @brief Compute the 3D axis-aligned bounding box of the point cloud.
   */
  void get_bounds(float& min_x, float& max_x,
                  float& min_y, float& max_y,
                  float& min_z, float& max_z) const noexcept {
    if (empty()) {
      min_x = max_x = min_y = max_y = min_z = max_z = 0.0f;
      return;
    }
    float m_min_x = x[0], m_max_x = x[0];
    float m_min_y = y[0], m_max_y = y[0];
    float m_min_z = z[0], m_max_z = z[0];
    for (std::size_t i = 1; i < n; ++i) {
      if (x[i] < m_min_x) m_min_x = x[i];
      if (x[i] > m_max_x) m_max_x = x[i];
      if (y[i] < m_min_y) m_min_y = y[i];
      if (y[i] > m_max_y) m_max_y = y[i];
      if (z[i] < m_min_z) m_min_z = z[i];
      if (z[i] > m_max_z) m_max_z = z[i];
    }
    min_x = m_min_x; max_x = m_max_x;
    min_y = m_min_y; max_y = m_max_y;
    min_z = m_min_z; max_z = m_max_z;
  }

  PointCloudView as_soa() const noexcept {
    return view();
  }

  PointCloudView as_soa_mut() noexcept {
    return view();
  }

  operator PointCloudView() const noexcept {
    return view();
  }
};

using OwnedPointCloud = PointCloud;

/**
 * @brief Planar geometric model: ax + by + cz + d = 0.
 */
struct PlaneModel {
  float a = 0.0f;
  float b = 0.0f;
  float c = 0.0f;
  float d = 0.0f;
  int inliers = 0;

  constexpr PlaneModel() noexcept = default;
  constexpr PlaneModel(float pa, float pb, float pc, float pd, int pinliers = 0) noexcept
      : a(pa), b(pb), c(pc), d(pd), inliers(pinliers) {}

  inline float dot(float px, float py, float pz) const noexcept {
    return a * px + b * py + c * pz + d;
  }

  inline float distance(float px, float py, float pz) const noexcept {
    return std::abs(dot(px, py, pz));
  }

  inline bool normalize() noexcept {
    float norm = std::sqrt(a * a + b * b + c * c);
    if (norm < 1e-6f) return false;
    float inv = 1.0f / norm;
    a *= inv;
    b *= inv;
    c *= inv;
    d *= inv;
    return true;
  }
};

/**
 * @brief Flat CSR-style cluster segmentation result.
 *
 * Eliminates heap-allocated nested vectors. Cluster k spans indices[offsets[k]] to indices[offsets[k+1]].
 */
struct ClusterResult {
  std::vector<uint32_t> indices;
  std::vector<uint32_t> offsets;

  void clear() noexcept {
    indices.clear();
    offsets.clear();
  }

  void reserve(std::size_t num_indices, std::size_t num_clusters) {
    indices.reserve(num_indices);
    offsets.reserve(num_clusters + 1);
  }

  void reserve(std::size_t num_indices) {
    indices.reserve(num_indices);
    offsets.reserve(65);
  }

  std::size_t capacity() const noexcept {
    return indices.capacity();
  }

  std::size_t num_clusters() const noexcept {
    return offsets.empty() ? 0 : offsets.size() - 1;
  }

  std::pair<const uint32_t*, std::size_t> cluster(std::size_t idx) const noexcept {
    if (idx >= num_clusters()) return {nullptr, 0};
    uint32_t start = offsets[idx];
    uint32_t len = offsets[idx + 1] - start;
    return {indices.data() + start, len};
  }

  const uint32_t* cluster_indices(std::size_t idx) const noexcept {
    if (idx >= num_clusters()) return nullptr;
    return indices.data() + offsets[idx];
  }

  std::size_t cluster_size(std::size_t idx) const noexcept {
    if (idx >= num_clusters()) return 0;
    return offsets[idx + 1] - offsets[idx];
  }
};

/**
 * @brief Flat CSR-style spatial query result for batch radius/kNN queries.
 */
struct NeighborQueryResult {
  std::vector<int32_t> indices;
  std::vector<uint32_t> offsets;

  void clear() noexcept {
    indices.clear();
    offsets.clear();
  }

  void reserve(std::size_t num_indices, std::size_t num_queries = 64) {
    indices.reserve(num_indices);
    offsets.reserve(num_queries + 1);
  }

  std::size_t num_queries() const noexcept {
    return offsets.empty() ? 0 : offsets.size() - 1;
  }

  std::pair<const int32_t*, std::size_t> neighbors(std::size_t query_idx) const noexcept {
    if (query_idx >= num_queries()) return {nullptr, 0};
    uint32_t start = offsets[query_idx];
    uint32_t len = offsets[query_idx + 1] - start;
    return {indices.data() + start, len};
  }
};

} // namespace rvpoint

namespace rvv_pcl = rvpoint;
