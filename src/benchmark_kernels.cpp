#include "include/benchmark_kernels.h"

#include <cmath>
#include <cstdint>
#include <vector>

namespace rvv_pcl::bench {

namespace {

constexpr float kL2QueryX = 41.0f;
constexpr float kL2QueryY = 29.0f;
constexpr float kL2QueryZ = 37.0f;
constexpr float kFilterThreshold = 120.0f;
constexpr float kRadiusQueryX = 17.0f;
constexpr float kRadiusQueryY = 23.0f;
constexpr float kRadiusQueryZ = 31.0f;
constexpr float kRadiusSquared = 2025.0f;

uint64_t sumQuantized(const std::vector<float> &values) {
  uint64_t checksum = 0;
  for (float value : values) {
    checksum += static_cast<uint64_t>(value);
  }
  return checksum;
}

} // namespace

PointCloudSoA makeDeterministicCloud(std::size_t size) {
  PointCloudSoA cloud;
  cloud.reserve(size);

  for (std::size_t index = 0; index < size; ++index) {
    const float x = static_cast<float>((index * 3u + 1u) % 97u);
    const float y = static_cast<float>((index * 5u + 7u) % 89u);
    const float z = static_cast<float>((index * 7u + 13u) % 83u);
    cloud.push_back({x, y, z});
  }

  return cloud;
}

bool parseKernel(const std::string &name, Kernel &kernel) {
  if (name == "l2") {
    kernel = Kernel::L2;
    return true;
  }
  if (name == "reduction") {
    kernel = Kernel::Reduction;
    return true;
  }
  if (name == "filter") {
    kernel = Kernel::Filter;
    return true;
  }
  if (name == "radius") {
    kernel = Kernel::Radius;
    return true;
  }
  if (name == "normal") {
    kernel = Kernel::Normal;
    return true;
  }
  return false;
}

const char *kernelName(Kernel kernel) {
  switch (kernel) {
  case Kernel::L2:
    return "l2";
  case Kernel::Reduction:
    return "reduction";
  case Kernel::Filter:
    return "filter";
  case Kernel::Radius:
    return "radius";
  case Kernel::Normal:
    return "normal";
  }
  return "unknown";
}

const char *modeName() {
#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
  return "rvv";
#else
  return "scalar";
#endif
}

uint64_t runL2Distance(const PointCloudSoA &cloud) {
  if (cloud.empty()) {
    return 0;
  }

  std::vector<float> distances(cloud.size(), 0.0f);
#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
  std::size_t index = 0;
  while (index < cloud.size()) {
    const std::size_t vl = __riscv_vsetvl_e32m8(cloud.size() - index);
    const vfloat32m8_t x = __riscv_vle32_v_f32m8(cloud.xData() + index, vl);
    const vfloat32m8_t y = __riscv_vle32_v_f32m8(cloud.yData() + index, vl);
    const vfloat32m8_t z = __riscv_vle32_v_f32m8(cloud.zData() + index, vl);
    const vfloat32m8_t dx = __riscv_vfsub_vf_f32m8(x, kL2QueryX, vl);
    const vfloat32m8_t dy = __riscv_vfsub_vf_f32m8(y, kL2QueryY, vl);
    const vfloat32m8_t dz = __riscv_vfsub_vf_f32m8(z, kL2QueryZ, vl);
    vfloat32m8_t d2 = __riscv_vfmul_vv_f32m8(dx, dx, vl);
    d2 = __riscv_vfmacc_vv_f32m8(d2, dy, dy, vl);
    d2 = __riscv_vfmacc_vv_f32m8(d2, dz, dz, vl);
    __riscv_vse32_v_f32m8(distances.data() + index, d2, vl);
    index += vl;
  }
#else
  for (std::size_t index = 0; index < cloud.size(); ++index) {
    const float dx = cloud.xData()[index] - kL2QueryX;
    const float dy = cloud.yData()[index] - kL2QueryY;
    const float dz = cloud.zData()[index] - kL2QueryZ;
    distances[index] = dx * dx + dy * dy + dz * dz;
  }
#endif

  return sumQuantized(distances);
}

uint64_t runReduction(const PointCloudSoA &cloud) {
  if (cloud.empty()) {
    return 0;
  }

  std::vector<float> values(cloud.size(), 0.0f);
#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
  std::size_t index = 0;
  while (index < cloud.size()) {
    const std::size_t vl = __riscv_vsetvl_e32m8(cloud.size() - index);
    const vfloat32m8_t x = __riscv_vle32_v_f32m8(cloud.xData() + index, vl);
    const vfloat32m8_t y = __riscv_vle32_v_f32m8(cloud.yData() + index, vl);
    const vfloat32m8_t z = __riscv_vle32_v_f32m8(cloud.zData() + index, vl);
    const vfloat32m8_t sum = __riscv_vfadd_vv_f32m8(__riscv_vfadd_vv_f32m8(x, y, vl), z, vl);
    __riscv_vse32_v_f32m8(values.data() + index, sum, vl);
    index += vl;
  }
#else
  for (std::size_t index = 0; index < cloud.size(); ++index) {
    values[index] = cloud.xData()[index] + cloud.yData()[index] + cloud.zData()[index];
  }
#endif

  uint64_t sum = 0;
  uint64_t sum_sq = 0;
  for (float value : values) {
    const uint64_t quantized = static_cast<uint64_t>(value);
    sum += quantized;
    sum_sq += quantized * quantized;
  }

  const double count = static_cast<double>(cloud.size());
  const double mean = static_cast<double>(sum) / count;
  const double variance = static_cast<double>(sum_sq) / count - mean * mean;
  const uint64_t mean_bits = static_cast<uint64_t>(std::llround(mean * 1000.0));
  const uint64_t variance_bits = static_cast<uint64_t>(std::llround(variance * 1000.0));
  return mean_bits ^ (variance_bits << 1);
}

uint64_t runMaskedFilter(const PointCloudSoA &cloud) {
  if (cloud.empty()) {
    return 0;
  }

  std::vector<float> sums(cloud.size(), 0.0f);
#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
  std::size_t index = 0;
  while (index < cloud.size()) {
    const std::size_t vl = __riscv_vsetvl_e32m8(cloud.size() - index);
    const vfloat32m8_t x = __riscv_vle32_v_f32m8(cloud.xData() + index, vl);
    const vfloat32m8_t y = __riscv_vle32_v_f32m8(cloud.yData() + index, vl);
    const vfloat32m8_t z = __riscv_vle32_v_f32m8(cloud.zData() + index, vl);
    const vfloat32m8_t sum = __riscv_vfadd_vv_f32m8(__riscv_vfadd_vv_f32m8(x, y, vl), z, vl);
    __riscv_vse32_v_f32m8(sums.data() + index, sum, vl);
    index += vl;
  }
#else
  for (std::size_t index = 0; index < cloud.size(); ++index) {
    sums[index] = cloud.xData()[index] + cloud.yData()[index] + cloud.zData()[index];
  }
#endif

  std::size_t count = 0;
  for (float value : sums) {
    if (value <= kFilterThreshold) {
      ++count;
    }
  }
  return static_cast<uint64_t>(count);
}

uint64_t runRadiusSearch(const PointCloudSoA &cloud) {
  if (cloud.empty()) {
    return 0;
  }

  std::vector<float> distances(cloud.size(), 0.0f);
#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
  std::size_t index = 0;
  while (index < cloud.size()) {
    const std::size_t vl = __riscv_vsetvl_e32m8(cloud.size() - index);
    const vfloat32m8_t x = __riscv_vle32_v_f32m8(cloud.xData() + index, vl);
    const vfloat32m8_t y = __riscv_vle32_v_f32m8(cloud.yData() + index, vl);
    const vfloat32m8_t z = __riscv_vle32_v_f32m8(cloud.zData() + index, vl);
    const vfloat32m8_t dx = __riscv_vfsub_vf_f32m8(x, kRadiusQueryX, vl);
    const vfloat32m8_t dy = __riscv_vfsub_vf_f32m8(y, kRadiusQueryY, vl);
    const vfloat32m8_t dz = __riscv_vfsub_vf_f32m8(z, kRadiusQueryZ, vl);
    vfloat32m8_t d2 = __riscv_vfmul_vv_f32m8(dx, dx, vl);
    d2 = __riscv_vfmacc_vv_f32m8(d2, dy, dy, vl);
    d2 = __riscv_vfmacc_vv_f32m8(d2, dz, dz, vl);
    __riscv_vse32_v_f32m8(distances.data() + index, d2, vl);
    index += vl;
  }
#else
  for (std::size_t index = 0; index < cloud.size(); ++index) {
    const float dx = cloud.xData()[index] - kRadiusQueryX;
    const float dy = cloud.yData()[index] - kRadiusQueryY;
    const float dz = cloud.zData()[index] - kRadiusQueryZ;
    distances[index] = dx * dx + dy * dy + dz * dz;
  }
#endif

  std::size_t count = 0;
  for (float distance : distances) {
    if (distance <= kRadiusSquared) {
      ++count;
    }
  }
  return static_cast<uint64_t>(count);
}

uint64_t runNormalEstimation(const PointCloudSoA &cloud) {
  if (cloud.size() < 3) {
    return 0;
  }

  const std::size_t output_size = cloud.size() - 2;
  std::vector<float> cross_x(output_size, 0.0f);
  std::vector<float> cross_y(output_size, 0.0f);
  std::vector<float> cross_z(output_size, 0.0f);

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
  std::size_t index = 0;
  while (index < output_size) {
    const std::size_t vl = __riscv_vsetvl_e32m8(output_size - index);
    const vfloat32m8_t x0 = __riscv_vle32_v_f32m8(cloud.xData() + index, vl);
    const vfloat32m8_t y0 = __riscv_vle32_v_f32m8(cloud.yData() + index, vl);
    const vfloat32m8_t z0 = __riscv_vle32_v_f32m8(cloud.zData() + index, vl);
    const vfloat32m8_t x1 = __riscv_vle32_v_f32m8(cloud.xData() + index + 1, vl);
    const vfloat32m8_t y1 = __riscv_vle32_v_f32m8(cloud.yData() + index + 1, vl);
    const vfloat32m8_t z1 = __riscv_vle32_v_f32m8(cloud.zData() + index + 1, vl);
    const vfloat32m8_t x2 = __riscv_vle32_v_f32m8(cloud.xData() + index + 2, vl);
    const vfloat32m8_t y2 = __riscv_vle32_v_f32m8(cloud.yData() + index + 2, vl);
    const vfloat32m8_t z2 = __riscv_vle32_v_f32m8(cloud.zData() + index + 2, vl);

    const vfloat32m8_t ux = __riscv_vfsub_vv_f32m8(x1, x0, vl);
    const vfloat32m8_t uy = __riscv_vfsub_vv_f32m8(y1, y0, vl);
    const vfloat32m8_t uz = __riscv_vfsub_vv_f32m8(z1, z0, vl);
    const vfloat32m8_t vx = __riscv_vfsub_vv_f32m8(x2, x0, vl);
    const vfloat32m8_t vy = __riscv_vfsub_vv_f32m8(y2, y0, vl);
    const vfloat32m8_t vz = __riscv_vfsub_vv_f32m8(z2, z0, vl);

    const vfloat32m8_t cx = __riscv_vfsub_vv_f32m8(__riscv_vfmul_vv_f32m8(uy, vz, vl),
                                                   __riscv_vfmul_vv_f32m8(uz, vy, vl), vl);
    const vfloat32m8_t cy = __riscv_vfsub_vv_f32m8(__riscv_vfmul_vv_f32m8(uz, vx, vl),
                                                   __riscv_vfmul_vv_f32m8(ux, vz, vl), vl);
    const vfloat32m8_t cz = __riscv_vfsub_vv_f32m8(__riscv_vfmul_vv_f32m8(ux, vy, vl),
                                                   __riscv_vfmul_vv_f32m8(uy, vx, vl), vl);

    __riscv_vse32_v_f32m8(cross_x.data() + index, cx, vl);
    __riscv_vse32_v_f32m8(cross_y.data() + index, cy, vl);
    __riscv_vse32_v_f32m8(cross_z.data() + index, cz, vl);
    index += vl;
  }
#else
  for (std::size_t index = 0; index < output_size; ++index) {
    const float ux = cloud.xData()[index + 1] - cloud.xData()[index];
    const float uy = cloud.yData()[index + 1] - cloud.yData()[index];
    const float uz = cloud.zData()[index + 1] - cloud.zData()[index];
    const float vx = cloud.xData()[index + 2] - cloud.xData()[index];
    const float vy = cloud.yData()[index + 2] - cloud.yData()[index];
    const float vz = cloud.zData()[index + 2] - cloud.zData()[index];
    cross_x[index] = uy * vz - uz * vy;
    cross_y[index] = uz * vx - ux * vz;
    cross_z[index] = ux * vy - uy * vx;
  }
#endif

  uint64_t checksum = 0;
  for (std::size_t index = 0; index < output_size; ++index) {
    checksum += static_cast<uint64_t>(std::fabs(cross_x[index]));
    checksum += static_cast<uint64_t>(std::fabs(cross_y[index]));
    checksum += static_cast<uint64_t>(std::fabs(cross_z[index]));
  }
  return checksum;
}

} // namespace rvv_pcl::bench