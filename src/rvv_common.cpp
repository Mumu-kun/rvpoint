#include "include/rvv_pcl.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rvv_pcl {

namespace {

void scalar_add(const float *a, const float *b, float *result, std::size_t n) {
  for (std::size_t i = 0; i < n; ++i) {
    result[i] = a[i] + b[i];
  }
}

void scalar_sub(const float *a, const float *b, float *result, std::size_t n) {
  for (std::size_t i = 0; i < n; ++i) {
    result[i] = a[i] - b[i];
  }
}

void scalar_mul(const float *a, const float *b, float *result, std::size_t n) {
  for (std::size_t i = 0; i < n; ++i) {
    result[i] = a[i] * b[i];
  }
}

void scalar_div(const float *a, const float *b, float *result, std::size_t n) {
  for (std::size_t i = 0; i < n; ++i) {
    result[i] = a[i] / b[i];
  }
}

} // namespace

void RVVHelper::vadd(const float *a, const float *b, float *result, std::size_t n) {
#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
  std::size_t i = 0;
  while (i < n) {
    std::size_t vl = __riscv_vsetvl_e32m8(n - i);
    vfloat32m8_t va = __riscv_vle32_v_f32m8(a + i, vl);
    vfloat32m8_t vb = __riscv_vle32_v_f32m8(b + i, vl);
    __riscv_vse32_v_f32m8(result + i, __riscv_vfadd_vv_f32m8(va, vb, vl), vl);
    i += vl;
  }
#else
  scalar_add(a, b, result, n);
#endif
}

void RVVHelper::vsub(const float *a, const float *b, float *result, std::size_t n) {
#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
  std::size_t i = 0;
  while (i < n) {
    std::size_t vl = __riscv_vsetvl_e32m8(n - i);
    vfloat32m8_t va = __riscv_vle32_v_f32m8(a + i, vl);
    vfloat32m8_t vb = __riscv_vle32_v_f32m8(b + i, vl);
    __riscv_vse32_v_f32m8(result + i, __riscv_vfsub_vv_f32m8(va, vb, vl), vl);
    i += vl;
  }
#else
  scalar_sub(a, b, result, n);
#endif
}

void RVVHelper::vmul(const float *a, const float *b, float *result, std::size_t n) {
#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
  std::size_t i = 0;
  while (i < n) {
    std::size_t vl = __riscv_vsetvl_e32m8(n - i);
    vfloat32m8_t va = __riscv_vle32_v_f32m8(a + i, vl);
    vfloat32m8_t vb = __riscv_vle32_v_f32m8(b + i, vl);
    __riscv_vse32_v_f32m8(result + i, __riscv_vfmul_vv_f32m8(va, vb, vl), vl);
    i += vl;
  }
#else
  scalar_mul(a, b, result, n);
#endif
}

void RVVHelper::vdiv(const float *a, const float *b, float *result, std::size_t n) {
#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
  std::size_t i = 0;
  while (i < n) {
    std::size_t vl = __riscv_vsetvl_e32m8(n - i);
    vfloat32m8_t va = __riscv_vle32_v_f32m8(a + i, vl);
    vfloat32m8_t vb = __riscv_vle32_v_f32m8(b + i, vl);
    __riscv_vse32_v_f32m8(result + i, __riscv_vfdiv_vv_f32m8(va, vb, vl), vl);
    i += vl;
  }
#else
  scalar_div(a, b, result, n);
#endif
}

void RVVHelper::vfmadd(const float *a, const float *b, const float *c, float *result,
                       std::size_t n) {
#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
  std::size_t i = 0;
  while (i < n) {
    std::size_t vl = __riscv_vsetvl_e32m8(n - i);
    vfloat32m8_t va = __riscv_vle32_v_f32m8(a + i, vl);
    vfloat32m8_t vb = __riscv_vle32_v_f32m8(b + i, vl);
    vfloat32m8_t vc = __riscv_vle32_v_f32m8(c + i, vl);
    __riscv_vse32_v_f32m8(result + i, __riscv_vfmacc_vv_f32m8(vc, va, vb, vl), vl);
    i += vl;
  }
#else
  for (std::size_t i = 0; i < n; ++i) {
    result[i] = a[i] * b[i] + c[i];
  }
#endif
}

void RVVHelper::vsqrt(const float *a, float *result, std::size_t n) {
  for (std::size_t i = 0; i < n; ++i) {
    result[i] = std::sqrt(a[i]);
  }
}

void RVVHelper::vrsqrt(const float *a, float *result, std::size_t n) {
  for (std::size_t i = 0; i < n; ++i) {
    result[i] = 1.0f / std::sqrt(a[i]);
  }
}

float RVVHelper::vsum(const float *a, std::size_t n) {
#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
  float total = 0.0f;
  std::size_t i = 0;
  while (i < n) {
    std::size_t vl = __riscv_vsetvl_e32m8(n - i);
    vfloat32m8_t v = __riscv_vle32_v_f32m8(a + i, vl);
    vfloat32m1_t seed = __riscv_vfmv_s_f_f32m1(0.0f, 1);
    vfloat32m1_t reduced = __riscv_vfredusum_vs_f32m8_f32m1(v, seed, vl);
    total += __riscv_vfmv_f_s_f32m1_f32(reduced);
    i += vl;
  }
  return total;
#else
  float total = 0.0f;
  for (std::size_t i = 0; i < n; ++i) {
    total += a[i];
  }
  return total;
#endif
}

float RVVHelper::vmax(const float *a, std::size_t n) {
  float value = std::numeric_limits<float>::lowest();
  for (std::size_t i = 0; i < n; ++i) {
    value = std::max(value, a[i]);
  }
  return value;
}

float RVVHelper::vmin(const float *a, std::size_t n) {
  float value = std::numeric_limits<float>::max();
  for (std::size_t i = 0; i < n; ++i) {
    value = std::min(value, a[i]);
  }
  return value;
}

float RVVHelper::vdot(const float *a, const float *b, std::size_t n) {
  float total = 0.0f;
  for (std::size_t i = 0; i < n; ++i) {
    total += a[i] * b[i];
  }
  return total;
}

void RVVHelper::distanceSquared(const float *px, const float *py, const float *pz, std::size_t n,
                                float qx, float qy, float qz, float *out_d2) {
#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
  std::size_t i = 0;
  while (i < n) {
    std::size_t vl = __riscv_vsetvl_e32m8(n - i);
    vfloat32m8_t vx = __riscv_vle32_v_f32m8(px + i, vl);
    vfloat32m8_t vy = __riscv_vle32_v_f32m8(py + i, vl);
    vfloat32m8_t vz = __riscv_vle32_v_f32m8(pz + i, vl);
    vfloat32m8_t dx = __riscv_vfsub_vf_f32m8(vx, qx, vl);
    vfloat32m8_t dy = __riscv_vfsub_vf_f32m8(vy, qy, vl);
    vfloat32m8_t dz = __riscv_vfsub_vf_f32m8(vz, qz, vl);
    vfloat32m8_t d2 = __riscv_vfmul_vv_f32m8(dx, dx, vl);
    d2 = __riscv_vfmacc_vv_f32m8(d2, dy, dy, vl);
    d2 = __riscv_vfmacc_vv_f32m8(d2, dz, dz, vl);
    __riscv_vse32_v_f32m8(out_d2 + i, d2, vl);
    i += vl;
  }
#else
  for (std::size_t i = 0; i < n; ++i) {
    const float dx = px[i] - qx;
    const float dy = py[i] - qy;
    const float dz = pz[i] - qz;
    out_d2[i] = dx * dx + dy * dy + dz * dz;
  }
#endif
}

void RVVHelper::planeDistances(const float *px, const float *py, const float *pz, std::size_t n,
                             const std::array<float, 4> &coeffs, float *out_dist) {
#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
  std::size_t i = 0;
  while (i < n) {
    std::size_t vl = __riscv_vsetvl_e32m8(n - i);
    vfloat32m8_t vx = __riscv_vle32_v_f32m8(px + i, vl);
    vfloat32m8_t vy = __riscv_vle32_v_f32m8(py + i, vl);
    vfloat32m8_t vz = __riscv_vle32_v_f32m8(pz + i, vl);
    vfloat32m8_t dist = __riscv_vfmul_vf_f32m8(vx, coeffs[0], vl);
    dist = __riscv_vfmacc_vf_f32m8(dist, coeffs[1], vy, vl);
    dist = __riscv_vfmacc_vf_f32m8(dist, coeffs[2], vz, vl);
    dist = __riscv_vfadd_vf_f32m8(dist, coeffs[3], vl);
    __riscv_vse32_v_f32m8(out_dist + i, dist, vl);
    i += vl;
  }
#else
  for (std::size_t i = 0; i < n; ++i) {
    out_dist[i] = coeffs[0] * px[i] + coeffs[1] * py[i] + coeffs[2] * pz[i] + coeffs[3];
  }
#endif
}

void RVVHelper::gatherDistanceSquared(const float *px, const float *py, const float *pz,
                                    const int *subset_indices, std::size_t n,
                                    float qx, float qy, float qz, float *out_d2) {
#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
  std::size_t i = 0;
  while (i < n) {
    std::size_t vl = __riscv_vsetvl_e32m2(n - i);
    vint32m2_t v_idx = __riscv_vle32_v_i32m2(subset_indices + i, vl);
    vuint32m2_t v_uidx = __riscv_vreinterpret_v_i32m2_u32m2(v_idx);
    vuint32m2_t v_byte_offsets = __riscv_vsll_vx_u32m2(v_uidx, 2, vl);
    vfloat32m2_t vx = __riscv_vluxei32_v_f32m2(px, v_byte_offsets, vl);
    vfloat32m2_t vy = __riscv_vluxei32_v_f32m2(py, v_byte_offsets, vl);
    vfloat32m2_t vz = __riscv_vluxei32_v_f32m2(pz, v_byte_offsets, vl);
    vfloat32m2_t dx = __riscv_vfsub_vf_f32m2(vx, qx, vl);
    vfloat32m2_t dy = __riscv_vfsub_vf_f32m2(vy, qy, vl);
    vfloat32m2_t dz = __riscv_vfsub_vf_f32m2(vz, qz, vl);
    vfloat32m2_t dist2 = __riscv_vfmul_vv_f32m2(dx, dx, vl);
    dist2 = __riscv_vfmacc_vv_f32m2(dist2, dy, dy, vl);
    dist2 = __riscv_vfmacc_vv_f32m2(dist2, dz, dz, vl);
    __riscv_vse32_v_f32m2(out_d2 + i, dist2, vl);
    i += vl;
  }
#else
  for (std::size_t i = 0; i < n; ++i) {
    const int idx = subset_indices[i];
    const float dx = px[idx] - qx;
    const float dy = py[idx] - qy;
    const float dz = pz[idx] - qz;
    out_d2[i] = dx * dx + dy * dy + dz * dz;
  }
#endif
}

} // namespace rvv_pcl
