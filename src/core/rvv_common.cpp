#include "core/rvv_common.h"
#include <cstdio>

namespace rvpoint {

// ============================================================================
// Helper: Squared Euclidean Distance Kernel (RVV)
// ============================================================================
void get_dist_sq_rvv(const float* x, const float* y, const float* z,
                     float qx, float qy, float qz,
                     float* out_d2, std::size_t n)
{
#if defined(__riscv_vector)
  std::size_t i = 0;
  while (i < n) {
    std::size_t vl = __riscv_vsetvl_e32m8(n - i);

    vfloat32m8_t vx = __riscv_vle32_v_f32m8(&x[i], vl);
    vfloat32m8_t vy = __riscv_vle32_v_f32m8(&y[i], vl);
    vfloat32m8_t vz = __riscv_vle32_v_f32m8(&z[i], vl);

    vfloat32m8_t dx = __riscv_vfsub_vf_f32m8(vx, qx, vl);
    vfloat32m8_t dy = __riscv_vfsub_vf_f32m8(vy, qy, vl);
    vfloat32m8_t dz = __riscv_vfsub_vf_f32m8(vz, qz, vl);

    vfloat32m8_t dx2 = __riscv_vfmul_vv_f32m8(dx, dx, vl);
    vfloat32m8_t dy2 = __riscv_vfmul_vv_f32m8(dy, dy, vl);
    vfloat32m8_t dz2 = __riscv_vfmul_vv_f32m8(dz, dz, vl);

    vfloat32m8_t sum = __riscv_vfadd_vv_f32m8(dx2, dy2, vl);
    sum = __riscv_vfadd_vv_f32m8(sum, dz2, vl);

    __riscv_vse32_v_f32m8(&out_d2[i], sum, vl);
    i += vl;
  }
#else
  for (std::size_t i = 0; i < n; ++i) {
    float dx = x[i] - qx;
    float dy = y[i] - qy;
    float dz = z[i] - qz;
    out_d2[i] = dx * dx + dy * dy + dz * dz;
  }
#endif
}

// ============================================================================
// Fused Gather-Filter Kernel
// ============================================================================
void get_inds_in_radius_rvv(const float* x, const float* y, const float* z,
                            const int* subset_indices, std::size_t n,
                            float qx, float qy, float qz, float r2,
                            std::vector<int>& out_indices,
                            std::vector<float>& out_dists)
{
#if defined(__riscv_vector)
    // ── Full RVV path: vluxei32 indexed gather + vectorized Euclidean distance ─────────────
    std::size_t i = 0;
    while (i < n) {
        std::size_t vl = __riscv_vsetvl_e32m2(n - i);

        vint32m2_t v_idx = __riscv_vle32_v_i32m2(&subset_indices[i], vl);
        vuint32m2_t v_uidx = __riscv_vreinterpret_v_i32m2_u32m2(v_idx);
        // vluxei32 expects BYTE offsets: multiply element index by 4
        vuint32m2_t v_byte_offsets = __riscv_vsll_vx_u32m2(v_uidx, 2, vl);

        vfloat32m2_t vx = __riscv_vluxei32_v_f32m2(x, v_byte_offsets, vl);
        vfloat32m2_t vy = __riscv_vluxei32_v_f32m2(y, v_byte_offsets, vl);
        vfloat32m2_t vz = __riscv_vluxei32_v_f32m2(z, v_byte_offsets, vl);

        vfloat32m2_t dx = __riscv_vfsub_vf_f32m2(vx, qx, vl);
        vfloat32m2_t dy = __riscv_vfsub_vf_f32m2(vy, qy, vl);
        vfloat32m2_t dz = __riscv_vfsub_vf_f32m2(vz, qz, vl);
        vfloat32m2_t dx2 = __riscv_vfmul_vv_f32m2(dx, dx, vl);
        vfloat32m2_t dy2 = __riscv_vfmul_vv_f32m2(dy, dy, vl);
        vfloat32m2_t dz2 = __riscv_vfmul_vv_f32m2(dz, dz, vl);
        vfloat32m2_t dist2 = __riscv_vfadd_vv_f32m2(
                             __riscv_vfadd_vv_f32m2(dx2, dy2, vl), dz2, vl);

        vbool16_t mask = __riscv_vmfle_vf_f32m2_b16(dist2, r2, vl);
        long count = __riscv_vcpop_m_b16(mask, vl);
        if (count > 0) {
            size_t old_size = out_indices.size();
            out_indices.resize(old_size + count);
            out_dists.resize(old_size + count);
#ifndef GEM5_BUILD
            __riscv_vse32_v_i32m2(&out_indices[old_size],
                __riscv_vcompress_vm_i32m2(v_idx, mask, vl), count);
            __riscv_vse32_v_f32m2(&out_dists[old_size],
                __riscv_vcompress_vm_f32m2(dist2, mask, vl), count);
#else
            alignas(64) int t_idx[kMaxVectorFloatsM8];
            alignas(64) float t_d2[kMaxVectorFloatsM8];
            alignas(8) uint8_t m_bytes[kMaxVectorMaskBytesM8] = {0};
            __riscv_vse32_v_i32m2(t_idx, v_idx, vl);
            __riscv_vse32_v_f32m2(t_d2, dist2, vl);
            __riscv_vsm_v_b16(m_bytes, mask, vl);
            size_t written = 0;
            for (size_t k = 0; k < vl; ++k) {
                if ((m_bytes[k >> 3] >> (k & 7)) & 1) {
                    out_indices[old_size + written] = t_idx[k];
                    out_dists[old_size + written] = t_d2[k];
                    written++;
                }
            }
#endif
        }
        i += vl;
    }
#else
    for (std::size_t i = 0; i < n; ++i) {
        int idx = subset_indices[i];
        float dx = x[idx] - qx, dy = y[idx] - qy, dz = z[idx] - qz;
        float d2 = dx*dx + dy*dy + dz*dz;
        if (d2 <= r2) {
            out_indices.push_back(idx);
            out_dists.push_back(d2);
        }
    }
#endif
}

} // namespace rvpoint
