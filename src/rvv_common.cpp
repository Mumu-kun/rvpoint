#include "include/rvv_pcl.h"
#include <cstdio>

namespace rvv_pcl {

// ============================================================================
// Helper: Squared Euclidean Distance Kernel (RVV)
// ============================================================================
void get_dist_sq_rvv(const float* x, const float* y, const float* z,
                     float qx, float qy, float qz,
                     float* out_d2, std::size_t n)
{
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
} // namespace rvv_pcl

// ============================================================================
// Fused Gather-Filter Kernel
// ============================================================================
void get_inds_in_radius_rvv(const float* x, const float* y, const float* z,
                            const int* subset_indices, std::size_t n,
                            float qx, float qy, float qz, float r2,
                            std::vector<int>& out_indices, 
                            std::vector<float>& out_dists)
{
    // Use LMUL=2 for better stability/register allocation
    std::size_t i = 0;
    while (i < n) {
        std::size_t vl = __riscv_vsetvl_e32m2(n - i);

        // 1. Load Indices
        vint32m2_t v_idx = __riscv_vle32_v_i32m2(&subset_indices[i], vl);
        vuint32m2_t v_uidx = __riscv_vreinterpret_v_i32m2_u32m2(v_idx);
        
        // IMPORTANT: vluxei32 expects BYTE OFFSETS, not element indices!
        // Shift left by 2 (multiply by 4) for float (32-bit) access
        vuint32m2_t v_byte_offsets = __riscv_vsll_vx_u32m2(v_uidx, 2, vl);

        // 2. Gather X, Y, Z (Indexed Load using Byte Offsets)
        vfloat32m2_t vx = __riscv_vluxei32_v_f32m2(x, v_byte_offsets, vl);
        vfloat32m2_t vy = __riscv_vluxei32_v_f32m2(y, v_byte_offsets, vl);
        vfloat32m2_t vz = __riscv_vluxei32_v_f32m2(z, v_byte_offsets, vl);

        // 3. Subtract Query
        vfloat32m2_t dx = __riscv_vfsub_vf_f32m2(vx, qx, vl);
        vfloat32m2_t dy = __riscv_vfsub_vf_f32m2(vy, qy, vl);
        vfloat32m2_t dz = __riscv_vfsub_vf_f32m2(vz, qz, vl);

        // 4. Square and Add
        vfloat32m2_t dx2 = __riscv_vfmul_vv_f32m2(dx, dx, vl);
        vfloat32m2_t dy2 = __riscv_vfmul_vv_f32m2(dy, dy, vl);
        vfloat32m2_t dz2 = __riscv_vfmul_vv_f32m2(dz, dz, vl);
        vfloat32m2_t dist2 = __riscv_vfadd_vv_f32m2(dx2, dy2, vl);
        dist2 = __riscv_vfadd_vv_f32m2(dist2, dz2, vl);

        // 5. Compare <= r2
        vbool16_t mask = __riscv_vmfle_vf_f32m2_b16(dist2, r2, vl);

        // 6. Compress (Filter)
        long count = __riscv_vcpop_m_b16(mask, vl);
        if (count > 0) {
            size_t old_size = out_indices.size();
            out_indices.resize(old_size + count);
            out_dists.resize(old_size + count);
            
            // Reload/Reuse v_idx (original element indices) for output
            vint32m2_t v_valid_idx = __riscv_vcompress_vm_i32m2(v_idx, mask, vl);
            vfloat32m2_t v_valid_dist = __riscv_vcompress_vm_f32m2(dist2, mask, vl);
            
            __riscv_vse32_v_i32m2(&out_indices[old_size], v_valid_idx, count);
            __riscv_vse32_v_f32m2(&out_dists[old_size], v_valid_dist, count);
        }

        i += vl;
    }
}

} // namespace rvv_pcl
