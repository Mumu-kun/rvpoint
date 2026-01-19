#include "include/rvv_pcl.h"

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
}

} // namespace rvv_pcl
