#include "filters/passthrough_filter/passthrough_filter.h"

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvpoint {

PassThroughFilter::PassThroughFilter(PassThroughParams params)
    : params_(params) {}

PassThroughFilter::PassThroughFilter(float min_z, float max_z) {
    params_.z_range = {min_z, max_z};
}

void PassThroughFilter::filter(const PointCloud& in, PointCloud& out, PointCloud* rejected) const {
    out.clear();
    out.reserve(in.size());
    if (rejected) {
        rejected->clear();
        rejected->reserve(in.size());
    }

    const float x_min = params_.x_range.min_val;
    const float x_max = params_.x_range.max_val;
    const float y_min = params_.y_range.min_val;
    const float y_max = params_.y_range.max_val;
    const float z_min = params_.z_range.min_val;
    const float z_max = params_.z_range.max_val;
    const bool keep_inliers = params_.keep_inliers;

    const std::size_t n = in.size();

#if defined(__riscv_vector)
    std::size_t i = 0;
    while (i < n) {
        std::size_t vl = __riscv_vsetvl_e32m8(n - i);
        vfloat32m8_t vx = __riscv_vle32_v_f32m8(in.x.data() + i, vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(in.y.data() + i, vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(in.z.data() + i, vl);

        vbool4_t m_xmin = __riscv_vmfge_vf_f32m8_b4(vx, x_min, vl);
        vbool4_t m_xmax = __riscv_vmfle_vf_f32m8_b4(vx, x_max, vl);
        vbool4_t m_x = __riscv_vmand_mm_b4(m_xmin, m_xmax, vl);

        vbool4_t m_ymin = __riscv_vmfge_vf_f32m8_b4(vy, y_min, vl);
        vbool4_t m_ymax = __riscv_vmfle_vf_f32m8_b4(vy, y_max, vl);
        vbool4_t m_y = __riscv_vmand_mm_b4(m_ymin, m_ymax, vl);

        vbool4_t m_zmin = __riscv_vmfge_vf_f32m8_b4(vz, z_min, vl);
        vbool4_t m_zmax = __riscv_vmfle_vf_f32m8_b4(vz, z_max, vl);
        vbool4_t m_z = __riscv_vmand_mm_b4(m_zmin, m_zmax, vl);

        vbool4_t m_box = __riscv_vmand_mm_b4(__riscv_vmand_mm_b4(m_x, m_y, vl), m_z, vl);

        // Extract scalar points for compaction
        for (std::size_t k = 0; k < vl; ++k) {
            float px = in.x[i + k];
            float py = in.y[i + k];
            float pz = in.z[i + k];
            bool inside = (px >= x_min && px <= x_max &&
                           py >= y_min && py <= y_max &&
                           pz >= z_min && pz <= z_max);
            bool pass = (inside == keep_inliers);
            if (pass) {
                out.push_back(px, py, pz);
            } else if (rejected) {
                rejected->push_back(px, py, pz);
            }
        }
        i += vl;
    }
#else
    for (std::size_t i = 0; i < n; ++i) {
        const float px = in.x[i];
        const float py = in.y[i];
        const float pz = in.z[i];

        bool inside = (px >= x_min && px <= x_max &&
                       py >= y_min && py <= y_max &&
                       pz >= z_min && pz <= z_max);
        bool pass = (inside == keep_inliers);
        if (pass) {
            out.push_back(px, py, pz);
        } else if (rejected) {
            rejected->push_back(px, py, pz);
        }
    }
#endif
}

} // namespace rvpoint
