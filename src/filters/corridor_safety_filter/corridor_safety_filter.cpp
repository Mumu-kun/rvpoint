#include "filters/corridor_safety_filter/corridor_safety_filter.h"

#include <cmath>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvpoint {

SafetyEvaluationResult ForwardCorridorSafetyFilter::evaluate(const PointCloud& obstacles, float vx_mps) const {
    SafetyEvaluationResult result;
    result.stopping_distance_m = compute_stopping_distance(vx_mps);

    const float x_min = params_.min_forward_dist_m;
    const float x_max = result.stopping_distance_m;
    const float y_margin = (params_.car_width_m * 0.5f) + params_.lateral_margin_m;
    const float z_min = params_.min_obstacle_height_m;
    const float z_max = params_.car_height_m;

    const std::size_t n = obstacles.size();
    if (n == 0) return result;

    uint32_t count = 0;
    float nearest_x = 1e9f;

#if defined(__riscv_vector)
    std::size_t i = 0;
    while (i < n) {
        std::size_t vl = __riscv_vsetvl_e32m8(n - i);
        vfloat32m8_t vx = __riscv_vle32_v_f32m8(obstacles.x.data() + i, vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(obstacles.y.data() + i, vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(obstacles.z.data() + i, vl);

        vbool4_t m_xmin = __riscv_vmfge_vf_f32m8_b4(vx, x_min, vl);
        vbool4_t m_xmax = __riscv_vmfle_vf_f32m8_b4(vx, x_max, vl);
        vbool4_t m_x = __riscv_vmand_mm_b4(m_xmin, m_xmax, vl);

        vbool4_t m_ymin = __riscv_vmfge_vf_f32m8_b4(vy, -y_margin, vl);
        vbool4_t m_ymax = __riscv_vmfle_vf_f32m8_b4(vy, y_margin, vl);
        vbool4_t m_y = __riscv_vmand_mm_b4(m_ymin, m_ymax, vl);

        vbool4_t m_zmin = __riscv_vmfge_vf_f32m8_b4(vz, z_min, vl);
        vbool4_t m_zmax = __riscv_vmfle_vf_f32m8_b4(vz, z_max, vl);
        vbool4_t m_z = __riscv_vmand_mm_b4(m_zmin, m_zmax, vl);

        vbool4_t m_box = __riscv_vmand_mm_b4(__riscv_vmand_mm_b4(m_x, m_y, vl), m_z, vl);
        std::size_t sub_count = __riscv_vcpop_m_b4(m_box, vl);

        if (sub_count > 0) {
            count += static_cast<uint32_t>(sub_count);
            // Process the matched points in this chunk to find nearest_x
            for (std::size_t k = 0; k < vl; ++k) {
                float px = obstacles.x[i + k];
                float py = obstacles.y[i + k];
                float pz = obstacles.z[i + k];
                if (px >= x_min && px <= x_max &&
                    py >= -y_margin && py <= y_margin &&
                    pz >= z_min && pz <= z_max) {
                    if (px < nearest_x) {
                        nearest_x = px;
                    }
                }
            }
        }
        i += vl;
    }
#else
    for (std::size_t i = 0; i < n; ++i) {
        const float px = obstacles.x[i];
        const float py = obstacles.y[i];
        const float pz = obstacles.z[i];

        if (px >= x_min && px <= x_max &&
            py >= -y_margin && py <= y_margin &&
            pz >= z_min && pz <= z_max) {
            count++;
            if (px < nearest_x) {
                nearest_x = px;
            }
        }
    }
#endif

    result.intrusion_points_count = count;
    result.nearest_hazard_x_m = nearest_x;
    result.emergency_stop_triggered = (count >= params_.trigger_point_threshold);

    return result;
}

bool ForwardCorridorSafetyFilter::evaluate_and_actuate(const PointCloud& obstacles,
                                                       float vx_mps,
                                                       MotorActuator& actuator,
                                                       SafetyEvaluationResult* result_out) {
    SafetyEvaluationResult res = evaluate(obstacles, vx_mps);
    if (result_out) {
        *result_out = res;
    }

    if (res.emergency_stop_triggered) {
        actuator.emergency_brake();
        return true;
    }

    return false;
}

} // namespace rvpoint

