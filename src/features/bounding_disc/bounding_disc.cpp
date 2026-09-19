#include "features/bounding_disc/bounding_disc.h"

#include <cmath>
#include <algorithm>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvpoint {

BoundingDiscExtractor::BoundingDiscExtractor(std::size_t max_points, Backend backend)
    : backend_(backend) {
    reserve(max_points);
}

void BoundingDiscExtractor::reserve(std::size_t max_points) {
    scratch_x_.resize(max_points, 0.0f);
    scratch_y_.resize(max_points, 0.0f);
}

void BoundingDiscExtractor::compute_from_points(const PointCloud& cloud,
                                                const uint32_t* indices,
                                                std::size_t count,
                                                BoundingDisc& out_disc) {
    out_disc = BoundingDisc{};
    if (count == 0) return;

    out_disc.point_count = static_cast<uint32_t>(count);

    if (scratch_x_.size() < count) {
        reserve(count * 2);
    }

    float z_min = cloud.z[indices[0]];
    float z_max = cloud.z[indices[0]];

    for (std::size_t i = 0; i < count; ++i) {
        uint32_t idx = indices[i];
        scratch_x_[i] = cloud.x[idx];
        scratch_y_[i] = cloud.y[idx];
        float pz = cloud.z[idx];
        if (pz < z_min) z_min = pz;
        if (pz > z_max) z_max = pz;
    }

    bool use_rvv = false;
#if defined(__riscv_vector)
    if (backend_ == Backend::Auto || backend_ == Backend::RVV) {
        use_rvv = true;
    }
#endif

    if (use_rvv) {
        compute_points_rvv(count, z_min, z_max, out_disc);
    } else {
        compute_points_scalar(count, z_min, z_max, out_disc);
    }
}

void BoundingDiscExtractor::compute_points_scalar(std::size_t count, float z_min, float z_max, BoundingDisc& out_disc) {
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    for (std::size_t i = 0; i < count; ++i) {
        sum_x += scratch_x_[i];
        sum_y += scratch_y_[i];
    }
    const float inv_k = 1.0f / static_cast<float>(count);
    const float cx = sum_x * inv_k;
    const float cy = sum_y * inv_k;

    float max_r2 = 0.0f;
    for (std::size_t i = 0; i < count; ++i) {
        float dx = scratch_x_[i] - cx;
        float dy = scratch_y_[i] - cy;
        float r2 = dx * dx + dy * dy;
        if (r2 > max_r2) max_r2 = r2;
    }

    out_disc.cx = cx;
    out_disc.cy = cy;
    out_disc.radius = std::sqrt(max_r2);
    out_disc.z_min = z_min;
    out_disc.z_max = z_max;
}

void BoundingDiscExtractor::compute_points_rvv(std::size_t count, float z_min, float z_max, BoundingDisc& out_disc) {
#if defined(__riscv_vector)
    // 1. RVV Sum Reduction for Centroid (cx, cy)
    std::size_t i = 0;
    vfloat32m1_t v_sum_x = __riscv_vfmv_s_f_f32m1(0.0f, 1);
    vfloat32m1_t v_sum_y = __riscv_vfmv_s_f_f32m1(0.0f, 1);

    while (i < count) {
        std::size_t vl = __riscv_vsetvl_e32m8(count - i);
        vfloat32m8_t vx = __riscv_vle32_v_f32m8(scratch_x_.data() + i, vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(scratch_y_.data() + i, vl);
        v_sum_x = __riscv_vfredusum_vs_f32m8_f32m1(vx, v_sum_x, vl);
        v_sum_y = __riscv_vfredusum_vs_f32m8_f32m1(vy, v_sum_y, vl);
        i += vl;
    }
    float sum_x = __riscv_vfmv_f_s_f32m1_f32(v_sum_x);
    float sum_y = __riscv_vfmv_f_s_f32m1_f32(v_sum_y);

    const float inv_k = 1.0f / static_cast<float>(count);
    const float cx = sum_x * inv_k;
    const float cy = sum_y * inv_k;

    // 2. RVV Max Distance Squared Reduction from Centroid
    i = 0;
    vfloat32m1_t v_max_r2 = __riscv_vfmv_s_f_f32m1(0.0f, 1);

    while (i < count) {
        std::size_t vl = __riscv_vsetvl_e32m8(count - i);
        vfloat32m8_t vx = __riscv_vle32_v_f32m8(scratch_x_.data() + i, vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(scratch_y_.data() + i, vl);
        vfloat32m8_t dx = __riscv_vfsub_vf_f32m8(vx, cx, vl);
        vfloat32m8_t dy = __riscv_vfsub_vf_f32m8(vy, cy, vl);
        vfloat32m8_t r2 = __riscv_vfmul_vv_f32m8(dx, dx, vl);
        r2 = __riscv_vfmacc_vv_f32m8(r2, dy, dy, vl);
        v_max_r2 = __riscv_vfredmax_vs_f32m8_f32m1(r2, v_max_r2, vl);
        i += vl;
    }
    float max_r2 = __riscv_vfmv_f_s_f32m1_f32(v_max_r2);

    out_disc.cx = cx;
    out_disc.cy = cy;
    out_disc.radius = std::sqrt(max_r2);
    out_disc.z_min = z_min;
    out_disc.z_max = z_max;
#else
    compute_points_scalar(count, z_min, z_max, out_disc);
#endif
}

void BoundingDiscExtractor::compute_concentric(const OrientedBoundingBox& box,
                                              BoundingDisc& out_disc) const {
    out_disc.cx = box.cx;
    out_disc.cy = box.cy;
    out_disc.radius = 0.5f * std::sqrt(box.extent_x * box.extent_x +
                                       box.extent_y * box.extent_y);
    out_disc.z_min = box.cz - 0.5f * box.extent_z;
    out_disc.z_max = box.cz + 0.5f * box.extent_z;
    out_disc.point_count = box.point_count;
}

} // namespace rvpoint
