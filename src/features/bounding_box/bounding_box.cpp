#include "features/bounding_box/bounding_box.h"

#include <algorithm>
#include <cmath>
#include <limits>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvpoint {

namespace {
constexpr float kPi = 3.14159265358979323846f;

inline float cross_2d(const Point2D& o, const Point2D& a, const Point2D& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

inline float normalize_angle(float a) {
    while (a > kPi) a -= 2.0f * kPi;
    while (a <= -kPi) a += 2.0f * kPi;
    return a;
}
} // namespace

ObstacleGeometryExtractor::ObstacleGeometryExtractor(std::size_t max_points) {
    reserve(max_points);
}

void ObstacleGeometryExtractor::reserve(std::size_t max_points) {
    pts_2d_scratch_.reserve(max_points);
    hull_scratch_.reserve(max_points);
    scratch_x_.reserve(max_points);
    scratch_y_.reserve(max_points);
    scratch_z_.reserve(max_points);
}

void ObstacleGeometryExtractor::compute_disc(const PointCloud& cloud,
                                            const uint32_t* indices,
                                            std::size_t count,
                                            BoundingDisc& out) {
    out = BoundingDisc{};
    if (count == 0) return;

    out.point_count = static_cast<uint32_t>(count);

    if (count == 1) {
        uint32_t idx = indices[0];
        out.cx = cloud.x[idx];
        out.cy = cloud.y[idx];
        out.radius = 0.0f;
        out.z_min = cloud.z[idx];
        out.z_max = cloud.z[idx];
        return;
    }

    // Gather coordinates into contiguous scratch buffers
    if (scratch_x_.size() < count) scratch_x_.resize(count);
    if (scratch_y_.size() < count) scratch_y_.resize(count);
    if (scratch_z_.size() < count) scratch_z_.resize(count);

    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float z_min = cloud.z[indices[0]];
    float z_max = cloud.z[indices[0]];

    for (std::size_t i = 0; i < count; ++i) {
        uint32_t idx = indices[i];
        float px = cloud.x[idx];
        float py = cloud.y[idx];
        float pz = cloud.z[idx];
        scratch_x_[i] = px;
        scratch_y_[i] = py;
        scratch_z_[i] = pz;
        sum_x += px;
        sum_y += py;
        if (pz < z_min) z_min = pz;
        if (pz > z_max) z_max = pz;
    }

    const float inv_k = 1.0f / static_cast<float>(count);
    const float cx = sum_x * inv_k;
    const float cy = sum_y * inv_k;

    float max_r2 = 0.0f;

#if defined(__riscv_vector)
    std::size_t i = 0;
    while (i < count) {
        std::size_t vl = __riscv_vsetvl_e32m8(count - i);
        vfloat32m8_t vx = __riscv_vle32_v_f32m8(scratch_x_.data() + i, vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(scratch_y_.data() + i, vl);

        vfloat32m8_t dx = __riscv_vfsub_vf_f32m8(vx, cx, vl);
        vfloat32m8_t dy = __riscv_vfsub_vf_f32m8(vy, cy, vl);

        vfloat32m8_t r2 = __riscv_vfmul_vv_f32m8(dx, dx, vl);
        r2 = __riscv_vfmacc_vv_f32m8(r2, dy, dy, vl);

        vfloat32m1_t v_max = __riscv_vfmv_s_f_f32m1(max_r2, vl);
        v_max = __riscv_vfredmax_vs_f32m8_f32m1(r2, v_max, vl);
        float local_max = __riscv_vfmv_f_s_f32m1_f32(v_max);
        if (local_max > max_r2) max_r2 = local_max;

        i += vl;
    }
#else
    for (std::size_t i = 0; i < count; ++i) {
        float dx = scratch_x_[i] - cx;
        float dy = scratch_y_[i] - cy;
        float r2 = dx * dx + dy * dy;
        if (r2 > max_r2) max_r2 = r2;
    }
#endif

    out.cx = cx;
    out.cy = cy;
    out.radius = std::sqrt(max_r2);
    out.z_min = z_min;
    out.z_max = z_max;
}

void ObstacleGeometryExtractor::compute_convex_hull_2d(const PointCloud& cloud,
                                                      const uint32_t* indices,
                                                      std::size_t count,
                                                      std::vector<Point2D>& out_hull) {
    out_hull.clear();
    if (count == 0) return;

    if (count <= 2) {
        for (std::size_t i = 0; i < count; ++i) {
            uint32_t idx = indices[i];
            out_hull.push_back({cloud.x[idx], cloud.y[idx], idx});
        }
        return;
    }

    if (pts_2d_scratch_.size() < count) pts_2d_scratch_.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        uint32_t idx = indices[i];
        pts_2d_scratch_[i] = {cloud.x[idx], cloud.y[idx], idx};
    }

    // Lexicographical sort by X then Y
    std::sort(pts_2d_scratch_.begin(), pts_2d_scratch_.begin() + count,
              [](const Point2D& a, const Point2D& b) {
                  if (std::abs(a.x - b.x) > 1e-6f) return a.x < b.x;
                  return a.y < b.y;
              });

    // Andrew's Monotone Chain
    // 1. Lower hull
    for (std::size_t i = 0; i < count; ++i) {
        while (out_hull.size() >= 2 &&
               cross_2d(out_hull[out_hull.size() - 2], out_hull.back(), pts_2d_scratch_[i]) <= 1e-6f) {
            out_hull.pop_back();
        }
        out_hull.push_back(pts_2d_scratch_[i]);
    }

    // 2. Upper hull
    std::size_t lower_hull_size = out_hull.size() + 1;
    for (int i = static_cast<int>(count) - 2; i >= 0; --i) {
        while (out_hull.size() >= lower_hull_size &&
               cross_2d(out_hull[out_hull.size() - 2], out_hull.back(), pts_2d_scratch_[i]) <= 1e-6f) {
            out_hull.pop_back();
        }
        out_hull.push_back(pts_2d_scratch_[i]);
    }

    // Remove duplicate first point appended at end of upper hull
    if (out_hull.size() > 1) {
        out_hull.pop_back();
    }
}

void ObstacleGeometryExtractor::compute_obb(const PointCloud& cloud,
                                           const uint32_t* indices,
                                           std::size_t count,
                                           OrientedBoundingBox& out) {
    out = OrientedBoundingBox{};
    if (count == 0) return;

    out.point_count = static_cast<uint32_t>(count);

    float z_min = cloud.z[indices[0]];
    float z_max = cloud.z[indices[0]];
    for (std::size_t i = 1; i < count; ++i) {
        float pz = cloud.z[indices[i]];
        if (pz < z_min) z_min = pz;
        if (pz > z_max) z_max = pz;
    }

    compute_convex_hull_2d(cloud, indices, count, hull_scratch_);
    const std::size_t M = hull_scratch_.size();

    // Fallback to Axis-Aligned Bounding Box if hull has fewer than 3 vertices
    if (M < 3) {
        float x_min = cloud.x[indices[0]], x_max = cloud.x[indices[0]];
        float y_min = cloud.y[indices[0]], y_max = cloud.y[indices[0]];
        for (std::size_t i = 1; i < count; ++i) {
            uint32_t idx = indices[i];
            if (cloud.x[idx] < x_min) x_min = cloud.x[idx];
            if (cloud.x[idx] > x_max) x_max = cloud.x[idx];
            if (cloud.y[idx] < y_min) y_min = cloud.y[idx];
            if (cloud.y[idx] > y_max) y_max = cloud.y[idx];
        }
        out.cx = 0.5f * (x_min + x_max);
        out.cy = 0.5f * (y_min + y_max);
        out.cz = 0.5f * (z_min + z_max);
        out.extent_x = x_max - x_min;
        out.extent_y = y_max - y_min;
        out.extent_z = z_max - z_min;
        out.yaw_rad = 0.0f;

        out.corners_x[0] = x_min; out.corners_y[0] = y_min;
        out.corners_x[1] = x_max; out.corners_y[1] = y_min;
        out.corners_x[2] = x_max; out.corners_y[2] = y_max;
        out.corners_x[3] = x_min; out.corners_y[3] = y_max;
        return;
    }

    // Rotating Calipers minimum-area bounding box
    float min_area = std::numeric_limits<float>::max();
    OrientedBoundingBox best_box;

    for (std::size_t i = 0; i < M; ++i) {
        std::size_t j = (i + 1) % M;
        float dx = hull_scratch_[j].x - hull_scratch_[i].x;
        float dy = hull_scratch_[j].y - hull_scratch_[i].y;
        float edge_len = std::sqrt(dx * dx + dy * dy);
        if (edge_len < 1e-6f) continue;

        // Orthonormal basis along edge
        float ux = dx / edge_len;
        float uy = dy / edge_len;
        float vx = -uy;
        float vy = ux;

        float u_min = std::numeric_limits<float>::max();
        float u_max = -std::numeric_limits<float>::max();
        float v_min = std::numeric_limits<float>::max();
        float v_max = -std::numeric_limits<float>::max();

        for (std::size_t k = 0; k < M; ++k) {
            float pu = hull_scratch_[k].x * ux + hull_scratch_[k].y * uy;
            float pv = hull_scratch_[k].x * vx + hull_scratch_[k].y * vy;
            if (pu < u_min) u_min = pu;
            if (pu > u_max) u_max = pu;
            if (pv < v_min) v_min = pv;
            if (pv > v_max) v_max = pv;
        }

        float len_u = u_max - u_min;
        float len_v = v_max - v_min;
        float area = len_u * len_v;

        if (area < min_area) {
            min_area = area;
            float cu = 0.5f * (u_min + u_max);
            float cv = 0.5f * (v_min + v_max);

            best_box.cx = cu * ux + cv * vx;
            best_box.cy = cu * uy + cv * vy;
            best_box.cz = 0.5f * (z_min + z_max);
            best_box.extent_x = len_u;
            best_box.extent_y = len_v;
            best_box.extent_z = z_max - z_min;
            best_box.yaw_rad = std::atan2(uy, ux);
            best_box.point_count = static_cast<uint32_t>(count);

            // 4 CCW footprint corners in (x, y)
            best_box.corners_x[0] = u_min * ux + v_min * vx;
            best_box.corners_y[0] = u_min * uy + v_min * vy;
            best_box.corners_x[1] = u_max * ux + v_min * vx;
            best_box.corners_y[1] = u_max * uy + v_min * vy;
            best_box.corners_x[2] = u_max * ux + v_max * vx;
            best_box.corners_y[2] = u_max * uy + v_max * vy;
            best_box.corners_x[3] = u_min * ux + v_max * vx;
            best_box.corners_y[3] = u_min * uy + v_max * vy;
        }
    }

    // Extents normalization: ensure extent_x >= extent_y (heading points along longer edge)
    if (best_box.extent_x < best_box.extent_y) {
        std::swap(best_box.extent_x, best_box.extent_y);
        best_box.yaw_rad += 0.5f * kPi;
    }
    best_box.yaw_rad = normalize_angle(best_box.yaw_rad);

    out = best_box;
}

void ObstacleGeometryExtractor::compute_single(const PointCloud& cloud,
                                              const uint32_t* indices,
                                              std::size_t count,
                                              ObstacleGeometry& out) {
    compute_disc(cloud, indices, count, out.disc);
    compute_obb(cloud, indices, count, out.obb);
}

void ObstacleGeometryExtractor::extract_discs(const PointCloud& cloud,
                                             const ClusterResult& clusters,
                                             std::vector<BoundingDisc>& out) {
    const std::size_t num_clusters = clusters.num_clusters();
    out.resize(num_clusters);
    for (std::size_t c = 0; c < num_clusters; ++c) {
        compute_disc(cloud, clusters.cluster_indices(c), clusters.cluster_size(c), out[c]);
    }
}

void ObstacleGeometryExtractor::extract_all(const PointCloud& cloud,
                                           const ClusterResult& clusters,
                                           std::vector<ObstacleGeometry>& out) {
    const std::size_t num_clusters = clusters.num_clusters();
    out.resize(num_clusters);
    for (std::size_t c = 0; c < num_clusters; ++c) {
        compute_single(cloud, clusters.cluster_indices(c), clusters.cluster_size(c), out[c]);
    }
}

} // namespace rvpoint

