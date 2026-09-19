#include "features/convex_hull/convex_hull.h"

#include <algorithm>
#include <cmath>
#include <limits>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvpoint {

namespace {

inline float cross_2d(float x1, float y1, float x2, float y2, float x3, float y3) {
    return (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
}

} // namespace

ConvexHull2D::ConvexHull2D(std::size_t max_points,
                           ConvexHullStrategy strategy,
                           Backend backend)
    : strategy_(strategy), backend_(backend) {
    reserve(max_points);
}

void ConvexHull2D::reserve(std::size_t max_points) {
    pts_x_.resize(max_points, 0.0f);
    pts_y_.resize(max_points, 0.0f);
    res_x_.resize(max_points, 0.0f);
    res_y_.resize(max_points, 0.0f);
    perm_.resize(max_points, 0);
}

#if defined(__riscv_vector)
void ConvexHull2D::filter_akl_toussaint_rvv(std::size_t count, std::size_t& out_count) {
    if (count < 16) {
        for (std::size_t i = 0; i < count; ++i) {
            res_x_[i] = pts_x_[i];
            res_y_[i] = pts_y_[i];
        }
        out_count = count;
        return;
    }

    // 1. Compute extrema along X, Y, X+Y, Y-X using vector reductions
    float min_x = 1e9f, max_x = -1e9f;
    float min_y = 1e9f, max_y = -1e9f;
    float min_s = 1e9f, max_s = -1e9f;
    float min_d = 1e9f, max_d = -1e9f;

    std::size_t i = 0;
    vfloat32m4_t v_minx = __riscv_vfmv_v_f_f32m4(1e9f, 32);
    vfloat32m4_t v_maxx = __riscv_vfmv_v_f_f32m4(-1e9f, 32);
    vfloat32m4_t v_miny = __riscv_vfmv_v_f_f32m4(1e9f, 32);
    vfloat32m4_t v_maxy = __riscv_vfmv_v_f_f32m4(-1e9f, 32);
    vfloat32m4_t v_mins = __riscv_vfmv_v_f_f32m4(1e9f, 32);
    vfloat32m4_t v_maxs = __riscv_vfmv_v_f_f32m4(-1e9f, 32);
    vfloat32m4_t v_mind = __riscv_vfmv_v_f_f32m4(1e9f, 32);
    vfloat32m4_t v_maxd = __riscv_vfmv_v_f_f32m4(-1e9f, 32);

    while (i < count) {
        std::size_t vl = __riscv_vsetvl_e32m4(count - i);
        vfloat32m4_t vx = __riscv_vle32_v_f32m4(pts_x_.data() + i, vl);
        vfloat32m4_t vy = __riscv_vle32_v_f32m4(pts_y_.data() + i, vl);
        vfloat32m4_t vs = __riscv_vfadd_vv_f32m4(vx, vy, vl);
        vfloat32m4_t vd = __riscv_vfsub_vv_f32m4(vy, vx, vl);

        v_minx = __riscv_vfmin_vv_f32m4_tu(v_minx, v_minx, vx, vl);
        v_maxx = __riscv_vfmax_vv_f32m4_tu(v_maxx, v_maxx, vx, vl);
        v_miny = __riscv_vfmin_vv_f32m4_tu(v_miny, v_miny, vy, vl);
        v_maxy = __riscv_vfmax_vv_f32m4_tu(v_maxy, v_maxy, vy, vl);
        v_mins = __riscv_vfmin_vv_f32m4_tu(v_mins, v_mins, vs, vl);
        v_maxs = __riscv_vfmax_vv_f32m4_tu(v_maxs, v_maxs, vs, vl);
        v_mind = __riscv_vfmin_vv_f32m4_tu(v_mind, v_mind, vd, vl);
        v_maxd = __riscv_vfmax_vv_f32m4_tu(v_maxd, v_maxd, vd, vl);
        i += vl;
    }

    vfloat32m1_t r_minx = __riscv_vfmv_s_f_f32m1(1e9f, 1);
    vfloat32m1_t r_maxx = __riscv_vfmv_s_f_f32m1(-1e9f, 1);
    vfloat32m1_t r_miny = __riscv_vfmv_s_f_f32m1(1e9f, 1);
    vfloat32m1_t r_maxy = __riscv_vfmv_s_f_f32m1(-1e9f, 1);
    vfloat32m1_t r_mins = __riscv_vfmv_s_f_f32m1(1e9f, 1);
    vfloat32m1_t r_maxs = __riscv_vfmv_s_f_f32m1(-1e9f, 1);
    vfloat32m1_t r_mind = __riscv_vfmv_s_f_f32m1(1e9f, 1);
    vfloat32m1_t r_maxd = __riscv_vfmv_s_f_f32m1(-1e9f, 1);

    min_x = __riscv_vfmv_f_s_f32m1_f32(__riscv_vfredmin_vs_f32m4_f32m1(v_minx, r_minx, 32));
    max_x = __riscv_vfmv_f_s_f32m1_f32(__riscv_vfredmax_vs_f32m4_f32m1(v_maxx, r_maxx, 32));
    min_y = __riscv_vfmv_f_s_f32m1_f32(__riscv_vfredmin_vs_f32m4_f32m1(v_miny, r_miny, 32));
    max_y = __riscv_vfmv_f_s_f32m1_f32(__riscv_vfredmax_vs_f32m4_f32m1(v_maxy, r_maxy, 32));
    min_s = __riscv_vfmv_f_s_f32m1_f32(__riscv_vfredmin_vs_f32m4_f32m1(v_mins, r_mins, 32));
    max_s = __riscv_vfmv_f_s_f32m1_f32(__riscv_vfredmax_vs_f32m4_f32m1(v_maxs, r_maxs, 32));
    min_d = __riscv_vfmv_f_s_f32m1_f32(__riscv_vfredmin_vs_f32m4_f32m1(v_mind, r_mind, 32));
    max_d = __riscv_vfmv_f_s_f32m1_f32(__riscv_vfredmax_vs_f32m4_f32m1(v_maxd, r_maxd, 32));

    // Extract 8 extreme point coordinates
    float ex[8], ey[8];
    for (std::size_t k = 0; k < count; ++k) {
        float px = pts_x_[k], py = pts_y_[k];
        float ps = px + py, pd = py - px;
        if (px == min_x) { ex[0] = px; ey[0] = py; }
        if (ps == min_s) { ex[1] = px; ey[1] = py; }
        if (py == min_y) { ex[2] = px; ey[2] = py; }
        if (pd == max_d) { ex[3] = px; ey[3] = py; }
        if (px == max_x) { ex[4] = px; ey[4] = py; }
        if (ps == max_s) { ex[5] = px; ey[5] = py; }
        if (py == max_y) { ex[6] = px; ey[6] = py; }
        if (pd == min_d) { ex[7] = px; ey[7] = py; }
    }

    // CCW edge half-plane coefficients A[j] * x + B[j] * y + C[j] <= 0
    float A[8], B[8], C[8];
    for (int j = 0; j < 8; ++j) {
        int nxt = (j + 1) % 8;
        float dx = ex[nxt] - ex[j];
        float dy = ey[nxt] - ey[j];
        A[j] = -dy;
        B[j] = dx;
        C[j] = dy * ex[j] - dx * ey[j];
    }

    // Batched half-plane vector test: retain points on or outside ANY edge
    out_count = 0;
    i = 0;
    while (i < count) {
        std::size_t vl = __riscv_vsetvl_e32m4(count - i);
        vfloat32m4_t vx = __riscv_vle32_v_f32m4(pts_x_.data() + i, vl);
        vfloat32m4_t vy = __riscv_vle32_v_f32m4(pts_y_.data() + i, vl);

        vbool8_t keep_mask = __riscv_vmclr_m_b8(vl);
        for (int j = 0; j < 8; ++j) {
            vfloat32m4_t v_cross = __riscv_vfmul_vf_f32m4(vx, A[j], vl);
            v_cross = __riscv_vfmacc_vf_f32m4(v_cross, B[j], vy, vl);
            v_cross = __riscv_vfadd_vf_f32m4(v_cross, C[j], vl);
            vbool8_t out_edge = __riscv_vmfle_vf_f32m4_b8(v_cross, 0.0f, vl);
            keep_mask = __riscv_vmor_mm_b8(keep_mask, out_edge, vl);
        }

        std::size_t n_survivors = __riscv_vcpop_m_b8(keep_mask, vl);
        vfloat32m4_t kept_x = __riscv_vcompress_vm_f32m4(vx, keep_mask, vl);
        vfloat32m4_t kept_y = __riscv_vcompress_vm_f32m4(vy, keep_mask, vl);

        __riscv_vse32_v_f32m4(res_x_.data() + out_count, kept_x, n_survivors);
        __riscv_vse32_v_f32m4(res_y_.data() + out_count, kept_y, n_survivors);
        out_count += n_survivors;
        i += vl;
    }
}
#endif

void ConvexHull2D::compute(const PointCloud& cloud,
                           const uint32_t* indices,
                           std::size_t count,
                           PointCloud2D& out_hull) {
    out_hull.clear();
    if (count == 0) return;

    if (count <= 2) {
        for (std::size_t i = 0; i < count; ++i) {
            uint32_t idx = indices[i];
            out_hull.push_back(cloud.x[idx], cloud.y[idx]);
        }
        return;
    }

    if (pts_x_.size() < count) {
        reserve(count * 2);
    }

    // Gather points into contiguous SoA buffers
    for (std::size_t i = 0; i < count; ++i) {
        uint32_t idx = indices[i];
        pts_x_[i] = cloud.x[idx];
        pts_y_[i] = cloud.y[idx];
    }

    switch (strategy_) {
        case ConvexHullStrategy::JARVIS_MARCH:
            compute_jarvis_march(count, out_hull);
            break;
        case ConvexHullStrategy::ANGULAR_BINNING:
            compute_angular_binning(count, out_hull);
            break;
        case ConvexHullStrategy::MONOTONE_CHAIN:
        default:
            compute_monotone_chain(count, out_hull);
            break;
    }
}

void ConvexHull2D::compute_monotone_chain(std::size_t count, PointCloud2D& out_hull) {
    const float* src_x = pts_x_.data();
    const float* src_y = pts_y_.data();
    std::size_t n_pts = count;

#if defined(__riscv_vector)
    if ((backend_ == Backend::Auto || backend_ == Backend::RVV) && count >= 16) {
        std::size_t akl_count = 0;
        filter_akl_toussaint_rvv(count, akl_count);
        if (akl_count >= 3) {
            src_x = res_x_.data();
            src_y = res_y_.data();
            n_pts = akl_count;
        }
    }
#endif

    for (std::size_t i = 0; i < n_pts; ++i) {
        perm_[i] = static_cast<uint32_t>(i);
    }

    // 1. Lexicographical permutation sort: X ascending, then Y ascending
    std::sort(perm_.begin(), perm_.begin() + n_pts, [src_x, src_y](uint32_t a, uint32_t b) {
        float dx = src_x[a] - src_x[b];
        if (std::abs(dx) > 1e-6f) return dx < 0.0f;
        return src_y[a] < src_y[b];
    });

    // 2. Prune coincident duplicate points
    std::size_t n = 0;
    for (std::size_t i = 0; i < n_pts; ++i) {
        uint32_t curr = perm_[i];
        if (n == 0) {
            perm_[n++] = curr;
        } else {
            uint32_t prev = perm_[n - 1];
            if (std::abs(src_x[curr] - src_x[prev]) > 1e-6f ||
                std::abs(src_y[curr] - src_y[prev]) > 1e-6f) {
                perm_[n++] = curr;
            }
        }
    }

    if (n <= 2) {
        for (std::size_t i = 0; i < n; ++i) {
            uint32_t idx = perm_[i];
            out_hull.push_back(src_x[idx], src_y[idx]);
        }
        return;
    }

    // 3. Andrew's Monotone Chain: Lower hull
    for (std::size_t i = 0; i < n; ++i) {
        uint32_t curr = perm_[i];
        float px = src_x[curr];
        float py = src_y[curr];

        while (out_hull.size() >= 2) {
            std::size_t sz = out_hull.size();
            float x1 = out_hull.x[sz - 2];
            float y1 = out_hull.y[sz - 2];
            float x2 = out_hull.x[sz - 1];
            float y2 = out_hull.y[sz - 1];

            if (cross_2d(x1, y1, x2, y2, px, py) <= 1e-6f) {
                out_hull.pop_back();
            } else {
                break;
            }
        }
        out_hull.push_back(px, py);
    }

    // 4. Andrew's Monotone Chain: Upper hull
    std::size_t lower_hull_size = out_hull.size() + 1;
    for (int i = static_cast<int>(n) - 2; i >= 0; --i) {
        uint32_t curr = perm_[i];
        float px = src_x[curr];
        float py = src_y[curr];

        while (out_hull.size() >= lower_hull_size) {
            std::size_t sz = out_hull.size();
            float x1 = out_hull.x[sz - 2];
            float y1 = out_hull.y[sz - 2];
            float x2 = out_hull.x[sz - 1];
            float y2 = out_hull.y[sz - 1];

            if (cross_2d(x1, y1, x2, y2, px, py) <= 1e-6f) {
                out_hull.pop_back();
            } else {
                break;
            }
        }
        out_hull.push_back(px, py);
    }

    // Remove redundant endpoint
    if (out_hull.size() > 1) {
        out_hull.pop_back();
    }
}

void ConvexHull2D::compute_jarvis_march(std::size_t count, PointCloud2D& out_hull) {
    const float* px = pts_x_.data();
    const float* py = pts_y_.data();

    // 1. Pivot point: minimum X (tie-break minimum Y)
    std::size_t pivot_idx = 0;
    float min_x = px[0];
    float min_y = py[0];
    for (std::size_t i = 1; i < count; ++i) {
        if (px[i] < min_x || (std::abs(px[i] - min_x) < 1e-6f && py[i] < min_y)) {
            min_x = px[i];
            min_y = py[i];
            pivot_idx = i;
        }
    }

    // 2. Gift-wrapping loop
    std::size_t current = pivot_idx;
    do {
        out_hull.push_back(px[current], py[current]);
        std::size_t next_cand = (current == 0) ? 1 : 0;
        float cx = px[current];
        float cy = py[current];
        float cand_x = px[next_cand];
        float cand_y = py[next_cand];

        for (std::size_t i = 0; i < count; ++i) {
            if (i == current) continue;
            float cross = (cand_x - cx) * (py[i] - cy) - (cand_y - cy) * (px[i] - cx);
            if (cross > 1e-6f) {
                next_cand = i;
                cand_x = px[i];
                cand_y = py[i];
            }
        }
        current = next_cand;
    } while (current != pivot_idx && out_hull.size() < 64);
}

void ConvexHull2D::compute_angular_binning(std::size_t count, PointCloud2D& out_hull) {
    constexpr std::size_t B = 16;
    const float* px = pts_x_.data();
    const float* py = pts_y_.data();

    float best_proj[B];
    std::size_t best_idx[B];
    for (std::size_t b = 0; b < B; ++b) {
        best_proj[b] = -1e30f;
        best_idx[b] = 0;
    }

    for (std::size_t b = 0; b < B; ++b) {
        float theta = static_cast<float>(b) * (6.28318530718f / static_cast<float>(B));
        float cos_t = std::cos(theta);
        float sin_t = std::sin(theta);

        for (std::size_t i = 0; i < count; ++i) {
            float proj = px[i] * cos_t + py[i] * sin_t;
            if (proj > best_proj[b]) {
                best_proj[b] = proj;
                best_idx[b] = i;
            }
        }
    }

    // Deduplicate and output CCW vertices
    for (std::size_t b = 0; b < B; ++b) {
        std::size_t idx = best_idx[b];
        if (out_hull.empty() ||
            std::abs(out_hull.x.back() - px[idx]) > 1e-4f ||
            std::abs(out_hull.y.back() - py[idx]) > 1e-4f) {
            out_hull.push_back(px[idx], py[idx]);
        }
    }
}

} // namespace rvpoint
