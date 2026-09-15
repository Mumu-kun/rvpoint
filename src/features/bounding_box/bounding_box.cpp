#include "features/bounding_box/bounding_box.h"

#include <algorithm>
#include <cmath>
#include <limits>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvpoint {

BoundingBoxExtractor::BoundingBoxExtractor(const BoundingBoxParams& params,
                                           std::size_t max_points,
                                           Backend backend)
    : params_(params), backend_(backend) {
    reserve(max_points);
}

void BoundingBoxExtractor::reserve(std::size_t max_points) {
    scratch_x_.resize(max_points, 0.0f);
    scratch_y_.resize(max_points, 0.0f);
    scratch_z_.resize(max_points, 0.0f);
    scratch_dec_x_.resize(128, 0.0f);
    scratch_dec_y_.resize(128, 0.0f);
    candidates_.resize(128); // Sized once for hot-path zero-heap invariant (ADR-0010)
}

void BoundingBoxExtractor::extract_candidates(const PointCloud2D& hull,
                                             std::size_t& candidate_count,
                                             std::size_t& min_area_idx,
                                             float& min_area) {
    const std::size_t M = hull.size();
    if (candidates_.size() < M) {
        candidates_.resize(M * 2);
    }

    candidate_count = 0;
    min_area = std::numeric_limits<float>::max();
    min_area_idx = 0;

    for (std::size_t i = 0; i < M; ++i) {
        std::size_t j = (i + 1) % M;
        float dx = hull.x[j] - hull.x[i];
        float dy = hull.y[j] - hull.y[i];
        float edge_len = std::sqrt(dx * dx + dy * dy);

        if (edge_len < 1e-6f) continue;

        float ux = dx / edge_len;
        float uy = dy / edge_len;

        float u_min = std::numeric_limits<float>::max();
        float u_max = std::numeric_limits<float>::lowest();
        float v_min = std::numeric_limits<float>::max();
        float v_max = std::numeric_limits<float>::lowest();

#if defined(__riscv_vector)
        bool use_rvv = (backend_ == Backend::Auto || backend_ == Backend::RVV);
        if (use_rvv && M >= 8) {
            std::size_t k = 0;
            vfloat32m1_t v_umin = __riscv_vfmv_s_f_f32m1(u_min, 1);
            vfloat32m1_t v_umax = __riscv_vfmv_s_f_f32m1(u_max, 1);
            vfloat32m1_t v_vmin = __riscv_vfmv_s_f_f32m1(v_min, 1);
            vfloat32m1_t v_vmax = __riscv_vfmv_s_f_f32m1(v_max, 1);

            while (k < M) {
                std::size_t vl = __riscv_vsetvl_e32m4(M - k);
                vfloat32m4_t hx = __riscv_vle32_v_f32m4(hull.x.data() + k, vl);
                vfloat32m4_t hy = __riscv_vle32_v_f32m4(hull.y.data() + k, vl);

                vfloat32m4_t pu = __riscv_vfmul_vf_f32m4(hx, ux, vl);
                pu = __riscv_vfmacc_vf_f32m4(pu, uy, hy, vl);

                vfloat32m4_t pv = __riscv_vfmul_vf_f32m4(hy, ux, vl);
                pv = __riscv_vfnmsac_vf_f32m4(pv, uy, hx, vl);

                v_umin = __riscv_vfredmin_vs_f32m4_f32m1(pu, v_umin, vl);
                v_umax = __riscv_vfredmax_vs_f32m4_f32m1(pu, v_umax, vl);
                v_vmin = __riscv_vfredmin_vs_f32m4_f32m1(pv, v_vmin, vl);
                v_vmax = __riscv_vfredmax_vs_f32m4_f32m1(pv, v_vmax, vl);
                k += vl;
            }

            u_min = __riscv_vfmv_f_s_f32m1_f32(v_umin);
            u_max = __riscv_vfmv_f_s_f32m1_f32(v_umax);
            v_min = __riscv_vfmv_f_s_f32m1_f32(v_vmin);
            v_max = __riscv_vfmv_f_s_f32m1_f32(v_vmax);
        } else
#endif
        {
            for (std::size_t k = 0; k < M; ++k) {
                float hx = hull.x[k];
                float hy = hull.y[k];
                float pu = hx * ux + hy * uy;
                float pv = -hx * uy + hy * ux;

                if (pu < u_min) u_min = pu;
                if (pu > u_max) u_max = pu;
                if (pv < v_min) v_min = pv;
                if (pv > v_max) v_max = pv;
            }
        }

        float area = (u_max - u_min) * (v_max - v_min);
        candidates_[candidate_count] = CandidateBox{ux, uy, u_min, u_max, v_min, v_max, area};

        if (area < min_area) {
            min_area = area;
            min_area_idx = candidate_count;
        }
        ++candidate_count;
    }
}

void BoundingBoxExtractor::populate_box(const CandidateBox& win, float min_z, float max_z,
                                      uint32_t pt_count, OrientedBoundingBox& out_box) {
    float u_center = 0.5f * (win.u_min + win.u_max);
    float v_center = 0.5f * (win.v_min + win.v_max);

    out_box.cx = u_center * win.ux - v_center * win.uy;
    out_box.cy = u_center * win.uy + v_center * win.ux;
    out_box.cz = 0.5f * (min_z + max_z);
    out_box.extent_x = std::max(0.05f, win.u_max - win.u_min);
    out_box.extent_y = std::max(0.05f, win.v_max - win.v_min);
    out_box.extent_z = std::max(0.05f, max_z - min_z);
    out_box.yaw_rad = std::atan2(win.uy, win.ux);
    out_box.point_count = pt_count;

    // Heading Normalization: ensure extent_x >= extent_y (ISO 8855 principal vehicle axis)
    if (out_box.extent_x < out_box.extent_y) {
        std::swap(out_box.extent_x, out_box.extent_y);
        out_box.yaw_rad += static_cast<float>(M_PI_2);
        if (out_box.yaw_rad > static_cast<float>(M_PI)) {
            out_box.yaw_rad -= static_cast<float>(2.0 * M_PI);
        }
    }

    // Synchronized Footprint Corners strictly computed AFTER heading normalization
    float half_l = 0.5f * out_box.extent_x;
    float half_w = 0.5f * out_box.extent_y;
    float cos_h = std::cos(out_box.yaw_rad);
    float sin_h = std::sin(out_box.yaw_rad);

    out_box.corners[0] = { out_box.cx + half_l * cos_h - half_w * sin_h,
                           out_box.cy + half_l * sin_h + half_w * cos_h };
    out_box.corners[1] = { out_box.cx - half_l * cos_h - half_w * sin_h,
                           out_box.cy - half_l * sin_h + half_w * cos_h };
    out_box.corners[2] = { out_box.cx - half_l * cos_h + half_w * sin_h,
                           out_box.cy - half_l * sin_h - half_w * cos_h };
    out_box.corners[3] = { out_box.cx + half_l * cos_h + half_w * sin_h,
                           out_box.cy + half_l * sin_h - half_w * cos_h };
}

void BoundingBoxExtractor::compute(const PointCloud& cloud,
                                  const uint32_t* indices,
                                  std::size_t count,
                                  const PointCloud2D& hull,
                                  OrientedBoundingBox& out_box) {
    switch (params_.strategy) {
        case BoundingBoxStrategy::MIN_AREA:
            compute_min_area(cloud, indices, count, hull, out_box);
            break;
        case BoundingBoxStrategy::L_SHAPE_ALIGN:
            compute_l_shape(cloud, indices, count, hull, out_box);
            break;
        case BoundingBoxStrategy::EDGE_ALIGN:
            compute_edge_align(cloud, indices, count, hull, out_box);
            break;
        case BoundingBoxStrategy::WIREFRAME_PCA: {
            out_box = OrientedBoundingBox{};
            if (count == 0) return;
            float min_z = std::numeric_limits<float>::max();
            float max_z = std::numeric_limits<float>::lowest();
            for (std::size_t i = 0; i < count; ++i) {
                float z = cloud.z[indices[i]];
                if (z < min_z) min_z = z;
                if (z > max_z) max_z = z;
            }
            compute_wireframe_pca(hull, min_z, max_z, static_cast<uint32_t>(count), out_box);
            break;
        }
    }
}

void BoundingBoxExtractor::compute_min_area(const PointCloud& cloud,
                                           const uint32_t* indices,
                                           std::size_t count,
                                           const PointCloud2D& hull,
                                           OrientedBoundingBox& out_box) {
    out_box = OrientedBoundingBox{};
    if (count == 0) return;

    if (scratch_x_.size() < count) {
        reserve(count * 2);
    }

    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    for (std::size_t i = 0; i < count; ++i) {
        uint32_t idx = indices[i];
        scratch_x_[i] = cloud.x[idx];
        scratch_y_[i] = cloud.y[idx];
        scratch_z_[i] = cloud.z[idx];
        if (scratch_z_[i] < min_z) min_z = scratch_z_[i];
        if (scratch_z_[i] > max_z) max_z = scratch_z_[i];
    }

    if (count < 3 || hull.size() < 3) {
        float min_x = scratch_x_[0], max_x = scratch_x_[0];
        float min_y = scratch_y_[0], max_y = scratch_y_[0];
        for (std::size_t i = 1; i < count; ++i) {
            if (scratch_x_[i] < min_x) min_x = scratch_x_[i];
            if (scratch_x_[i] > max_x) max_x = scratch_x_[i];
            if (scratch_y_[i] < min_y) min_y = scratch_y_[i];
            if (scratch_y_[i] > max_y) max_y = scratch_y_[i];
        }
        CandidateBox aabb{1.0f, 0.0f, min_x, max_x, min_y, max_y, (max_x - min_x) * (max_y - min_y)};
        populate_box(aabb, min_z, max_z, static_cast<uint32_t>(count), out_box);
        return;
    }

    std::size_t candidate_count = 0, min_area_idx = 0;
    float min_area = std::numeric_limits<float>::max();
    extract_candidates(hull, candidate_count, min_area_idx, min_area);

    if (candidate_count == 0) return;
    populate_box(candidates_[min_area_idx], min_z, max_z, static_cast<uint32_t>(count), out_box);
}

void BoundingBoxExtractor::compute_edge_align(const PointCloud& cloud,
                                             const uint32_t* indices,
                                             std::size_t count,
                                             const PointCloud2D& hull,
                                             OrientedBoundingBox& out_box) {
    out_box = OrientedBoundingBox{};
    if (count == 0) return;

    if (scratch_x_.size() < count) {
        reserve(count * 2);
    }

    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    for (std::size_t i = 0; i < count; ++i) {
        uint32_t idx = indices[i];
        scratch_x_[i] = cloud.x[idx];
        scratch_y_[i] = cloud.y[idx];
        scratch_z_[i] = cloud.z[idx];
        if (scratch_z_[i] < min_z) min_z = scratch_z_[i];
        if (scratch_z_[i] > max_z) max_z = scratch_z_[i];
    }

    const std::size_t M = hull.size();
    if (count < 3 || M < 3) {
        compute_min_area(cloud, indices, count, hull, out_box);
        return;
    }

    std::size_t candidate_count = 0, min_area_idx = 0;
    float min_area = std::numeric_limits<float>::max();
    extract_candidates(hull, candidate_count, min_area_idx, min_area);

    if (candidate_count == 0) return;

    // Precalculate hull edge vectors and lengths (O(M), M <= 64)
    float edge_len[64];
    float edge_dx[64];
    float edge_dy[64];
    std::size_t num_edges = std::min(M, std::size_t(64));

    for (std::size_t k = 0; k < num_edges; ++k) {
        std::size_t nxt = (k + 1) % M;
        float dx = hull.x[nxt] - hull.x[k];
        float dy = hull.y[nxt] - hull.y[k];
        float l = std::sqrt(dx * dx + dy * dy);
        edge_len[k] = l;
        if (l > 1e-6f) {
            edge_dx[k] = dx / l;
            edge_dy[k] = dy / l;
        } else {
            edge_dx[k] = 1.0f;
            edge_dy[k] = 0.0f;
        }
    }

    const float area_threshold = min_area * params_.area_constraint_ratio;
    float best_score = -1.0f;
    std::size_t best_idx = min_area_idx;

    // Unconditional length-weighted collinearity evaluation: S = sum(L_k * max(|d_k.u|, |d_k.v|)^4)
    for (std::size_t i = 0; i < candidate_count; ++i) {
        if (candidates_[i].area > area_threshold) continue;

        float score = 0.0f;
        float ux = candidates_[i].ux;
        float uy = candidates_[i].uy;

        for (std::size_t k = 0; k < num_edges; ++k) {
            float pu = std::abs(edge_dx[k] * ux + edge_dy[k] * uy);
            float pv = std::abs(-edge_dx[k] * uy + edge_dy[k] * ux);
            float alpha = std::max(pu, pv);
            float a2 = alpha * alpha;
            score += edge_len[k] * (a2 * a2);
        }

        if (score > best_score) {
            best_score = score;
            best_idx = i;
        }
    }

    populate_box(candidates_[best_idx], min_z, max_z, static_cast<uint32_t>(count), out_box);
}

void BoundingBoxExtractor::compute_l_shape(const PointCloud& cloud,
                                          const uint32_t* indices,
                                          std::size_t count,
                                          const PointCloud2D& hull,
                                          OrientedBoundingBox& out_box) {
    out_box = OrientedBoundingBox{};
    if (count == 0) return;

    if (scratch_x_.size() < count) {
        reserve(count * 2);
    }

    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    for (std::size_t i = 0; i < count; ++i) {
        uint32_t idx = indices[i];
        scratch_x_[i] = cloud.x[idx];
        scratch_y_[i] = cloud.y[idx];
        scratch_z_[i] = cloud.z[idx];
        if (scratch_z_[i] < min_z) min_z = scratch_z_[i];
        if (scratch_z_[i] > max_z) max_z = scratch_z_[i];
    }

    const std::size_t M = hull.size();
    if (count < 3 || M < 3) {
        compute_min_area(cloud, indices, count, hull, out_box);
        return;
    }

    std::size_t candidate_count = 0, min_area_idx = 0;
    float min_area = std::numeric_limits<float>::max();
    extract_candidates(hull, candidate_count, min_area_idx, min_area);

    if (candidate_count == 0) return;

    const float area_threshold = min_area * params_.area_constraint_ratio;

    // Collect valid candidate indices within the feasible area window
    uint8_t valid_indices[32];
    std::size_t num_valid = 0;
    for (std::size_t i = 0; i < candidate_count && num_valid < 32; ++i) {
        if (candidates_[i].area <= area_threshold) {
            valid_indices[num_valid++] = static_cast<uint8_t>(i);
        }
    }

    // Trivial candidate bypass: if only 1 candidate passes area threshold, zero closeness needed!
    if (num_valid <= 1) {
        populate_box(candidates_[min_area_idx], min_z, max_z, static_cast<uint32_t>(count), out_box);
        return;
    }

    // Stratified decimation for orientation scoring: caps scoring pass at <= 64 points
    std::size_t eval_count = count;
    const float* eval_x = scratch_x_.data();
    const float* eval_y = scratch_y_.data();

    if (count > params_.decimation_threshold) {
        std::size_t stride = std::max(std::size_t(1), count / params_.decimation_threshold);
        eval_count = 0;
        for (std::size_t k = 0; k < count && eval_count < 128; k += stride) {
            scratch_dec_x_[eval_count] = scratch_x_[k];
            scratch_dec_y_[eval_count] = scratch_y_[k];
            ++eval_count;
        }
        eval_x = scratch_dec_x_.data();
        eval_y = scratch_dec_y_.data();
    }

    // Prepare batched candidate constants (midpoint formulation)
    const float d0 = params_.truncation_dist;
    BatchedCandidate batched[32];
    for (std::size_t k = 0; k < num_valid; ++k) {
        const auto& c = candidates_[valid_indices[k]];
        batched[k].ux = c.ux;
        batched[k].uy = c.uy;
        batched[k].u_mid = 0.5f * (c.u_min + c.u_max);
        batched[k].v_mid = 0.5f * (c.v_min + c.v_max);
        batched[k].thresh_u = 0.5f * (c.u_max - c.u_min) - d0;
        batched[k].thresh_v = 0.5f * (c.v_max - c.v_min) - d0;
    }

    float scores[32] = {0.0f};

#if defined(__riscv_vector)
    bool use_rvv = (backend_ == Backend::Auto || backend_ == Backend::RVV);
    if (use_rvv) {
        std::size_t k = 0;
        // Process in batches of 3 concurrently in vector registers (zero stack spills)
        while (k + 3 <= num_valid) {
            evaluate_closeness_batch3_rvv(eval_x, eval_y, eval_count,
                                         batched[k], batched[k + 1], batched[k + 2],
                                         scores[k], scores[k + 1], scores[k + 2]);
            k += 3;
        }
        // Remainder evaluated individually
        while (k < num_valid) {
            scores[k] = evaluate_closeness_single_rvv(candidates_[valid_indices[k]], eval_x, eval_y, eval_count);
            ++k;
        }
    } else
#endif
    {
        for (std::size_t k = 0; k < num_valid; ++k) {
            scores[k] = evaluate_closeness_single_scalar(candidates_[valid_indices[k]], eval_x, eval_y, eval_count);
        }
    }

    float best_score = -1.0f;
    std::size_t best_idx = min_area_idx;

    for (std::size_t k = 0; k < num_valid; ++k) {
        if (scores[k] > best_score) {
            best_score = scores[k];
            best_idx = valid_indices[k];
        }
    }

    populate_box(candidates_[best_idx], min_z, max_z, static_cast<uint32_t>(count), out_box);
}

float BoundingBoxExtractor::evaluate_closeness_single(const CandidateBox& cand, const float* px, const float* py, std::size_t count) {
#if defined(__riscv_vector)
    if (backend_ == Backend::Auto || backend_ == Backend::RVV) {
        return evaluate_closeness_single_rvv(cand, px, py, count);
    }
#endif
    return evaluate_closeness_single_scalar(cand, px, py, count);
}

float BoundingBoxExtractor::evaluate_closeness_single_scalar(const CandidateBox& cand, const float* px, const float* py, std::size_t count) {
    const float d0 = params_.truncation_dist;
    float u_mid = 0.5f * (cand.u_min + cand.u_max);
    float v_mid = 0.5f * (cand.v_min + cand.v_max);
    float thresh_u = 0.5f * (cand.u_max - cand.u_min) - d0;
    float thresh_v = 0.5f * (cand.v_max - cand.v_min) - d0;

    float total = 0.0f;
    for (std::size_t i = 0; i < count; ++i) {
        float u = px[i] * cand.ux + py[i] * cand.uy;
        float v = -px[i] * cand.uy + py[i] * cand.ux;

        float su = std::abs(u - u_mid) - thresh_u;
        float sv = std::abs(v - v_mid) - thresh_v;
        float c = std::max(0.0f, std::max(su, sv));
        total += c;
    }
    return total;
}

float BoundingBoxExtractor::evaluate_closeness_single_rvv(const CandidateBox& cand, const float* px, const float* py, std::size_t count) {
#if defined(__riscv_vector)
    const float d0 = params_.truncation_dist;
    float u_mid = 0.5f * (cand.u_min + cand.u_max);
    float v_mid = 0.5f * (cand.v_min + cand.v_max);
    float thresh_u = 0.5f * (cand.u_max - cand.u_min) - d0;
    float thresh_v = 0.5f * (cand.v_max - cand.v_min) - d0;

    std::size_t i = 0;
    vfloat32m4_t v_acc = __riscv_vfmv_v_f_f32m4(0.0f, 32);

    while (i < count) {
        std::size_t vl = __riscv_vsetvl_e32m4(count - i);
        vfloat32m4_t vx = __riscv_vle32_v_f32m4(px + i, vl);
        vfloat32m4_t vy = __riscv_vle32_v_f32m4(py + i, vl);

        // vu = vx * ux + vy * uy
        vfloat32m4_t vu = __riscv_vfmul_vf_f32m4(vx, cand.ux, vl);
        vu = __riscv_vfmacc_vf_f32m4(vu, cand.uy, vy, vl);

        // vv = vy * ux - vx * uy
        vfloat32m4_t vv = __riscv_vfmul_vf_f32m4(vy, cand.ux, vl);
        vv = __riscv_vfnmsac_vf_f32m4(vv, cand.uy, vx, vl);

        // In-place midpoint interval evaluation: |u - u_mid| - thresh_u
        vu = __riscv_vfsub_vf_f32m4(vu, u_mid, vl);
        vu = __riscv_vfsgnjx_vv_f32m4(vu, vu, vl);
        vu = __riscv_vfsub_vf_f32m4(vu, thresh_u, vl);

        // In-place midpoint interval evaluation: |v - v_mid| - thresh_v
        vv = __riscv_vfsub_vf_f32m4(vv, v_mid, vl);
        vv = __riscv_vfsgnjx_vv_f32m4(vv, vv, vl);
        vv = __riscv_vfsub_vf_f32m4(vv, thresh_v, vl);

        vfloat32m4_t sc = __riscv_vfmax_vv_f32m4(vu, vv, vl);
        sc = __riscv_vfmax_vf_f32m4(sc, 0.0f, vl);

        v_acc = __riscv_vfadd_vv_f32m4_tu(v_acc, v_acc, sc, vl);
        i += vl;
    }

    vfloat32m1_t zero = __riscv_vfmv_s_f_f32m1(0.0f, 1);
    vfloat32m1_t v_final = __riscv_vfredusum_vs_f32m4_f32m1(v_acc, zero, 32);
    return __riscv_vfmv_f_s_f32m1_f32(v_final);
#else
    return evaluate_closeness_single_scalar(cand, px, py, count);
#endif
}

#if defined(__riscv_vector)
void BoundingBoxExtractor::evaluate_closeness_batch3_rvv(
    const float* px, const float* py, std::size_t count,
    const BatchedCandidate& c0, const BatchedCandidate& c1, const BatchedCandidate& c2,
    float& s0, float& s1, float& s2)
{
    std::size_t i = 0;
    vfloat32m4_t v_acc0 = __riscv_vfmv_v_f_f32m4(0.0f, 32);
    vfloat32m4_t v_acc1 = __riscv_vfmv_v_f_f32m4(0.0f, 32);
    vfloat32m4_t v_acc2 = __riscv_vfmv_v_f_f32m4(0.0f, 32);

    while (i < count) {
        std::size_t vl = __riscv_vsetvl_e32m4(count - i);
        vfloat32m4_t vx = __riscv_vle32_v_f32m4(px + i, vl);
        vfloat32m4_t vy = __riscv_vle32_v_f32m4(py + i, vl);

        // --- Candidate 0 ---
        {
            vfloat32m4_t vu = __riscv_vfmul_vf_f32m4(vx, c0.ux, vl);
            vu = __riscv_vfmacc_vf_f32m4(vu, c0.uy, vy, vl);
            vfloat32m4_t vv = __riscv_vfmul_vf_f32m4(vy, c0.ux, vl);
            vv = __riscv_vfnmsac_vf_f32m4(vv, c0.uy, vx, vl);

            vu = __riscv_vfsub_vf_f32m4(vu, c0.u_mid, vl);
            vu = __riscv_vfsgnjx_vv_f32m4(vu, vu, vl);
            vu = __riscv_vfsub_vf_f32m4(vu, c0.thresh_u, vl);

            vv = __riscv_vfsub_vf_f32m4(vv, c0.v_mid, vl);
            vv = __riscv_vfsgnjx_vv_f32m4(vv, vv, vl);
            vv = __riscv_vfsub_vf_f32m4(vv, c0.thresh_v, vl);

            vfloat32m4_t sc = __riscv_vfmax_vv_f32m4(vu, vv, vl);
            sc = __riscv_vfmax_vf_f32m4(sc, 0.0f, vl);
            v_acc0 = __riscv_vfadd_vv_f32m4_tu(v_acc0, v_acc0, sc, vl);
        }

        // --- Candidate 1 ---
        {
            vfloat32m4_t vu = __riscv_vfmul_vf_f32m4(vx, c1.ux, vl);
            vu = __riscv_vfmacc_vf_f32m4(vu, c1.uy, vy, vl);
            vfloat32m4_t vv = __riscv_vfmul_vf_f32m4(vy, c1.ux, vl);
            vv = __riscv_vfnmsac_vf_f32m4(vv, c1.uy, vx, vl);

            vu = __riscv_vfsub_vf_f32m4(vu, c1.u_mid, vl);
            vu = __riscv_vfsgnjx_vv_f32m4(vu, vu, vl);
            vu = __riscv_vfsub_vf_f32m4(vu, c1.thresh_u, vl);

            vv = __riscv_vfsub_vf_f32m4(vv, c1.v_mid, vl);
            vv = __riscv_vfsgnjx_vv_f32m4(vv, vv, vl);
            vv = __riscv_vfsub_vf_f32m4(vv, c1.thresh_v, vl);

            vfloat32m4_t sc = __riscv_vfmax_vv_f32m4(vu, vv, vl);
            sc = __riscv_vfmax_vf_f32m4(sc, 0.0f, vl);
            v_acc1 = __riscv_vfadd_vv_f32m4_tu(v_acc1, v_acc1, sc, vl);
        }

        // --- Candidate 2 ---
        {
            vfloat32m4_t vu = __riscv_vfmul_vf_f32m4(vx, c2.ux, vl);
            vu = __riscv_vfmacc_vf_f32m4(vu, c2.uy, vy, vl);
            vfloat32m4_t vv = __riscv_vfmul_vf_f32m4(vy, c2.ux, vl);
            vv = __riscv_vfnmsac_vf_f32m4(vv, c2.uy, vx, vl);

            vu = __riscv_vfsub_vf_f32m4(vu, c2.u_mid, vl);
            vu = __riscv_vfsgnjx_vv_f32m4(vu, vu, vl);
            vu = __riscv_vfsub_vf_f32m4(vu, c2.thresh_u, vl);

            vv = __riscv_vfsub_vf_f32m4(vv, c2.v_mid, vl);
            vv = __riscv_vfsgnjx_vv_f32m4(vv, vv, vl);
            vv = __riscv_vfsub_vf_f32m4(vv, c2.thresh_v, vl);

            vfloat32m4_t sc = __riscv_vfmax_vv_f32m4(vu, vv, vl);
            sc = __riscv_vfmax_vf_f32m4(sc, 0.0f, vl);
            v_acc2 = __riscv_vfadd_vv_f32m4_tu(v_acc2, v_acc2, sc, vl);
        }

        i += vl;
    }

    vfloat32m1_t zero = __riscv_vfmv_s_f_f32m1(0.0f, 1);
    s0 = __riscv_vfmv_f_s_f32m1_f32(__riscv_vfredusum_vs_f32m4_f32m1(v_acc0, zero, 32));
    s1 = __riscv_vfmv_f_s_f32m1_f32(__riscv_vfredusum_vs_f32m4_f32m1(v_acc1, zero, 32));
    s2 = __riscv_vfmv_f_s_f32m1_f32(__riscv_vfredusum_vs_f32m4_f32m1(v_acc2, zero, 32));
}
#endif

void BoundingBoxExtractor::compute_wireframe_pca(const PointCloud2D& hull,
                                                float z_min, float z_max,
                                                uint32_t pt_count,
                                                OrientedBoundingBox& out_box) {
    const std::size_t M = hull.size();
    double total_len = 0.0;
    double cx = 0.0, cy = 0.0;

    for (std::size_t j = 0; j < M; ++j) {
        std::size_t j_next = (j + 1) % M;
        double x1 = hull.x[j], y1 = hull.y[j];
        double x2 = hull.x[j_next], y2 = hull.y[j_next];
        double dx = x2 - x1, dy = y2 - y1;
        double len = std::sqrt(dx * dx + dy * dy);
        total_len += len;
        cx += 0.5 * (x1 + x2) * len;
        cy += 0.5 * (y1 + y2) * len;
    }

    if (total_len > 1e-6) {
        cx /= total_len;
        cy /= total_len;
    }

    double cxx = 0.0, cxy = 0.0, cyy = 0.0;
    for (std::size_t j = 0; j < M; ++j) {
        std::size_t j_next = (j + 1) % M;
        double x1 = hull.x[j] - cx, y1 = hull.y[j] - cy;
        double x2 = hull.x[j_next] - cx, y2 = hull.y[j_next] - cy;
        double dx = x2 - x1, dy = y2 - y1;
        double len = std::sqrt(dx * dx + dy * dy);

        cxx += (x1 * x1 + x1 * x2 + x2 * x2) * len / 3.0;
        cxy += (x1 * y1 + 0.5 * (x1 * y2 + x2 * y1) + x2 * y2) * len / 3.0;
        cyy += (y1 * y1 + y1 * y2 + y2 * y2) * len / 3.0;
    }

    if (total_len > 1e-6) {
        cxx /= total_len;
        cxy /= total_len;
        cyy /= total_len;
    }

    double diff = cxx - cyy;
    double theta = 0.5 * std::atan2(2.0 * cxy, diff);

    float ux = static_cast<float>(std::cos(theta));
    float uy = static_cast<float>(std::sin(theta));

    float u_min = std::numeric_limits<float>::max();
    float u_max = std::numeric_limits<float>::lowest();
    float v_min = std::numeric_limits<float>::max();
    float v_max = std::numeric_limits<float>::lowest();

    for (std::size_t k = 0; k < M; ++k) {
        float hx = hull.x[k];
        float hy = hull.y[k];
        float pu = hx * ux + hy * uy;
        float pv = -hx * uy + hy * ux;
        if (pu < u_min) u_min = pu;
        if (pu > u_max) u_max = pu;
        if (pv < v_min) v_min = pv;
        if (pv > v_max) v_max = pv;
    }

    CandidateBox win{ux, uy, u_min, u_max, v_min, v_max, (u_max - u_min) * (v_max - v_min)};
    populate_box(win, z_min, z_max, pt_count, out_box);
}

} // namespace rvpoint
