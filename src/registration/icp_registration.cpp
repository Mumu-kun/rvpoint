// icp_registration.cpp
//
// Point-to-Plane ICP Registration — RVV-accelerated implementation
// ================================================================
// Implements Stages 2–5 of the Tracking Mode pipeline:
//   Stage 2: Correspondence Search (spatial hash + RVV distance)
//   Stage 3: Residual + Jacobian (fully vectorized per-pair)
//   Stage 4: Reduction to 6×6 (tree-sum via vfredusum)
//   Stage 5: Solve 6×6 (scalar Cholesky — too small to vectorize)
//
// RVV Parallelism Strategy:
//   - All vectorized stages use LMUL=m8 for maximum throughput
//   - Same intrinsic patterns as SOR distance kernel & RANSAC inlier counting
//   - GEM5_BUILD fallback paths avoid vluxei32/vsseg

#include "icp_registration.h"
#include "../include/profiler.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <vector>

#include <riscv_vector.h>

namespace rvv_pcl {

// ============================================================================
// Stage 2: Correspondence Search — RVV distance kernel
// ============================================================================

void correspondence_distance_rvv(const float* src_x, const float* src_y, const float* src_z,
                                  const float* tgt_x, const float* tgt_y, const float* tgt_z,
                                  const int* candidate_indices, std::size_t n_candidates,
                                  float qx, float qy, float qz,
                                  int& best_idx, float& best_dist_sq) {
    best_idx = -1;
    best_dist_sq = 1e30f;

    if (n_candidates == 0) return;

    // Gather candidate target points and compute distances using RVV
    // Pattern: same as SOR's inlined distance kernel (vfsub_vf → vfmul_vv → vfadd_vv)
#ifdef GEM5_BUILD
    // gem5-safe: scalar gather + RVV distances
    std::vector<float> gx(n_candidates), gy(n_candidates), gz(n_candidates);
    for (std::size_t i = 0; i < n_candidates; ++i) {
        int idx = candidate_indices[i];
        gx[i] = tgt_x[idx];
        gy[i] = tgt_y[idx];
        gz[i] = tgt_z[idx];
    }

    std::vector<float> dists(n_candidates);
    std::size_t j = 0;
    while (j < n_candidates) {
        std::size_t vl = __riscv_vsetvl_e32m8(n_candidates - j);
        vfloat32m8_t vx = __riscv_vle32_v_f32m8(&gx[j], vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(&gy[j], vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(&gz[j], vl);

        vfloat32m8_t dx = __riscv_vfsub_vf_f32m8(vx, qx, vl);
        vfloat32m8_t dy = __riscv_vfsub_vf_f32m8(vy, qy, vl);
        vfloat32m8_t dz = __riscv_vfsub_vf_f32m8(vz, qz, vl);

        vfloat32m8_t d2 = __riscv_vfmul_vv_f32m8(dx, dx, vl);
        d2 = __riscv_vfmacc_vv_f32m8(d2, dy, dy, vl);
        d2 = __riscv_vfmacc_vv_f32m8(d2, dz, dz, vl);

        __riscv_vse32_v_f32m8(&dists[j], d2, vl);
        j += vl;
    }

    // Scalar find-min
    for (std::size_t i = 0; i < n_candidates; ++i) {
        if (dists[i] < best_dist_sq) {
            best_dist_sq = dists[i];
            best_idx = candidate_indices[i];
        }
    }
#else
    // Full RVV path with indexed gather
    std::size_t i = 0;
    while (i < n_candidates) {
        std::size_t vl = __riscv_vsetvl_e32m2(n_candidates - i);

        // Load candidate indices and convert to byte offsets
        vint32m2_t v_idx = __riscv_vle32_v_i32m2(&candidate_indices[i], vl);
        vuint32m2_t v_uidx = __riscv_vreinterpret_v_i32m2_u32m2(v_idx);
        vuint32m2_t v_byte_off = __riscv_vsll_vx_u32m2(v_uidx, 2, vl);

        // Indexed gather of target coordinates
        vfloat32m2_t vx = __riscv_vluxei32_v_f32m2(tgt_x, v_byte_off, vl);
        vfloat32m2_t vy = __riscv_vluxei32_v_f32m2(tgt_y, v_byte_off, vl);
        vfloat32m2_t vz = __riscv_vluxei32_v_f32m2(tgt_z, v_byte_off, vl);

        // Compute squared distances
        vfloat32m2_t dx = __riscv_vfsub_vf_f32m2(vx, qx, vl);
        vfloat32m2_t dy = __riscv_vfsub_vf_f32m2(vy, qy, vl);
        vfloat32m2_t dz = __riscv_vfsub_vf_f32m2(vz, qz, vl);

        vfloat32m2_t d2 = __riscv_vfmul_vv_f32m2(dx, dx, vl);
        d2 = __riscv_vfmacc_vv_f32m2(d2, dy, dy, vl);
        d2 = __riscv_vfmacc_vv_f32m2(d2, dz, dz, vl);

        // Store distances temporarily and find min (scalar scan for now)
        float tmp_dists[64]; // max VL for m2
        int tmp_indices[64];
        __riscv_vse32_v_f32m2(tmp_dists, d2, vl);
        __riscv_vse32_v_i32m2(tmp_indices, v_idx, vl);

        for (std::size_t k = 0; k < vl; ++k) {
            if (tmp_dists[k] < best_dist_sq) {
                best_dist_sq = tmp_dists[k];
                best_idx = tmp_indices[k];
            }
        }

        i += vl;
    }
#endif
}

// ============================================================================
// Stage 3 + 4: Residual + Jacobian + Reduction — RVV kernel
// ============================================================================

void residual_jacobian_rvv(std::size_t n,
                            const float* src_x, const float* src_y, const float* src_z,
                            const float* tgt_x, const float* tgt_y, const float* tgt_z,
                            const float* tgt_nx, const float* tgt_ny, const float* tgt_nz,
                            float* JtJ_upper, float* Jtr, float& sum_error) {
    // Zero output accumulators
    for (int i = 0; i < 21; ++i) JtJ_upper[i] = 0.0f;
    for (int i = 0; i < 6; ++i) Jtr[i] = 0.0f;
    sum_error = 0.0f;

    if (n == 0) return;

    // Per-pair Jacobian row:
    //   J = [nz*sy - ny*sz,  nx*sz - nz*sx,  ny*sx - nx*sy,  nx, ny, nz]
    // where (sx, sy, sz) = transformed source point, (nx, ny, nz) = target normal
    //
    // Residual: r = nx*(sx-tx) + ny*(sy-ty) + nz*(sz-tz)
    //
    // We accumulate JtJ (6×6 symmetric, 21 upper-triangle entries) and Jtr (6 entries)
    // using RVV vectorized computation + reduction.

    // Accumulator arrays for the 21 JtJ entries and 6 Jtr entries
    // We'll compute per-pair contributions vectorized and reduce.

    // Temporary SoA buffers for Jacobian columns
    std::vector<float> j0(n), j1(n), j2(n), j3(n), j4(n), j5(n), res(n);

    // Phase 1: Compute Jacobian rows and residuals (fully vectorized, LMUL=m8)
    std::size_t i = 0;
    while (i < n) {
        std::size_t vl = __riscv_vsetvl_e32m8(n - i);

        // Load source (transformed) and target points
        vfloat32m8_t v_sx = __riscv_vle32_v_f32m8(&src_x[i], vl);
        vfloat32m8_t v_sy = __riscv_vle32_v_f32m8(&src_y[i], vl);
        vfloat32m8_t v_sz = __riscv_vle32_v_f32m8(&src_z[i], vl);

        vfloat32m8_t v_tx = __riscv_vle32_v_f32m8(&tgt_x[i], vl);
        vfloat32m8_t v_ty = __riscv_vle32_v_f32m8(&tgt_y[i], vl);
        vfloat32m8_t v_tz = __riscv_vle32_v_f32m8(&tgt_z[i], vl);

        // Load target normals
        vfloat32m8_t v_nx = __riscv_vle32_v_f32m8(&tgt_nx[i], vl);
        vfloat32m8_t v_ny = __riscv_vle32_v_f32m8(&tgt_ny[i], vl);
        vfloat32m8_t v_nz = __riscv_vle32_v_f32m8(&tgt_nz[i], vl);

        // Differences: diff = source_transformed - target
        vfloat32m8_t v_dx = __riscv_vfsub_vv_f32m8(v_sx, v_tx, vl);
        vfloat32m8_t v_dy = __riscv_vfsub_vv_f32m8(v_sy, v_ty, vl);
        vfloat32m8_t v_dz = __riscv_vfsub_vv_f32m8(v_sz, v_tz, vl);

        // Residual: r = nx*dx + ny*dy + nz*dz (point-to-plane distance)
        vfloat32m8_t v_r = __riscv_vfmul_vv_f32m8(v_nx, v_dx, vl);
        v_r = __riscv_vfmacc_vv_f32m8(v_r, v_ny, v_dy, vl);
        v_r = __riscv_vfmacc_vv_f32m8(v_r, v_nz, v_dz, vl);

        // Jacobian row: J = [cross(s, n), n]
        // j0 = nz*sy - ny*sz
        vfloat32m8_t v_j0 = __riscv_vfmul_vv_f32m8(v_nz, v_sy, vl);
        v_j0 = __riscv_vfnmsac_vv_f32m8(v_j0, v_ny, v_sz, vl);

        // j1 = nx*sz - nz*sx
        vfloat32m8_t v_j1 = __riscv_vfmul_vv_f32m8(v_nx, v_sz, vl);
        v_j1 = __riscv_vfnmsac_vv_f32m8(v_j1, v_nz, v_sx, vl);

        // j2 = ny*sx - nx*sy
        vfloat32m8_t v_j2 = __riscv_vfmul_vv_f32m8(v_ny, v_sx, vl);
        v_j2 = __riscv_vfnmsac_vv_f32m8(v_j2, v_nx, v_sy, vl);

        // j3 = nx, j4 = ny, j5 = nz
        // (already loaded as v_nx, v_ny, v_nz)

        // Store Jacobian columns and residuals for phase 2 reduction
        __riscv_vse32_v_f32m8(&j0[i], v_j0, vl);
        __riscv_vse32_v_f32m8(&j1[i], v_j1, vl);
        __riscv_vse32_v_f32m8(&j2[i], v_j2, vl);
        __riscv_vse32_v_f32m8(&j3[i], v_nx, vl);
        __riscv_vse32_v_f32m8(&j4[i], v_ny, vl);
        __riscv_vse32_v_f32m8(&j5[i], v_nz, vl);
        __riscv_vse32_v_f32m8(&res[i], v_r, vl);

        i += vl;
    }

    // Phase 2: Reduce to 6×6 system using RVV tree-reduction (vfredusum)
    // JtJ[a][b] = sum_i(J_a_i * J_b_i)
    // Jtr[a]    = sum_i(J_a_i * r_i)
    const float* jcols[6] = {j0.data(), j1.data(), j2.data(),
                              j3.data(), j4.data(), j5.data()};

    // Compute 21 upper-triangle entries of JtJ
    int jtj_idx = 0;
    for (int a = 0; a < 6; ++a) {
        for (int b = a; b < 6; ++b) {
            float acc = 0.0f;
            std::size_t k = 0;
            while (k < n) {
                std::size_t vl = __riscv_vsetvl_e32m8(n - k);
                vfloat32m8_t va = __riscv_vle32_v_f32m8(&jcols[a][k], vl);
                vfloat32m8_t vb = __riscv_vle32_v_f32m8(&jcols[b][k], vl);
                vfloat32m8_t prod = __riscv_vfmul_vv_f32m8(va, vb, vl);
                vfloat32m1_t zero = __riscv_vfmv_s_f_f32m1(0.0f, 1);
                vfloat32m1_t vsum = __riscv_vfredusum_vs_f32m8_f32m1(prod, zero, vl);
                acc += __riscv_vfmv_f_s_f32m1_f32(vsum);
                k += vl;
            }
            JtJ_upper[jtj_idx++] = acc;
        }
    }

    // Compute 6 entries of Jtr
    for (int a = 0; a < 6; ++a) {
        float acc = 0.0f;
        std::size_t k = 0;
        while (k < n) {
            std::size_t vl = __riscv_vsetvl_e32m8(n - k);
            vfloat32m8_t va = __riscv_vle32_v_f32m8(&jcols[a][k], vl);
            vfloat32m8_t vr = __riscv_vle32_v_f32m8(&res[k], vl);
            vfloat32m8_t prod = __riscv_vfmul_vv_f32m8(va, vr, vl);
            vfloat32m1_t zero = __riscv_vfmv_s_f_f32m1(0.0f, 1);
            vfloat32m1_t vsum = __riscv_vfredusum_vs_f32m8_f32m1(prod, zero, vl);
            acc += __riscv_vfmv_f_s_f32m1_f32(vsum);
            k += vl;
        }
        Jtr[a] = acc;
    }

    // Compute sum of |residual| for mean error
    {
        std::size_t k = 0;
        while (k < n) {
            std::size_t vl = __riscv_vsetvl_e32m8(n - k);
            vfloat32m8_t vr = __riscv_vle32_v_f32m8(&res[k], vl);
            // Absolute value via max(r, -r) pattern
            vfloat32m8_t neg_vr = __riscv_vfneg_v_f32m8(vr, vl);
            vfloat32m8_t abs_r = __riscv_vfmax_vv_f32m8(vr, neg_vr, vl);
            vfloat32m1_t zero = __riscv_vfmv_s_f_f32m1(0.0f, 1);
            vfloat32m1_t vsum = __riscv_vfredusum_vs_f32m8_f32m1(abs_r, zero, vl);
            sum_error += __riscv_vfmv_f_s_f32m1_f32(vsum);
            k += vl;
        }
    }
}

// ============================================================================
// Stage 2 implementation: findCorrespondences
// ============================================================================

void ICPRegistration::findCorrespondences(const PointCloudSoA& source,
                                           const PointCloudSoA& target,
                                           const SpatialHash& hash,
                                           float max_dist,
                                           const SE3Transform& current_transform,
                                           std::vector<CorrespondencePair>& corr) {
    corr.clear();
    corr.reserve(source.n);

    float max_dist_sq = max_dist * max_dist;

    for (std::size_t i = 0; i < source.n; ++i) {
        // Transform source point by current estimate
        PointXYZ src_pt = {source.x[i], source.y[i], source.z[i]};
        PointXYZ transformed = current_transform.apply(src_pt);

        // Query spatial hash for candidates within max_dist
        std::vector<int> nbr_indices;
        std::vector<float> nbr_dists;
        hash.radiusSearch(transformed, max_dist, nbr_indices, nbr_dists);

        if (nbr_indices.empty()) continue;

        // Find nearest among candidates using RVV distance kernel
        int best_idx = -1;
        float best_dist_sq = max_dist_sq;

        // Use RVV-accelerated distance computation
        correspondence_distance_rvv(nullptr, nullptr, nullptr,
                                     target.x, target.y, target.z,
                                     nbr_indices.data(), nbr_indices.size(),
                                     transformed.x, transformed.y, transformed.z,
                                     best_idx, best_dist_sq);

        if (best_idx >= 0 && best_dist_sq < max_dist_sq) {
            corr.push_back({(int)i, best_idx, best_dist_sq});
        }
    }
}

// ============================================================================
// Stage 3+4 implementation: computeResidualsJacobians
// ============================================================================

void ICPRegistration::computeResidualsJacobians(const PointCloudSoA& source,
                                                 const PointCloudSoA& target,
                                                 const float* target_nx,
                                                 const float* target_ny,
                                                 const float* target_nz,
                                                 const SE3Transform& current_transform,
                                                 const std::vector<CorrespondencePair>& corr,
                                                 float* JtJ_upper,
                                                 float* Jtr,
                                                 float& mean_error) {
    std::size_t n = corr.size();
    if (n == 0) {
        for (int i = 0; i < 21; ++i) JtJ_upper[i] = 0.0f;
        for (int i = 0; i < 6; ++i) Jtr[i] = 0.0f;
        mean_error = 1e10f;
        return;
    }

    // Gather matched pairs into contiguous SoA buffers for RVV processing
    std::vector<float> src_tx(n), src_ty(n), src_tz(n);
    std::vector<float> tgt_px(n), tgt_py(n), tgt_pz(n);
    std::vector<float> tgt_pnx(n), tgt_pny(n), tgt_pnz(n);

    for (std::size_t i = 0; i < n; ++i) {
        int si = corr[i].source_idx;
        int ti = corr[i].target_idx;

        // Transform source point
        PointXYZ sp = {source.x[si], source.y[si], source.z[si]};
        PointXYZ tp = current_transform.apply(sp);
        src_tx[i] = tp.x;
        src_ty[i] = tp.y;
        src_tz[i] = tp.z;

        // Copy target point and normal
        tgt_px[i] = target.x[ti];
        tgt_py[i] = target.y[ti];
        tgt_pz[i] = target.z[ti];
        tgt_pnx[i] = target_nx[ti];
        tgt_pny[i] = target_ny[ti];
        tgt_pnz[i] = target_nz[ti];
    }

    // Call the vectorized kernel
    float sum_error = 0.0f;
    residual_jacobian_rvv(n,
                           src_tx.data(), src_ty.data(), src_tz.data(),
                           tgt_px.data(), tgt_py.data(), tgt_pz.data(),
                           tgt_pnx.data(), tgt_pny.data(), tgt_pnz.data(),
                           JtJ_upper, Jtr, sum_error);

    mean_error = sum_error / (float)n;
}

// ============================================================================
// Stage 5: Solve 6×6 system (Cholesky decomposition, scalar)
// ============================================================================

bool ICPRegistration::solve6x6(const float* JtJ_upper, const float* Jtr, float* dx) {
    // Expand upper-triangle to full 6×6 matrix
    float A[6][6];
    int idx = 0;
    for (int i = 0; i < 6; ++i) {
        for (int j = i; j < 6; ++j) {
            A[i][j] = JtJ_upper[idx];
            A[j][i] = JtJ_upper[idx];
            idx++;
        }
    }

    // RHS: b = -Jtr
    float b[6];
    for (int i = 0; i < 6; ++i) b[i] = -Jtr[i];

    // Cholesky decomposition: A = L * L^T
    float L[6][6] = {};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j <= i; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < j; ++k) {
                sum += L[i][k] * L[j][k];
            }
            if (i == j) {
                float diag = A[i][i] - sum;
                if (diag <= 0.0f) return false; // Not positive definite
                L[i][j] = std::sqrt(diag);
            } else {
                L[i][j] = (A[i][j] - sum) / L[j][j];
            }
        }
    }

    // Forward substitution: L * y = b
    float y[6];
    for (int i = 0; i < 6; ++i) {
        float sum = 0.0f;
        for (int k = 0; k < i; ++k) sum += L[i][k] * y[k];
        y[i] = (b[i] - sum) / L[i][i];
    }

    // Backward substitution: L^T * dx = y
    for (int i = 5; i >= 0; --i) {
        float sum = 0.0f;
        for (int k = i + 1; k < 6; ++k) sum += L[k][i] * dx[k];
        dx[i] = (y[i] - sum) / L[i][i];
    }

    return true;
}

// ============================================================================
// Convert 6-DOF increment to SE3Transform
// ============================================================================

SE3Transform ICPRegistration::incrementToTransform(const float* dx) {
    // dx = [alpha, beta, gamma, tx, ty, tz]
    // Small-angle approximation for rotation
    return SE3Transform::fromAxisAngle(dx[0], dx[1], dx[2], dx[3], dx[4], dx[5]);
}

// ============================================================================
// Main ICP alignment function
// ============================================================================

void ICPRegistration::align(const PointCloudSoA& source_soa,
                             const PointCloudSoA& target_soa,
                             const float* target_nx, const float* target_ny, const float* target_nz,
                             const SpatialHash& hash,
                             const TrackingConfig& config,
                             SE3Transform& out_transform,
                             int& out_iterations,
                             float& out_error,
                             double& timing_correspondence_ms,
                             double& timing_residual_ms,
                             double& timing_reduction_ms,
                             double& timing_solve_ms) {
    // Initialize
    SE3Transform current_transform; // identity
    out_iterations = 0;
    out_error = 1e10f;
    timing_correspondence_ms = 0;
    timing_residual_ms = 0;
    timing_reduction_ms = 0;
    timing_solve_ms = 0;

    float prev_error = 1e20f;

    for (int iter = 0; iter < config.icp_max_iterations; ++iter) {
        // --- Stage 2: Correspondence Search ---
        auto t0 = std::chrono::high_resolution_clock::now();
        findCorrespondences(source_soa, target_soa, hash,
                           config.correspondence_max_dist,
                           current_transform, correspondences_);
        auto t1 = std::chrono::high_resolution_clock::now();
        timing_correspondence_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();

        if (correspondences_.size() < 6) {
            // Not enough correspondences to solve
            break;
        }

        // --- Stage 3 + 4: Residual + Jacobian + Reduction ---
        float JtJ_upper[21];
        float Jtr_vec[6];
        float mean_err = 0.0f;

        t0 = std::chrono::high_resolution_clock::now();
        computeResidualsJacobians(source_soa, target_soa,
                                   target_nx, target_ny, target_nz,
                                   current_transform, correspondences_,
                                   JtJ_upper, Jtr_vec, mean_err);
        t1 = std::chrono::high_resolution_clock::now();
        timing_residual_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();

        // --- Stage 5: Solve ---
        float dx[6] = {};
        t0 = std::chrono::high_resolution_clock::now();
        bool solved = solve6x6(JtJ_upper, Jtr_vec, dx);
        t1 = std::chrono::high_resolution_clock::now();
        timing_solve_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();

        if (!solved) break;

        // Apply increment
        SE3Transform increment = incrementToTransform(dx);
        current_transform = increment.compose(current_transform);

        out_error = mean_err;
        out_iterations = iter + 1;

        // Convergence check: is the update small enough?
        float update_norm_sq = 0.0f;
        for (int k = 0; k < 6; ++k) update_norm_sq += dx[k] * dx[k];

        if (update_norm_sq < config.icp_convergence_thresh) {
            break;
        }

        // Also check if error is increasing (diverging)
        if (mean_err > prev_error * 1.1f && iter > 2) {
            break;
        }
        prev_error = mean_err;
    }

    out_transform = current_transform;
}

} // namespace rvv_pcl
