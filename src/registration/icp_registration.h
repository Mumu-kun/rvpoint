#pragma once

#include "tracking_types.h"
#include "../include/rvv_pcl.h"
#include <vector>

namespace rvv_pcl {

// ============================================================================
// ICP Registration — Stages 2–5 of the Tracking Mode Pipeline
// ============================================================================

/**
 * @brief Point-to-Plane ICP Registration using RVV acceleration.
 *
 * Implements the iterative loop of Stages 2–5:
 *   Stage 2: Correspondence Search
 *   Stage 3: Residual + Jacobian Computation
 *   Stage 4: Reduction to 6×6 System
 *   Stage 5: Solve for Transform
 *
 * All vectorizable stages use LMUL=m8 for maximum throughput.
 */
class ICPRegistration {
public:
    ICPRegistration() = default;

    /**
     * @brief Run ICP registration between source and target clouds.
     *
     * @param source_soa  New frame point cloud (SoA, downsampled).
     * @param target_soa  Reference cloud (SoA, from keyframe).
     * @param target_nx   Reference cloud normals X.
     * @param target_ny   Reference cloud normals Y.
     * @param target_nz   Reference cloud normals Z.
     * @param hash        Pre-built spatial hash on the target cloud.
     * @param config      ICP configuration parameters.
     * @param out_transform  Output: the computed transform.
     * @param out_iterations Output: number of ICP iterations used.
     * @param out_error      Output: final mean residual error.
     * @param timing_correspondence_ms  Output: total time in correspondence stage.
     * @param timing_residual_ms        Output: total time in residual+jacobian stage.
     * @param timing_reduction_ms       Output: total time in reduction stage.
     * @param timing_solve_ms           Output: total time in solve stage.
     */
    void align(const PointCloudSoA& source_soa,
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
               double& timing_solve_ms);

    /**
     * @brief Get the correspondences from the last ICP iteration (for debugging).
     */
    const std::vector<CorrespondencePair>& getLastCorrespondences() const {
        return correspondences_;
    }

private:
    std::vector<CorrespondencePair> correspondences_;

    // === Stage 2: Correspondence Search ===
    // For each source point, find nearest neighbor in target cloud via spatial hash.
    // RVV-accelerated distance evaluation.
    void findCorrespondences(const PointCloudSoA& source,
                             const PointCloudSoA& target,
                             const SpatialHash& hash,
                             float max_dist,
                             const SE3Transform& current_transform,
                             std::vector<CorrespondencePair>& corr);

    // === Stage 3: Residual + Jacobian ===
    // Compute point-to-plane residual and 1×6 Jacobian for each pair.
    // Fully vectorized per-pair — same data-parallel structure as RANSAC inlier counting.
    //
    // For point-to-plane ICP:
    //   residual_i = n_i . (T*p_source_i - p_target_i)
    //   J_i = [n_i × (T*p_source_i), n_i]  (1×6 row vector)
    //
    // Output: JtJ (6×6 upper triangle, 21 values) and Jtr (6×1)
    void computeResidualsJacobians(const PointCloudSoA& source,
                                    const PointCloudSoA& target,
                                    const float* target_nx,
                                    const float* target_ny,
                                    const float* target_nz,
                                    const SE3Transform& current_transform,
                                    const std::vector<CorrespondencePair>& corr,
                                    float* JtJ_upper,  // 21 values
                                    float* Jtr,        // 6 values
                                    float& mean_error);

    // === Stage 4: Reduction to 6×6 ===
    // Tree-reduce per-pair JtJ/Jtr contributions into one 6×6 system.
    // This is fused into computeResidualsJacobians using vfredusum.

    // === Stage 5: Solve 6×6 system ===
    // Scalar Cholesky solve: JtJ * dx = -Jtr
    // Returns the 6-DOF incremental update (3 rotation + 3 translation).
    bool solve6x6(const float* JtJ_upper, const float* Jtr, float* dx);

    // Convert 6-DOF incremental update to SE3Transform
    SE3Transform incrementToTransform(const float* dx);
};

// ============================================================================
// Standalone RVV Kernels for ICP (exposed for testing/benchmarking)
// ============================================================================

/**
 * @brief RVV-accelerated batch distance computation for correspondence.
 *
 * For each source point (transformed), compute squared distance to candidate
 * target points identified by the spatial hash, keeping only the nearest.
 *
 * Uses the same RVV distance kernel pattern as SOR's inlined distance computation.
 */
void correspondence_distance_rvv(const float* src_x, const float* src_y, const float* src_z,
                                  const float* tgt_x, const float* tgt_y, const float* tgt_z,
                                  const int* candidate_indices, std::size_t n_candidates,
                                  float qx, float qy, float qz,
                                  int& best_idx, float& best_dist_sq);

/**
 * @brief RVV-accelerated point-to-plane residual + Jacobian computation.
 *
 * Processes N correspondence pairs in vector batches.
 * For each pair i:
 *   src_transformed_i = T * source_i
 *   diff_i = src_transformed_i - target_i
 *   residual_i = nx_i * diff_x + ny_i * diff_y + nz_i * diff_z
 *   J_i = [cross(src_transformed_i, normal_i), normal_i]
 *
 * Accumulates JtJ (21 upper-triangle values) and Jtr (6 values) via reduction.
 *
 * @param n          Number of correspondence pairs.
 * @param src_x/y/z  Transformed source point coordinates (SoA).
 * @param tgt_x/y/z  Target point coordinates (SoA).
 * @param tgt_nx/ny/nz Target normals (SoA).
 * @param JtJ_upper  Output: accumulated 6×6 upper triangle (21 floats).
 * @param Jtr        Output: accumulated 6×1 vector (6 floats).
 * @param sum_error  Output: accumulated sum of |residual|.
 */
void residual_jacobian_rvv(std::size_t n,
                            const float* src_x, const float* src_y, const float* src_z,
                            const float* tgt_x, const float* tgt_y, const float* tgt_z,
                            const float* tgt_nx, const float* tgt_ny, const float* tgt_nz,
                            float* JtJ_upper, float* Jtr, float& sum_error);

} // namespace rvv_pcl
