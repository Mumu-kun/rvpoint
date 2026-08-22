#pragma once

#include "registration/tracking_types.h"
#include "core/point_types.h"
#include "search/spatial_hashing.h"
#include <vector>

namespace rvpoint {

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
 */
class ICPRegistration {
public:
    ICPRegistration() = default;

    /**
     * @brief Run ICP registration between source and target clouds.
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
     * @brief Get the correspondences from the last ICP iteration.
     */
    const std::vector<CorrespondencePair>& getLastCorrespondences() const {
        return correspondences_;
    }

private:
    std::vector<CorrespondencePair> correspondences_;

    // Stage 2: Correspondence Search
    void findCorrespondences(const PointCloudSoA& source,
                             const PointCloudSoA& target,
                             const SpatialHash& hash,
                             float max_dist,
                             const SE3Transform& current_transform,
                             std::vector<CorrespondencePair>& corr);

    // Stage 3+4: Residual + Jacobian + Reduction
    void computeResidualsJacobians(const PointCloudSoA& source,
                                   const PointCloudSoA& target,
                                   const float* target_nx,
                                   const float* target_ny,
                                   const float* target_nz,
                                   const SE3Transform& current_transform,
                                   const std::vector<CorrespondencePair>& corr,
                                   float* JtJ_upper,
                                   float* Jtr,
                                   float& mean_error);

    // Stage 5: Solve 6×6 system
    bool solve6x6(const float* JtJ_upper, const float* Jtr, float* dx);

    // Convert 6-DOF incremental update to SE3Transform
    SE3Transform incrementToTransform(const float* dx);
};

// ============================================================================
// Standalone Kernels for ICP
// ============================================================================

void correspondence_distance_rvv(const float* src_x, const float* src_y, const float* src_z,
                                  const float* tgt_x, const float* tgt_y, const float* tgt_z,
                                  const int* candidate_indices, std::size_t n_candidates,
                                  float qx, float qy, float qz,
                                  int& best_idx, float& best_dist_sq);

void residual_jacobian_rvv(std::size_t n,
                            const float* src_x, const float* src_y, const float* src_z,
                            const float* tgt_x, const float* tgt_y, const float* tgt_z,
                            const float* tgt_nx, const float* tgt_ny, const float* tgt_nz,
                            float* JtJ_upper, float* Jtr, float& sum_error);

} // namespace rvpoint
