#pragma once

#include "tracking_types.h"
#include "icp_registration.h"
#include "../include/rvv_pcl.h"
#include "../include/euclidean_clustering.h"
#include <vector>

namespace rvv_pcl {

// ============================================================================
// Tracking Pipeline — Full 9-Stage Tracking Mode Orchestrator
// ============================================================================

/**
 * @brief Orchestrates the full 9-stage tracking mode pipeline.
 *
 * Pipeline stages:
 *   1. Voxel Downsampling — reuses existing voxel_grid_downsamp_rvv_v2()
 *   2–5. ICP Registration — via ICPRegistration class
 *   6. Propagate — apply transform to stored planes/clusters/normals (RVV)
 *   7. Verify — check propagated models against new frame (RVV)
 *   8. Split — separate confirmed vs residual points (RVV)
 *   9. Update Spatial Index — incremental insert/remove
 *
 * After stage 8, residual points are optionally passed to the existing
 * full pipeline (RANSAC + clustering + normal estimation) to detect
 * new geometric primitives.
 */
class TrackingPipeline {
public:
    TrackingPipeline();

    /**
     * @brief Initialize tracking from a keyframe.
     *
     * Runs the full pipeline on the keyframe and stores the results
     * as the initial tracking state.
     *
     * @param keyframe_points  Keyframe point cloud (AoS).
     * @param n_points         Number of points.
     * @param config           Tracking configuration.
     */
    void initializeKeyframe(const PointXYZ* keyframe_points,
                            std::size_t n_points,
                            const TrackingConfig& config);

    /**
     * @brief Process a tracking frame (non-keyframe).
     *
     * Runs the 9-stage tracking pipeline against the stored keyframe state.
     *
     * @param frame_points  New frame point cloud (AoS).
     * @param n_points      Number of points.
     * @param config        Tracking configuration.
     * @return TrackingResult with transform, confirmed/new primitives, timings.
     */
    TrackingResult processTrackingFrame(const PointXYZ* frame_points,
                                        std::size_t n_points,
                                        const TrackingConfig& config);

    /**
     * @brief Get the current tracking state (for inspection/debugging).
     */
    const TrackingState& getState() const { return state_; }

    /**
     * @brief Run the full pipeline on a point cloud (for comparison/benchmarking).
     *
     * This is the "keyframe mode" pipeline that tracking mode replaces on
     * non-keyframes. Exposed as a static method for benchmarking.
     *
     * @param points     Input points (AoS).
     * @param n_points   Number of points.
     * @param config     Configuration.
     * @param out_time_ms Output: total time in milliseconds.
     */
    static void runFullPipeline(const PointXYZ* points,
                                std::size_t n_points,
                                const TrackingConfig& config,
                                double& out_time_ms);

private:
    TrackingState state_;
    ICPRegistration icp_;

    // === Stage 6: Propagate ===
    // Apply transform to stored planes, clusters, and reference normals.
    // Fully vectorized: batched matrix-vector transforms on SoA data.
    void propagateModels(const SE3Transform& transform,
                          std::vector<PlaneModel>& planes,
                          std::vector<ClusterModel>& clusters);

    // === Stage 7: Verify ===
    // Check propagated planes/clusters against new frame's points.
    // Per-point distance-to-plane and distance-to-centroid checks.
    // Returns vectors of confirmed/rejected indices.
    void verifyModels(const PointCloudSoA& new_frame,
                       const std::vector<PlaneModel>& planes,
                       const std::vector<ClusterModel>& clusters,
                       std::vector<bool>& point_confirmed,
                       std::vector<int>& confirmed_plane_indices,
                       std::vector<int>& confirmed_cluster_indices,
                       const TrackingConfig& config);

    // === Stage 8: Split ===
    // Partition points into confirmed (matched existing model) and residual
    // (unmatched — needs full pipeline). RVV compaction.
    void splitConfirmedResidual(const PointCloudSoA& new_frame,
                                 const std::vector<bool>& point_confirmed,
                                 std::vector<PointXYZ>& confirmed_out,
                                 std::vector<PointXYZ>& residual_out);

    // === Stage 9: Update Spatial Index ===
    // Insert new/residual points, remove stale points.
    void updateSpatialIndex(const std::vector<PointXYZ>& new_points,
                            const TrackingConfig& config);
};

// ============================================================================
// Standalone RVV Kernels for Tracking Pipeline Stages (exposed for testing)
// ============================================================================

/**
 * @brief Stage 6: RVV-accelerated batched transform application.
 *
 * Applies SE3Transform to N points stored in SoA layout.
 * Uses LMUL=m8 for maximum throughput.
 * Pattern: batched matrix-vector multiply (3 fmacc per coordinate).
 */
void propagate_transform_rvv(const SE3Transform& T,
                              const float* in_x, const float* in_y, const float* in_z,
                              float* out_x, float* out_y, float* out_z,
                              std::size_t n);

/**
 * @brief Stage 7: RVV-accelerated plane verification.
 *
 * For each point, compute |a*x + b*y + c*z + d| and check <= threshold.
 * Returns count of inliers and sets mask bits.
 * Same pattern as RANSAC inlier counting.
 */
std::size_t verify_plane_rvv(const PointCloudSoA& cloud,
                              const PlaneModel& plane,
                              float dist_thresh,
                              std::vector<bool>& inlier_mask);

/**
 * @brief Stage 7: RVV-accelerated cluster verification.
 *
 * For each point, compute squared distance to centroid and check <= radius².
 * Same distance kernel pattern as SOR.
 */
std::size_t verify_cluster_rvv(const PointCloudSoA& cloud,
                                const ClusterModel& cluster,
                                float dist_thresh,
                                std::vector<bool>& inlier_mask);

/**
 * @brief Stage 8: RVV-accelerated point compaction (split).
 *
 * Given a boolean mask, compacts points into two output arrays:
 * confirmed (mask=true) and residual (mask=false).
 * Uses vcompress pattern from RANSAC extract.
 */
void compact_points_rvv(const PointCloudSoA& cloud,
                         const std::vector<bool>& mask,
                         std::vector<PointXYZ>& confirmed,
                         std::vector<PointXYZ>& residual);

} // namespace rvv_pcl
