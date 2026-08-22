#pragma once

#include "registration/tracking_types.h"
#include "registration/icp_registration.h"
#include "core/point_types.h"
#include "segmentation/euclidean_clustering.h"
#include <vector>

namespace rvpoint {

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
 */
class TrackingPipeline {
public:
    TrackingPipeline();

    /**
     * @brief Initialize tracking from a keyframe.
     */
    void initializeKeyframe(const PointXYZ* keyframe_points,
                            std::size_t n_points,
                            const TrackingConfig& config);

    /**
     * @brief Process a tracking frame (non-keyframe).
     */
    TrackingResult processTrackingFrame(const PointXYZ* frame_points,
                                        std::size_t n_points,
                                        const TrackingConfig& config);

    /**
     * @brief Get the current tracking state.
     */
    const TrackingState& getState() const { return state_; }

    /**
     * @brief Run the full pipeline on a point cloud (for comparison/benchmarking).
     */
    static void runFullPipeline(const PointXYZ* points,
                                std::size_t n_points,
                                const TrackingConfig& config,
                                double& out_time_ms);

private:
    TrackingState state_;
    ICPRegistration icp_;

    // Stage 6: Propagate
    void propagateModels(const SE3Transform& transform,
                          std::vector<PlaneModel>& planes,
                          std::vector<ClusterModel>& clusters);

    // Stage 7: Verify
    void verifyModels(const PointCloudSoA& new_frame,
                       const std::vector<PlaneModel>& planes,
                       const std::vector<ClusterModel>& clusters,
                       std::vector<bool>& point_confirmed,
                       std::vector<int>& confirmed_plane_indices,
                       std::vector<int>& confirmed_cluster_indices,
                       const TrackingConfig& config);

    // Stage 8: Split
    void splitConfirmedResidual(const PointCloudSoA& new_frame,
                                 const std::vector<bool>& point_confirmed,
                                 std::vector<PointXYZ>& confirmed_out,
                                 std::vector<PointXYZ>& residual_out);

    // Stage 9: Update Spatial Index
    void updateSpatialIndex(const std::vector<PointXYZ>& new_points,
                            const TrackingConfig& config);
};

// ============================================================================
// Standalone RVV Kernels for Tracking Pipeline Stages
// ============================================================================

void propagate_transform_rvv(const SE3Transform& T,
                              const float* in_x, const float* in_y, const float* in_z,
                              float* out_x, float* out_y, float* out_z,
                              std::size_t n);

std::size_t verify_plane_rvv(const PointCloudSoA& cloud,
                              const PlaneModel& plane,
                              float dist_thresh,
                              std::vector<bool>& inlier_mask);

std::size_t verify_cluster_rvv(const PointCloudSoA& cloud,
                                const ClusterModel& cluster,
                                float dist_thresh,
                                std::vector<bool>& inlier_mask);

void compact_points_rvv(const PointCloudSoA& cloud,
                         const std::vector<bool>& mask,
                         std::vector<PointXYZ>& confirmed,
                         std::vector<PointXYZ>& residual);

} // namespace rvpoint
