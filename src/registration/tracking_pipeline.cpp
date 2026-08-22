// tracking_pipeline.cpp
//
// Full 9-Stage Tracking Mode Pipeline — RVV-accelerated
// =====================================================
// Orchestrates all tracking stages and wires residual points back into
// the existing full pipeline (RANSAC, clustering, normal estimation).

#include "registration/tracking_pipeline.h"
#include "filters/voxel_grid.h"
#include "features/normal_estimation.h"
#include "segmentation/ransac_plane.h"
#include "segmentation/euclidean_clustering.h"
#include "core/profiler.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <vector>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvpoint {

// ============================================================================
// Stage 6 Kernel: Propagate Transform (RVV, LMUL=m8)
// ============================================================================

void propagate_transform_rvv(const SE3Transform& T,
                              const float* in_x, const float* in_y, const float* in_z,
                              float* out_x, float* out_y, float* out_z,
                              std::size_t n) {
    const float r00 = T.R(0,0), r01 = T.R(0,1), r02 = T.R(0,2), t_x = T.tx();
    const float r10 = T.R(1,0), r11 = T.R(1,1), r12 = T.R(1,2), t_y = T.ty();
    const float r20 = T.R(2,0), r21 = T.R(2,1), r22 = T.R(2,2), t_z = T.tz();

#if defined(__riscv_vector)
    std::size_t i = 0;
    while (i < n) {
        std::size_t vl = __riscv_vsetvl_e32m8(n - i);

        vfloat32m8_t vx = __riscv_vle32_v_f32m8(&in_x[i], vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(&in_y[i], vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(&in_z[i], vl);

        vfloat32m8_t ox = __riscv_vfmv_v_f_f32m8(t_x, vl);
        ox = __riscv_vfmacc_vf_f32m8(ox, r00, vx, vl);
        ox = __riscv_vfmacc_vf_f32m8(ox, r01, vy, vl);
        ox = __riscv_vfmacc_vf_f32m8(ox, r02, vz, vl);

        vfloat32m8_t oy = __riscv_vfmv_v_f_f32m8(t_y, vl);
        oy = __riscv_vfmacc_vf_f32m8(oy, r10, vx, vl);
        oy = __riscv_vfmacc_vf_f32m8(oy, r11, vy, vl);
        oy = __riscv_vfmacc_vf_f32m8(oy, r12, vz, vl);

        vfloat32m8_t oz = __riscv_vfmv_v_f_f32m8(t_z, vl);
        oz = __riscv_vfmacc_vf_f32m8(oz, r20, vx, vl);
        oz = __riscv_vfmacc_vf_f32m8(oz, r21, vy, vl);
        oz = __riscv_vfmacc_vf_f32m8(oz, r22, vz, vl);

        __riscv_vse32_v_f32m8(&out_x[i], ox, vl);
        __riscv_vse32_v_f32m8(&out_y[i], oy, vl);
        __riscv_vse32_v_f32m8(&out_z[i], oz, vl);

        i += vl;
    }
#else
    for (std::size_t i = 0; i < n; ++i) {
        out_x[i] = r00*in_x[i] + r01*in_y[i] + r02*in_z[i] + t_x;
        out_y[i] = r10*in_x[i] + r11*in_y[i] + r12*in_z[i] + t_y;
        out_z[i] = r20*in_x[i] + r21*in_y[i] + r22*in_z[i] + t_z;
    }
#endif
}

// ============================================================================
// Stage 7 Kernel: Verify Plane (RVV, LMUL=m8)
// ============================================================================

std::size_t verify_plane_rvv(const PointCloudSoA& cloud,
                              const PlaneModel& plane,
                              float dist_thresh,
                              std::vector<bool>& inlier_mask) {
    const float a = plane.a, b = plane.b, c = plane.c, d = plane.d;
    std::size_t count = 0;

#if defined(__riscv_vector)
    std::size_t i = 0;
    while (i < cloud.n) {
        std::size_t vl = __riscv_vsetvl_e32m8(cloud.n - i);

        vfloat32m8_t vx = __riscv_vle32_v_f32m8(&cloud.x[i], vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(&cloud.y[i], vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(&cloud.z[i], vl);

        vfloat32m8_t dist = __riscv_vfmv_v_f_f32m8(d, vl);
        dist = __riscv_vfmacc_vf_f32m8(dist, a, vx, vl);
        dist = __riscv_vfmacc_vf_f32m8(dist, b, vy, vl);
        dist = __riscv_vfmacc_vf_f32m8(dist, c, vz, vl);

        vbool4_t mask_le = __riscv_vmfle_vf_f32m8_b4(dist, dist_thresh, vl);
        vbool4_t mask_ge = __riscv_vmfge_vf_f32m8_b4(dist, -dist_thresh, vl);
        vbool4_t mask_in = __riscv_vmand_mm_b4(mask_le, mask_ge, vl);

        long cnt = __riscv_vcpop_m_b4(mask_in, vl);
        count += (std::size_t)cnt;

        uint8_t mask_bytes[64];
        __riscv_vsm_v_b4(mask_bytes, mask_in, vl);
        for (std::size_t lane = 0; lane < vl; ++lane) {
            if ((mask_bytes[lane >> 3] >> (lane & 7u)) & 1u) {
                inlier_mask[i + lane] = true;
            }
        }

        i += vl;
    }
#else
    for (std::size_t i = 0; i < cloud.n; ++i) {
        float dist = a*cloud.x[i] + b*cloud.y[i] + c*cloud.z[i] + d;
        if (std::abs(dist) <= dist_thresh) {
            inlier_mask[i] = true;
            count++;
        }
    }
#endif

    return count;
}

// ============================================================================
// Stage 7 Kernel: Verify Cluster (RVV, LMUL=m8)
// ============================================================================

std::size_t verify_cluster_rvv(const PointCloudSoA& cloud,
                                const ClusterModel& cluster,
                                float dist_thresh,
                                std::vector<bool>& inlier_mask) {
    float cx = cluster.cx, cy = cluster.cy, cz = cluster.cz;
    float dist_thresh_sq = dist_thresh * dist_thresh;
    std::size_t count = 0;

#if defined(__riscv_vector)
    std::size_t i = 0;
    while (i < cloud.n) {
        std::size_t vl = __riscv_vsetvl_e32m8(cloud.n - i);

        vfloat32m8_t vx = __riscv_vle32_v_f32m8(&cloud.x[i], vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(&cloud.y[i], vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(&cloud.z[i], vl);

        vfloat32m8_t dx = __riscv_vfsub_vf_f32m8(vx, cx, vl);
        vfloat32m8_t dy = __riscv_vfsub_vf_f32m8(vy, cy, vl);
        vfloat32m8_t dz = __riscv_vfsub_vf_f32m8(vz, cz, vl);

        vfloat32m8_t d2 = __riscv_vfmul_vv_f32m8(dx, dx, vl);
        d2 = __riscv_vfmacc_vv_f32m8(d2, dy, dy, vl);
        d2 = __riscv_vfmacc_vv_f32m8(d2, dz, dz, vl);

        vbool4_t mask = __riscv_vmfle_vf_f32m8_b4(d2, dist_thresh_sq, vl);
        long cnt = __riscv_vcpop_m_b4(mask, vl);
        count += (std::size_t)cnt;

        uint8_t mask_bytes[64];
        __riscv_vsm_v_b4(mask_bytes, mask, vl);
        for (std::size_t lane = 0; lane < vl; ++lane) {
            if ((mask_bytes[lane >> 3] >> (lane & 7u)) & 1u) {
                inlier_mask[i + lane] = true;
            }
        }

        i += vl;
    }
#else
    for (std::size_t i = 0; i < cloud.n; ++i) {
        float dx = cloud.x[i] - cx;
        float dy = cloud.y[i] - cy;
        float dz = cloud.z[i] - cz;
        float d2 = dx*dx + dy*dy + dz*dz;
        if (d2 <= dist_thresh_sq) {
            inlier_mask[i] = true;
            count++;
        }
    }
#endif

    return count;
}

// ============================================================================
// Stage 8 Kernel: Compact Points (Split) (RVV, LMUL=m8)
// ============================================================================

void compact_points_rvv(const PointCloudSoA& cloud,
                         const std::vector<bool>& mask,
                         std::vector<PointXYZ>& confirmed,
                         std::vector<PointXYZ>& residual) {
    std::size_t n_confirmed = 0;
    for (std::size_t i = 0; i < cloud.n; ++i) {
        if (mask[i]) n_confirmed++;
    }
    std::size_t n_residual = cloud.n - n_confirmed;

    confirmed.resize(n_confirmed);
    residual.resize(n_residual);

#if defined(__riscv_vector) && !defined(GEM5_BUILD)
    std::vector<uint8_t> flat_mask(cloud.n);
    for (std::size_t i = 0; i < cloud.n; ++i) {
        flat_mask[i] = mask[i] ? 1 : 0;
    }

    std::size_t conf_idx = 0;
    std::size_t res_idx = 0;

    float* conf_base = reinterpret_cast<float*>(confirmed.data());
    float* res_base = reinterpret_cast<float*>(residual.data());
    const ptrdiff_t stride = (ptrdiff_t)sizeof(PointXYZ);

    std::size_t i = 0;
    while (i < cloud.n) {
        std::size_t vl = __riscv_vsetvl_e32m8(cloud.n - i);

        vfloat32m8_t vx = __riscv_vle32_v_f32m8(&cloud.x[i], vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(&cloud.y[i], vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(&cloud.z[i], vl);

        vuint8m2_t vm_bytes = __riscv_vle8_v_u8m2(&flat_mask[i], vl);
        vuint16m4_t vm_16 = __riscv_vzext_vf2_u16m4(vm_bytes, vl);
        vuint32m8_t vm_32 = __riscv_vzext_vf2_u32m8(vm_16, vl);
        vbool4_t conf_mask = __riscv_vmsne_vx_u32m8_b4(vm_32, 0, vl);
        vbool4_t res_mask = __riscv_vmseq_vx_u32m8_b4(vm_32, 0, vl);

        long conf_cnt = __riscv_vcpop_m_b4(conf_mask, vl);
        if (conf_cnt > 0) {
            __riscv_vsse32_v_f32m8(conf_base + conf_idx * 3 + 0, stride,
                                    __riscv_vcompress_vm_f32m8(vx, conf_mask, vl),
                                    (size_t)conf_cnt);
            __riscv_vsse32_v_f32m8(conf_base + conf_idx * 3 + 1, stride,
                                    __riscv_vcompress_vm_f32m8(vy, conf_mask, vl),
                                    (size_t)conf_cnt);
            __riscv_vsse32_v_f32m8(conf_base + conf_idx * 3 + 2, stride,
                                    __riscv_vcompress_vm_f32m8(vz, conf_mask, vl),
                                    (size_t)conf_cnt);
            conf_idx += (size_t)conf_cnt;
        }

        long res_cnt = __riscv_vcpop_m_b4(res_mask, vl);
        if (res_cnt > 0) {
            __riscv_vsse32_v_f32m8(res_base + res_idx * 3 + 0, stride,
                                    __riscv_vcompress_vm_f32m8(vx, res_mask, vl),
                                    (size_t)res_cnt);
            __riscv_vsse32_v_f32m8(res_base + res_idx * 3 + 1, stride,
                                    __riscv_vcompress_vm_f32m8(vy, res_mask, vl),
                                    (size_t)res_cnt);
            __riscv_vsse32_v_f32m8(res_base + res_idx * 3 + 2, stride,
                                    __riscv_vcompress_vm_f32m8(vz, res_mask, vl),
                                    (size_t)res_cnt);
            res_idx += (size_t)res_cnt;
        }

        i += vl;
    }
#else
    std::size_t c_idx = 0;
    std::size_t r_idx = 0;
    for (std::size_t i = 0; i < cloud.n; ++i) {
        PointXYZ pt = {cloud.x[i], cloud.y[i], cloud.z[i]};
        if (mask[i]) {
            confirmed[c_idx++] = pt;
        } else {
            residual[r_idx++] = pt;
        }
    }
#endif
}

// ============================================================================
// TrackingPipeline implementation
// ============================================================================

TrackingPipeline::TrackingPipeline() {}

void TrackingPipeline::initializeKeyframe(const PointXYZ* keyframe_points,
                                           std::size_t n_points,
                                           const TrackingConfig& config) {
    RVPOINT_PROFILE_SCOPE("TrackingPipeline::initializeKeyframe");

    // --- Step 1: Voxel downsample the keyframe ---
    std::vector<float> raw_x(n_points), raw_y(n_points), raw_z(n_points);
    for (std::size_t i = 0; i < n_points; ++i) {
        raw_x[i] = keyframe_points[i].x;
        raw_y[i] = keyframe_points[i].y;
        raw_z[i] = keyframe_points[i].z;
    }
    PointCloudSoA raw_soa = {raw_x.data(), raw_y.data(), raw_z.data(), n_points};

    std::vector<PointXYZ> downsampled(n_points);
    std::size_t n_down = voxel_grid_downsamp_rvv_v2(raw_soa, downsampled.data(), config.voxel_leaf_size);
    downsampled.resize(n_down);

    state_.setRefCloud(downsampled.data(), n_down);

    // --- Step 2: Compute normals on the reference cloud ---
    state_.ref_nx.resize(n_down);
    state_.ref_ny.resize(n_down);
    state_.ref_nz.resize(n_down);

    PointCloudSoA ref_soa = state_.getRefCloudSoA();
    normal_estimation_rvv(ref_soa,
                           state_.ref_nx.data(), state_.ref_ny.data(), state_.ref_nz.data(),
                           config.normal_k, config.normal_radius);

    // --- Step 3: Detect initial planes via RANSAC ---
    {
        float model[4];
        int inliers = ransac_plane_rvv(ref_soa, config.ransac_dist_thresh,
                                        config.ransac_max_iters, model);
        if (inliers > 10) {
            state_.planes.push_back(PlaneModel::fromArray(model, inliers));
        }
    }

    // --- Step 4: Detect initial clusters ---
    {
        EuclideanClustering ec;
        ec.setInputCloud(ref_soa);
        ec.setClusterTolerance(config.cluster_tolerance);
        ec.setMinClusterSize(config.min_cluster_size);
        ec.setMaxClusterSize(config.max_cluster_size);
        auto clusters = ec.extract();

        for (const auto& cl : clusters) {
            float cx = 0, cy = 0, cz = 0;
            for (int idx : cl.indices) {
                cx += ref_soa.x[idx];
                cy += ref_soa.y[idx];
                cz += ref_soa.z[idx];
            }
            float inv_n = 1.0f / (float)cl.indices.size();
            cx *= inv_n; cy *= inv_n; cz *= inv_n;

            float max_r2 = 0.0f;
            for (int idx : cl.indices) {
                float dx = ref_soa.x[idx] - cx;
                float dy = ref_soa.y[idx] - cy;
                float dz = ref_soa.z[idx] - cz;
                float r2 = dx*dx + dy*dy + dz*dz;
                if (r2 > max_r2) max_r2 = r2;
            }

            state_.clusters.push_back(ClusterModel(cx, cy, cz, std::sqrt(max_r2),
                                                     (int)cl.indices.size()));
        }
    }

    // --- Step 5: Build spatial index ---
    state_.buildSpatialIndex(config.spatial_hash_cell_size);
    state_.accumulated_transform = SE3Transform();
}

TrackingResult TrackingPipeline::processTrackingFrame(const PointXYZ* frame_points,
                                                       std::size_t n_points,
                                                       const TrackingConfig& config) {
    TrackingResult result;
    auto total_start = std::chrono::high_resolution_clock::now();

    // =========================================================================
    // STAGE 1: Voxel Downsampling
    // =========================================================================
    auto t0 = std::chrono::high_resolution_clock::now();

    std::vector<float> raw_x(n_points), raw_y(n_points), raw_z(n_points);
    for (std::size_t i = 0; i < n_points; ++i) {
        raw_x[i] = frame_points[i].x;
        raw_y[i] = frame_points[i].y;
        raw_z[i] = frame_points[i].z;
    }
    PointCloudSoA raw_soa = {raw_x.data(), raw_y.data(), raw_z.data(), n_points};

    std::vector<PointXYZ> downsampled(n_points);
    std::size_t n_down = voxel_grid_downsamp_rvv_v2(raw_soa, downsampled.data(), config.voxel_leaf_size);

    std::vector<float> ds_x(n_down), ds_y(n_down), ds_z(n_down);
    for (std::size_t i = 0; i < n_down; ++i) {
        ds_x[i] = downsampled[i].x;
        ds_y[i] = downsampled[i].y;
        ds_z[i] = downsampled[i].z;
    }
    PointCloudSoA frame_soa = {ds_x.data(), ds_y.data(), ds_z.data(), n_down};

    auto t1 = std::chrono::high_resolution_clock::now();
    result.timing.voxel_downsample_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // =========================================================================
    // STAGES 2–5: ICP Registration
    // =========================================================================
    PointCloudSoA ref_soa = state_.getRefCloudSoA();

    if (!state_.spatial_index_built) {
        state_.buildSpatialIndex(config.spatial_hash_cell_size);
    }

    icp_.align(frame_soa, ref_soa,
               state_.ref_nx.data(), state_.ref_ny.data(), state_.ref_nz.data(),
               state_.spatial_index,
               config,
               result.frame_transform,
               result.icp_iterations_used,
               result.icp_final_error,
               result.timing.correspondence_ms,
               result.timing.residual_jacobian_ms,
               result.timing.reduction_ms,
               result.timing.solve_ms);

    // =========================================================================
    // STAGE 6: Propagate Models
    // =========================================================================
    t0 = std::chrono::high_resolution_clock::now();

    std::vector<PlaneModel> propagated_planes = state_.planes;
    std::vector<ClusterModel> propagated_clusters = state_.clusters;
    propagateModels(result.frame_transform, propagated_planes, propagated_clusters);

    t1 = std::chrono::high_resolution_clock::now();
    result.timing.propagate_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // =========================================================================
    // STAGE 7: Verify Models
    // =========================================================================
    t0 = std::chrono::high_resolution_clock::now();

    std::vector<bool> point_confirmed(n_down, false);
    std::vector<int> confirmed_plane_idx;
    std::vector<int> confirmed_cluster_idx;

    verifyModels(frame_soa, propagated_planes, propagated_clusters,
                 point_confirmed, confirmed_plane_idx, confirmed_cluster_idx,
                 config);

    for (int idx : confirmed_plane_idx) {
        result.confirmed_planes.push_back(propagated_planes[idx]);
    }
    for (int idx : confirmed_cluster_idx) {
        result.confirmed_clusters.push_back(propagated_clusters[idx]);
    }

    t1 = std::chrono::high_resolution_clock::now();
    result.timing.verify_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // =========================================================================
    // STAGE 8: Split Confirmed vs Residual Points
    // =========================================================================
    t0 = std::chrono::high_resolution_clock::now();

    std::vector<PointXYZ> confirmed_points;
    splitConfirmedResidual(frame_soa, point_confirmed, confirmed_points, result.residual_points);

    result.n_confirmed = confirmed_points.size();
    result.n_residual = result.residual_points.size();

    t1 = std::chrono::high_resolution_clock::now();
    result.timing.split_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // =========================================================================
    // Residual Processing (wire into existing full pipeline)
    // =========================================================================
    if (result.n_residual > 30) {
        auto res_t0 = std::chrono::high_resolution_clock::now();

        std::vector<float> res_x(result.n_residual), res_y(result.n_residual), res_z(result.n_residual);
        for (std::size_t i = 0; i < result.n_residual; ++i) {
            res_x[i] = result.residual_points[i].x;
            res_y[i] = result.residual_points[i].y;
            res_z[i] = result.residual_points[i].z;
        }
        PointCloudSoA res_soa = {res_x.data(), res_y.data(), res_z.data(), result.n_residual};

        // RANSAC plane on residual
        float res_model[4];
        int res_inliers = ransac_plane_rvv(res_soa, config.ransac_dist_thresh,
                                            config.ransac_max_iters, res_model);
        if (res_inliers > 10) {
            result.new_planes.push_back(PlaneModel::fromArray(res_model, res_inliers));
        }

        // Euclidean clustering on residual
        EuclideanClustering ec;
        ec.setInputCloud(res_soa);
        ec.setClusterTolerance(config.cluster_tolerance);
        ec.setMinClusterSize(config.min_cluster_size);
        ec.setMaxClusterSize(config.max_cluster_size);
        auto res_clusters = ec.extract();

        for (const auto& cl : res_clusters) {
            float cx = 0, cy = 0, cz = 0;
            for (int idx : cl.indices) {
                cx += res_soa.x[idx];
                cy += res_soa.y[idx];
                cz += res_soa.z[idx];
            }
            float inv_n = 1.0f / (float)cl.indices.size();
            cx *= inv_n; cy *= inv_n; cz *= inv_n;

            float max_r2 = 0.0f;
            for (int idx : cl.indices) {
                float dx = res_soa.x[idx] - cx;
                float dy = res_soa.y[idx] - cy;
                float dz = res_soa.z[idx] - cz;
                float r2 = dx*dx + dy*dy + dz*dz;
                if (r2 > max_r2) max_r2 = r2;
            }

            result.new_clusters.push_back(ClusterModel(cx, cy, cz, std::sqrt(max_r2),
                                                         (int)cl.indices.size()));
        }

        auto res_t1 = std::chrono::high_resolution_clock::now();
        result.timing.residual_full_pipeline_ms =
            std::chrono::duration<double, std::milli>(res_t1 - res_t0).count();
    }

    // =========================================================================
    // STAGE 9: Update Spatial Index
    // =========================================================================
    t0 = std::chrono::high_resolution_clock::now();

    updateSpatialIndex(result.residual_points, config);

    t1 = std::chrono::high_resolution_clock::now();
    result.timing.update_index_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Update accumulated transform
    state_.accumulated_transform = result.frame_transform.compose(state_.accumulated_transform);

    auto total_end = std::chrono::high_resolution_clock::now();
    result.timing.total_tracking_ms = std::chrono::duration<double, std::milli>(total_end - total_start).count();

    return result;
}

// ============================================================================
// Stage 6 implementation: propagateModels
// ============================================================================

void TrackingPipeline::propagateModels(const SE3Transform& transform,
                                        std::vector<PlaneModel>& planes,
                                        std::vector<ClusterModel>& clusters) {
    for (auto& plane : planes) {
        plane = plane.transformed(transform);
    }

    for (auto& cluster : clusters) {
        cluster = cluster.transformed(transform);
    }

    if (state_.ref_n > 0) {
        std::vector<float> new_nx(state_.ref_n), new_ny(state_.ref_n), new_nz(state_.ref_n);
        SE3Transform R_only = transform;
        R_only.tx() = 0; R_only.ty() = 0; R_only.tz() = 0;
        propagate_transform_rvv(R_only,
                                 state_.ref_nx.data(), state_.ref_ny.data(), state_.ref_nz.data(),
                                 new_nx.data(), new_ny.data(), new_nz.data(),
                                 state_.ref_n);
        state_.ref_nx = std::move(new_nx);
        state_.ref_ny = std::move(new_ny);
        state_.ref_nz = std::move(new_nz);
    }
}

// ============================================================================
// Stage 7 implementation: verifyModels
// ============================================================================

void TrackingPipeline::verifyModels(const PointCloudSoA& new_frame,
                                     const std::vector<PlaneModel>& planes,
                                     const std::vector<ClusterModel>& clusters,
                                     std::vector<bool>& point_confirmed,
                                     std::vector<int>& confirmed_plane_indices,
                                     std::vector<int>& confirmed_cluster_indices,
                                     const TrackingConfig& config) {
    confirmed_plane_indices.clear();
    confirmed_cluster_indices.clear();

    for (std::size_t p = 0; p < planes.size(); ++p) {
        std::vector<bool> plane_mask(new_frame.n, false);
        std::size_t inlier_count = verify_plane_rvv(new_frame, planes[p],
                                                      config.plane_verify_dist, plane_mask);

        float ratio = (float)inlier_count / (float)new_frame.n;
        if (ratio >= config.plane_verify_ratio) {
            confirmed_plane_indices.push_back((int)p);
            for (std::size_t i = 0; i < new_frame.n; ++i) {
                if (plane_mask[i]) point_confirmed[i] = true;
            }
        }
    }

    for (std::size_t c = 0; c < clusters.size(); ++c) {
        std::vector<bool> cluster_mask(new_frame.n, false);
        std::size_t nearby_count = verify_cluster_rvv(new_frame, clusters[c],
                                                        config.cluster_verify_dist, cluster_mask);

        float ratio = (float)nearby_count / std::max(1.0f, (float)clusters[c].point_count);
        if (ratio >= config.cluster_verify_ratio) {
            confirmed_cluster_indices.push_back((int)c);
            for (std::size_t i = 0; i < new_frame.n; ++i) {
                if (cluster_mask[i]) point_confirmed[i] = true;
            }
        }
    }
}

// ============================================================================
// Stage 8 implementation: splitConfirmedResidual
// ============================================================================

void TrackingPipeline::splitConfirmedResidual(const PointCloudSoA& new_frame,
                                               const std::vector<bool>& point_confirmed,
                                               std::vector<PointXYZ>& confirmed_out,
                                               std::vector<PointXYZ>& residual_out) {
    compact_points_rvv(new_frame, point_confirmed, confirmed_out, residual_out);
}

// ============================================================================
// Stage 9 implementation: updateSpatialIndex
// ============================================================================

void TrackingPipeline::updateSpatialIndex(const std::vector<PointXYZ>& /*new_points*/,
                                           const TrackingConfig& config) {
    if (state_.ref_n > 0) {
        state_.buildSpatialIndex(config.spatial_hash_cell_size);
    }
}

// ============================================================================
// Static: Run full pipeline for comparison benchmarking
// ============================================================================

void TrackingPipeline::runFullPipeline(const PointXYZ* points,
                                        std::size_t n_points,
                                        const TrackingConfig& config,
                                        double& out_time_ms) {
    auto t0 = std::chrono::high_resolution_clock::now();

    std::vector<float> raw_x(n_points), raw_y(n_points), raw_z(n_points);
    for (std::size_t i = 0; i < n_points; ++i) {
        raw_x[i] = points[i].x;
        raw_y[i] = points[i].y;
        raw_z[i] = points[i].z;
    }
    PointCloudSoA raw_soa = {raw_x.data(), raw_y.data(), raw_z.data(), n_points};

    std::vector<PointXYZ> downsampled(n_points);
    std::size_t n_down = voxel_grid_downsamp_rvv_v2(raw_soa, downsampled.data(), config.voxel_leaf_size);

    std::vector<float> ds_x(n_down), ds_y(n_down), ds_z(n_down);
    for (std::size_t i = 0; i < n_down; ++i) {
        ds_x[i] = downsampled[i].x;
        ds_y[i] = downsampled[i].y;
        ds_z[i] = downsampled[i].z;
    }
    PointCloudSoA ds_soa = {ds_x.data(), ds_y.data(), ds_z.data(), n_down};

    std::vector<float> nx(n_down), ny(n_down), nz(n_down);
    normal_estimation_rvv(ds_soa, nx.data(), ny.data(), nz.data(),
                           config.normal_k, config.normal_radius);

    float model[4];
    ransac_plane_rvv(ds_soa, config.ransac_dist_thresh,
                      config.ransac_max_iters, model);

    EuclideanClustering ec;
    ec.setInputCloud(ds_soa);
    ec.setClusterTolerance(config.cluster_tolerance);
    ec.setMinClusterSize(config.min_cluster_size);
    ec.setMaxClusterSize(config.max_cluster_size);
    ec.extract();

    auto t1 = std::chrono::high_resolution_clock::now();
    out_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
}

} // namespace rvpoint
