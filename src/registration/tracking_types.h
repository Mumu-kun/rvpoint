#pragma once

#include "../include/rvv_pcl.h"
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace rvv_pcl {

// ============================================================================
// SE3 Transform — 4×4 rigid body transformation (rotation + translation)
// ============================================================================

/**
 * @brief 4×4 rigid body transformation (rotation + translation).
 *
 * Stored as a flat row-major 4×4 matrix for simplicity.
 * The upper-left 3×3 block is the rotation matrix R,
 * the rightmost column is the translation vector t.
 *
 *   [ R00 R01 R02 tx ]
 *   [ R10 R11 R12 ty ]
 *   [ R20 R21 R22 tz ]
 *   [  0   0   0   1 ]
 */
struct SE3Transform {
    float data[16];

    SE3Transform() {
        // Initialize to identity
        for (int i = 0; i < 16; ++i) data[i] = 0.0f;
        data[0] = data[5] = data[10] = data[15] = 1.0f;
    }

    // Accessors for rotation matrix elements (row-major)
    float& R(int row, int col) { return data[row * 4 + col]; }
    float  R(int row, int col) const { return data[row * 4 + col]; }

    // Accessors for translation vector
    float& tx() { return data[3]; }
    float& ty() { return data[7]; }
    float& tz() { return data[11]; }
    float  tx() const { return data[3]; }
    float  ty() const { return data[7]; }
    float  tz() const { return data[11]; }

    // Apply transform to a single point
    PointXYZ apply(const PointXYZ& p) const {
        PointXYZ out;
        out.x = R(0,0)*p.x + R(0,1)*p.y + R(0,2)*p.z + tx();
        out.y = R(1,0)*p.x + R(1,1)*p.y + R(1,2)*p.z + ty();
        out.z = R(2,0)*p.x + R(2,1)*p.y + R(2,2)*p.z + tz();
        return out;
    }

    // Compose: this * other
    SE3Transform compose(const SE3Transform& other) const {
        SE3Transform result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                float sum = 0.0f;
                for (int k = 0; k < 4; ++k) {
                    sum += data[i * 4 + k] * other.data[k * 4 + j];
                }
                result.data[i * 4 + j] = sum;
            }
        }
        return result;
    }

    // Create from axis-angle rotation + translation
    static SE3Transform fromAxisAngle(float ax, float ay, float az,
                                       float tx, float ty, float tz) {
        float angle = std::sqrt(ax*ax + ay*ay + az*az);
        SE3Transform T;
        if (angle < 1e-8f) {
            T.tx() = tx; T.ty() = ty; T.tz() = tz;
            return T;
        }
        float nx = ax / angle, ny = ay / angle, nz = az / angle;
        float c = std::cos(angle), s = std::sin(angle), t = 1.0f - c;
        T.R(0,0) = t*nx*nx + c;       T.R(0,1) = t*nx*ny - s*nz;  T.R(0,2) = t*nx*nz + s*ny;
        T.R(1,0) = t*nx*ny + s*nz;    T.R(1,1) = t*ny*ny + c;     T.R(1,2) = t*ny*nz - s*nx;
        T.R(2,0) = t*nx*nz - s*ny;    T.R(2,1) = t*ny*nz + s*nx;  T.R(2,2) = t*nz*nz + c;
        T.tx() = tx; T.ty() = ty; T.tz() = tz;
        return T;
    }
};

// ============================================================================
// Plane Model — ax + by + cz + d = 0
// ============================================================================

/**
 * @brief Detected plane model with metadata.
 */
struct PlaneModel {
    float a, b, c, d;          ///< Plane equation: ax + by + cz + d = 0
    int inlier_count;          ///< Number of inlier points
    float confidence;          ///< Detection confidence [0, 1]

    PlaneModel() : a(0), b(0), c(0), d(0), inlier_count(0), confidence(0) {}
    PlaneModel(float a_, float b_, float c_, float d_, int n = 0, float conf = 1.0f)
        : a(a_), b(b_), c(c_), d(d_), inlier_count(n), confidence(conf) {}

    // Get plane coefficients as array (for compatibility with existing RANSAC API)
    void toArray(float* model) const {
        model[0] = a; model[1] = b; model[2] = c; model[3] = d;
    }

    // Create from array (for compatibility with existing RANSAC API)
    static PlaneModel fromArray(const float* model, int n = 0) {
        return PlaneModel(model[0], model[1], model[2], model[3], n);
    }

    // Apply a transform to this plane
    PlaneModel transformed(const SE3Transform& T) const {
        // Transform normal: n' = R * n (normals transform by rotation only)
        float na = T.R(0,0)*a + T.R(0,1)*b + T.R(0,2)*c;
        float nb = T.R(1,0)*a + T.R(1,1)*b + T.R(1,2)*c;
        float nc = T.R(2,0)*a + T.R(2,1)*b + T.R(2,2)*c;
        // Transform d: d' = d - n' . t
        float nd = d - (na * T.tx() + nb * T.ty() + nc * T.tz());
        return PlaneModel(na, nb, nc, nd, inlier_count, confidence);
    }
};

// ============================================================================
// Cluster Model — centroid + radius + point count
// ============================================================================

/**
 * @brief Detected cluster model (centroid-based).
 */
struct ClusterModel {
    float cx, cy, cz;         ///< Cluster centroid
    float radius;             ///< Approximate bounding radius
    int point_count;          ///< Number of points in cluster

    ClusterModel() : cx(0), cy(0), cz(0), radius(0), point_count(0) {}
    ClusterModel(float x, float y, float z, float r, int n)
        : cx(x), cy(y), cz(z), radius(r), point_count(n) {}

    // Apply a transform to this cluster
    ClusterModel transformed(const SE3Transform& T) const {
        PointXYZ c_in = {cx, cy, cz};
        PointXYZ c_out = T.apply(c_in);
        return ClusterModel(c_out.x, c_out.y, c_out.z, radius, point_count);
    }
};

// ============================================================================
// Tracking Configuration
// ============================================================================

/**
 * @brief Configuration parameters for the tracking mode pipeline.
 */
struct TrackingConfig {
    // ICP parameters
    int icp_max_iterations = 15;        ///< Max ICP iterations per frame
    float icp_convergence_thresh = 1e-6f; ///< Convergence threshold (delta transform norm²)
    float correspondence_max_dist = 0.5f; ///< Max distance for correspondence matching (meters)

    // Voxel downsampling
    float voxel_leaf_size = 0.05f;      ///< Voxel grid leaf size for downsampling

    // Verification thresholds
    float plane_verify_dist = 0.05f;    ///< Max distance from plane to count as inlier
    float plane_verify_ratio = 0.5f;    ///< Min ratio of inliers to confirm plane
    float cluster_verify_dist = 0.2f;   ///< Max distance from centroid for cluster verification
    float cluster_verify_ratio = 0.3f;  ///< Min ratio of nearby points to confirm cluster

    // Residual processing
    float ransac_dist_thresh = 0.02f;   ///< RANSAC distance threshold for residual processing
    int ransac_max_iters = 500;         ///< RANSAC iterations for residual plane fitting
    float cluster_tolerance = 0.05f;    ///< Euclidean clustering tolerance for residuals
    int min_cluster_size = 10;          ///< Min cluster size for residual clustering
    int max_cluster_size = 25000;       ///< Max cluster size for residual clustering
    float normal_radius = 0.1f;         ///< Radius for normal estimation on residuals
    int normal_k = 20;                  ///< K neighbors for normal estimation

    // Spatial index
    float spatial_hash_cell_size = 0.1f; ///< SpatialHash cell size for neighbor search
};

// ============================================================================
// Tracking State — accumulated inter-frame state
// ============================================================================

/**
 * @brief Per-keyframe state that persists across tracking frames.
 *
 * Created from a keyframe's full-pipeline output and incrementally updated
 * by each tracking frame.
 */
struct TrackingState {
    // Reference cloud (from keyframe, in SoA layout for RVV)
    std::vector<float> ref_x, ref_y, ref_z;
    std::size_t ref_n = 0;

    // Reference normals (from keyframe normal estimation)
    std::vector<float> ref_nx, ref_ny, ref_nz;

    // Known geometric primitives
    std::vector<PlaneModel> planes;
    std::vector<ClusterModel> clusters;

    // Accumulated transform from keyframe to current frame
    SE3Transform accumulated_transform;

    // Spatial index for fast neighbor search
    // Rebuilt or incrementally updated each tracking frame
    SpatialHash spatial_index;
    bool spatial_index_built = false;

    // Helper: get SoA view of reference cloud
    PointCloudSoA getRefCloudSoA() {
        return {ref_x.data(), ref_y.data(), ref_z.data(), ref_n};
    }

    // Helper: set reference cloud from AoS points
    void setRefCloud(const PointXYZ* points, std::size_t n) {
        ref_n = n;
        ref_x.resize(n); ref_y.resize(n); ref_z.resize(n);
        for (std::size_t i = 0; i < n; ++i) {
            ref_x[i] = points[i].x;
            ref_y[i] = points[i].y;
            ref_z[i] = points[i].z;
        }
    }

    // Helper: set reference cloud from SoA
    void setRefCloudSoA(const PointCloudSoA& cloud) {
        ref_n = cloud.n;
        ref_x.assign(cloud.x, cloud.x + cloud.n);
        ref_y.assign(cloud.y, cloud.y + cloud.n);
        ref_z.assign(cloud.z, cloud.z + cloud.n);
    }

    // Build spatial index from current reference cloud
    void buildSpatialIndex(float cell_size) {
        PointCloudSoA soa = getRefCloudSoA();
        spatial_index.setInputCloud(soa, cell_size);
        spatial_index.build();
        spatial_index_built = true;
    }
};

// ============================================================================
// Tracking Result — output of one tracking-mode frame
// ============================================================================

/**
 * @brief Output from processing one tracking-mode frame.
 */
struct TrackingResult {
    // The computed frame-to-frame transform
    SE3Transform frame_transform;

    // Confirmed (propagated + verified) geometric primitives
    std::vector<PlaneModel> confirmed_planes;
    std::vector<ClusterModel> confirmed_clusters;

    // Newly detected primitives from residual processing
    std::vector<PlaneModel> new_planes;
    std::vector<ClusterModel> new_clusters;

    // Residual points (not matched by tracking)
    std::vector<PointXYZ> residual_points;
    std::size_t n_residual = 0;

    // Confirmed points (matched by tracking)
    std::size_t n_confirmed = 0;

    // ICP convergence info
    int icp_iterations_used = 0;
    float icp_final_error = 0.0f;

    // Per-stage timing (ms) — 9 stages
    struct StageTiming {
        double voxel_downsample_ms = 0;
        double correspondence_ms = 0;
        double residual_jacobian_ms = 0;
        double reduction_ms = 0;
        double solve_ms = 0;
        double propagate_ms = 0;
        double verify_ms = 0;
        double split_ms = 0;
        double update_index_ms = 0;
        double residual_full_pipeline_ms = 0; // Time for RANSAC+clustering on residuals
        double total_tracking_ms = 0;
    } timing;
};

// ============================================================================
// Correspondence Pair — used internally by ICP
// ============================================================================

/**
 * @brief A matched point pair between new frame and reference cloud.
 */
struct CorrespondencePair {
    int source_idx;    ///< Index in new (downsampled) frame
    int target_idx;    ///< Index in reference cloud
    float distance_sq; ///< Squared distance between matched points
};

} // namespace rvv_pcl
