#pragma once

#include "core/point_types.h"
#include "search/spatial_hashing.h"
#include <vector>
#include <cmath>
#include <cstddef>
#include <algorithm>

namespace rvpoint {

// ============================================================================
// SE(3) Rigid Body Transformation — 4×4 Matrix representation
// ============================================================================

/**
 * @brief Represents a 3D rigid body transformation in SE(3): p' = R * p + t
 *
 * Stored as a row-major 4×4 matrix:
 *   [ R00  R01  R02  tx ]
 *   [ R10  R11  R12  ty ]
 *   [ R20  R21  R22  tz ]
 *   [  0    0    0    1 ]
 */
struct SE3Transform {
    float data[16]; // Row-major 4×4

    // Default: Identity transformation
    SE3Transform() {
        for (int i = 0; i < 16; ++i) data[i] = 0.0f;
        data[0] = 1.0f; data[5] = 1.0f; data[10] = 1.0f; data[15] = 1.0f;
    }

    // Element access (row, col)
    float& operator()(int r, int c) { return data[r * 4 + c]; }
    const float& operator()(int r, int c) const { return data[r * 4 + c]; }

    // Rotation matrix elements
    float R(int r, int c) const { return data[r * 4 + c]; }

    // Translation components
    float tx() const { return data[3]; }
    float ty() const { return data[7]; }
    float tz() const { return data[11]; }
    float& tx() { return data[3]; }
    float& ty() { return data[7]; }
    float& tz() { return data[11]; }

    // Apply transformation to a single point: p' = R * p + t
    PointXYZ apply(const PointXYZ& p) const {
        return {
            data[0]*p.x + data[1]*p.y + data[2]*p.z + data[3],
            data[4]*p.x + data[5]*p.y + data[6]*p.z + data[7],
            data[8]*p.x + data[9]*p.y + data[10]*p.z + data[11]
        };
    }

    // Compose two transforms: T_composed = this * other
    SE3Transform compose(const SE3Transform& other) const {
        SE3Transform result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                float sum = 0.0f;
                for (int k = 0; k < 4; ++k) {
                    sum += (*this)(i, k) * other(k, j);
                }
                result(i, j) = sum;
            }
        }
        return result;
    }

    // Construct from axis-angle (exponential map approximation for small angles)
    // plus translation
    static SE3Transform fromAxisAngle(float ax, float ay, float az,
                                      float tx, float ty, float tz) {
        SE3Transform T;
        float theta = std::sqrt(ax*ax + ay*ay + az*az);

        if (theta < 1e-7f) {
            // First-order Taylor (small-angle approximation)
            // R ≈ I + [w]_x
            T(0,0) = 1.0f;  T(0,1) = -az;   T(0,2) =  ay;
            T(1,0) =  az;   T(1,1) = 1.0f;  T(1,2) = -ax;
            T(2,0) = -ay;   T(2,1) =  ax;   T(2,2) = 1.0f;
        } else {
            // Rodrigues formula: R = I + sin(θ)/θ [w]_x + (1-cos(θ))/θ² [w]_x²
            float kx = ax / theta, ky = ay / theta, kz = az / theta;
            float c = std::cos(theta), s = std::sin(theta), v = 1.0f - c;

            T(0,0) = kx*kx*v + c;      T(0,1) = kx*ky*v - kz*s;  T(0,2) = kx*kz*v + ky*s;
            T(1,0) = kx*ky*v + kz*s;  T(1,1) = ky*ky*v + c;      T(1,2) = ky*kz*v - kx*s;
            T(2,0) = kx*kz*v - ky*s;  T(2,1) = ky*kz*v + kx*s;  T(2,2) = kz*kz*v + c;
        }

        T.tx() = tx;
        T.ty() = ty;
        T.tz() = tz;
        return T;
    }

    // Invert rigid transform: T^{-1} = [ R^T  -R^T*t ]
    SE3Transform inverse() const {
        SE3Transform inv;
        // R^T
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                inv(i, j) = (*this)(j, i);
            }
        }
        // -R^T * t
        inv.tx() = -(inv(0,0)*tx() + inv(0,1)*ty() + inv(0,2)*tz());
        inv.ty() = -(inv(1,0)*tx() + inv(1,1)*ty() + inv(1,2)*tz());
        inv.tz() = -(inv(2,0)*tx() + inv(2,1)*ty() + inv(2,2)*tz());
        return inv;
    }
};

// ============================================================================
// Geometric Primitive Models
// ============================================================================

/**
 * @brief Detected plane model: a*x + b*y + c*z + d = 0 (normalized: a²+b²+c²=1).
 */
struct PlaneModel {
    float a, b, c, d;          ///< Plane equation coefficients
    int inlier_count;          ///< Number of inliers that support this plane
    float confidence;          ///< Detection confidence [0, 1]

    PlaneModel() : a(0), b(0), c(0), d(0), inlier_count(0), confidence(0) {}
    PlaneModel(float a_, float b_, float c_, float d_, int n = 0, float conf = 1.0f)
        : a(a_), b(b_), c(c_), d(d_), inlier_count(n), confidence(conf) {}

    void toArray(float* model) const {
        model[0] = a; model[1] = b; model[2] = c; model[3] = d;
    }

    static PlaneModel fromArray(const float* model, int n = 0) {
        return PlaneModel(model[0], model[1], model[2], model[3], n);
    }

    // Apply a transform to this plane: normal transforms by R, d transforms by d - n' . t
    PlaneModel transformed(const SE3Transform& T) const {
        float na = T.R(0,0)*a + T.R(0,1)*b + T.R(0,2)*c;
        float nb = T.R(1,0)*a + T.R(1,1)*b + T.R(1,2)*c;
        float nc = T.R(2,0)*a + T.R(2,1)*b + T.R(2,2)*c;
        float nd = d - (na * T.tx() + nb * T.ty() + nc * T.tz());
        return PlaneModel(na, nb, nc, nd, inlier_count, confidence);
    }
};

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
    int icp_max_iterations = 15;          ///< Max ICP iterations per frame
    float icp_convergence_thresh = 1e-6f; ///< Convergence threshold (delta transform norm²)
    float correspondence_max_dist = 0.5f; ///< Max distance for correspondence matching (meters)

    // Voxel downsampling
    float voxel_leaf_size = 0.05f;        ///< Voxel grid leaf size for downsampling

    // Verification thresholds
    float plane_verify_dist = 0.05f;      ///< Max distance from plane to count as inlier
    float plane_verify_ratio = 0.5f;      ///< Min ratio of inliers to confirm plane
    float cluster_verify_dist = 0.2f;     ///< Max distance from centroid for cluster verification
    float cluster_verify_ratio = 0.3f;    ///< Min ratio of nearby points to confirm cluster

    // Residual processing
    float ransac_dist_thresh = 0.02f;     ///< RANSAC distance threshold for residual processing
    int ransac_max_iters = 500;           ///< RANSAC iterations for residual plane fitting
    float cluster_tolerance = 0.05f;      ///< Euclidean clustering tolerance for residuals
    int min_cluster_size = 10;            ///< Min cluster size for residual clustering
    int max_cluster_size = 25000;         ///< Max cluster size for residual clustering
    float normal_radius = 0.1f;           ///< Radius for normal estimation on residuals
    int normal_k = 20;                    ///< K neighbors for normal estimation

    // Spatial index
    float spatial_hash_cell_size = 0.1f;  ///< SpatialHash cell size for neighbor search
};

// ============================================================================
// Tracking State — accumulated inter-frame state
// ============================================================================

/**
 * @brief Per-keyframe state that persists across tracking frames.
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

} // namespace rvpoint
