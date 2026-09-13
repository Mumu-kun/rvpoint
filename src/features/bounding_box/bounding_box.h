#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include "core/point_types.h"

namespace rvpoint {

/**
 * @brief 2D Point representation for convex hull computation.
 */
struct Point2D {
    float x = 0.0f;
    float y = 0.0f;
    uint32_t orig_idx = 0;
};

/**
 * @brief Lightweight Bounding Disc for high-rate reactive evasion (ADR-0009).
 *
 * Provides isotropic safety margin around the obstacle centroid for O(1)
 * branchless collision clearance checks in the 50 Hz vehicle control loop.
 */
struct BoundingDisc {
    float cx = 0.0f;          ///< 2D Centroid X in body frame (meters)
    float cy = 0.0f;          ///< 2D Centroid Y in body frame (meters)
    float radius = 0.0f;      ///< Maximum Euclidean radius from centroid (meters)
    float z_min = 0.0f;       ///< Minimum vertical elevation (meters)
    float z_max = 0.0f;       ///< Maximum vertical elevation (meters)
    uint32_t point_count = 0; ///< Number of points belonging to the obstacle cluster
};

/**
 * @brief 3D Oriented Bounding Box (OBB) computed via Rotating Calipers (ADR-0009).
 *
 * Provides exact minimal-area oriented rectangular volume for visual telemetry,
 * Foxglove Studio display, and downstream tracking.
 */
struct OrientedBoundingBox {
    float cx = 0.0f;          ///< 3D Center X in body frame (meters)
    float cy = 0.0f;          ///< 3D Center Y in body frame (meters)
    float cz = 0.0f;          ///< 3D Center Z in body frame (meters)
    float extent_x = 0.0f;    ///< Extent / length along heading yaw axis (meters)
    float extent_y = 0.0f;    ///< Extent / width perpendicular to heading axis (meters)
    float extent_z = 0.0f;    ///< Vertical height span (meters)
    float yaw_rad = 0.0f;     ///< Heading angle in horizontal plane relative to +X axis (radians, [-pi, pi])
    float corners_x[4] = {0}; ///< 2D ground footprint corner vertices (CCW order: X)
    float corners_y[4] = {0}; ///< 2D ground footprint corner vertices (CCW order: Y)
    uint32_t point_count = 0; ///< Number of points belonging to the obstacle cluster
};

/**
 * @brief Paired dual representation of an extracted obstacle cluster.
 */
struct ObstacleGeometry {
    BoundingDisc disc;
    OrientedBoundingBox obb;
};

/**
 * @brief Zero-heap obstacle geometry extractor (ADR-0009, ADR-0010, ADR-0011).
 *
 * Implements:
 * 1. Vectorized Bounding Disc extraction ($O(K)$)
 * 2. Andrew's Monotone Chain 2D Convex Hull ($O(K \log K)$)
 * 3. Freeman-Shapira Rotating Calipers minimum-area OBB ($O(M)$)
 *
 * Reuses internal scratch workspaces across frames to guarantee zero allocations on steady-state hot paths.
 */
class ObstacleGeometryExtractor {
public:
    explicit ObstacleGeometryExtractor(std::size_t max_points = 4096);

    /**
     * @brief Pre-allocates scratch memory for hot-path zero-heap operation.
     */
    void reserve(std::size_t max_points);

    /**
     * @brief Extract lightweight BoundingDisc for a single cluster of points.
     * @param cloud Source point cloud.
     * @param indices Array of indices into the cloud belonging to the cluster.
     * @param count Number of indices.
     * @param out Extracted BoundingDisc.
     */
    void compute_disc(const PointCloud& cloud,
                      const uint32_t* indices,
                      std::size_t count,
                      BoundingDisc& out);

    /**
     * @brief Extract 2D Convex Hull from a single cluster of points (Andrew's Monotone Chain).
     * @param cloud Source point cloud.
     * @param indices Array of indices into the cloud.
     * @param count Number of indices.
     * @param out_hull Output convex polygon vertices in counter-clockwise order.
     */
    void compute_convex_hull_2d(const PointCloud& cloud,
                                const uint32_t* indices,
                                std::size_t count,
                                std::vector<Point2D>& out_hull);

    /**
     * @brief Extract 3D Oriented Bounding Box for a single cluster (Rotating Calipers).
     * @param cloud Source point cloud.
     * @param indices Array of indices into the cloud.
     * @param count Number of indices.
     * @param out Extracted OrientedBoundingBox.
     */
    void compute_obb(const PointCloud& cloud,
                     const uint32_t* indices,
                     std::size_t count,
                     OrientedBoundingBox& out);

    /**
     * @brief Extract both BoundingDisc and OrientedBoundingBox for a single cluster.
     */
    void compute_single(const PointCloud& cloud,
                        const uint32_t* indices,
                        std::size_t count,
                        ObstacleGeometry& out);

    /**
     * @brief Batch extraction: extracts BoundingDiscs for all clusters in ClusterResult.
     */
    void extract_discs(const PointCloud& cloud,
                       const ClusterResult& clusters,
                       std::vector<BoundingDisc>& out);

    /**
     * @brief Batch extraction: extracts full dual ObstacleGeometry for all clusters in ClusterResult.
     */
    void extract_all(const PointCloud& cloud,
                     const ClusterResult& clusters,
                     std::vector<ObstacleGeometry>& out);

    /**
     * @brief Functor operator: extracts dual ObstacleGeometry for all clusters.
     */
    void operator()(const PointCloud& cloud,
                    const ClusterResult& clusters,
                    std::vector<ObstacleGeometry>& out) {
        extract_all(cloud, clusters, out);
    }

private:
    std::vector<Point2D> pts_2d_scratch_;
    std::vector<Point2D> hull_scratch_;
    std::vector<float>   scratch_x_;
    std::vector<float>   scratch_y_;
    std::vector<float>   scratch_z_;
};

} // namespace rvpoint

