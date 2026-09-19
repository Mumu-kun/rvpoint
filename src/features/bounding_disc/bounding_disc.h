#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include "core/point_types.h"

namespace rvpoint {

struct OrientedBoundingBox;

/**
 * @brief Bounding Disc for reactive evasion and corridor safety (ADR-0009).
 */
struct BoundingDisc {
    float cx = 0.0f;          ///< 2D Center X in body frame (meters)
    float cy = 0.0f;          ///< 2D Center Y in body frame (meters)
    float radius = 0.0f;      ///< Enclosing safety radius (meters)
    float z_min = 0.0f;       ///< Minimum vertical elevation (meters)
    float z_max = 0.0f;       ///< Maximum vertical elevation (meters)
    uint32_t point_count = 0; ///< Cluster point count
};

/**
 * @brief Pure leaf kernel extracting 2D Bounding Discs.
 *
 * Conforms to AGENTS.md Leaf-Kernel Invariant, ADR-0010 (zero heap), and ADR-0011 (backend dispatch).
 * Supports two distinct mathematical paths:
 * 1. compute_from_points: Point-Centroid Disc (RVV reduction sum mean + max radius)
 * 2. compute_concentric: OBB-Concentric Disc (branchless O(1) circumscribed radius)
 */
class BoundingDiscExtractor {
public:
    explicit BoundingDiscExtractor(std::size_t max_points = 2048,
                                  Backend backend = Backend::Auto);

    void reserve(std::size_t max_points);

    void set_backend(Backend b) noexcept { backend_ = b; }
    Backend backend() const noexcept { return backend_; }

    /**
     * @brief Path 1: Extract Point-Centroid Disc directly from point cloud.
     * Uses RVV vector sum reduction for centroid and vector max reduction for radius.
     */
    void compute_from_points(const PointCloud& cloud,
                             const uint32_t* indices,
                             std::size_t count,
                             BoundingDisc& out_disc);

    /**
     * @brief Path 2: Extract Concentric Disc circumscribed from an OrientedBoundingBox.
     * Centers exactly on OBB (cx, cy) with R = 0.5 * sqrt(extent_x^2 + extent_y^2).
     */
    void compute_concentric(const OrientedBoundingBox& box,
                            BoundingDisc& out_disc) const;

private:
    Backend backend_;
    std::vector<float> scratch_x_;
    std::vector<float> scratch_y_;

    void compute_points_rvv(std::size_t count, float z_min, float z_max, BoundingDisc& out_disc);
    void compute_points_scalar(std::size_t count, float z_min, float z_max, BoundingDisc& out_disc);
};

} // namespace rvpoint
