#pragma once

#include "core/point_types.h"

#include <cstddef>
#include <limits>
#include <cstdint>
#include <vector>

namespace rvpoint {

class Octree;
class PointerOctree;

/// Thin wrapper that holds the point indices belonging to one cluster.
struct ClusterIndices {
    std::vector<int> indices; ///< Point indices into the source cloud.
};

/**
 * @brief Euclidean cluster extraction for PointCloudSoA.
 */
class EuclideanClustering {
public:
    EuclideanClustering() = default;

    /// Attach the cloud to segment.
    void setInputCloud(const PointCloudSoA &cloud);

    /// Maximum distance between two points for them to be neighbours.
    void setClusterTolerance(float tolerance);

    /// Clusters with fewer points than this are discarded.
    void setMinClusterSize(int min_size);

    /// Clusters with more points than this are discarded.
    void setMaxClusterSize(int max_size);

    /// Optional neighbor search structure for accelerated queries.
    void setNeighborSearch(const Octree *search);
    void setNeighborSearch(const PointerOctree *search);

    // Accessors
    float clusterTolerance()  const { return clusterTolerance_; }
    int   minClusterSize()    const { return minClusterSize_; }
    int   maxClusterSize()    const { return maxClusterSize_; }

    /**
     * @brief Run cluster extraction.
     * @return Vector of ClusterIndices.
     */
    std::vector<ClusterIndices> extract() const;

private:
    void radiusQueryUnvisited(
        float qx, float qy, float qz,
        float tol_sq,
        const std::vector<bool> &visited,
        std::vector<int>        &out
    ) const;

    const PointCloudSoA *cloud_            = nullptr;
    const Octree        *searcher_         = nullptr;
    const PointerOctree *ptr_searcher_     = nullptr;
    float                clusterTolerance_ = 0.05f;
    int                  minClusterSize_   = 1;
    int                  maxClusterSize_   = std::numeric_limits<int>::max();
};

} // namespace rvpoint
