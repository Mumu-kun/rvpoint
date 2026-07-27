// euclidean_clustering.h
//
// Euclidean Cluster Extraction
// ----------------------------
// Region-growing segmentation over a PointCloudSoA using a radius-based
// neighbor search.  Each seed is expanded by a BFS/flood-fill: a point is
// added to the current cluster if its Euclidean distance to any already-
// assigned member is ≤ clusterTolerance_.
//
// The inner neighbor-query loop is accelerated with RVV (RISC-V Vector
// Extension) when compiled with RVV_PCL_USE_RVV and __riscv_vector:
//
//   RVV path  – processes VLEN/32 cloud points per vsetvl cycle; computes
//               squared distances with vfsub/vfmul/vfmacc and collects
//               candidate indices via a bit-mask scatter, identical in style
//               to CaravanRadiusSearch.
//
//   Scalar fallback – plain O(N) sweep per BFS expansion step.
//
// Usage:
//   EuclideanClustering ec;
//   ec.setInputCloud(cloud);
//   ec.setClusterTolerance(0.05f);
//   ec.setMinClusterSize(10);
//   ec.setMaxClusterSize(25000);
//   std::vector<std::vector<int>> clusters = ec.extract();

#pragma once

#include "rvv_pcl.h"

#include <cstddef>
#include <limits>
#include <cstdint>
#include <vector>

namespace rvv_pcl {

// ─── ClusterIndices ───────────────────────────────────────────────────────────
/// Thin wrapper that holds the point indices belonging to one cluster.
struct ClusterIndices {
    std::vector<int> indices; ///< Point indices into the source cloud.
};

// ─── EuclideanClustering ──────────────────────────────────────────────────────
/**
 * @brief Euclidean cluster extraction for PointCloudSoA.
 *
 * Algorithm: BFS flood-fill.  For each unvisited seed point the search
 * expands to all points within clusterTolerance_.  Clusters outside
 * [minClusterSize_, maxClusterSize_] are discarded.
 *
 * The inner radius-check kernel has an RVV-accelerated and a scalar path
 * selected at compile time via the RVV_PCL_USE_RVV preprocessor guard.
 */
class EuclideanClustering {
public:
    EuclideanClustering() = default;

    // ── Setters ──────────────────────────────────────────────────────────────

    /// Attach the cloud to segment (reference must outlive this object).
    void setInputCloud(const PointCloudSoA &cloud);

    /// Maximum distance (metres) between two points for them to be neighbours.
    void setClusterTolerance(float tolerance);

    /// Clusters with fewer points than this are discarded.
    void setMinClusterSize(int min_size);

    /// Clusters with more points than this are discarded.
    void setMaxClusterSize(int max_size);

    /// Optional neighbor search structure for accelerated queries.
    void setNeighborSearch(NeighborSearch *search);

    // ── Accessors ─────────────────────────────────────────────────────────────
    float clusterTolerance()  const { return clusterTolerance_; }
    int   minClusterSize()    const { return minClusterSize_; }
    int   maxClusterSize()    const { return maxClusterSize_; }

    // ── Core API ──────────────────────────────────────────────────────────────

    /**
     * @brief Run cluster extraction.
     * @return Vector of ClusterIndices; each entry holds the point indices of
     *         one cluster, sorted in ascending order.
     */
    std::vector<ClusterIndices> extract() const;

private:
    // ── Internal helpers ──────────────────────────────────────────────────────

    /// Collect all indices in [cloud] within tolerance² of (qx,qy,qz) that
    /// are not yet visited. Appended into @p out.
    void radiusQueryUnvisited(
        float qx, float qy, float qz,
        float tol_sq,
        const std::vector<bool> &visited,
        std::vector<int>        &out
    ) const;

    // ── State ─────────────────────────────────────────────────────────────────
    const PointCloudSoA *cloud_            = nullptr;
    NeighborSearch      *searcher_         = nullptr;
    float                clusterTolerance_ = 0.05f;
    int                  minClusterSize_   = 1;
    int                  maxClusterSize_   = std::numeric_limits<int>::max();
};

} // namespace rvv_pcl
