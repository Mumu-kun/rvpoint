#pragma once

#include "core/point_types.h"
#include "search/fast_3d_spatial_grid.h"

#include <cstddef>
#include <limits>
#include <cstdint>
#include <vector>

namespace rvpoint {

class Octree;
class PointerOctree;

/// Thin wrapper that holds the point indices belonging to one cluster (legacy).
struct ClusterIndices {
    std::vector<int> indices; ///< Point indices into the source cloud.
};

/**
 * @brief Euclidean Cluster Extraction for point clouds.
 *
 * Partitions a point cloud into disconnected clusters based on Euclidean distance.
 * Implements a stateful, zero-vtable Functor pattern owning visited bitmaps,
 * a flat BFS queue, and neighbor scratch buffers (ADR-0010, ADR-0011).
 *
 * Writes results directly into flat CSR ClusterResult to eliminate nested vector allocations.
 */
enum class ClusteringMethod {
    BFS,
    SymmetricUnionFind
};

class EuclideanClustering {
public:
    explicit EuclideanClustering(float tolerance = 0.05f, int min_size = 1,
                                 int max_size = std::numeric_limits<int>::max(),
                                 Backend backend = Backend::Auto);

    // Primary modern accessors
    void set_cluster_tolerance(float tolerance) noexcept { clusterTolerance_ = tolerance; }
    float cluster_tolerance() const noexcept { return clusterTolerance_; }

    void set_min_cluster_size(int min_size) noexcept { minClusterSize_ = min_size; }
    int min_cluster_size() const noexcept { return minClusterSize_; }

    void set_max_cluster_size(int max_size) noexcept { maxClusterSize_ = max_size; }
    int max_cluster_size() const noexcept { return maxClusterSize_; }

    void set_use_spatial_grid(bool enable) noexcept { use_spatial_grid_ = enable; }
    bool use_spatial_grid() const noexcept { return use_spatial_grid_; }

    void set_method(ClusteringMethod method) noexcept { method_ = method; }
    ClusteringMethod method() const noexcept { return method_; }

    void threads(int t) noexcept { num_threads_ = t; }
    int threads() const noexcept { return num_threads_; }

    void set_backend(Backend b) noexcept { backend_ = b; }
    Backend backend() const noexcept { return backend_; }

    // Legacy PCL-style setters
    void setInputCloud(const PointCloudSoA &cloud) noexcept { cloud_ = &cloud; }
    void setClusterTolerance(float tolerance) noexcept { set_cluster_tolerance(tolerance); }
    void setMinClusterSize(int min_size) noexcept { set_min_cluster_size(min_size); }
    void setMaxClusterSize(int max_size) noexcept { set_max_cluster_size(max_size); }
    float clusterTolerance() const noexcept { return clusterTolerance_; }
    int minClusterSize() const noexcept { return minClusterSize_; }
    int maxClusterSize() const noexcept { return maxClusterSize_; }

    // Optional legacy neighbor search structures
    void setNeighborSearch(const Octree *search);
    void setNeighborSearch(const PointerOctree *search);

    /**
     * @brief Pre-allocate clustering workspaces for zero-heap execution.
     */
    void reserve(std::size_t max_points);

    /**
     * @brief Extract clusters into flat CSR ClusterResult.
     */
    void operator()(const PointCloudView& in, ClusterResult& out, float tolerance, int min_size, int max_size);
    void operator()(const PointCloudView& in, ClusterResult& out) {
        (*this)(in, out, clusterTolerance_, minClusterSize_, maxClusterSize_);
    }
    void operator()(const PointCloud& in, ClusterResult& out, float tolerance, int min_size, int max_size) {
        (*this)(in.view(), out, tolerance, min_size, max_size);
    }
    void operator()(const PointCloud& in, ClusterResult& out) {
        (*this)(in.view(), out, clusterTolerance_, minClusterSize_, maxClusterSize_);
    }

    void apply(const PointCloudView& in, ClusterResult& out, float tolerance, int min_size, int max_size) {
        (*this)(in, out, tolerance, min_size, max_size);
    }
    void apply(const PointCloudView& in, ClusterResult& out) {
        (*this)(in, out, clusterTolerance_, minClusterSize_, maxClusterSize_);
    }
    void apply(const PointCloud& in, ClusterResult& out, float tolerance, int min_size, int max_size) {
        (*this)(in.view(), out, tolerance, min_size, max_size);
    }
    void apply(const PointCloud& in, ClusterResult& out) {
        (*this)(in, out, clusterTolerance_, minClusterSize_, maxClusterSize_);
    }

    /**
     * @brief Legacy extraction returning vector of ClusterIndices.
     */
    std::vector<ClusterIndices> extract() const;

private:
    void radiusQueryUnvisited(
        const PointCloudView& in,
        float qx, float qy, float qz,
        float tol_sq,
        std::vector<int> &out
    );

    const PointCloudSoA *cloud_            = nullptr;
    const Octree        *searcher_         = nullptr;
    const PointerOctree *ptr_searcher_     = nullptr;
    float                clusterTolerance_ = 0.05f;
    int                  minClusterSize_   = 1;
    int                  maxClusterSize_   = std::numeric_limits<int>::max();
    bool                 use_spatial_grid_ = true;
    ClusteringMethod     method_           = ClusteringMethod::SymmetricUnionFind;
    int                  num_threads_      = 0;
    Backend              backend_          = Backend::Auto;

    void extract_bfs(const PointCloudView& in, ClusterResult& out, float tolerance, int min_size, int max_size);
    void extract_symmetric_union_find(const PointCloudView& in, ClusterResult& out, float tolerance, int min_size, int max_size);

    // Instance-owned scratch workspaces (ADR-0010 zero-heap hot-path)
    std::vector<uint8_t> visited_;
    std::vector<int>     bfs_queue_;
    std::vector<int>     neighbors_;
    std::vector<int>     search_raw_nb_scratch_;
    std::vector<float>   search_dists_scratch_;
    Fast3DSpatialGrid    grid_;

    // Symmetric Union-Find scratch workspaces
    std::vector<int>     uf_parent_;
    std::vector<int>     uf_rank_;
    std::vector<int>     root_counts_;
    std::vector<int>     root_to_cid_;

    struct UfThreadScratch {
        std::vector<int> self_pts;
        std::vector<int> cand_idx;
        std::vector<float> cand_x;
        std::vector<float> cand_y;
        std::vector<float> cand_z;
        std::vector<std::pair<int, int>> edges;
    };
    std::vector<UfThreadScratch> thread_scratch_;
};

} // namespace rvpoint
