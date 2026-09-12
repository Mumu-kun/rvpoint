#pragma once

#include "core/point_types.h"
#include "search/fast_3d_spatial_grid.h"
#include "segmentation/spatial_slab_engine.h" // For SlabUnionFind
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>

namespace rvpoint {

/**
 * @brief Forward 13-cell relative offsets for cell-centric spatial clustering.
 *
 * Imposes canonical forward direction (half of 26 3D neighbors) to eliminate
 * redundant pairwise distance calculations by symmetry.
 */
struct ForwardOffset3D {
    int dx, dy, dz;
};

inline constexpr ForwardOffset3D kForward13Offsets[13] = {
    // Layer dz = 0 (4 forward neighbors)
    { 1,  0, 0},
    {-1,  1, 0},
    { 0,  1, 0},
    { 1,  1, 0},
    // Layer dz = 1 (9 forward neighbors)
    {-1, -1, 1},
    { 0, -1, 1},
    { 1, -1, 1},
    {-1,  0, 1},
    { 0,  0, 1},
    { 1,  0, 1},
    {-1,  1, 1},
    { 0,  1, 1},
    { 1,  1, 1}
};

/**
 * @brief 13-Forward-Offset Cell-Centric Euclidean Clustering Functor.
 *
 * Implements symmetric pairwise grid neighbor traversal (ADR-0010, ADR-0011).
 * Traverses occupied spatial hash cells and merges connected components using
 * zero-heap Union-Find across the 13 canonical forward neighborhood vectors.
 */
class ForwardCellClustering {
public:
    explicit ForwardCellClustering(float cluster_tolerance = 0.15f,
                                   int min_cluster_size = 10,
                                   int max_cluster_size = 25000)
        : cluster_tolerance_(cluster_tolerance),
          min_cluster_size_(min_cluster_size),
          max_cluster_size_(max_cluster_size) {}

    void set_cluster_tolerance(float tol) noexcept { cluster_tolerance_ = tol; }
    void set_min_cluster_size(int min_sz) noexcept { min_cluster_size_ = min_sz; }
    void set_max_cluster_size(int max_sz) noexcept { max_cluster_size_ = max_sz; }

    float cluster_tolerance() const noexcept { return cluster_tolerance_; }
    int min_cluster_size() const noexcept { return min_cluster_size_; }
    int max_cluster_size() const noexcept { return max_cluster_size_; }

    void reserve(std::size_t max_points) {
        grid_ = Fast3DSpatialGrid(cluster_tolerance_, max_points);
        uf_parent_.reserve(max_points);
        uf_rank_.reserve(max_points);
        root_counts_.reserve(max_points);
        root_to_cid_.reserve(max_points);
        fill_offsets_.reserve(max_points);
        occupied_cells_.reserve(max_points);
    }

    void operator()(const PointCloudView& in, ClusterResult& out) {
        compute(in, out, cluster_tolerance_, min_cluster_size_, max_cluster_size_);
    }

    void operator()(const PointCloud& in, ClusterResult& out) {
        compute(in.view(), out, cluster_tolerance_, min_cluster_size_, max_cluster_size_);
    }

    void compute(const PointCloudView& in, ClusterResult& out,
                 float cluster_tolerance, int min_cluster_size, int max_cluster_size)
    {
        out.clear();
        const std::size_t n = in.n;
        if (n == 0) return;

        // Build spatial hash grid with cell size equal to cluster tolerance
        if (grid_.capacity_ < n || grid_.cell_size_ != cluster_tolerance) {
            grid_ = Fast3DSpatialGrid(cluster_tolerance, std::max<std::size_t>(n, 1024));
        }
        grid_.build(in);

        // Pre-allocate Union-Find structures
        if (uf_parent_.size() < n) uf_parent_.resize(n);
        if (uf_rank_.size() < n) uf_rank_.resize(n);

        SlabUnionFind uf(uf_parent_.data(), uf_rank_.data(), n);

        const float tol2 = cluster_tolerance * cluster_tolerance;
        const float* px = in.x;
        const float* py = in.y;
        const float* pz = in.z;

        const auto& cells = grid_.cells_;
        const auto& next  = grid_.next_;
        const std::size_t mask = grid_.mask_;

        // Collect occupied hash bucket indices
        occupied_cells_.clear();
        for (std::size_t h = 0; h < cells.size(); ++h) {
            if (cells[h].head != -1) {
                occupied_cells_.push_back(static_cast<uint32_t>(h));
            }
        }

        // Iterate over occupied grid cells
        for (uint32_t h_idx : occupied_cells_) {
            const auto& cell = cells[h_idx];
            int cx = cell.cx;
            int cy = cell.cy;
            int cz = cell.cz;

            // 1. Pairwise checks within the same home cell
            int p1 = cell.head;
            while (p1 != -1) {
                float p1x = px[p1], p1y = py[p1], p1z = pz[p1];
                int p2 = next[p1];
                while (p2 != -1) {
                    float dx = px[p2] - p1x;
                    float dy = py[p2] - p1y;
                    float dz = pz[p2] - p1z;
                    if (dx*dx + dy*dy + dz*dz <= tol2) {
                        uf.unite(p1, p2);
                    }
                    p2 = next[p2];
                }
                p1 = next[p1];
            }

            // 2. Pairwise checks against the 13 canonical forward neighbor cells
            for (const auto& off : kForward13Offsets) {
                int ncx = cx + off.dx;
                int ncy = cy + off.dy;
                int ncz = cz + off.dz;

                std::size_t nh = grid_.hash3D(ncx, ncy, ncz);
                int probes = 0;
                while (cells[nh].head != -1 && probes < Fast3DSpatialGrid::kMaxProbes) {
                    if (cells[nh].cx == ncx && cells[nh].cy == ncy && cells[nh].cz == ncz) {
                        // Found neighbor cell: check each p1 in home cell against p2 in neighbor cell
                        int q1 = cell.head;
                        while (q1 != -1) {
                            float q1x = px[q1], q1y = py[q1], q1z = pz[q1];
                            int q2 = cells[nh].head;
                            while (q2 != -1) {
                                float dx = px[q2] - q1x;
                                float dy = py[q2] - q1y;
                                float dz = pz[q2] - q1z;
                                if (dx*dx + dy*dy + dz*dz <= tol2) {
                                    uf.unite(q1, q2);
                                }
                                q2 = next[q2];
                            }
                            q1 = next[q1];
                        }
                        break;
                    }
                    nh = (nh + 1) & mask;
                    probes++;
                }
            }
        }

        // Extract CSR clusters
        if (root_counts_.size() < n) root_counts_.resize(n);
        if (root_to_cid_.size() < n) root_to_cid_.resize(n);

        std::fill(root_counts_.begin(), root_counts_.begin() + n, 0);
        for (std::size_t i = 0; i < n; ++i) {
            root_counts_[uf.find(static_cast<int>(i))]++;
        }

        std::fill(root_to_cid_.begin(), root_to_cid_.begin() + n, -1);
        int num_valid_clusters = 0;
        for (std::size_t r = 0; r < n; ++r) {
            int sz = root_counts_[r];
            if (sz >= min_cluster_size && sz <= max_cluster_size) {
                root_to_cid_[r] = num_valid_clusters++;
            }
        }

        out.offsets.assign(num_valid_clusters + 1, 0);
        for (std::size_t r = 0; r < n; ++r) {
            int cid = root_to_cid_[r];
            if (cid != -1) {
                out.offsets[cid + 1] = root_counts_[r];
            }
        }
        for (int c = 0; c < num_valid_clusters; ++c) {
            out.offsets[c + 1] += out.offsets[c];
        }

        out.indices.resize(out.offsets.back());
        if (fill_offsets_.size() < static_cast<std::size_t>(num_valid_clusters + 1)) {
            fill_offsets_.resize(num_valid_clusters + 1);
        }
        std::copy(out.offsets.begin(), out.offsets.end(), fill_offsets_.begin());

        for (std::size_t i = 0; i < n; ++i) {
            int r = uf.find(static_cast<int>(i));
            int cid = root_to_cid_[r];
            if (cid != -1) {
                out.indices[fill_offsets_[cid]++] = static_cast<uint32_t>(i);
            }
        }

        // Sort point indices in each cluster for determinism
        for (int cid = 0; cid < num_valid_clusters; ++cid) {
            uint32_t start = out.offsets[cid];
            uint32_t end = out.offsets[cid + 1];
            std::sort(out.indices.begin() + start, out.indices.begin() + end);
        }
    }

private:
    float cluster_tolerance_ = 0.15f;
    int min_cluster_size_ = 10;
    int max_cluster_size_ = 25000;

    Fast3DSpatialGrid grid_;
    std::vector<int> uf_parent_, uf_rank_;
    std::vector<int> root_counts_, root_to_cid_;
    std::vector<uint32_t> fill_offsets_;
    std::vector<uint32_t> occupied_cells_;
};

} // namespace rvpoint
