#pragma once

#include "core/point_types.h"
#include "core/radix_sort.h"
#include "search/fast_3d_spatial_grid.h"
#include "filters/radius_outlier_removal.h"
#include "segmentation/euclidean_clustering.h"
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <numeric>

#if defined(_OPENMP)
#include <omp.h>
#endif

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvpoint {

enum class SlabAxis { X = 0, Y = 1, Z = 2, Range = 3 };

inline uint32_t float_to_sortable_uint32(float f) noexcept {
    uint32_t u;
    std::memcpy(&u, &f, sizeof(float));
    uint32_t mask = static_cast<uint32_t>(static_cast<int32_t>(u) >> 31);
    return u ^ (mask | 0x80000000u);
}

/**
 * @brief Zero-allocation Union-Find supporting RVV vector-vector parent initialization.
 */
struct SlabUnionFind {
    int* parent = nullptr;
    int* rank = nullptr;
    size_t n = 0;

    SlabUnionFind(int* p, int* r, size_t count) : parent(p), rank(r), n(count) {
#if defined(__riscv_vector)
        size_t i = 0;
        while (i < n) {
            size_t vl = __riscv_vsetvl_e32m8(n - i);
            vuint32m8_t vid = __riscv_vid_v_u32m8(vl);
            vid = __riscv_vadd_vx_u32m8(vid, static_cast<uint32_t>(i), vl);
            __riscv_vse32_v_u32m8(reinterpret_cast<uint32_t*>(parent + i), vid, vl);
            i += vl;
        }
#else
        std::iota(parent, parent + n, 0);
#endif
        std::memset(rank, 0, n * sizeof(int));
    }

    int find(int i) noexcept {
        int root = i;
        while (root != parent[root]) root = parent[root];
        int curr = i;
        while (curr != root) {
            int nxt = parent[curr];
            parent[curr] = root;
            curr = nxt;
        }
        return root;
    }

    void unite(int i, int j) noexcept {
        int root_i = find(i);
        int root_j = find(j);
        if (root_i != root_j) {
            if (rank[root_i] < rank[root_j]) {
                parent[root_i] = root_j;
            } else if (rank[root_i] > rank[root_j]) {
                parent[root_j] = root_i;
            } else {
                parent[root_j] = root_i;
                rank[root_i]++;
            }
        }
    }
};

/**
 * @brief Geometric Domain Decomposition Engine for multi-core obstacle filtering & clustering.
 *
 * Implements ADR-0013: Equal-population quantile slicing, parallel local spatial grids,
 * independent intra-slab ROR/clustering, and fast boundary component stitching.
 * Adheres to ADR-0010 (Zero-Heap Steady-State Execution Invariant) and ADR-0011.
 */
class SpatialSlabEngine {
public:
    explicit SpatialSlabEngine(float ror_radius = 0.25f, int ror_min_pts = 5,
                               float cluster_tolerance = 0.15f, int min_cluster_size = 10,
                               int max_cluster_size = 25000, int num_slabs = 8,
                               SlabAxis axis = SlabAxis::X)
        : ror_radius_(ror_radius), ror_min_pts_(ror_min_pts),
          cluster_tolerance_(cluster_tolerance), min_cluster_size_(min_cluster_size),
          max_cluster_size_(max_cluster_size), num_slabs_(num_slabs), axis_(axis) {}

    // Fluent configuration interface
    SpatialSlabEngine& ror_radius(float r) noexcept { ror_radius_ = r; return *this; }
    SpatialSlabEngine& ror_min_pts(int k) noexcept { ror_min_pts_ = k; return *this; }
    SpatialSlabEngine& cluster_tolerance(float tol) noexcept { cluster_tolerance_ = tol; return *this; }
    SpatialSlabEngine& min_cluster_size(int min_sz) noexcept { min_cluster_size_ = min_sz; return *this; }
    SpatialSlabEngine& max_cluster_size(int max_sz) noexcept { max_cluster_size_ = max_sz; return *this; }
    SpatialSlabEngine& num_slabs(int slabs) noexcept { num_slabs_ = slabs; return *this; }
    SpatialSlabEngine& axis(SlabAxis ax) noexcept { axis_ = ax; return *this; }

    void set_ror_radius(float r) noexcept { ror_radius_ = r; }
    void set_ror_min_pts(int k) noexcept { ror_min_pts_ = k; }
    void set_cluster_tolerance(float tol) noexcept { cluster_tolerance_ = tol; }
    void set_min_cluster_size(int min_sz) noexcept { min_cluster_size_ = min_sz; }
    void set_max_cluster_size(int max_sz) noexcept { max_cluster_size_ = max_sz; }
    void set_num_slabs(int slabs) noexcept { num_slabs_ = slabs; }
    void set_axis(SlabAxis ax) noexcept { axis_ = ax; }

    float get_ror_radius() const noexcept { return ror_radius_; }
    int get_ror_min_pts() const noexcept { return ror_min_pts_; }
    float get_cluster_tolerance() const noexcept { return cluster_tolerance_; }
    int get_min_cluster_size() const noexcept { return min_cluster_size_; }
    int get_max_cluster_size() const noexcept { return max_cluster_size_; }
    int get_num_slabs() const noexcept { return num_slabs_; }
    SlabAxis get_axis() const noexcept { return axis_; }

    static inline float extract_coord(float x, float y, float z, SlabAxis ax) noexcept {
        switch (ax) {
            case SlabAxis::Y: return y;
            case SlabAxis::Z: return z;
            case SlabAxis::Range: return std::sqrt(x * x + y * y + z * z);
            case SlabAxis::X:
            default: return x;
        }
    }

    void reserve(size_t max_points) {
        keys_.reserve(max_points);
        sorted_idx_.reserve(max_points);
        sorted_x_.reserve(max_points);
        sorted_y_.reserve(max_points);
        sorted_z_.reserve(max_points);
        sorted_coords_.reserve(max_points);
        f_coords_.reserve(max_points);
        uf_parent_.reserve(max_points);
        uf_rank_.reserve(max_points);
        root_counts_.reserve(max_points);
        root_to_cid_.reserve(max_points);

        size_t max_threads = 16;
        local_grids_.resize(max_threads);
        clust_grids_.resize(max_threads);
        slab_kept_.resize(max_threads);
        thread_neighbors_.resize(max_threads);
        thread_dists2_.resize(max_threads);
        for (size_t t = 0; t < max_threads; ++t) {
            local_grids_[t] = Fast3DSpatialGrid(ror_radius_, max_points / 2);
            clust_grids_[t] = Fast3DSpatialGrid(cluster_tolerance_, max_points / 2);
            slab_kept_[t].reserve(max_points / 4);
            thread_neighbors_[t].reserve(256);
            thread_dists2_[t].reserve(256);
        }

        slab_cut_.resize(max_threads + 1);
        slab_start_.resize(max_threads);
        slab_end_.resize(max_threads);
        halo_start_.resize(max_threads);
        halo_end_.resize(max_threads);
        slab_offset_.resize(max_threads + 1);
        fill_offsets_.reserve(max_points);

        fallback_ror_ = std::make_unique<RadiusOutlierRemoval>(ror_radius_, ror_min_pts_);
        fallback_ror_->reserve(max_points);
        fallback_ec_ = std::make_unique<EuclideanClustering>(cluster_tolerance_, min_cluster_size_, max_cluster_size_);
        fallback_ec_->reserve(max_points);
    }

    void operator()(const PointCloudView& in, PointCloud& filtered_out, ClusterResult& clusters_out) {
        compute(in, filtered_out, clusters_out, ror_radius_, ror_min_pts_,
                cluster_tolerance_, min_cluster_size_, max_cluster_size_, num_slabs_, axis_);
    }

    void operator()(const PointCloud& in, PointCloud& filtered_out, ClusterResult& clusters_out) {
        compute(in.view(), filtered_out, clusters_out, ror_radius_, ror_min_pts_,
                cluster_tolerance_, min_cluster_size_, max_cluster_size_, num_slabs_, axis_);
    }

    void compute(const PointCloudView& in, PointCloud& filtered_out, ClusterResult& clusters_out,
                 float ror_radius, int ror_min_pts, float cluster_tolerance,
                 int min_cluster_size, int max_cluster_size, int num_slabs, SlabAxis axis)
    {
        clusters_out.clear();
        filtered_out.clear();
        size_t n = in.n;
        if (n == 0) return;

        int eff_slabs = num_slabs;
#if defined(_OPENMP)
        if (eff_slabs <= 0) eff_slabs = omp_get_max_threads();
        if (omp_in_parallel()) eff_slabs = 1; // anti-oversubscription
#else
        eff_slabs = 1;
#endif
        if (eff_slabs > 16) eff_slabs = 16;
        if (static_cast<size_t>(eff_slabs) > n) eff_slabs = static_cast<int>(n);

        // Fallback to sequential ROR and clustering if single slab
        if (eff_slabs <= 1) {
            if (!fallback_ror_) {
                fallback_ror_ = std::make_unique<RadiusOutlierRemoval>(ror_radius, ror_min_pts);
                fallback_ror_->reserve(n);
            }
            if (!fallback_ec_) {
                fallback_ec_ = std::make_unique<EuclideanClustering>(cluster_tolerance, min_cluster_size, max_cluster_size);
                fallback_ec_->reserve(n);
            }
            (*fallback_ror_)(in, filtered_out);
            (*fallback_ec_)(filtered_out.view(), clusters_out);
            return;
        }

        // 1. Compute sortable keys along primary axis
        if (keys_.size() < n) keys_.resize(n);
        if (sorted_idx_.size() < n) sorted_idx_.resize(n);

        const float* px = in.x;
        const float* py = in.y;
        const float* pz = in.z;

        for (size_t i = 0; i < n; ++i) {
            float coord = extract_coord(px[i], py[i], pz[i], axis);
            keys_[i] = float_to_sortable_uint32(coord);
        }

        // 2. Global Parallel Radix Sort
        for (size_t i = 0; i < n; ++i) {
            sorted_idx_[i] = static_cast<uint32_t>(i);
        }
        radix_sort_pairs_u32_parallel(keys_.data(), sorted_idx_.data(), n, eff_slabs);

        if (sorted_x_.size() < n) sorted_x_.resize(n);
        if (sorted_y_.size() < n) sorted_y_.resize(n);
        if (sorted_z_.size() < n) sorted_z_.resize(n);
        if (sorted_coords_.size() < n) sorted_coords_.resize(n);

        for (size_t i = 0; i < n; ++i) {
            uint32_t orig_i = sorted_idx_[i];
            sorted_x_[i] = px[orig_i];
            sorted_y_[i] = py[orig_i];
            sorted_z_[i] = pz[orig_i];
            sorted_coords_[i] = extract_coord(px[orig_i], py[orig_i], pz[orig_i], axis);
        }

        // 3. Compute Quantile Equal-Population Slab Boundaries & Halo Overlap
        if (slab_cut_.size() <= static_cast<size_t>(eff_slabs)) slab_cut_.resize(eff_slabs + 1);
        slab_cut_[0] = -1e9f;
        slab_cut_[eff_slabs] = 1e9f;
        for (int s = 1; s < eff_slabs; ++s) {
            size_t split_i = (n * s) / eff_slabs;
            slab_cut_[s] = 0.5f * (sorted_coords_[split_i - 1] + sorted_coords_[split_i]);
        }

        float halo = std::max(ror_radius, cluster_tolerance);
        if (slab_start_.size() < static_cast<size_t>(eff_slabs)) {
            slab_start_.resize(eff_slabs);
            slab_end_.resize(eff_slabs);
            halo_start_.resize(eff_slabs);
            halo_end_.resize(eff_slabs);
        }

        for (int s = 0; s < eff_slabs; ++s) {
            slab_start_[s] = (n * s) / eff_slabs;
            slab_end_[s]   = (n * (s + 1)) / eff_slabs;

            float low_cut  = slab_cut_[s];
            float high_cut = slab_cut_[s + 1];

            halo_start_[s] = std::lower_bound(sorted_coords_.begin(), sorted_coords_.begin() + n, low_cut - halo) - sorted_coords_.begin();
            halo_end_[s]   = std::upper_bound(sorted_coords_.begin(), sorted_coords_.begin() + n, high_cut + halo) - sorted_coords_.begin();
            if (halo_end_[s] < halo_start_[s]) halo_end_[s] = halo_start_[s];
        }

        // 4. Build local grids in parallel
        if (local_grids_.size() < static_cast<size_t>(eff_slabs)) {
            local_grids_.resize(eff_slabs);
        }
        for (int s = 0; s < eff_slabs; ++s) {
            size_t ng = halo_end_[s] - halo_start_[s];
            if (local_grids_[s].capacity_ < ng || local_grids_[s].cell_size_ != ror_radius) {
                local_grids_[s] = Fast3DSpatialGrid(ror_radius, std::max<size_t>(ng, 1024));
            }
        }

#if defined(_OPENMP)
        #pragma omp parallel for num_threads(eff_slabs) schedule(static, 1)
#endif
        for (int s = 0; s < eff_slabs; ++s) {
            size_t h_st = halo_start_[s];
            size_t ng   = halo_end_[s] - h_st;
            if (ng > 0) {
                local_grids_[s].build(&sorted_x_[h_st], &sorted_y_[h_st], &sorted_z_[h_st], ng);
            }
        }

        // 5. Parallel Intra-Slab ROR Filtering
        if (slab_kept_.size() < static_cast<size_t>(eff_slabs)) {
            slab_kept_.resize(eff_slabs);
        }

        float r2 = ror_radius * ror_radius;
#if defined(_OPENMP)
        #pragma omp parallel for num_threads(eff_slabs) schedule(static, 1)
#endif
        for (int s = 0; s < eff_slabs; ++s) {
            size_t s_st = slab_start_[s];
            size_t s_en = slab_end_[s];
            auto& kept = slab_kept_[s];
            kept.clear();
            if (kept.capacity() < (s_en - s_st)) kept.reserve(s_en - s_st);

            const auto& grid = local_grids_[s];
            for (size_t i = s_st; i < s_en; ++i) {
                int count = grid.countNeighbors(sorted_x_[i], sorted_y_[i], sorted_z_[i], r2, ror_min_pts);
                if (count >= ror_min_pts) {
                    kept.push_back(static_cast<uint32_t>(i));
                }
            }
        }

        // Assemble filtered points
        if (slab_offset_.size() <= static_cast<size_t>(eff_slabs)) {
            slab_offset_.resize(eff_slabs + 1);
        }
        slab_offset_[0] = 0;
        for (int s = 0; s < eff_slabs; ++s) {
            slab_offset_[s + 1] = slab_offset_[s] + slab_kept_[s].size();
        }
        size_t n_filtered = slab_offset_[eff_slabs];
        filtered_out.clear();
        filtered_out.resize(n_filtered);
        if (f_coords_.size() < n_filtered) f_coords_.resize(n_filtered);

#if defined(_OPENMP)
        #pragma omp parallel for num_threads(eff_slabs) schedule(static, 1)
#endif
        for (int s = 0; s < eff_slabs; ++s) {
            size_t base = slab_offset_[s];
            const auto& kept = slab_kept_[s];
            for (size_t k = 0; k < kept.size(); ++k) {
                uint32_t id = kept[k];
                filtered_out.x[base + k] = sorted_x_[id];
                filtered_out.y[base + k] = sorted_y_[id];
                filtered_out.z[base + k] = sorted_z_[id];
                f_coords_[base + k]      = sorted_coords_[id];
            }
        }

        if (n_filtered == 0) return;

        // 6. Spatial Slab Clustering + Boundary Stitching
        if (uf_parent_.size() < n_filtered) uf_parent_.resize(n_filtered);
        if (uf_rank_.size() < n_filtered) uf_rank_.resize(n_filtered);

        SlabUnionFind global_uf(uf_parent_.data(), uf_rank_.data(), n_filtered);

        // 6A: Intra-slab clustering in parallel
        if (clust_grids_.size() < static_cast<size_t>(eff_slabs)) {
            clust_grids_.resize(eff_slabs);
        }
        if (thread_neighbors_.size() < static_cast<size_t>(eff_slabs)) {
            thread_neighbors_.resize(eff_slabs);
            thread_dists2_.resize(eff_slabs);
        }

        for (int s = 0; s < eff_slabs; ++s) {
            size_t base = slab_offset_[s];
            size_t n_s  = slab_offset_[s + 1] - base;
            if (clust_grids_[s].capacity_ < n_s || clust_grids_[s].cell_size_ != cluster_tolerance) {
                clust_grids_[s] = Fast3DSpatialGrid(cluster_tolerance, std::max<size_t>(n_s, 1024));
            }
            if (thread_neighbors_[s].capacity() < 256) {
                thread_neighbors_[s].reserve(256);
                thread_dists2_[s].reserve(256);
            }
        }

#if defined(_OPENMP)
        #pragma omp parallel for num_threads(eff_slabs) schedule(static, 1)
#endif
        for (int s = 0; s < eff_slabs; ++s) {
            size_t base = slab_offset_[s];
            size_t n_s  = slab_offset_[s + 1] - base;
            if (n_s <= 1) continue;

            const float* sx = filtered_out.x.data() + base;
            const float* sy = filtered_out.y.data() + base;
            const float* sz = filtered_out.z.data() + base;

            clust_grids_[s].build(sx, sy, sz, n_s);

            auto& neighbors = thread_neighbors_[s];
            auto& dists2 = thread_dists2_[s];

            float tol2 = cluster_tolerance * cluster_tolerance;
            for (size_t u = 0; u < n_s; ++u) {
                clust_grids_[s].radiusSearch(sx[u], sy[u], sz[u], tol2, neighbors, dists2);
                for (int v_idx : neighbors) {
                    if (v_idx > static_cast<int>(u)) {
                        global_uf.unite(static_cast<int>(base + u), static_cast<int>(base + v_idx));
                    }
                }
            }
        }

        // 6B: Boundary Stitching across (eff_slabs - 1) boundary planes
        for (int s = 1; s < eff_slabs; ++s) {
            float split_coord = slab_cut_[s];
            size_t left_base  = slab_offset_[s - 1];
            size_t left_end   = slab_offset_[s];
            size_t right_base = slab_offset_[s];
            size_t right_end  = slab_offset_[s + 1];

            if (left_end == left_base || right_end == right_base) continue;

            size_t left_bnd_start = std::lower_bound(f_coords_.begin() + left_base,
                                                     f_coords_.begin() + left_end,
                                                     split_coord - cluster_tolerance) - f_coords_.begin();
            size_t right_bnd_end  = std::upper_bound(f_coords_.begin() + right_base,
                                                     f_coords_.begin() + right_end,
                                                     split_coord + cluster_tolerance) - f_coords_.begin();

            float tol2 = cluster_tolerance * cluster_tolerance;
            for (size_t li = left_bnd_start; li < left_end; ++li) {
                float lc = f_coords_[li];
                float lx = filtered_out.x[li], ly = filtered_out.y[li], lz = filtered_out.z[li];
                for (size_t rj = right_base; rj < right_bnd_end; ++rj) {
                    float rc = f_coords_[rj];
                    if (rc - lc > cluster_tolerance) break;
                    float ddx = filtered_out.x[rj] - lx;
                    float ddy = filtered_out.y[rj] - ly;
                    float ddz = filtered_out.z[rj] - lz;
                    if (ddx*ddx + ddy*ddy + ddz*ddz <= tol2) {
                        global_uf.unite(static_cast<int>(li), static_cast<int>(rj));
                    }
                }
            }
        }

        // 6C: Extract CSR clusters
        if (root_counts_.size() < n_filtered) root_counts_.resize(n_filtered);
        if (root_to_cid_.size() < n_filtered) root_to_cid_.resize(n_filtered);

        std::fill(root_counts_.begin(), root_counts_.begin() + n_filtered, 0);
        for (size_t i = 0; i < n_filtered; ++i) {
            root_counts_[global_uf.find(static_cast<int>(i))]++;
        }

        std::fill(root_to_cid_.begin(), root_to_cid_.begin() + n_filtered, -1);
        int num_valid_clusters = 0;
        for (size_t r = 0; r < n_filtered; ++r) {
            int sz = root_counts_[r];
            if (sz >= min_cluster_size && sz <= max_cluster_size) {
                root_to_cid_[r] = num_valid_clusters++;
            }
        }

        clusters_out.offsets.assign(num_valid_clusters + 1, 0);
        for (size_t r = 0; r < n_filtered; ++r) {
            int cid = root_to_cid_[r];
            if (cid != -1) {
                clusters_out.offsets[cid + 1] = root_counts_[r];
            }
        }
        for (int c = 0; c < num_valid_clusters; ++c) {
            clusters_out.offsets[c + 1] += clusters_out.offsets[c];
        }

        clusters_out.indices.resize(clusters_out.offsets.back());
        if (fill_offsets_.size() < static_cast<size_t>(num_valid_clusters + 1)) {
            fill_offsets_.resize(num_valid_clusters + 1);
        }
        std::copy(clusters_out.offsets.begin(), clusters_out.offsets.end(), fill_offsets_.begin());

        for (size_t i = 0; i < n_filtered; ++i) {
            int r = global_uf.find(static_cast<int>(i));
            int cid = root_to_cid_[r];
            if (cid != -1) {
                clusters_out.indices[fill_offsets_[cid]++] = static_cast<uint32_t>(i);
            }
        }

        // Sort point indices in each cluster for determinism
        for (int cid = 0; cid < num_valid_clusters; ++cid) {
            uint32_t start = clusters_out.offsets[cid];
            uint32_t end = clusters_out.offsets[cid + 1];
            std::sort(clusters_out.indices.begin() + start, clusters_out.indices.begin() + end);
        }
    }

private:
    float ror_radius_ = 0.25f;
    int ror_min_pts_ = 5;
    float cluster_tolerance_ = 0.15f;
    int min_cluster_size_ = 10;
    int max_cluster_size_ = 25000;
    int num_slabs_ = 8;
    SlabAxis axis_ = SlabAxis::X;

    std::vector<uint32_t> keys_;
    std::vector<uint32_t> sorted_idx_;
    std::vector<float> sorted_x_, sorted_y_, sorted_z_, sorted_coords_;
    std::vector<float> f_coords_;
    std::vector<int> uf_parent_, uf_rank_;
    std::vector<int> root_counts_, root_to_cid_;

    std::vector<float> slab_cut_;
    std::vector<size_t> slab_start_, slab_end_;
    std::vector<size_t> halo_start_, halo_end_;
    std::vector<size_t> slab_offset_;
    std::vector<uint32_t> fill_offsets_;

    std::vector<std::vector<uint32_t>> slab_kept_;
    std::vector<Fast3DSpatialGrid> local_grids_;
    std::vector<Fast3DSpatialGrid> clust_grids_;
    std::vector<std::vector<int>> thread_neighbors_;
    std::vector<std::vector<float>> thread_dists2_;

    std::unique_ptr<RadiusOutlierRemoval> fallback_ror_;
    std::unique_ptr<EuclideanClustering> fallback_ec_;
};

} // namespace rvpoint

