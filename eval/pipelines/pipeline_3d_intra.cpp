// pipeline_3d_intra.cpp
// ============================================================================
// Intra-Frame Single-Core Perception Pipeline
// ============================================================================
// Algorithmic optimizations over pipeline_3d_rvv_clust.cpp:
//   1. Radix sort replaces std::sort in voxel grid (O(N) vs O(N log N))
//   2. Pipeline inversion: RANSAC runs on downsampled cloud BEFORE grid build
//      - Grid is built on ~36k obstacle points instead of ~52k (30% less work)
//      - ROR runs on obstacle-only points (30% less work)
//   3. Clustering runs on ROR-filtered obstacles
//
// This pipeline targets <30ms per-frame on the SpacemiT K1 (single-core first,
// then 8-core spatial slab decomposition in Phase 2).
// ============================================================================

#include "include/rvpoint.h"
#include "search/fast_3d_spatial_grid.h"
#include "io/simple_pcd_loader.h"
#include "filters/voxel_grid.h"
#include "filters/radix_sort.h"
#include "segmentation/euclidean_clustering.h"
#include "segmentation/ransac_plane.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

#if defined(_OPENMP)
#include <omp.h>
#endif

using Clock = std::chrono::high_resolution_clock;
using namespace rvpoint;

namespace {

constexpr int kStageCount = 7;

struct StageTiming {
    int index;
    const char *label;
    double ms;
    std::size_t point_count;
};

struct PipelineConfig {
    float voxel_leaf_size = 0.10f;
    float ror_search_radius = 0.25f;
    int ror_min_pts = 2;
    float ransac_distance_threshold = 0.22f;
    int ransac_max_iterations = 250;
    float cluster_tolerance = 0.15f;
    int min_cluster_size = 50;
    int max_cluster_size = 100000;
};

constexpr PipelineConfig kPipelineConfig;

struct FastPRNG {
    uint64_t state;
    explicit FastPRNG(uint64_t seed = 0x853c49e6748fea9bULL) : state(seed != 0 ? seed : 0x853c49e6748fea9bULL) {}
    inline uint32_t next() {
        uint64_t x = state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        state = x;
        return static_cast<uint32_t>((x * 0x2545F4914F6CDD1DULL) >> 32);
    }
    inline uint32_t next_bounded(uint32_t bound) {
        return bound > 0 ? (next() % bound) : 0;
    }
};

// ============================================================================
// RANSAC Plane Fitting (Vectorized SPRT)
// ============================================================================
static bool compute_plane_coeffs(float x1, float y1, float z1,
                                 float x2, float y2, float z2,
                                 float x3, float y3, float z3,
                                 float* model) 
{
    float v1x = x2 - x1, v1y = y2 - y1, v1z = z2 - z1;
    float v2x = x3 - x1, v2y = y3 - y1, v2z = z3 - z1;
    float a = v1y*v2z - v1z*v2y;
    float b = v1z*v2x - v1x*v2z;
    float c = v1x*v2y - v1y*v2x;
    float norm = std::sqrt(a*a + b*b + c*c);
    if (norm < 1e-4f) return false;
    a /= norm; b /= norm; c /= norm;
    model[0] = a; model[1] = b; model[2] = c;
    model[3] = -(a*x1 + b*y1 + c*z1);
    return true;
}

static int ransac_plane_intra(
    const PointCloudSoA& cloud, float dist_thresh, int max_iters, float* model,
    const float* ground_normal_prior = nullptr, float min_ground_dot = 0.707f, uint64_t seed = 42)
{
    if (cloud.n < 3) return 0;
    FastPRNG rng(seed);
    float best_model[4] = {0,0,0,0};
    int best_sample_inliers = 0;
    int k_iters = max_iters;
    const double log_p = std::log(1.0 - 0.99);

    const size_t sample_sz = std::min(cloud.n, static_cast<size_t>(2048));
    std::vector<float> sx(sample_sz), sy(sample_sz), sz(sample_sz);
    FastPRNG s_rng(seed ^ 0x9e3779b97f4a7c15ULL);
    for (size_t k = 0; k < sample_sz; ++k) {
        size_t idx = s_rng.next_bounded(static_cast<uint32_t>(cloud.n));
        sx[k] = cloud.x[idx]; sy[k] = cloud.y[idx]; sz[k] = cloud.z[idx];
    }

    for (int iter = 0; iter < k_iters && iter < max_iters; ++iter) {
        int i1 = static_cast<int>(rng.next_bounded(static_cast<uint32_t>(cloud.n)));
        int i2 = static_cast<int>(rng.next_bounded(static_cast<uint32_t>(cloud.n)));
        int i3 = static_cast<int>(rng.next_bounded(static_cast<uint32_t>(cloud.n)));
        if (i1 == i2 || i1 == i3 || i2 == i3) continue;

        float cand_model[4];
        if (!compute_plane_coeffs(cloud.x[i1], cloud.y[i1], cloud.z[i1],
                                 cloud.x[i2], cloud.y[i2], cloud.z[i2],
                                 cloud.x[i3], cloud.y[i3], cloud.z[i3], cand_model)) continue;

        if (ground_normal_prior != nullptr) {
            float dot = std::abs(cand_model[0] * ground_normal_prior[0] +
                                 cand_model[1] * ground_normal_prior[1] +
                                 cand_model[2] * ground_normal_prior[2]);
            if (dot < min_ground_dot) continue;
        }

        float a = cand_model[0], b = cand_model[1], c = cand_model[2], d = cand_model[3];
        int sample_inliers = 0;
#if defined(__riscv_vector)
        size_t si = 0;
        while (si < sample_sz) {
            size_t vl = __riscv_vsetvl_e32m8(sample_sz - si);
            vfloat32m8_t vx = __riscv_vle32_v_f32m8(&sx[si], vl);
            vfloat32m8_t vy = __riscv_vle32_v_f32m8(&sy[si], vl);
            vfloat32m8_t vz = __riscv_vle32_v_f32m8(&sz[si], vl);
            vfloat32m8_t dist = __riscv_vfmul_vf_f32m8(vx, a, vl);
            dist = __riscv_vfmacc_vf_f32m8(dist, b, vy, vl);
            dist = __riscv_vfmacc_vf_f32m8(dist, c, vz, vl);
            dist = __riscv_vfadd_vf_f32m8(dist, d, vl);
            vbool4_t mask_le = __riscv_vmfle_vf_f32m8_b4(dist, dist_thresh, vl);
            vbool4_t mask_ge = __riscv_vmfge_vf_f32m8_b4(dist, -dist_thresh, vl);
            vbool4_t mask_in = __riscv_vmand_mm_b4(mask_le, mask_ge, vl);
            sample_inliers += static_cast<int>(__riscv_vcpop_m_b4(mask_in, vl));
            si += vl;
        }
#else
        for (size_t pt = 0; pt < sample_sz; ++pt) {
            float dist = std::abs(a * sx[pt] + b * sy[pt] + c * sz[pt] + d);
            if (dist <= dist_thresh) sample_inliers++;
        }
#endif

        if (sample_inliers > best_sample_inliers) {
            best_sample_inliers = sample_inliers;
            for (int k = 0; k < 4; k++) best_model[k] = cand_model[k];
            double w = static_cast<double>(best_sample_inliers) / static_cast<double>(sample_sz);
            double p_no_outliers = std::clamp(1.0 - std::pow(w, 3.0), 1e-7, 1.0 - 1e-7);
            double log_no_outliers = std::log(p_no_outliers);
            if (std::abs(log_no_outliers) > 1e-7) {
                int dynamic_k = static_cast<int>(std::ceil(log_p / log_no_outliers));
                if (dynamic_k > 0 && dynamic_k < k_iters) k_iters = dynamic_k;
            }
        }
    }

    // Refine plane coefficients using inlier covariance
    if (best_sample_inliers >= 10) {
        float a0 = best_model[0], b0 = best_model[1], c0 = best_model[2], d0 = best_model[3];
        double sum_x = 0, sum_y = 0, sum_z = 0;
        int inlier_cnt = 0;
        for (size_t k = 0; k < sample_sz; ++k) {
            float dist = std::abs(a0 * sx[k] + b0 * sy[k] + c0 * sz[k] + d0);
            if (dist <= dist_thresh) {
                sum_x += sx[k]; sum_y += sy[k]; sum_z += sz[k];
                inlier_cnt++;
            }
        }
        if (inlier_cnt >= 10) {
            double cx = sum_x / inlier_cnt, cy = sum_y / inlier_cnt, cz = sum_z / inlier_cnt;
            double c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
            for (size_t k = 0; k < sample_sz; ++k) {
                float dist = std::abs(a0 * sx[k] + b0 * sy[k] + c0 * sz[k] + d0);
                if (dist <= dist_thresh) {
                    double dx = sx[k] - cx, dy = sy[k] - cy, dz = sz[k] - cz;
                    c00 += dx * dx; c01 += dx * dy; c02 += dx * dz;
                    c11 += dy * dy; c12 += dy * dz; c22 += dz * dz;
                }
            }
            double rnx = c01 * c12 - c02 * c11;
            double rny = c01 * c02 - c00 * c12;
            double rnz = c00 * c11 - c01 * c01;
            double rlen = std::sqrt(rnx * rnx + rny * rny + rnz * rnz);
            if (rlen > 1e-6) {
                rnx /= rlen; rny /= rlen; rnz /= rlen;
                if (rnz < 0) { rnx = -rnx; rny = -rny; rnz = -rnz; }
                best_model[0] = static_cast<float>(rnx);
                best_model[1] = static_cast<float>(rny);
                best_model[2] = static_cast<float>(rnz);
                best_model[3] = static_cast<float>(-(rnx * cx + rny * cy + rnz * cz));
            }
        }
    }

    for (int k = 0; k < 4; k++) model[k] = best_model[k];

    // Count total inliers on full cloud
    float a = model[0], b = model[1], c = model[2], d = model[3];
    int total_inliers = 0;
#if defined(__riscv_vector)
    size_t n = cloud.n, i = 0;
    while (i < n) {
        size_t vl = __riscv_vsetvl_e32m8(n - i);
        vfloat32m8_t vx = __riscv_vle32_v_f32m8(&cloud.x[i], vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(&cloud.y[i], vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(&cloud.z[i], vl);
        vfloat32m8_t dist = __riscv_vfmul_vf_f32m8(vx, a, vl);
        dist = __riscv_vfmacc_vf_f32m8(dist, b, vy, vl);
        dist = __riscv_vfmacc_vf_f32m8(dist, c, vz, vl);
        dist = __riscv_vfadd_vf_f32m8(dist, d, vl);
        vbool4_t mask_le = __riscv_vmfle_vf_f32m8_b4(dist, dist_thresh, vl);
        vbool4_t mask_ge = __riscv_vmfge_vf_f32m8_b4(dist, -dist_thresh, vl);
        vbool4_t mask_in = __riscv_vmand_mm_b4(mask_le, mask_ge, vl);
        total_inliers += static_cast<int>(__riscv_vcpop_m_b4(mask_in, vl));
        i += vl;
    }
#else
    for (size_t pt = 0; pt < cloud.n; ++pt) {
        float dist = std::abs(a * cloud.x[pt] + b * cloud.y[pt] + c * cloud.z[pt] + d);
        if (dist <= dist_thresh) total_inliers++;
    }
#endif
    return total_inliers;
}

// ============================================================================
// Vectorized Inlier/Outlier Extraction
// ============================================================================
static size_t extract_outliers_soa(
    const PointCloudSoA& in, const float* model, float thresh,
    std::vector<float>& ox, std::vector<float>& oy, std::vector<float>& oz)
{
    float a = model[0], b = model[1], c = model[2], d = model[3];
    const size_t n = in.n;
    ox.clear(); oy.clear(); oz.clear();

    size_t in_count = 0;
#if defined(__riscv_vector)
    ox.resize(n); oy.resize(n); oz.resize(n);
    size_t out_count = 0;
    size_t i = 0;
    while (i < n) {
        size_t vl = __riscv_vsetvl_e32m8(n - i);
        vfloat32m8_t vx = __riscv_vle32_v_f32m8(&in.x[i], vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(&in.y[i], vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(&in.z[i], vl);

        vfloat32m8_t dist = __riscv_vfmul_vf_f32m8(vx, a, vl);
        dist = __riscv_vfmacc_vf_f32m8(dist, b, vy, vl);
        dist = __riscv_vfmacc_vf_f32m8(dist, c, vz, vl);
        dist = __riscv_vfadd_vf_f32m8(dist, d, vl);

        vbool4_t mask_le = __riscv_vmfle_vf_f32m8_b4(dist, thresh, vl);
        vbool4_t mask_ge = __riscv_vmfge_vf_f32m8_b4(dist, -thresh, vl);
        vbool4_t inlier_mask = __riscv_vmand_mm_b4(mask_le, mask_ge, vl);
        vbool4_t outlier_mask = __riscv_vmnot_m_b4(inlier_mask, vl);

        long cnt_out = __riscv_vcpop_m_b4(outlier_mask, vl);
        if (cnt_out > 0) {
            vfloat32m8_t cx = __riscv_vcompress_vm_f32m8(vx, outlier_mask, vl);
            vfloat32m8_t cy = __riscv_vcompress_vm_f32m8(vy, outlier_mask, vl);
            vfloat32m8_t cz = __riscv_vcompress_vm_f32m8(vz, outlier_mask, vl);
            __riscv_vse32_v_f32m8(&ox[out_count], cx, cnt_out);
            __riscv_vse32_v_f32m8(&oy[out_count], cy, cnt_out);
            __riscv_vse32_v_f32m8(&oz[out_count], cz, cnt_out);
            out_count += cnt_out;
        }

        long cnt_in = __riscv_vcpop_m_b4(inlier_mask, vl);
        in_count += cnt_in;
        i += vl;
    }
    ox.resize(out_count);
    oy.resize(out_count);
    oz.resize(out_count);
#else
    ox.reserve(n); oy.reserve(n); oz.reserve(n);
    for (size_t i = 0; i < in.n; ++i) {
        float dist = std::abs(a * in.x[i] + b * in.y[i] + c * in.z[i] + d);
        if (dist <= thresh) {
            in_count++;
        } else {
            ox.push_back(in.x[i]);
            oy.push_back(in.y[i]);
            oz.push_back(in.z[i]);
        }
    }
#endif
    return in_count;
}

// ============================================================================
// Fused ROR (Radius Outlier Removal) using spatial grid
// ============================================================================
static void execute_ror_inplace(
    const float* px, const float* py, const float* pz, size_t n,
    const Fast3DSpatialGrid& grid,
    float search_radius, int min_neighbors,
    std::vector<float>& out_x, std::vector<float>& out_y, std::vector<float>& out_z)
{
    float r2 = search_radius * search_radius;
    std::vector<uint8_t> keep(n, 0);

    const auto& cells = grid.cells_;
    const auto& next = grid.next_;
    const size_t mask = grid.mask_;
    const float inv_cell = grid.inv_cell_;

#if defined(_OPENMP)
    #pragma omp parallel for schedule(dynamic, 128)
#endif
    for (size_t i = 0; i < n; ++i) {
        float qx = px[i], qy = py[i], qz = pz[i];
        int qcx = rv_fast_floor(qx * inv_cell);
        int qcy = rv_fast_floor(qy * inv_cell);
        int qcz = rv_fast_floor(qz * inv_cell);

        int in_radius_count = 0;

        // Check own cell first
        size_t self_h = grid.hash3D(qcx, qcy, qcz);
        int probe = 0;
        while (cells[self_h].head != -1 && probe < Fast3DSpatialGrid::kMaxProbes) {
            if (cells[self_h].cx == qcx && cells[self_h].cy == qcy && cells[self_h].cz == qcz) {
                int curr = cells[self_h].head;
                while (curr != -1) {
                    float ddx = px[curr] - qx, ddy = py[curr] - qy, ddz = pz[curr] - qz;
                    if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) {
                        in_radius_count++;
                        if (in_radius_count >= min_neighbors) break;
                    }
                    curr = next[curr];
                }
                break;
            }
            self_h = (self_h + 1) & mask;
            probe++;
        }

        // Check neighboring cells if needed
        if (in_radius_count < min_neighbors) {
            for (int dz = -1; dz <= 1 && in_radius_count < min_neighbors; ++dz) {
                for (int dy = -1; dy <= 1 && in_radius_count < min_neighbors; ++dy) {
                    for (int dx = -1; dx <= 1 && in_radius_count < min_neighbors; ++dx) {
                        if (dx == 0 && dy == 0 && dz == 0) continue;
                        int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
                        size_t h = grid.hash3D(tcx, tcy, tcz);
                        int p = 0;
                        while (cells[h].head != -1 && p < Fast3DSpatialGrid::kMaxProbes) {
                            if (cells[h].cx == tcx && cells[h].cy == tcy && cells[h].cz == tcz) {
                                int curr = cells[h].head;
                                while (curr != -1) {
                                    float ddx = px[curr] - qx, ddy = py[curr] - qy, ddz = pz[curr] - qz;
                                    if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) {
                                        in_radius_count++;
                                        if (in_radius_count >= min_neighbors) break;
                                    }
                                    curr = next[curr];
                                }
                                break;
                            }
                            h = (h + 1) & mask;
                            p++;
                        }
                    }
                }
            }
        }

        if (in_radius_count >= min_neighbors) {
            keep[i] = 1;
        }
    }

    out_x.clear(); out_y.clear(); out_z.clear();
    out_x.reserve(n); out_y.reserve(n); out_z.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        if (keep[i]) {
            out_x.push_back(px[i]);
            out_y.push_back(py[i]);
            out_z.push_back(pz[i]);
        }
    }
}

// ============================================================================
// Union-Find Clustering (same as pipeline_3d_rvv_clust.cpp)
// ============================================================================
struct ClusterResult {
    std::vector<int> indices;
};

struct UnionFind {
    std::vector<int> parent;
    std::vector<int> rank;
    explicit UnionFind(size_t n) : parent(n), rank(n, 0) {
#if defined(__riscv_vector)
        size_t i = 0;
        int* p = parent.data();
        while (i < n) {
            size_t vl = __riscv_vsetvl_e32m8(n - i);
            vuint32m8_t vid = __riscv_vid_v_u32m8(vl);
            vid = __riscv_vadd_vx_u32m8(vid, static_cast<uint32_t>(i), vl);
            __riscv_vse32_v_u32m8(reinterpret_cast<uint32_t*>(p + i), vid, vl);
            i += vl;
        }
#else
        std::iota(parent.begin(), parent.end(), 0);
#endif
    }
    int find(int i) {
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
    void unite(int i, int j) {
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

class FastClustGrid {
public:
    struct Cell {
        int cx = 0, cy = 0, cz = 0;
        int head = -1;
    };
    static constexpr int kMaxProbes = 64;
    size_t capacity_ = 65536;
    size_t mask_ = 65535;
    float inv_cell_;
    std::vector<Cell> cells_;
    std::vector<int> next_;
    std::vector<uint32_t> touched_slots_;

    FastClustGrid(float cell_size, size_t expected_pts)
        : inv_cell_(1.0f / cell_size)
    {
        capacity_ = next_power_of_2(std::max<size_t>(1024, expected_pts * 2));
        mask_ = capacity_ - 1;
        cells_.resize(capacity_);
        touched_slots_.reserve(std::min<size_t>(capacity_, 65536));
    }

    inline size_t hash3D(int x, int y, int z) const {
        return ((static_cast<size_t>(x) * 73856093) ^
                (static_cast<size_t>(y) * 19349663) ^
                (static_cast<size_t>(z) * 83492791)) & mask_;
    }

    bool build(const float* x, const float* y, const float* z, size_t n) {
        if (capacity_ < n * 2) {
            capacity_ = next_power_of_2(std::max<size_t>(1024, n * 2));
            mask_ = capacity_ - 1;
            cells_.resize(capacity_);
            for (size_t i = 0; i < capacity_; ++i) cells_[i].head = -1;
            touched_slots_.clear();
        }
        for (uint32_t slot : touched_slots_) {
            cells_[slot].head = -1;
        }
        touched_slots_.clear();
        if (next_.size() < n) next_.resize(n);

        for (size_t i = 0; i < n; ++i) {
            int cx = rv_fast_floor(x[i] * inv_cell_);
            int cy = rv_fast_floor(y[i] * inv_cell_);
            int cz = rv_fast_floor(z[i] * inv_cell_);

            size_t h = hash3D(cx, cy, cz);
            int probe = 0;
            while (cells_[h].head != -1 && (cells_[h].cx != cx || cells_[h].cy != cy || cells_[h].cz != cz) && probe < kMaxProbes) {
                h = (h + 1) & mask_;
                probe++;
            }
            if (cells_[h].head == -1) {
                cells_[h].cx = cx; cells_[h].cy = cy; cells_[h].cz = cz;
                touched_slots_.push_back(static_cast<uint32_t>(h));
            }
            next_[i] = cells_[h].head;
            cells_[h].head = static_cast<int>(i);
        }
        return true;
    }
};

static std::vector<ClusterResult> execute_clustering_rvv(
    const PointCloudSoA& cloud, float tolerance, int min_cluster_size, int max_cluster_size)
{
    std::vector<ClusterResult> clusters;
    const size_t n = cloud.n;
    if (n == 0) return clusters;

    const float tol_sq = tolerance * tolerance;
    const float cell_size = tolerance;
    FastClustGrid grid(cell_size, n);
    grid.build(cloud.x, cloud.y, cloud.z, n);

    const int max_threads =
#if defined(_OPENMP)
        omp_get_max_threads();
#else
        1;
#endif

    std::vector<std::vector<std::pair<int, int>>> thread_edges(max_threads);
    std::vector<UnionFind> thread_ufs;
    thread_ufs.reserve(max_threads);
    for (int t = 0; t < max_threads; ++t) {
        thread_edges[t].reserve(n / max_threads);
        thread_ufs.emplace_back(n);
    }

    static const int kForwardOffsets[13][3] = {
        {-1, -1, 1}, { 0, -1, 1}, { 1, -1, 1},
        {-1,  0, 1}, { 0,  0, 1}, { 1,  0, 1},
        {-1,  1, 1}, { 0,  1, 1}, { 1,  1, 1},
        {-1,  1, 0}, { 0,  1, 0}, { 1,  1, 0},
        { 1,  0, 0}
    };

    const auto& touched = grid.touched_slots_;
    const size_t num_cells = touched.size();

#if defined(_OPENMP)
    #pragma omp parallel
#endif
    {
        int tid = 0;
#if defined(_OPENMP)
        tid = omp_get_thread_num();
#endif
        auto& local_edges = thread_edges[tid];
        auto& local_uf = thread_ufs[tid];
        std::vector<int> self_pts;
        std::vector<int> cand_idx;
        std::vector<float> cand_x, cand_y, cand_z;
        self_pts.reserve(64);
        cand_idx.reserve(128);
        cand_x.reserve(128);
        cand_y.reserve(128);
        cand_z.reserve(128);

#if defined(_OPENMP)
        #pragma omp for schedule(dynamic, 32)
#endif
        for (size_t s_idx = 0; s_idx < num_cells; ++s_idx) {
            uint32_t slot = touched[s_idx];
            const auto& cell = grid.cells_[slot];
            if (cell.head == -1) continue;

            self_pts.clear();
            int curr = cell.head;
            while (curr != -1) {
                self_pts.push_back(curr);
                curr = grid.next_[curr];
            }
            size_t n_self = self_pts.size();

            // Intra-cell pairwise
            for (size_t u = 0; u < n_self; ++u) {
                int p_u = self_pts[u];
                float ux = cloud.x[p_u], uy = cloud.y[p_u], uz = cloud.z[p_u];
                for (size_t v = u + 1; v < n_self; ++v) {
                    int p_v = self_pts[v];
                    float ddx = cloud.x[p_v] - ux, ddy = cloud.y[p_v] - uy, ddz = cloud.z[p_v] - uz;
                    if (ddx * ddx + ddy * ddy + ddz * ddz <= tol_sq) {
                        if (local_uf.find(p_u) != local_uf.find(p_v)) {
                            local_uf.unite(p_u, p_v);
                            local_edges.push_back({p_u, p_v});
                        }
                    }
                }
            }

            // Gather candidates from 13 forward neighbors
            cand_idx.clear();
            cand_x.clear();
            cand_y.clear();
            cand_z.clear();

            for (int k = 0; k < 13; ++k) {
                int tcx = cell.cx + kForwardOffsets[k][0];
                int tcy = cell.cy + kForwardOffsets[k][1];
                int tcz = cell.cz + kForwardOffsets[k][2];

                size_t h = grid.hash3D(tcx, tcy, tcz);
                int probe = 0;
                while (grid.cells_[h].head != -1 && probe < Fast3DSpatialGrid::kMaxProbes) {
                    if (grid.cells_[h].cx == tcx && grid.cells_[h].cy == tcy && grid.cells_[h].cz == tcz) {
                        int c_nbr = grid.cells_[h].head;
                        while (c_nbr != -1) {
                            cand_idx.push_back(c_nbr);
                            cand_x.push_back(cloud.x[c_nbr]);
                            cand_y.push_back(cloud.y[c_nbr]);
                            cand_z.push_back(cloud.z[c_nbr]);
                            c_nbr = grid.next_[c_nbr];
                        }
                        break;
                    }
                    h = (h + 1) & grid.mask_;
                    probe++;
                }
            }

            // Vector distance checks
            size_t M = cand_idx.size();
            if (M > 0) {
                for (size_t u = 0; u < n_self; ++u) {
                    int p_u = self_pts[u];
                    float qx = cloud.x[p_u], qy = cloud.y[p_u], qz = cloud.z[p_u];

#if defined(__riscv_vector)
                    size_t k = 0;
                    while (k < M) {
                        size_t vl = __riscv_vsetvl_e32m8(M - k);
                        vfloat32m8_t vx = __riscv_vle32_v_f32m8(&cand_x[k], vl);
                        vfloat32m8_t vy = __riscv_vle32_v_f32m8(&cand_y[k], vl);
                        vfloat32m8_t vz = __riscv_vle32_v_f32m8(&cand_z[k], vl);

                        vfloat32m8_t ddx = __riscv_vfsub_vf_f32m8(vx, qx, vl);
                        vfloat32m8_t ddy = __riscv_vfsub_vf_f32m8(vy, qy, vl);
                        vfloat32m8_t ddz = __riscv_vfsub_vf_f32m8(vz, qz, vl);

                        vfloat32m8_t d2 = __riscv_vfmul_vv_f32m8(ddx, ddx, vl);
                        d2 = __riscv_vfmacc_vv_f32m8(d2, ddy, ddy, vl);
                        d2 = __riscv_vfmacc_vv_f32m8(d2, ddz, ddz, vl);

                        vbool4_t in_tol = __riscv_vmfle_vf_f32m8_b4(d2, tol_sq, vl);
                        if (__riscv_vcpop_m_b4(in_tol, vl) > 0) {
                            uint8_t mbytes[64];
                            __riscv_vsm_v_b4(mbytes, in_tol, vl);
                            for (size_t lane = 0; lane < vl; ++lane) {
                                if ((mbytes[lane >> 3] >> (lane & 7u)) & 1u) {
                                    int p_v = cand_idx[k + lane];
                                    if (local_uf.find(p_u) != local_uf.find(p_v)) {
                                        local_uf.unite(p_u, p_v);
                                        local_edges.push_back({p_u, p_v});
                                    }
                                }
                            }
                        }
                        k += vl;
                    }
#else
                    for (size_t k = 0; k < M; ++k) {
                        float ddx = cand_x[k] - qx;
                        float ddy = cand_y[k] - qy;
                        float ddz = cand_z[k] - qz;
                        if (ddx * ddx + ddy * ddy + ddz * ddz <= tol_sq) {
                            int p_v = cand_idx[k];
                            if (local_uf.find(p_u) != local_uf.find(p_v)) {
                                local_uf.unite(p_u, p_v);
                                local_edges.push_back({p_u, p_v});
                            }
                        }
                    }
#endif
                }
            }
        }
    }

    UnionFind uf(n);
    for (int t = 0; t < max_threads; ++t) {
        for (const auto& edge : thread_edges[t]) {
            uf.unite(edge.first, edge.second);
        }
    }

    // CSR Grouping
    std::vector<int> root_counts(n, 0);
    for (size_t i = 0; i < n; ++i) {
        root_counts[uf.find(static_cast<int>(i))]++;
    }

    std::vector<int> root_to_cid(n, -1);
    int num_valid_clusters = 0;
    for (size_t r = 0; r < n; ++r) {
        int sz = root_counts[r];
        if (sz >= min_cluster_size && sz <= max_cluster_size) {
            root_to_cid[r] = num_valid_clusters++;
        }
    }

    clusters.resize(num_valid_clusters);
    for (size_t r = 0; r < n; ++r) {
        int cid = root_to_cid[r];
        if (cid != -1) {
            clusters[cid].indices.reserve(root_counts[r]);
        }
    }

    for (size_t i = 0; i < n; ++i) {
        int r = uf.find(static_cast<int>(i));
        int cid = root_to_cid[r];
        if (cid != -1) {
            clusters[cid].indices.push_back(static_cast<int>(i));
        }
    }
    return clusters;
}

// ============================================================================
// 8-Core Spatial Slab Perception Pipeline (Stages 4, 5, 6)
// ============================================================================
struct SpatialSlabPipelineResult {
    std::vector<float> fx, fy, fz;
    std::vector<ClusterResult> clusters;
    double partition_grid_ms = 0.0;
    double sort_ms = 0.0;
    double grid_ms = 0.0;
    double ror_ms = 0.0;
    double clustering_ms = 0.0;
};

inline uint32_t float_to_sortable_uint32(float f) {
    uint32_t u;
    std::memcpy(&u, &f, sizeof(float));
    uint32_t mask = static_cast<uint32_t>(static_cast<int32_t>(u) >> 31);
    return u ^ (mask | 0x80000000u);
}

static SpatialSlabPipelineResult execute_spatial_slab_stages(
    const std::vector<float>& ox, const std::vector<float>& oy, const std::vector<float>& oz,
    float ror_radius, int ror_min_pts, bool skip_ror,
    float cluster_tolerance, int min_cluster_size, int max_cluster_size,
    int num_slabs, bool verbose = false)
{
    SpatialSlabPipelineResult res;
    size_t n_obstacles = ox.size();
    if (n_obstacles == 0) return res;
    if (num_slabs < 1) num_slabs = 1;

    // --- STAGE 4: Spatial Slab Partitioning & Local Grids ---
    auto t_start = Clock::now();

    std::vector<uint32_t> keys(n_obstacles);
    std::vector<uint32_t> sorted_idx(n_obstacles);
#if defined(__riscv_vector)
    size_t vi_idx = 0;
    const int32_t* iox = reinterpret_cast<const int32_t*>(ox.data());
    int32_t* okeys = reinterpret_cast<int32_t*>(keys.data());
    while (vi_idx < n_obstacles) {
        size_t vl = __riscv_vsetvl_e32m8(n_obstacles - vi_idx);
        vint32m8_t vi = __riscv_vle32_v_i32m8(iox + vi_idx, vl);
        vint32m8_t vmask = __riscv_vsra_vx_i32m8(vi, 31, vl);
        vint32m8_t vor = __riscv_vor_vx_i32m8(vmask, 0x80000000u, vl);
        vint32m8_t vres = __riscv_vxor_vv_i32m8(vi, vor, vl);
        __riscv_vse32_v_i32m8(okeys + vi_idx, vres, vl);

        vuint32m8_t vid = __riscv_vid_v_u32m8(vl);
        vid = __riscv_vadd_vx_u32m8(vid, static_cast<uint32_t>(vi_idx), vl);
        __riscv_vse32_v_u32m8(&sorted_idx[vi_idx], vid, vl);
        vi_idx += vl;
    }
#else
#if defined(_OPENMP)
    #pragma omp parallel for num_threads(num_slabs) schedule(static)
#endif
    for (size_t i = 0; i < n_obstacles; ++i) {
        keys[i] = float_to_sortable_uint32(ox[i]);
        sorted_idx[i] = static_cast<uint32_t>(i);
    }
#endif
    radix_sort_pairs_u32_parallel(keys.data(), sorted_idx.data(), n_obstacles, num_slabs);

    std::vector<float> sox(n_obstacles), soy(n_obstacles), soz(n_obstacles);
#if defined(_OPENMP)
    #pragma omp parallel for num_threads(num_slabs) schedule(static)
#endif
    for (size_t i = 0; i < n_obstacles; ++i) {
        uint32_t id = sorted_idx[i];
        sox[i] = ox[id];
        soy[i] = oy[id];
        soz[i] = oz[id];
    }
    auto t_sort_end = Clock::now();
    double t_sort_ms = std::chrono::duration<double, std::milli>(t_sort_end - t_start).count();

    std::vector<float> slab_cut(num_slabs + 1);
    slab_cut[0] = -1e9f;
    slab_cut[num_slabs] = 1e9f;
    for (int s = 1; s < num_slabs; ++s) {
        size_t split_i = (n_obstacles * s) / num_slabs;
        slab_cut[s] = 0.5f * (sox[split_i - 1] + sox[split_i]);
    }

    float halo = std::max(ror_radius, cluster_tolerance);

    std::vector<size_t> slab_start(num_slabs);
    std::vector<size_t> slab_end(num_slabs);
    std::vector<size_t> halo_start(num_slabs);
    std::vector<size_t> halo_end(num_slabs);

    for (int s = 0; s < num_slabs; ++s) {
        slab_start[s] = (n_obstacles * s) / num_slabs;
        slab_end[s]   = (n_obstacles * (s + 1)) / num_slabs;

        float low_cut  = slab_cut[s];
        float high_cut = slab_cut[s + 1];

        halo_start[s] = std::lower_bound(sox.begin(), sox.end(), low_cut - halo) - sox.begin();
        halo_end[s]   = std::upper_bound(sox.begin(), sox.end(), high_cut + halo) - sox.begin();
        if (halo_end[s] < halo_start[s]) halo_end[s] = halo_start[s];
    }

    // Build local grids in parallel across 8 threads
    std::vector<Fast3DSpatialGrid> local_grids;
    local_grids.reserve(num_slabs);
    for (int s = 0; s < num_slabs; ++s) {
        local_grids.emplace_back(ror_radius, halo_end[s] - halo_start[s]);
    }

#if defined(_OPENMP)
    #pragma omp parallel for num_threads(num_slabs) schedule(static, 1)
#endif
    for (int s = 0; s < num_slabs; ++s) {
        size_t h_st = halo_start[s];
        size_t ng   = halo_end[s] - h_st;
        if (ng > 0) {
            bool ok = local_grids[s].build(&sox[h_st], &soy[h_st], &soz[h_st], ng);
            if (!ok) {
                std::cerr << "[ERROR] local_grids[" << s << "].build FAILED! ng=" << ng << std::endl;
            }
        }
    }

    double t_grid_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_sort_end).count();
    res.sort_ms = t_sort_ms;
    res.grid_ms = t_grid_ms;
    res.partition_grid_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_start).count();

    // --- STAGE 5: Embarrassingly Parallel Per-Slab ROR ---
    t_start = Clock::now();
    std::vector<std::vector<uint32_t>> slab_kept(num_slabs);

#if defined(_OPENMP)
    #pragma omp parallel for num_threads(num_slabs) schedule(static, 1)
#endif
    for (int s = 0; s < num_slabs; ++s) {
        size_t s_st = slab_start[s];
        size_t s_en = slab_end[s];
        size_t h_st = halo_start[s];
        auto& kept = slab_kept[s];
        kept.reserve(s_en - s_st);

        if (skip_ror) {
            for (size_t i = s_st; i < s_en; ++i) kept.push_back(static_cast<uint32_t>(i));
        } else if (s_en > s_st) {
            const auto& grid = local_grids[s];
            const float* px = &sox[h_st];
            const float* py = &soy[h_st];
            const float* pz = &soz[h_st];
            float r2 = ror_radius * ror_radius;

            for (size_t i = s_st; i < s_en; ++i) {
                float qx = sox[i], qy = soy[i], qz = soz[i];
                int qcx = rv_fast_floor(qx * grid.inv_cell_);
                int qcy = rv_fast_floor(qy * grid.inv_cell_);
                int qcz = rv_fast_floor(qz * grid.inv_cell_);

                int count = 0;
                size_t h = grid.hash3D(qcx, qcy, qcz);
                int p = 0;
                while (grid.cells_[h].head != -1 && p < Fast3DSpatialGrid::kMaxProbes) {
                    if (grid.cells_[h].cx == qcx && grid.cells_[h].cy == qcy && grid.cells_[h].cz == qcz) {
                        int curr = grid.cells_[h].head;
                        while (curr != -1) {
                            float ddx = px[curr] - qx, ddy = py[curr] - qy, ddz = pz[curr] - qz;
                            if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) {
                                count++;
                                if (count >= ror_min_pts) break;
                            }
                            curr = grid.next_[curr];
                        }
                        break;
                    }
                    h = (h + 1) & grid.mask_;
                    p++;
                }

                if (count < ror_min_pts) {
                    for (int dz = -1; dz <= 1 && count < ror_min_pts; ++dz) {
                        for (int dy = -1; dy <= 1 && count < ror_min_pts; ++dy) {
                            for (int dx = -1; dx <= 1 && count < ror_min_pts; ++dx) {
                                if (dx == 0 && dy == 0 && dz == 0) continue;
                                int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
                                size_t nh = grid.hash3D(tcx, tcy, tcz);
                                int np = 0;
                                while (grid.cells_[nh].head != -1 && np < Fast3DSpatialGrid::kMaxProbes) {
                                    if (grid.cells_[nh].cx == tcx && grid.cells_[nh].cy == tcy && grid.cells_[nh].cz == tcz) {
                                        int curr = grid.cells_[nh].head;
                                        while (curr != -1) {
                                            float ddx = px[curr] - qx, ddy = py[curr] - qy, ddz = pz[curr] - qz;
                                            if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) {
                                                count++;
                                                if (count >= ror_min_pts) break;
                                            }
                                            curr = grid.next_[curr];
                                        }
                                        break;
                                    }
                                    nh = (nh + 1) & grid.mask_;
                                    np++;
                                }
                            }
                        }
                    }
                }

                if (count >= ror_min_pts) {
                    kept.push_back(static_cast<uint32_t>(i));
                }
            }
        }
    }

    res.ror_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_start).count();

    // --- Assemble filtered points with slab offsets ---
    std::vector<size_t> slab_offset(num_slabs + 1, 0);
    for (int s = 0; s < num_slabs; ++s) {
        slab_offset[s + 1] = slab_offset[s] + slab_kept[s].size();
    }
    size_t n_filtered = slab_offset[num_slabs];
    res.fx.resize(n_filtered);
    res.fy.resize(n_filtered);
    res.fz.resize(n_filtered);

#if defined(_OPENMP)
    #pragma omp parallel for num_threads(num_slabs) schedule(static, 1)
#endif
    for (int s = 0; s < num_slabs; ++s) {
        size_t base = slab_offset[s];
        const auto& kept = slab_kept[s];
        for (size_t k = 0; k < kept.size(); ++k) {
            uint32_t id = kept[k];
            res.fx[base + k] = sox[id];
            res.fy[base + k] = soy[id];
            res.fz[base + k] = soz[id];
        }
    }

    // --- STAGE 6: Spatial Slab Clustering + Boundary Stitching ---
    t_start = Clock::now();
    UnionFind global_uf(n_filtered);
    const float tol_sq = cluster_tolerance * cluster_tolerance;

    static const int kForwardOffsets[13][3] = {
        {-1, -1, 1}, { 0, -1, 1}, { 1, -1, 1},
        {-1,  0, 1}, { 0,  0, 1}, { 1,  0, 1},
        {-1,  1, 1}, { 0,  1, 1}, { 1,  1, 1},
        {-1,  1, 0}, { 0,  1, 0}, { 1,  1, 0},
        { 1,  0, 0}
    };

    std::vector<double> slab_6a_ms(num_slabs, 0.0);

    // Step 6A: Intra-slab clustering in parallel across all cores
#if defined(_OPENMP)
    #pragma omp parallel for num_threads(num_slabs) schedule(static, 1)
#endif
    for (int s = 0; s < num_slabs; ++s) {
        auto t_s_start = Clock::now();
        size_t base = slab_offset[s];
        size_t n_s  = slab_offset[s + 1] - base;
        if (n_s <= 1) continue;

        const float* sx = res.fx.data() + base;
        const float* sy = res.fy.data() + base;
        const float* sz = res.fz.data() + base;

        FastClustGrid grid(cluster_tolerance, n_s);
        grid.build(sx, sy, sz, n_s);

        std::vector<int> self_pts;
        std::vector<int> cand_idx;
        std::vector<float> cand_x, cand_y, cand_z;
        self_pts.reserve(64);
        cand_idx.reserve(128); cand_x.reserve(128); cand_y.reserve(128); cand_z.reserve(128);

        const auto& touched = grid.touched_slots_;
        for (uint32_t slot : touched) {
            const auto& cell = grid.cells_[slot];
            if (cell.head == -1) continue;

            self_pts.clear();
            int curr = cell.head;
            while (curr != -1) {
                self_pts.push_back(curr);
                curr = grid.next_[curr];
            }
            size_t n_self = self_pts.size();

            for (size_t u = 0; u < n_self; ++u) {
                int p_u = self_pts[u];
                float ux = sx[p_u], uy = sy[p_u], uz = sz[p_u];
                for (size_t v = u + 1; v < n_self; ++v) {
                    int p_v = self_pts[v];
                    float ddx = sx[p_v] - ux, ddy = sy[p_v] - uy, ddz = sz[p_v] - uz;
                    if (ddx * ddx + ddy * ddy + ddz * ddz <= tol_sq) {
                        global_uf.unite(static_cast<int>(base + p_u), static_cast<int>(base + p_v));
                    }
                }
            }

            cand_idx.clear(); cand_x.clear(); cand_y.clear(); cand_z.clear();
            for (int k = 0; k < 13; ++k) {
                int tcx = cell.cx + kForwardOffsets[k][0];
                int tcy = cell.cy + kForwardOffsets[k][1];
                int tcz = cell.cz + kForwardOffsets[k][2];
                size_t h = grid.hash3D(tcx, tcy, tcz);
                int probe = 0;
                while (grid.cells_[h].head != -1 && probe < Fast3DSpatialGrid::kMaxProbes) {
                    if (grid.cells_[h].cx == tcx && grid.cells_[h].cy == tcy && grid.cells_[h].cz == tcz) {
                        int c_nbr = grid.cells_[h].head;
                        while (c_nbr != -1) {
                            cand_idx.push_back(c_nbr);
                            cand_x.push_back(sx[c_nbr]);
                            cand_y.push_back(sy[c_nbr]);
                            cand_z.push_back(sz[c_nbr]);
                            c_nbr = grid.next_[c_nbr];
                        }
                        break;
                    }
                    h = (h + 1) & grid.mask_;
                    probe++;
                }
            }

            size_t M = cand_idx.size();
            if (M > 0) {
                for (size_t u = 0; u < n_self; ++u) {
                    int p_u = self_pts[u];
                    float qx = sx[p_u], qy = sy[p_u], qz = sz[p_u];
#if defined(__riscv_vector)
                    size_t k = 0;
                    while (k < M) {
                        size_t vl = __riscv_vsetvl_e32m8(M - k);
                        vfloat32m8_t vx = __riscv_vle32_v_f32m8(&cand_x[k], vl);
                        vfloat32m8_t vy = __riscv_vle32_v_f32m8(&cand_y[k], vl);
                        vfloat32m8_t vz = __riscv_vle32_v_f32m8(&cand_z[k], vl);

                        vfloat32m8_t ddx = __riscv_vfsub_vf_f32m8(vx, qx, vl);
                        vfloat32m8_t ddy = __riscv_vfsub_vf_f32m8(vy, qy, vl);
                        vfloat32m8_t ddz = __riscv_vfsub_vf_f32m8(vz, qz, vl);

                        vfloat32m8_t d2 = __riscv_vfmul_vv_f32m8(ddx, ddx, vl);
                        d2 = __riscv_vfmacc_vv_f32m8(d2, ddy, ddy, vl);
                        d2 = __riscv_vfmacc_vv_f32m8(d2, ddz, ddz, vl);

                        vbool4_t in_tol = __riscv_vmfle_vf_f32m8_b4(d2, tol_sq, vl);
                        if (__riscv_vcpop_m_b4(in_tol, vl) > 0) {
                            uint8_t mbytes[64];
                            __riscv_vsm_v_b4(mbytes, in_tol, vl);
                            for (size_t lane = 0; lane < vl; ++lane) {
                                if ((mbytes[lane >> 3] >> (lane & 7u)) & 1u) {
                                    global_uf.unite(static_cast<int>(base + p_u), static_cast<int>(base + cand_idx[k + lane]));
                                }
                            }
                        }
                        k += vl;
                    }
#else
                    for (size_t k = 0; k < M; ++k) {
                        float ddx = cand_x[k] - qx;
                        float ddy = cand_y[k] - qy;
                        float ddz = cand_z[k] - qz;
                        if (ddx * ddx + ddy * ddy + ddz * ddz <= tol_sq) {
                            global_uf.unite(static_cast<int>(base + p_u), static_cast<int>(base + cand_idx[k]));
                        }
                    }
#endif
                }
            }
        }
        slab_6a_ms[s] = std::chrono::duration<double, std::milli>(Clock::now() - t_s_start).count();
    }

    auto t_6a_end = Clock::now();
    if (verbose) {
        std::cout << "[slab-threads-6A] per-slab ms: ";
        for (int s = 0; s < num_slabs; ++s) std::cout << "S" << s << ":" << slab_6a_ms[s] << "ms ";
        std::cout << std::endl;
    }

    // Step 6B: Boundary component stitching across the (num_slabs - 1) boundary planes
    for (int s = 1; s < num_slabs; ++s) {
        float split_x = slab_cut[s];
        size_t left_base  = slab_offset[s - 1];
        size_t left_size  = slab_offset[s] - left_base;
        size_t right_base = slab_offset[s];
        size_t right_size = slab_offset[s + 1] - right_base;

        size_t left_end  = left_base + left_size;
        size_t right_end = right_base + right_size;

        size_t left_bnd_start = std::lower_bound(res.fx.begin() + left_base, res.fx.begin() + left_end, split_x - cluster_tolerance) - res.fx.begin();
        size_t right_bnd_end  = std::upper_bound(res.fx.begin() + right_base, res.fx.begin() + right_end, split_x + cluster_tolerance) - res.fx.begin();

        for (size_t li = left_bnd_start; li < left_end; ++li) {
            float lx = res.fx[li], ly = res.fy[li], lz = res.fz[li];
            for (size_t rj = right_base; rj < right_bnd_end; ++rj) {
                float ddx = res.fx[rj] - lx;
                if (ddx > cluster_tolerance) break;
                if (ddx < -cluster_tolerance) continue;
                float ddy = std::abs(res.fy[rj] - ly);
                if (ddy > cluster_tolerance) continue;
                float ddz = std::abs(res.fz[rj] - lz);
                if (ddz > cluster_tolerance) continue;
                if (ddx * ddx + ddy * ddy + ddz * ddz <= tol_sq) {
                    global_uf.unite(static_cast<int>(li), static_cast<int>(rj));
                }
            }
        }
    }
    auto t_6b_end = Clock::now();

    // Step 6C: Filter and extract valid clusters
#if defined(_OPENMP)
    #pragma omp parallel for num_threads(num_slabs) schedule(static)
#endif
    for (size_t i = 0; i < n_filtered; ++i) {
        global_uf.parent[i] = global_uf.find(static_cast<int>(i));
    }

    std::vector<int> root_counts(n_filtered, 0);
    for (size_t i = 0; i < n_filtered; ++i) {
        root_counts[global_uf.parent[i]]++;
    }

    std::vector<int> root_to_cid(n_filtered, -1);
    int num_valid_clusters = 0;
    for (size_t r = 0; r < n_filtered; ++r) {
        int sz = root_counts[r];
        if (sz >= min_cluster_size && sz <= max_cluster_size) {
            root_to_cid[r] = num_valid_clusters++;
        }
    }

    res.clusters.resize(num_valid_clusters);
    for (size_t r = 0; r < n_filtered; ++r) {
        int cid = root_to_cid[r];
        if (cid != -1) {
            res.clusters[cid].indices.reserve(root_counts[r]);
        }
    }

    for (size_t i = 0; i < n_filtered; ++i) {
        int r = global_uf.parent[i];
        int cid = root_to_cid[r];
        if (cid != -1) {
            res.clusters[cid].indices.push_back(static_cast<int>(i));
        }
    }

    // Physical obstacle sanity check: filter out flat road surface undulations
    std::vector<ClusterResult> valid_clusters;
    valid_clusters.reserve(res.clusters.size());
    for (auto& cl : res.clusters) {
        if (cl.indices.empty()) continue;
        float min_z = res.fz[cl.indices[0]], max_z = min_z;
        for (int idx : cl.indices) {
            float z = res.fz[idx];
            if (z < min_z) min_z = z;
            if (z > max_z) max_z = z;
        }
        // If a cluster is at ground level and completely flat (thickness < 0.12m), it is road pavement
        if ((max_z - min_z) < 0.12f && min_z < -1.2f) {
            continue;
        }
        valid_clusters.push_back(std::move(cl));
    }
    res.clusters = std::move(valid_clusters);

    auto t_6c_end = Clock::now();
    res.clustering_ms = std::chrono::duration<double, std::milli>(t_6c_end - t_start).count();

    double t_6a_ms = std::chrono::duration<double, std::milli>(t_6a_end - t_start).count();
    double t_6b_ms = std::chrono::duration<double, std::milli>(t_6b_end - t_6a_end).count();
    double t_6c_ms = std::chrono::duration<double, std::milli>(t_6c_end - t_6b_end).count();

    if (verbose) {
        std::cout << "[slab-breakdown] Stage 4 [sort: " << res.sort_ms << " ms, grid: " << res.grid_ms << " ms, total: " << res.partition_grid_ms << " ms] | "
                  << "Stage 5 ROR: " << res.ror_ms << " ms | "
                  << "Stage 6 [6A intra: " << t_6a_ms << " ms, 6B stitch: " << t_6b_ms << " ms, 6C extract: " << t_6c_ms << " ms]" << std::endl;
    }

    return res;
}

// ============================================================================
// Stage Management & Progress Logging
// ============================================================================
void beginStage(int stage_idx, const char *label, bool enabled) {
    if (enabled) {
        std::cout << "[progress] [" << stage_idx << "/" << kStageCount << "] "
                  << label << "..." << std::endl;
    }
}

double endStage(int stage_idx, const char *label,
                const Clock::time_point &start_time, bool enabled) {
    const auto end_time = Clock::now();
    const double ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    if (enabled) {
        std::cout << "[progress] [" << stage_idx << "/" << kStageCount << "] "
                  << label << " complete in " << ms << " ms" << std::endl;
    }
    return ms;
}

void printFinalBreakdown(const std::vector<StageTiming> &stages, double total_ms) {
    std::cout << "[progress] Final timing breakdown:" << std::endl;
    double measured_sum = 0.0;
    for (const auto &st : stages) {
        measured_sum += st.ms;
        const double pct = total_ms > 0.0 ? (st.ms / total_ms) * 100.0 : 0.0;
        std::cout << "[progress] [" << std::setw(2) << st.index << "/" << kStageCount << "] "
                  << st.label << ": " << std::fixed << std::setprecision(3)
                  << st.ms << " ms (" << std::setprecision(2) << pct << "%), pts="
                  << st.point_count << std::endl;
    }
    const double outside_ms = total_ms - measured_sum;
    const double outside_pct = total_ms > 0.0 ? (outside_ms / total_ms) * 100.0 : 0.0;
    std::cout << "[progress] [--] Outside timed stages: "
              << std::fixed << std::setprecision(3) << outside_ms << " ms ("
              << std::setprecision(2) << outside_pct << "%)" << std::endl;
    std::cout << "[progress] [--] Total: " << total_ms << " ms (100.00%)" << std::endl;
}

std::string resolveInputPath(const std::string &raw_path) {
    if (std::filesystem::exists(raw_path)) return raw_path;
    const std::filesystem::path p(raw_path);
    const std::filesystem::path data_dir("data");
    const std::filesystem::path alt1 = data_dir / p.filename();
    if (std::filesystem::exists(alt1)) return alt1.string();
    const std::filesystem::path alt2 = data_dir / "pcd_compressed" / p.filename();
    if (std::filesystem::exists(alt2)) return alt2.string();
    return raw_path;
}

struct FrameMetrics {
    int frame_id = 0;
    std::string filename;
    double io_read_ms = 0.0;
    double compute_ms = 0.0;
    double io_write_ms = 0.0;
    double total_ms = 0.0;

    double voxel_ms = 0.0;
    double ransac_ms = 0.0;
    double grid_build_ms = 0.0;
    double ror_ms = 0.0;
    double cluster_ms = 0.0;

    size_t raw_points = 0;
    size_t downsampled_points = 0;
    size_t obstacle_points = 0;
    size_t filtered_points = 0;
    size_t ground_inliers = 0;
    size_t cluster_count = 0;

    std::string s4_label;
    std::string s5_label;
    std::string s6_label;
    std::vector<StageTiming> stage_timings;
};

static bool process_single_frame(
    const std::string& input_path,
    int f_idx,
    float voxel_leaf_size,
    float cluster_tolerance,
    int min_cluster_size,
    int max_cluster_size,
    float ransac_distance_threshold,
    int ransac_max_iters,
    float ror_radius,
    int ror_min_pts,
    uint64_t seed,
    bool skip_ror,
    int num_slabs,
    const std::vector<float>& ground_normal_prior,
    float min_ground_dot,
    bool progress_enabled,
    bool disable_disk,
    const std::string& output_dir_base,
    FrameMetrics& m)
{
    const auto overall_start = Clock::now();
    m.frame_id = f_idx;
    m.filename = std::filesystem::path(input_path).filename().string();
    m.stage_timings.clear();
    m.stage_timings.reserve(kStageCount);

    // ========================================================================
    // STAGE 1: Load PCD
    // ========================================================================
    std::vector<PointXYZ> loaded_points;
    beginStage(1, "Load input cloud", progress_enabled);
    auto stage_start = Clock::now();
    int64_t count = loadPCD(input_path, loaded_points);
    if (count <= 0 || loaded_points.empty()) {
        std::cerr << "Failed to load input PCD: " << input_path << std::endl;
        return false;
    }
    const size_t n_input = loaded_points.size();
    m.raw_points = n_input;
    m.io_read_ms = endStage(1, "Load input cloud", stage_start, progress_enabled);
    m.stage_timings.push_back({1, "Load input cloud", m.io_read_ms, n_input});

    std::vector<float> ix(n_input), iy(n_input), iz(n_input);
    for (size_t i = 0; i < n_input; ++i) {
        ix[i] = loaded_points[i].x; iy[i] = loaded_points[i].y; iz[i] = loaded_points[i].z;
    }
    PointCloudSoA input_cloud{ix.data(), iy.data(), iz.data(), n_input};

    // ========================================================================
    // STAGE 2: Voxel Downsampling (with LSD radix sort)
    // ========================================================================
    std::vector<PointXYZ> downsampled_pts(n_input);
    beginStage(2, "Downsampling (Radix Sort)", progress_enabled);
    stage_start = Clock::now();
    size_t n_down = voxel_grid_downsamp_rvv_v2(input_cloud, downsampled_pts.data(), voxel_leaf_size);
    downsampled_pts.resize(n_down);
    m.voxel_ms = endStage(2, "Downsampling (Radix Sort)", stage_start, progress_enabled);
    m.downsampled_points = n_down;
    m.stage_timings.push_back({2, "Downsampling (Radix Sort)", m.voxel_ms, n_down});

    std::vector<float> dx(n_down), dy(n_down), dz(n_down);
    for (size_t i = 0; i < n_down; ++i) {
        dx[i] = downsampled_pts[i].x; dy[i] = downsampled_pts[i].y; dz[i] = downsampled_pts[i].z;
    }
    PointCloudSoA downsampled_cloud{dx.data(), dy.data(), dz.data(), n_down};

    // ========================================================================
    // STAGE 3: RANSAC Ground Removal (INVERTED)
    // ========================================================================
    float model[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    std::vector<float> ox, oy, oz;

    beginStage(3, "RANSAC ground removal (inverted)", progress_enabled);
    stage_start = Clock::now();
    size_t n_inliers = 0;
    if (n_down >= 3) {
        ransac_plane_intra(downsampled_cloud, ransac_distance_threshold,
                           ransac_max_iters, model,
                           ground_normal_prior.data(), min_ground_dot, seed);
    }
    n_inliers = extract_outliers_soa(downsampled_cloud, model, ransac_distance_threshold,
                                     ox, oy, oz);
    size_t n_obstacles = ox.size();
    m.ransac_ms = endStage(3, "RANSAC ground removal (inverted)", stage_start, progress_enabled);
    m.ground_inliers = n_inliers;
    m.obstacle_points = n_obstacles;
    m.stage_timings.push_back({3, "RANSAC ground removal (inverted)", m.ransac_ms, n_obstacles});

    // ========================================================================
    // STAGES 4, 5, 6: Spatial Slab Decomposition (Scalable Cores)
    // ========================================================================
    m.s4_label = "Build search grid (" + std::to_string(num_slabs) + "-core slab)";
    m.s5_label = "Radius outlier removal (" + std::to_string(num_slabs) + "-core slab)";
    m.s6_label = "Euclidean clustering (" + std::to_string(num_slabs) + "-core slab + merge)";

    beginStage(4, m.s4_label.c_str(), progress_enabled);
    beginStage(5, m.s5_label.c_str(), progress_enabled);
    beginStage(6, m.s6_label.c_str(), progress_enabled);

    auto slab_res = execute_spatial_slab_stages(
        ox, oy, oz,
        ror_radius, ror_min_pts, skip_ror,
        cluster_tolerance, min_cluster_size, max_cluster_size,
        num_slabs, progress_enabled);

    m.grid_build_ms = slab_res.partition_grid_ms;
    m.ror_ms = slab_res.ror_ms;
    m.cluster_ms = slab_res.clustering_ms;
    m.filtered_points = slab_res.fx.size();
    m.cluster_count = slab_res.clusters.size();

    m.stage_timings.push_back({4, m.s4_label.c_str(), slab_res.partition_grid_ms, n_obstacles});
    m.stage_timings.push_back({5, m.s5_label.c_str(), slab_res.ror_ms, slab_res.fx.size()});
    m.stage_timings.push_back({6, m.s6_label.c_str(), slab_res.clustering_ms, slab_res.clusters.size()});

    const auto& fx = slab_res.fx;
    const auto& fy = slab_res.fy;
    const auto& fz = slab_res.fz;
    size_t n_filtered = fx.size();
    const auto& clusters = slab_res.clusters;

    // ========================================================================
    // STAGE 7: Write Output (if enabled)
    // ========================================================================
    beginStage(7, "Write output", progress_enabled);
    stage_start = Clock::now();
    if (!disable_disk) {
        const std::filesystem::path output_dir =
            !output_dir_base.empty() ? std::filesystem::path(output_dir_base)
                                     : std::filesystem::path("output") / "intra_clusters";
        std::error_code dir_ec;
        std::filesystem::create_directories(output_dir, dir_ec);

        struct RGBColor { std::uint8_t r, g, b; };
        auto generateColors = [](size_t count) {
            std::vector<RGBColor> colors(count);
            for (size_t i = 0; i < count; ++i) {
                float hue = std::fmod(i * 0.618033988749895f, 1.0f);
                float c = 0.95f * 0.85f;
                float x = c * (1.0f - std::abs(std::fmod(hue * 6.0f, 2.0f) - 1.0f));
                float m = 0.95f - c;
                float r = 0, g = 0, b = 0;
                int h = static_cast<int>(hue * 6.0f) % 6;
                if (h == 0) { r = c; g = x; } else if (h == 1) { r = x; g = c; }
                else if (h == 2) { g = c; b = x; } else if (h == 3) { g = x; b = c; }
                else if (h == 4) { r = x; b = c; } else { r = c; b = x; }
                colors[i] = {static_cast<uint8_t>((r + m) * 255.0f), static_cast<uint8_t>((g + m) * 255.0f), static_cast<uint8_t>((b + m) * 255.0f)};
            }
            return colors;
        };

        auto colors = generateColors(clusters.size());
        std::vector<PointXYZRGB> colored_pts;
        for (size_t c_idx = 0; c_idx < clusters.size(); ++c_idx) {
            const auto& col = colors[c_idx];
            for (int pt_idx : clusters[c_idx].indices) {
                if (pt_idx >= 0 && static_cast<size_t>(pt_idx) < n_filtered) {
                    colored_pts.push_back({fx[pt_idx], fy[pt_idx], fz[pt_idx], col.r, col.g, col.b});
                }
            }
        }
        std::stringstream ss;
        ss << "frame_" << std::setfill('0') << std::setw(6) << f_idx << "_clusters.pcd";
        savePCDRGB((output_dir / ss.str()).string(), colored_pts, true);
    }
    m.io_write_ms = endStage(7, "Write output", stage_start, progress_enabled);
    m.stage_timings.push_back({7, "Write output", m.io_write_ms, clusters.size()});

    const auto overall_end = Clock::now();
    m.total_ms = std::chrono::duration<double, std::milli>(overall_end - overall_start).count();
    m.compute_ms = m.voxel_ms + m.ransac_ms + m.grid_build_ms + m.ror_ms + m.cluster_ms;

    return true;
}

} // anonymous namespace

// ============================================================================
// MAIN — Intra-Frame Scalable Pipeline (Single-Frame or Continuous Stream)
// ============================================================================
int main(int argc, char** argv) {
    bool progress_enabled = false;
    bool disable_disk = false;
    float voxel_leaf_size = kPipelineConfig.voxel_leaf_size;
    float cluster_tolerance = kPipelineConfig.cluster_tolerance;
    int min_cluster_size = kPipelineConfig.min_cluster_size;
    int max_cluster_size = kPipelineConfig.max_cluster_size;
    float ransac_distance_threshold = kPipelineConfig.ransac_distance_threshold;
    int ransac_max_iters = kPipelineConfig.ransac_max_iterations;
    float ror_radius = 0.25f;
    int ror_min_pts = 3;
    uint64_t seed = 42;
    bool skip_ror = false;
    int requested_threads = 0;
    int max_frames = -1;
    bool json_metrics = false;

    std::vector<float> ground_normal_prior = {0.0f, 0.0f, 1.0f};
    float min_ground_dot = 0.707f;

    std::vector<std::string> positional_args;

    try {
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg == "--progress") {
                progress_enabled = true;
            } else if (arg == "--no-write" || arg == "--disable-disk") {
                disable_disk = true;
            } else if (arg == "--skip-ror" || arg == "--no-ror" || arg == "--skip-sor" || arg == "--no-sor") {
                skip_ror = true;
            } else if (arg == "--json" || arg == "--json-metrics") {
                json_metrics = true;
            } else if (arg == "--threads" || arg == "--cores" || arg == "--num-slabs") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for " << arg << "\n"; return 1; }
                requested_threads = std::stoi(argv[++i]);
                if (requested_threads < 1) requested_threads = 1;
                if (requested_threads > 64) requested_threads = 64;
#if defined(_OPENMP)
                omp_set_num_threads(requested_threads);
#endif
            } else if (arg == "--max-frames" || arg == "--frames") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --max-frames\n"; return 1; }
                max_frames = std::stoi(argv[++i]);
            } else if (arg == "--leaf-size") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --leaf-size\n"; return 1; }
                voxel_leaf_size = std::stof(argv[++i]);
            } else if (arg == "--cluster-tolerance") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --cluster-tolerance\n"; return 1; }
                cluster_tolerance = std::stof(argv[++i]);
            } else if (arg == "--min-cluster") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --min-cluster\n"; return 1; }
                min_cluster_size = std::stoi(argv[++i]);
            } else if (arg == "--max-cluster") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --max-cluster\n"; return 1; }
                max_cluster_size = std::stoi(argv[++i]);
            } else if (arg == "--ransac-dist" || arg == "--ransac-distance-threshold") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for " << arg << "\n"; return 1; }
                ransac_distance_threshold = std::stof(argv[++i]);
            } else if (arg == "--ransac-iters") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --ransac-iters\n"; return 1; }
                ransac_max_iters = std::stoi(argv[++i]);
            } else if (arg == "--ror-radius") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --ror-radius\n"; return 1; }
                ror_radius = std::stof(argv[++i]);
            } else if (arg == "--ror-min-pts") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --ror-min-pts\n"; return 1; }
                ror_min_pts = std::stoi(argv[++i]);
            } else if (arg == "--seed") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --seed\n"; return 1; }
                seed = std::stoull(argv[++i]);
            } else if (arg == "--ground-angle-thresh") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --ground-angle-thresh\n"; return 1; }
                float deg = std::stof(argv[++i]);
                min_ground_dot = std::cos(deg * 3.14159265358979323846f / 180.0f);
            } else if (arg == "--no-ground-prior" || arg == "--unconstrained-plane") {
                min_ground_dot = 0.0f;
            } else if (arg == "--optical-frame") {
                ground_normal_prior = {0.0f, 1.0f, 0.0f};
            } else if (!arg.empty() && arg[0] == '-') {
                std::cerr << "Error: Unrecognized option '" << arg << "'\n";
                return 1;
            } else {
                positional_args.push_back(arg);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error parsing CLI arguments: " << e.what() << std::endl;
        return 1;
    }

    if (positional_args.empty()) {
        std::cerr << "Usage: " << argv[0]
                  << " [--threads <N>] [--max-frames <N>] [--progress] [--no-write] [--skip-ror] "
                     "[--leaf-size <val>] [--cluster-tolerance <val>] "
                     "[--min-cluster <val>] [--max-cluster <val>] "
                     "[--ransac-dist <val>] [--ransac-iters <val>] [--ror-radius <val>] [--ror-min-pts <val>] "
                     "[--seed <val>] <input.pcd | input_directory/>\n";
        return 1;
    }

    int num_slabs = (requested_threads > 0) ? requested_threads : 8;
#if defined(_OPENMP)
    if (requested_threads <= 0) {
        num_slabs = omp_get_max_threads();
    }
#endif
    if (num_slabs > 8) num_slabs = 8;
    if (num_slabs < 1) num_slabs = 1;

    // Collect PCD files
    std::string raw_input = positional_args[0];
    std::string input_path = resolveInputPath(raw_input);
    std::vector<std::string> pcd_files;

    if (std::filesystem::is_directory(input_path)) {
        for (const auto& entry : std::filesystem::directory_iterator(input_path)) {
            if (entry.path().extension() == ".pcd") {
                pcd_files.push_back(entry.path().string());
            }
        }
        std::sort(pcd_files.begin(), pcd_files.end());
    } else if (std::filesystem::exists(input_path)) {
        pcd_files.push_back(input_path);
    } else {
        std::cerr << "Error: Input path not found: " << raw_input << std::endl;
        return 1;
    }

    if (pcd_files.empty()) {
        std::cerr << "Error: No .pcd files found in " << input_path << std::endl;
        return 1;
    }

    if (max_frames > 0 && pcd_files.size() > static_cast<size_t>(max_frames)) {
        pcd_files.resize(max_frames);
    }

    std::string output_dir_base = positional_args.size() >= 2 ? positional_args[1] : "";

    // ========================================================================
    // CASE A: Single-Frame Mode
    // ========================================================================
    if (pcd_files.size() == 1) {
        FrameMetrics m;
        bool ok = process_single_frame(
            pcd_files[0], 0,
            voxel_leaf_size, cluster_tolerance, min_cluster_size, max_cluster_size,
            ransac_distance_threshold,
            ransac_max_iters, ror_radius, ror_min_pts, seed, skip_ror,
            num_slabs, ground_normal_prior, min_ground_dot,
            progress_enabled, disable_disk, output_dir_base, m);
        if (!ok) return 1;

        std::cout << "\n========================================================================\n"
                  << "  RVPoint Intra-Frame Pipeline (" << num_slabs << "-Core Spatial Slab, Inverted)\n"
                  << "========================================================================\n"
                  << "  Pipeline Order: Load → Downsample(Radix) → RANSAC → Grid → ROR → Cluster\n"
                  << "  (RANSAC runs BEFORE grid build — 30% fewer points in expensive stages)\n"
                  << "========================================================================\n";
        printFinalBreakdown(m.stage_timings, m.total_ms);
        std::cout << "[result] Total clusters found: " << m.cluster_count << std::endl;
        std::cout << "[result] Ground inliers: " << m.ground_inliers << " | Obstacles: " << m.obstacle_points
                  << " | Filtered: " << m.filtered_points << std::endl;
        std::cout << "========================================================================\n";

        if (json_metrics) {
            std::cout << "[metrics_json] {"
                      << "\"total_ms\": " << m.total_ms << ", "
                      << "\"compute_ms\": " << m.compute_ms << ", "
                      << "\"full_fps\": " << (m.total_ms > 0 ? 1000.0 / m.total_ms : 0.0) << ", "
                      << "\"compute_fps\": " << (m.compute_ms > 0 ? 1000.0 / m.compute_ms : 0.0) << ", "
                      << "\"load_ms\": " << m.io_read_ms << ", "
                      << "\"downsample_ms\": " << m.voxel_ms << ", "
                      << "\"ransac_ms\": " << m.ransac_ms << ", "
                      << "\"grid_ms\": " << m.grid_build_ms << ", "
                      << "\"ror_ms\": " << m.ror_ms << ", "
                      << "\"clustering_ms\": " << m.cluster_ms << ", "
                      << "\"num_threads\": " << num_slabs << ", "
                      << "\"clusters\": " << m.cluster_count
                      << "}" << std::endl;
        }
        return 0;
    }

    // ========================================================================
    // CASE B: Multi-Frame Continuous Stream Benchmark Mode
    // ========================================================================
    std::cout << "========================================================================\n"
              << "  RVPoint Multi-Core Continuous Stream Pipeline (Intra-Frame Spatial Slab)\n"
              << "  Target Architecture: SpacemiT K1 / Orange Pi RV2 (RV64GCV Octa-Core)\n"
              << "========================================================================\n"
              << "  Input Directory : " << raw_input << " (" << pcd_files.size() << " frames)\n"
              << "  Streaming Mode  : INTRA-FRAME (" << num_slabs << "-Core Parallel Spatial Slab Decomposition)\n"
              << "  Worker Threads  : " << num_slabs << "\n"
              << "  Voxel Leaf Size : " << voxel_leaf_size << " m\n"
              << "  Cluster Tol     : " << cluster_tolerance << " m (min: " << min_cluster_size << ")\n"
              << "  Cluster Export  : " << (disable_disk ? "DISABLED (Zero Disk I/O)" : "ENABLED") << "\n"
              << "========================================================================\n\n";

    auto stream_start_wall = Clock::now();
    std::vector<FrameMetrics> all_metrics;
    all_metrics.reserve(pcd_files.size());

    for (size_t f_idx = 0; f_idx < pcd_files.size(); ++f_idx) {
        FrameMetrics m;
        bool ok = process_single_frame(
            pcd_files[f_idx], static_cast<int>(f_idx),
            voxel_leaf_size, cluster_tolerance, min_cluster_size, max_cluster_size,
            ransac_distance_threshold,
            ransac_max_iters, ror_radius, ror_min_pts, seed, skip_ror,
            num_slabs, ground_normal_prior, min_ground_dot,
            /*progress_enabled=*/false, disable_disk, output_dir_base, m);
        if (!ok) continue;

        all_metrics.push_back(m);
        double fps = (m.compute_ms > 0.0) ? (1000.0 / m.compute_ms) : 0.0;
        std::cout << "[stream-intra] [" << num_slabs << "-Core] Frame " << std::setw(4) << f_idx << " (" << m.filename << ") | "
                  << "Compute: " << std::fixed << std::setprecision(2) << std::setw(6) << m.compute_ms << " ms ("
                  << std::setprecision(1) << std::setw(4) << fps << " FPS) | "
                  << "Clust: " << std::setw(5) << m.cluster_ms << " ms | "
                  << "Pts: " << std::setw(6) << m.raw_points << " -> " << std::setw(5) << m.downsampled_points << " | "
                  << "Obstacles: " << std::setw(5) << m.obstacle_points << " | "
                  << "Clusters: " << std::setw(2) << m.cluster_count << std::endl;
    }

    auto stream_end_wall = Clock::now();
    double total_wall_ms = std::chrono::duration<double, std::milli>(stream_end_wall - stream_start_wall).count();
    size_t count = all_metrics.size();
    if (count == 0) return 1;

    double sum_comp = 0, min_comp = 1e9, max_comp = 0;
    double sum_read = 0, sum_write = 0;
    double sum_vox = 0, sum_grid = 0, sum_ror = 0, sum_ransac = 0, sum_clust = 0;

    for (const auto& m : all_metrics) {
        sum_comp += m.compute_ms;
        min_comp = std::min(min_comp, m.compute_ms);
        max_comp = std::max(max_comp, m.compute_ms);
        sum_read += m.io_read_ms;
        sum_write += m.io_write_ms;
        sum_vox += m.voxel_ms;
        sum_grid += m.grid_build_ms;
        sum_ror += m.ror_ms;
        sum_ransac += m.ransac_ms;
        sum_clust += m.cluster_ms;
    }

    double avg_comp = sum_comp / count;
    double avg_read = sum_read / count;
    double avg_write = sum_write / count;
    double effective_fps = (avg_comp > 0.0) ? (1000.0 / avg_comp) : 0.0;
    double stream_wall_fps = (total_wall_ms > 0.0) ? (count * 1000.0 / total_wall_ms) : 0.0;

    std::cout << "\n========================================================================\n"
              << "  CONTINUOUS STREAM PERCEPTION BENCHMARK SUMMARY (INTRA-FRAME SLABS) (" << count << " frames)\n"
              << "========================================================================\n"
              << "  [Overall Stream Throughput & Wall-Clock Runtime]:\n"
              << "    • Total Wall-Clock Time   : " << std::fixed << std::setprecision(2) << total_wall_ms << " ms\n"
              << "    • Sustained Stream Rate   : " << std::setprecision(2) << stream_wall_fps << " FPS (Full FPS w/ Disk I/O & Load)\n"
              << "------------------------------------------------------------------------\n"
              << "  [Pure Compute Pipeline Latency] (Excluding all Disk I/O):\n"
              << "    • Average Compute Latency : " << std::fixed << std::setprecision(2) << avg_comp << " ms\n"
              << "    • Min Compute Latency     : " << min_comp << " ms\n"
              << "    • Max Compute Latency     : " << max_comp << " ms\n"
              << "    • Per-Frame Compute Rate  : " << std::setprecision(2) << effective_fps << " FPS (Computational FPS)\n"
              << "------------------------------------------------------------------------\n"
              << "  [Compute Stage Breakdown (Average per Frame)]:\n"
              << "    1. Voxel Downsample (Radix): " << std::setw(6) << (sum_vox / count) << " ms ("
              << std::setprecision(1) << ((sum_vox / sum_comp) * 100.0) << "%)\n"
              << "    2. Fast Spatial Grid Build : " << std::setw(6) << (sum_grid / count) << " ms ("
              << std::setprecision(1) << ((sum_grid / sum_comp) * 100.0) << "%)\n"
              << "    3. " << num_slabs << "-Core Slab ROR         : " << std::setw(6) << (sum_ror / count) << " ms ("
              << std::setprecision(1) << ((sum_ror / sum_comp) * 100.0) << "%)\n"
              << "    4. RVV RANSAC Ground Fit   : " << std::setw(6) << (sum_ransac / count) << " ms ("
              << std::setprecision(1) << ((sum_ransac / sum_comp) * 100.0) << "%)\n"
              << "    5. " << num_slabs << "-Core Slab Clustering : " << std::setw(6) << (sum_clust / count) << " ms ("
              << std::setprecision(1) << ((sum_clust / sum_comp) * 100.0) << "%)\n"
              << "------------------------------------------------------------------------\n"
              << "  [Disk I/O Telemetry (Isolated from Compute)]:\n"
              << "    • Average PCD Load Time    : " << avg_read << " ms\n"
              << "    • Average Cluster Save     : " << avg_write << " ms\n"
              << "========================================================================\n";

    if (json_metrics) {
        std::cout << "[metrics_json] {"
                  << "\"total_wall_ms\": " << total_wall_ms << ", "
                  << "\"avg_compute_ms\": " << avg_comp << ", "
                  << "\"full_fps\": " << stream_wall_fps << ", "
                  << "\"compute_fps\": " << effective_fps << ", "
                  << "\"avg_load_ms\": " << avg_read << ", "
                  << "\"avg_downsample_ms\": " << (sum_vox / count) << ", "
                  << "\"avg_ransac_ms\": " << (sum_ransac / count) << ", "
                  << "\"avg_grid_ms\": " << (sum_grid / count) << ", "
                  << "\"avg_ror_ms\": " << (sum_ror / count) << ", "
                  << "\"avg_clustering_ms\": " << (sum_clust / count) << ", "
                  << "\"num_threads\": " << num_slabs << ", "
                  << "\"frames_count\": " << count
                  << "}" << std::endl;
    }

    return 0;
}
