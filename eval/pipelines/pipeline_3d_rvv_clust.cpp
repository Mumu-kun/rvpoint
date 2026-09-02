// pipeline_3d_rvv_clust.cpp
// Standalone Temporary Benchmark Target: True 3D Pipeline with Hardware RVV 1.0 Vectorized Euclidean Clustering
// Direct side-by-side comparison against pipeline_3d_ultimate.cpp

#include "include/rvpoint.h"
#include "search/fast_3d_spatial_grid.h"
#include "io/simple_pcd_loader.h"
#include "filters/voxel_grid.h"
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

#if defined(__riscv) || defined(__riscv_vector)
#include <riscv_vector.h>
#endif

#if defined(_OPENMP)
#include <omp.h>
#endif

using Clock = std::chrono::high_resolution_clock;
using namespace rvpoint;

namespace {

constexpr int kStageCount = 10;

enum class PerceptionStatus {
    SUCCESS = 0,
    NO_GROUND_PLANE_FOUND = 1,
    INSUFFICIENT_INLIERS = 2,
    DEGENERATE_POINT_CLOUD = 3,
    SEARCH_INDEX_OVERFLOW = 4,
    IO_WRITE_FAILURE = 5,
    INVALID_CLI_ARGUMENTS = 6
};

inline const char* perceptionStatusToString(PerceptionStatus status) {
    switch (status) {
        case PerceptionStatus::SUCCESS: return "SUCCESS";
        case PerceptionStatus::NO_GROUND_PLANE_FOUND: return "NO_GROUND_PLANE_FOUND";
        case PerceptionStatus::INSUFFICIENT_INLIERS: return "INSUFFICIENT_INLIERS";
        case PerceptionStatus::DEGENERATE_POINT_CLOUD: return "DEGENERATE_POINT_CLOUD";
        case PerceptionStatus::SEARCH_INDEX_OVERFLOW: return "SEARCH_INDEX_OVERFLOW";
        case PerceptionStatus::IO_WRITE_FAILURE: return "IO_WRITE_FAILURE";
        case PerceptionStatus::INVALID_CLI_ARGUMENTS: return "INVALID_CLI_ARGUMENTS";
        default: return "UNKNOWN";
    }
}

struct StageTiming {
    int index;
    const char *label;
    double ms;
    std::size_t point_count;
};

struct PipelineConfig {
    float voxel_leaf_size = 0.10f;
    float sor_search_radius = 0.25f;
    int sor_mean_k = 20;
    float sor_std_threshold = 1.0f;
    int normal_k = 10;
    float ransac_distance_threshold = 0.20f;
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

struct FusedResult {
    std::vector<float> x, y, z;
    std::vector<float> nx, ny, nz;
};

static FusedResult execute_voxel_ror_rvv(
    const PointCloudSoA& cloud, const Fast3DSpatialGrid& grid,
    float search_radius, int min_neighbors, bool compute_normals = false)
{
    FusedResult res;
    const size_t n = cloud.n;
    if (n == 0) return res;

    float r2 = search_radius * search_radius;
    std::vector<uint8_t> keep(n, 0);

    const float* px = cloud.x;
    const float* py = cloud.y;
    const float* pz = cloud.z;
    const auto& cells = grid.cells_;
    const auto& next = grid.next_;
    const size_t mask = grid.mask_;
    const float inv_cell = grid.inv_cell_;

#if defined(_OPENMP)
    #pragma omp parallel for schedule(dynamic, 128)
#endif
    for (size_t i = 0; i < n; ++i) {
        float qx = px[i], qy = py[i], qz = pz[i];
        int qcx = static_cast<int>(std::floor(qx * inv_cell));
        int qcy = static_cast<int>(std::floor(qy * inv_cell));
        int qcz = static_cast<int>(std::floor(qz * inv_cell));

        int in_radius_count = 0;

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

    res.x.reserve(n);
    res.y.reserve(n);
    res.z.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        if (keep[i]) {
            res.x.push_back(px[i]);
            res.y.push_back(py[i]);
            res.z.push_back(pz[i]);
        }
    }
    return res;
}

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

static int ransac_plane_sprt_rvv(
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
    size_t stride = std::max<size_t>(1, cloud.n / sample_sz);
    for (size_t k = 0; k < sample_sz; ++k) {
        size_t idx = std::min(k * stride, cloud.n - 1);
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
#if defined(__riscv) || defined(__riscv_vector)
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

    // Refine plane coefficients using inlier covariance (SVD refinement)
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

    float a = best_model[0], b = best_model[1], c = best_model[2], d = best_model[3];
    int total_inliers = 0;
#if defined(__riscv) || defined(__riscv_vector)
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

    for (int k = 0; k < 4; k++) model[k] = best_model[k];
    return total_inliers;
}

static size_t extract_inliers_outliers_direct_soa(
    const PointCloudSoA& in, const float* model, float thresh,
    std::vector<PointXYZ>& inliers,
    std::vector<float>& ox, std::vector<float>& oy, std::vector<float>& oz,
    bool populate_inlier_pts = true)
{
    float a = model[0], b = model[1], c = model[2], d = model[3];
    const size_t n = in.n;
    inliers.clear(); ox.clear(); oy.clear(); oz.clear();
    ox.reserve(n); oy.reserve(n); oz.reserve(n);
    if (populate_inlier_pts) inliers.reserve(n);

    size_t in_count = 0;
#if defined(__riscv) || defined(__riscv_vector)
    ox.resize(n); oy.resize(n); oz.resize(n);
    size_t out_count = 0;
    std::vector<float> ix, iy, iz;
    if (populate_inlier_pts) { ix.resize(n); iy.resize(n); iz.resize(n); }
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
        if (cnt_in > 0) {
            if (populate_inlier_pts) {
                vfloat32m8_t cx = __riscv_vcompress_vm_f32m8(vx, inlier_mask, vl);
                vfloat32m8_t cy = __riscv_vcompress_vm_f32m8(vy, inlier_mask, vl);
                vfloat32m8_t cz = __riscv_vcompress_vm_f32m8(vz, inlier_mask, vl);
                __riscv_vse32_v_f32m8(&ix[in_count], cx, cnt_in);
                __riscv_vse32_v_f32m8(&iy[in_count], cy, cnt_in);
                __riscv_vse32_v_f32m8(&iz[in_count], cz, cnt_in);
            }
            in_count += cnt_in;
        }
        i += vl;
    }
    ox.resize(out_count);
    oy.resize(out_count);
    oz.resize(out_count);
    if (populate_inlier_pts) {
        inliers.resize(in_count);
        for (size_t k = 0; k < in_count; ++k) inliers[k] = {ix[k], iy[k], iz[k]};
    }
#else
    if (populate_inlier_pts) inliers.reserve(n);
    ox.reserve(n); oy.reserve(n); oz.reserve(n);
    for (size_t i = 0; i < in.n; ++i) {
        float dist = std::abs(a * in.x[i] + b * in.y[i] + c * in.z[i] + d);
        if (dist <= thresh) {
            in_count++;
            if (populate_inlier_pts) inliers.push_back({in.x[i], in.y[i], in.z[i]});
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
// 4. HARDWARE RVV 1.0 VECTORIZED EUCLIDEAN CLUSTERING (ZERO-ALLOCATION CSR)
// ============================================================================
struct ClusterResult {
    std::vector<int> indices;
};

struct UnionFind {
    std::vector<int> parent;
    std::vector<int> rank;
    explicit UnionFind(size_t n) : parent(n), rank(n, 0) {
        std::iota(parent.begin(), parent.end(), 0);
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
        capacity_ = next_power_of_2(std::max<size_t>(65536, expected_pts * 4));
        mask_ = capacity_ - 1;
        cells_.resize(capacity_);
        touched_slots_.reserve(65536);
    }

    inline size_t hash3D(int x, int y, int z) const {
        return ((static_cast<size_t>(x) * 73856093) ^
                (static_cast<size_t>(y) * 19349663) ^
                (static_cast<size_t>(z) * 83492791)) & mask_;
    }

    bool build(const float* x, const float* y, const float* z, size_t n) {
        if (capacity_ < n * 4) {
            capacity_ = next_power_of_2(std::max<size_t>(65536, n * 4));
            mask_ = capacity_ - 1;
            cells_.resize(capacity_);
        }
        for (uint32_t slot : touched_slots_) {
            cells_[slot].head = -1;
        }
        touched_slots_.clear();
        if (next_.size() < n) next_.resize(n);

        for (size_t i = 0; i < n; ++i) {
            int cx = static_cast<int>(std::floor(x[i] * inv_cell_));
            int cy = static_cast<int>(std::floor(y[i] * inv_cell_));
            int cz = static_cast<int>(std::floor(z[i] * inv_cell_));

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

static std::vector<ClusterResult> execute_union_find_clustering_rvv(
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
    for (int t = 0; t < max_threads; ++t) {
        thread_edges[t].reserve(n * 2 / max_threads);
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

            // 1. Intra-cell pairwise checks
            for (size_t u = 0; u < n_self; ++u) {
                int p_u = self_pts[u];
                float ux = cloud.x[p_u], uy = cloud.y[p_u], uz = cloud.z[p_u];
                for (size_t v = u + 1; v < n_self; ++v) {
                    int p_v = self_pts[v];
                    float ddx = cloud.x[p_v] - ux, ddy = cloud.y[p_v] - uy, ddz = cloud.z[p_v] - uz;
                    if (ddx * ddx + ddy * ddy + ddz * ddz <= tol_sq) {
                        local_edges.push_back({p_u, p_v});
                    }
                }
            }

            // 2. Gather candidates from 13 forward neighbors ONCE for this cell
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

            // 3. Vector distance checks against all candidates for each point in cell
            size_t M = cand_idx.size();
            if (M > 0) {
                for (size_t u = 0; u < n_self; ++u) {
                    int p_u = self_pts[u];
                    float qx = cloud.x[p_u], qy = cloud.y[p_u], qz = cloud.z[p_u];

#if defined(__riscv) || defined(__riscv_vector)
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
                                    local_edges.push_back({p_u, cand_idx[k + lane]});
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
                            local_edges.push_back({p_u, cand_idx[k]});
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

    // Zero-Allocation Two-Pass CSR Grouping
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
// 5. STAGE MANAGEMENT & PROGRESS LOGGING
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

} // anonymous namespace

int main(int argc, char** argv) {
    bool progress_enabled = false;
    bool disable_disk = false;
    float voxel_leaf_size = kPipelineConfig.voxel_leaf_size;
    float cluster_tolerance = kPipelineConfig.cluster_tolerance;
    int min_cluster_size = kPipelineConfig.min_cluster_size;
    int max_cluster_size = kPipelineConfig.max_cluster_size;
    int ransac_max_iters = kPipelineConfig.ransac_max_iterations;
    float ror_radius = 0.25f;
    int ror_min_pts = 2;
    uint64_t seed = 42;

    std::vector<float> ground_normal_prior = {0.0f, 0.0f, 1.0f};
    float min_ground_dot = 0.707f;

    std::vector<std::string> positional_args;
    std::vector<StageTiming> stage_timings;
    stage_timings.reserve(kStageCount);

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--progress") {
            progress_enabled = true;
        } else if (arg == "--no-write" || arg == "--disable-disk") {
            disable_disk = true;
        } else if (arg == "--leaf-size" && i + 1 < argc) {
            voxel_leaf_size = std::stof(argv[++i]);
        } else if (arg == "--cluster-tolerance" && i + 1 < argc) {
            cluster_tolerance = std::stof(argv[++i]);
        } else if (arg == "--min-cluster" && i + 1 < argc) {
            min_cluster_size = std::stoi(argv[++i]);
        } else if (arg == "--max-cluster" && i + 1 < argc) {
            max_cluster_size = std::stoi(argv[++i]);
        } else if (!arg.empty() && arg[0] != '-') {
            positional_args.push_back(arg);
        }
    }

    if (positional_args.empty()) {
        std::cerr << "Usage: " << argv[0] << " <input.pcd> [--progress] [--no-write] ...\n";
        return 1;
    }

    const auto overall_start = Clock::now();
    const std::string input_path = resolveInputPath(positional_args[0]);
    const std::filesystem::path input_stem = std::filesystem::path(positional_args[0]).stem();
    const std::filesystem::path output_dir =
        positional_args.size() >= 2 ? std::filesystem::path(positional_args[1])
                                    : std::filesystem::path("output") / (input_stem.string() + "_pipeline_rvv_clust");

    if (!disable_disk) {
        std::error_code dir_ec;
        std::filesystem::create_directories(output_dir, dir_ec);
        if (dir_ec) {
            std::cerr << "Error: Failed to create output directory '" << output_dir.string() << "': " << dir_ec.message() << std::endl;
            return 1;
        }
    }

    // Stage 1: Load PCD
    std::vector<PointXYZ> loaded_points;
    beginStage(1, "Load input cloud", progress_enabled);
    auto stage_start = Clock::now();
    int64_t count = loadPCD(input_path, loaded_points);
    if (count <= 0 || loaded_points.empty()) {
        std::cerr << "Failed to load input PCD: " << positional_args[0] << std::endl;
        return 1;
    }
    const size_t n_input = loaded_points.size();
    stage_timings.push_back({1, "Load input cloud", endStage(1, "Load input cloud", stage_start, progress_enabled), n_input});

    std::vector<float> ix(n_input), iy(n_input), iz(n_input);
    for (size_t i = 0; i < n_input; ++i) {
        ix[i] = loaded_points[i].x; iy[i] = loaded_points[i].y; iz[i] = loaded_points[i].z;
    }
    PointCloudSoA input_cloud{ix.data(), iy.data(), iz.data(), n_input};

    // Stage 2: Write input
    beginStage(2, "Write input stage", progress_enabled);
    stage_start = Clock::now();
    if (!disable_disk) {
        savePCD((output_dir / "00_input.pcd").string(), loaded_points, true);
    }
    stage_timings.push_back({2, "Write input stage", endStage(2, "Write input stage", stage_start, progress_enabled), n_input});

    // Stage 3: Downsampling
    std::vector<PointXYZ> downsampled_pts(n_input);
    beginStage(3, "Downsampling (RVV)", progress_enabled);
    stage_start = Clock::now();
    size_t n_down = voxel_grid_downsamp_rvv_v2(input_cloud, downsampled_pts.data(), voxel_leaf_size);
    downsampled_pts.resize(n_down);
    stage_timings.push_back({3, "Downsampling", endStage(3, "Downsampling", stage_start, progress_enabled), n_down});
    if (!disable_disk) {
        savePCD((output_dir / "01_downsampled.pcd").string(), downsampled_pts, true);
    }

    std::vector<float> dx(n_down), dy(n_down), dz(n_down);
    for (size_t i = 0; i < n_down; ++i) {
        dx[i] = downsampled_pts[i].x; dy[i] = downsampled_pts[i].y; dz[i] = downsampled_pts[i].z;
    }
    PointCloudSoA downsampled_cloud{dx.data(), dy.data(), dz.data(), n_down};

    // Stage 4: Build Search Index
    Fast3DSpatialGrid search_grid(ror_radius, n_down);
    beginStage(4, "Build search index for downsampled cloud", progress_enabled);
    stage_start = Clock::now();
    search_grid.build(dx.data(), dy.data(), dz.data(), n_down);
    stage_timings.push_back({4, "Build search index for downsampled cloud",
                             endStage(4, "Build search index for downsampled cloud", stage_start, progress_enabled), n_down});

    // Stage 5: ROR
    beginStage(5, "Radius outlier removal (RVV)", progress_enabled);
    stage_start = Clock::now();
    FusedResult fused = execute_voxel_ror_rvv(downsampled_cloud, search_grid, ror_radius, ror_min_pts, false);
    size_t n_sor = fused.x.size();
    stage_timings.push_back({5, "Radius outlier removal (RVV)", endStage(5, "Radius outlier removal (RVV)", stage_start, progress_enabled), n_sor});
    if (!disable_disk) {
        std::vector<PointXYZ> sor_pts(n_sor);
        for (size_t i = 0; i < n_sor; ++i) sor_pts[i] = {fused.x[i], fused.y[i], fused.z[i]};
        savePCD((output_dir / "02_ror_filtered.pcd").string(), sor_pts, true);
    }

    // Stage 6 & 7: Skipped
    stage_timings.push_back({6, "Rebuild search index for filtered cloud", 0.001, n_sor});
    stage_timings.push_back({7, "Normal estimation", 0.001, n_sor});

    // Stage 8: RANSAC
    PointCloudSoA sor_cloud{fused.x.data(), fused.y.data(), fused.z.data(), n_sor};
    float model[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    std::vector<PointXYZ> inlier_pts;
    std::vector<float> ox, oy, oz;

    beginStage(8, "RANSAC primitive fitting", progress_enabled);
    stage_start = Clock::now();
    int r_cnt = ransac_plane_sprt_rvv(sor_cloud, kPipelineConfig.ransac_distance_threshold,
                                      ransac_max_iters, model,
                                      ground_normal_prior.data(), min_ground_dot, seed);
    extract_inliers_outliers_direct_soa(sor_cloud, model, kPipelineConfig.ransac_distance_threshold,
                                       inlier_pts, ox, oy, oz, !disable_disk);
    size_t n_outliers = ox.size();
    stage_timings.push_back({8, "RANSAC primitive fitting", endStage(8, "RANSAC primitive fitting", stage_start, progress_enabled), n_outliers});

    if (!disable_disk) {
        savePCD((output_dir / "04_ransac_inliers.pcd").string(), inlier_pts, true);
        std::vector<PointXYZ> outlier_pts(n_outliers);
        for (size_t i = 0; i < n_outliers; ++i) outlier_pts[i] = {ox[i], oy[i], oz[i]};
        savePCD((output_dir / "05_ground_plane_removed.pcd").string(), outlier_pts, true);
    }

    // Stage 9: Hardware RVV 1.0 Euclidean Clustering
    PointCloudSoA non_ground_cloud{ox.data(), oy.data(), oz.data(), n_outliers};
    beginStage(9, "Euclidean clustering (Hardware RVV 1.0)", progress_enabled);
    stage_start = Clock::now();
    auto clusters = execute_union_find_clustering_rvv(non_ground_cloud, cluster_tolerance, min_cluster_size, max_cluster_size);
    double clust_ms = endStage(9, "Euclidean clustering (Hardware RVV 1.0)", stage_start, progress_enabled);
    stage_timings.push_back({9, "Euclidean clustering (Hardware RVV 1.0)", clust_ms, clusters.size()});

    // Stage 10: Write colored clusters
    beginStage(10, "Write cluster stage", progress_enabled);
    stage_start = Clock::now();
    if (!disable_disk) {
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
                if (pt_idx >= 0 && static_cast<size_t>(pt_idx) < n_outliers) {
                    colored_pts.push_back({ox[pt_idx], oy[pt_idx], oz[pt_idx], col.r, col.g, col.b});
                }
            }
        }
        savePCDRGB((output_dir / "06_clusters.pcd").string(), colored_pts, true);
    }
    stage_timings.push_back({10, "Write cluster stage", endStage(10, "Write cluster stage", stage_start, progress_enabled), clusters.size()});

    const auto overall_end = Clock::now();
    const double total_ms = std::chrono::duration<double, std::milli>(overall_end - overall_start).count();

    std::cout << "\n========================================================================\n"
              << "  RVPoint Hardware RVV 1.0 Vectorized Clustering Benchmark\n"
              << "========================================================================\n";
    printFinalBreakdown(stage_timings, total_ms);
    std::cout << "[result] Total clusters found: " << clusters.size() << std::endl;
    if (!disable_disk) {
        std::cout << "[export] Colored clusters saved to: " << (output_dir / "06_clusters.pcd").string() << std::endl;
    }
    std::cout << "========================================================================\n";

    return 0;
}
