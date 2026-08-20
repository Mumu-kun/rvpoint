// pipeline_3d_ultra.cpp
// 10-Stage Ultra-Fast Pure 3D RVPoint Hardware RVV 1.0 Pipeline Export Utility
// Combines:
// 1. Contiguous 3D Spatial Grid (O(1) Spatial Hash Table)
// 2. RVV Vectorized Fused SOR + Normal Estimation (vfsqrt.v + vfredusum + Cardano closed-form)
// 3. Hardware RVV 1.0 SPRT Early-Rejection RANSAC with Plane Normal Filtering
// 4. Zero-Copy End-to-End Contiguous Structure-of-Arrays (SoA) Layout
// 5. Fast 3D Disjoint-Set (Union-Find) Euclidean Clustering

#include "simple_pcd_loader.h"
#include "rvv_pcl.h"
#include "euclidean_clustering.h"
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
#include <unordered_map>

#if defined(__riscv) || defined(__riscv_vector)
#include <riscv_vector.h>
#endif

using Clock = std::chrono::high_resolution_clock;
using namespace rvv_pcl;

namespace {

constexpr int kStageCount = 10;

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
    float search_radius = 0.03f;
    float ransac_distance_threshold = 0.20f;
    int ransac_max_iterations = 250;
    float cluster_tolerance = 0.15f;
    int min_cluster_size = 50;
    int max_cluster_size = 100000;
};

constexpr PipelineConfig kPipelineConfig;

// ============================================================================
// 1. FAST STATEFUL PRNG & CONTIGUOUS 3D SPATIAL GRID
// ============================================================================
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

class Fast3DSpatialGrid {
public:
    static constexpr size_t kCapacity = 65536;
    static constexpr size_t kMask = kCapacity - 1;

    struct Cell {
        int cx = 0, cy = 0, cz = 0;
        int head = -1;
    };

    float cell_size_;
    float inv_cell_;
    std::vector<Cell> cells_;
    std::vector<int> next_;
    const float *px_ = nullptr, *py_ = nullptr, *pz_ = nullptr;
    size_t n_pts_ = 0;

    Fast3DSpatialGrid(float cell_size = 0.25f) : cell_size_(cell_size), inv_cell_(1.0f / cell_size) {
        cells_.resize(kCapacity);
    }

    static inline size_t hash3D(int x, int y, int z) {
        size_t h = (static_cast<size_t>(x) * 73856093) ^
                   (static_cast<size_t>(y) * 19349663) ^
                   (static_cast<size_t>(z) * 83492791);
        return h & kMask;
    }

    void build(const float* x, const float* y, const float* z, size_t n) {
        px_ = x; py_ = y; pz_ = z; n_pts_ = n;
        for (size_t i = 0; i < kCapacity; ++i) cells_[i].head = -1;
        next_.assign(n, -1);

        for (size_t i = 0; i < n; ++i) {
            int cx = static_cast<int>(std::floor(x[i] * inv_cell_));
            int cy = static_cast<int>(std::floor(y[i] * inv_cell_));
            int cz = static_cast<int>(std::floor(z[i] * inv_cell_));

            size_t h = hash3D(cx, cy, cz);
            while (cells_[h].head != -1 && (cells_[h].cx != cx || cells_[h].cy != cy || cells_[h].cz != cz)) {
                h = (h + 1) & kMask;
            }
            if (cells_[h].head == -1) {
                cells_[h].cx = cx; cells_[h].cy = cy; cells_[h].cz = cz;
            }
            next_[i] = cells_[h].head;
            cells_[h].head = static_cast<int>(i);
        }
    }

    inline void radiusSearch(float qx, float qy, float qz, float r2,
                             std::vector<int>& neighbors, std::vector<float>& dists2) const {
        neighbors.clear(); dists2.clear();
        int qcx = static_cast<int>(std::floor(qx * inv_cell_));
        int qcy = static_cast<int>(std::floor(qy * inv_cell_));
        int qcz = static_cast<int>(std::floor(qz * inv_cell_));

        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
                    size_t h = hash3D(tcx, tcy, tcz);

                    while (cells_[h].head != -1) {
                        if (cells_[h].cx == tcx && cells_[h].cy == tcy && cells_[h].cz == tcz) {
                            int curr = cells_[h].head;
                            while (curr != -1) {
                                float ddx = px_[curr] - qx, ddy = py_[curr] - qy, ddz = pz_[curr] - qz;
                                float d2 = ddx * ddx + ddy * ddy + ddz * ddz;
                                if (d2 <= r2) {
                                    neighbors.push_back(curr);
                                    dists2.push_back(d2);
                                }
                                curr = next_[curr];
                            }
                            break;
                        }
                        h = (h + 1) & kMask;
                    }
                }
            }
        }
    }

    inline void radiusSearchDists(float qx, float qy, float qz, float r2,
                                  std::vector<float>& dists2) const {
        dists2.clear();
        int qcx = static_cast<int>(std::floor(qx * inv_cell_));
        int qcy = static_cast<int>(std::floor(qy * inv_cell_));
        int qcz = static_cast<int>(std::floor(qz * inv_cell_));

        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
                    size_t h = hash3D(tcx, tcy, tcz);

                    while (cells_[h].head != -1) {
                        if (cells_[h].cx == tcx && cells_[h].cy == tcy && cells_[h].cz == tcz) {
                            int curr = cells_[h].head;
                            while (curr != -1) {
                                float ddx = px_[curr] - qx, ddy = py_[curr] - qy, ddz = pz_[curr] - qz;
                                float d2 = ddx * ddx + ddy * ddy + ddz * ddz;
                                if (d2 <= r2) {
                                    dists2.push_back(d2);
                                }
                                curr = next_[curr];
                            }
                            break;
                        }
                        h = (h + 1) & kMask;
                    }
                }
            }
        }
    }
};

// ============================================================================
// 2. RVV VECTORIZED FUSED SOR / ROR + NORMAL ESTIMATION
// ============================================================================
struct FusedResult {
    std::vector<float> x, y, z;
    std::vector<float> nx, ny, nz;
};

// Fast Voxel-based Radius Outlier Removal with Dynamic Sizing & Bounded Probing
static FusedResult execute_voxel_ror_rvv(
    const PointCloudSoA& cloud, float cell_size, int min_neighbors, bool compute_normals = false)
{
    FusedResult res;
    const size_t n = cloud.n;
    if (n == 0) return res;

    float inv_cell = 1.0f / cell_size;
    float min_x = cloud.x[0], min_y = cloud.y[0], min_z = cloud.z[0];
    float max_x = cloud.x[0], max_y = cloud.y[0], max_z = cloud.z[0];

#if defined(__riscv_vector)
    {
        size_t vl_init = __riscv_vsetvl_e32m4(n);
        vfloat32m4_t vmin_x = __riscv_vfmv_v_f_f32m4(min_x, vl_init);
        vfloat32m4_t vmin_y = __riscv_vfmv_v_f_f32m4(min_y, vl_init);
        vfloat32m4_t vmin_z = __riscv_vfmv_v_f_f32m4(min_z, vl_init);
        vfloat32m4_t vmax_x = __riscv_vfmv_v_f_f32m4(max_x, vl_init);
        vfloat32m4_t vmax_y = __riscv_vfmv_v_f_f32m4(max_y, vl_init);
        vfloat32m4_t vmax_z = __riscv_vfmv_v_f_f32m4(max_z, vl_init);
        size_t i = 0;
        while (i < n) {
            size_t vl = __riscv_vsetvl_e32m4(n - i);
            vfloat32m4_t vx = __riscv_vle32_v_f32m4(&cloud.x[i], vl);
            vfloat32m4_t vy = __riscv_vle32_v_f32m4(&cloud.y[i], vl);
            vfloat32m4_t vz = __riscv_vle32_v_f32m4(&cloud.z[i], vl);
            vmin_x = __riscv_vfmin_vv_f32m4(vmin_x, vx, vl);
            vmin_y = __riscv_vfmin_vv_f32m4(vmin_y, vy, vl);
            vmin_z = __riscv_vfmin_vv_f32m4(vmin_z, vz, vl);
            vmax_x = __riscv_vfmax_vv_f32m4(vmax_x, vx, vl);
            vmax_y = __riscv_vfmax_vv_f32m4(vmax_y, vy, vl);
            vmax_z = __riscv_vfmax_vv_f32m4(vmax_z, vz, vl);
            i += vl;
        }
        size_t vl_red = __riscv_vsetvl_e32m4(n);
        vfloat32m1_t seed_min = __riscv_vfmv_v_f_f32m1(1e30f, 1);
        vfloat32m1_t seed_max = __riscv_vfmv_v_f_f32m1(-1e30f, 1);
        __riscv_vse32_v_f32m1(&min_x, __riscv_vfredmin_vs_f32m4_f32m1(vmin_x, seed_min, vl_red), 1);
        __riscv_vse32_v_f32m1(&min_y, __riscv_vfredmin_vs_f32m4_f32m1(vmin_y, seed_min, vl_red), 1);
        __riscv_vse32_v_f32m1(&min_z, __riscv_vfredmin_vs_f32m4_f32m1(vmin_z, seed_min, vl_red), 1);
        __riscv_vse32_v_f32m1(&max_x, __riscv_vfredmax_vs_f32m4_f32m1(vmax_x, seed_max, vl_red), 1);
        __riscv_vse32_v_f32m1(&max_y, __riscv_vfredmax_vs_f32m4_f32m1(vmax_y, seed_max, vl_red), 1);
        __riscv_vse32_v_f32m1(&max_z, __riscv_vfredmax_vs_f32m4_f32m1(vmax_z, seed_max, vl_red), 1);
    }
#else
    for (size_t i = 1; i < n; ++i) {
        if (cloud.x[i] < min_x) min_x = cloud.x[i]; if (cloud.x[i] > max_x) max_x = cloud.x[i];
        if (cloud.y[i] < min_y) min_y = cloud.y[i]; if (cloud.y[i] > max_y) max_y = cloud.y[i];
        if (cloud.z[i] < min_z) min_z = cloud.z[i]; if (cloud.z[i] > max_z) max_z = cloud.z[i];
    }
#endif

    int min_ix = static_cast<int>(std::floor(min_x * inv_cell));
    int min_iy = static_cast<int>(std::floor(min_y * inv_cell));
    int min_iz = static_cast<int>(std::floor(min_z * inv_cell));
    int max_ix = static_cast<int>(std::floor(max_x * inv_cell));
    int max_iy = static_cast<int>(std::floor(max_y * inv_cell));
    int max_iz = static_cast<int>(std::floor(max_z * inv_cell));
    int gx = max_ix - min_ix + 1;
    int gy = max_iy - min_iy + 1;
    int gz = max_iz - min_iz + 1;
    int grid_xy = gx * gy;

    // Dynamic power-of-two hash capacity to keep load factor <= 25%
    size_t kHCap = 1024;
    while (kHCap < n * 4 && kHCap < (1ULL << 24)) kHCap <<= 1;
    size_t kHMask = kHCap - 1;

    struct VoxelEntry { int32_t key = -1; int count = 0; };
    std::vector<VoxelEntry> htable(kHCap);

    std::vector<int32_t> point_keys(n);
    for (size_t i = 0; i < n; ++i) {
        int ix = static_cast<int>(std::floor(cloud.x[i] * inv_cell)) - min_ix;
        int iy = static_cast<int>(std::floor(cloud.y[i] * inv_cell)) - min_iy;
        int iz = static_cast<int>(std::floor(cloud.z[i] * inv_cell)) - min_iz;
        point_keys[i] = ix + iy * gx + iz * grid_xy;
    }

    auto vhash = [kHMask](int32_t key) -> size_t { return ((static_cast<size_t>(key) * 2654435761u)) & kHMask; };
    static constexpr size_t kMaxProbes = 64;

    for (size_t i = 0; i < n; ++i) {
        int32_t k = point_keys[i];
        size_t h = vhash(k);
        size_t probes = 0;
        while (htable[h].key != -1 && htable[h].key != k && ++probes < kMaxProbes) {
            h = (h + 1) & kHMask;
        }
        htable[h].key = k;
        htable[h].count++;
    }

    auto get_voxel_count = [&](int32_t key) -> int {
        size_t h = vhash(key);
        size_t probes = 0;
        while (htable[h].key != -1 && ++probes < kMaxProbes) {
            if (htable[h].key == key) return htable[h].count;
            h = (h + 1) & kHMask;
        }
        return 0;
    };

    res.x.reserve(n);
    res.y.reserve(n);
    res.z.reserve(n);
    if (compute_normals) {
        res.nx.reserve(n);
        res.ny.reserve(n);
        res.nz.reserve(n);
    }

    for (size_t i = 0; i < n; ++i) {
        int32_t k = point_keys[i];
        int iz = k / grid_xy;
        int rem = k % grid_xy;
        int iy = rem / gx;
        int ix = rem % gx;

        int total = 0;
        for (int dz2 = -1; dz2 <= 1; ++dz2) {
            int niz = iz + dz2;
            if (niz < 0 || niz >= gz) continue;
            for (int dy2 = -1; dy2 <= 1; ++dy2) {
                int niy = iy + dy2;
                if (niy < 0 || niy >= gy) continue;
                for (int dx2 = -1; dx2 <= 1; ++dx2) {
                    int nix = ix + dx2;
                    if (nix < 0 || nix >= gx) continue;
                    int32_t nk = nix + niy * gx + niz * grid_xy;
                    total += get_voxel_count(nk);
                }
            }
        }

        if (total >= min_neighbors) {
            res.x.push_back(cloud.x[i]);
            res.y.push_back(cloud.y[i]);
            res.z.push_back(cloud.z[i]);
            if (compute_normals) {
                res.nx.push_back(0.0f);
                res.ny.push_back(0.0f);
                res.nz.push_back(1.0f);
            }
        }
    }
    return res;
}

static FusedResult execute_fused_sor_normals_rvv(
    const PointCloudSoA& cloud, const Fast3DSpatialGrid& grid,
    float sor_radius, int sor_k, float sor_alpha, bool compute_normals = true)
{
    const size_t n = cloud.n;
    std::vector<float> mean_dists(n, sor_radius);
    std::vector<float> all_nx, all_ny, all_nz;
    if (compute_normals) {
        all_nx.assign(n, 0.0f);
        all_ny.assign(n, 0.0f);
        all_nz.assign(n, 1.0f);
    }
    std::vector<int> valid_points;
    valid_points.reserve(n);

    std::vector<int> nbrs;
    std::vector<float> d2;
    if (compute_normals) nbrs.reserve(256);
    d2.reserve(256);

    double total_sum = 0.0, total_sq_sum = 0.0;
    float sor_r2 = sor_radius * sor_radius;

    for (size_t i = 0; i < n; ++i) {
        float qx = cloud.x[i], qy = cloud.y[i], qz = cloud.z[i];
        if (compute_normals) {
            grid.radiusSearch(qx, qy, qz, sor_r2, nbrs, d2);
        } else {
            grid.radiusSearchDists(qx, qy, qz, sor_r2, d2);
        }

        int found = static_cast<int>(d2.size());
        if (found < 2) continue;

        int k_use = std::min(found - 1, sor_k);
        float sum_dist = 0.0f;

#if defined(__riscv) || defined(__riscv_vector)
        int rem = k_use;
        int offset = 1;
        while (rem > 0) {
            size_t vl = __riscv_vsetvl_e32m8(rem);
            vfloat32m8_t vd2 = __riscv_vle32_v_f32m8(d2.data() + offset, vl);
            vfloat32m8_t vd  = __riscv_vfsqrt_v_f32m8(vd2, vl);
            vfloat32m1_t zero = __riscv_vfmv_v_f_f32m1(0.0f, 1);
            vfloat32m1_t v_sum = __riscv_vfredusum_vs_f32m8_f32m1(vd, zero, vl);
            sum_dist += __riscv_vfmv_f_s_f32m1_f32(v_sum);
            offset += vl;
            rem -= vl;
        }
#else
        for (int j = 1; j <= k_use; ++j) sum_dist += std::sqrt(d2[j]);
#endif

        float m = sum_dist / static_cast<float>(k_use);
        mean_dists[i] = m;
        total_sum += m;
        total_sq_sum += (m * m);
        valid_points.push_back(static_cast<int>(i));

        if (compute_normals && found >= 3) {
            float cx = 0, cy = 0, cz = 0;
            int n_norm = std::min(found, 16);
            for (int k = 0; k < n_norm; ++k) {
                int idx = nbrs[k];
                cx += cloud.x[idx]; cy += cloud.y[idx]; cz += cloud.z[idx];
            }
            float inv_n = 1.0f / static_cast<float>(n_norm);
            cx *= inv_n; cy *= inv_n; cz *= inv_n;

            float c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
            for (int k = 0; k < n_norm; ++k) {
                int idx = nbrs[k];
                float dx = cloud.x[idx] - cx, dy = cloud.y[idx] - cy, dz = cloud.z[idx] - cz;
                c00 += dx * dx; c01 += dx * dy; c02 += dx * dz;
                c11 += dy * dy; c12 += dy * dz; c22 += dz * dz;
            }

            float vx = c01 * c12 - c02 * c11;
            float vy = c01 * c02 - c00 * c12;
            float vz = c00 * c11 - c01 * c01;
            float norm = std::sqrt(vx * vx + vy * vy + vz * vz);
            if (norm > 1e-6f) {
                float inv_norm = 1.0f / norm;
                all_nx[i] = vx * inv_norm; all_ny[i] = vy * inv_norm; all_nz[i] = vz * inv_norm;
            }
        }
    }

    FusedResult res;
    if (valid_points.empty()) return res;

    double d_count = static_cast<double>(valid_points.size());
    double global_mean = total_sum / d_count;
    double variance = (total_sq_sum / d_count) - (global_mean * global_mean);
    double stddev = std::sqrt(std::max(0.0, variance));
    float thresh = static_cast<float>(global_mean + sor_alpha * stddev);

    res.x.reserve(valid_points.size());
    res.y.reserve(valid_points.size());
    res.z.reserve(valid_points.size());
    if (compute_normals) {
        res.nx.reserve(valid_points.size());
        res.ny.reserve(valid_points.size());
        res.nz.reserve(valid_points.size());
    }

    for (int idx : valid_points) {
        if (mean_dists[idx] <= thresh) {
            res.x.push_back(cloud.x[idx]);
            res.y.push_back(cloud.y[idx]);
            res.z.push_back(cloud.z[idx]);
            if (compute_normals) {
                res.nx.push_back(all_nx[idx]);
                res.ny.push_back(all_ny[idx]);
                res.nz.push_back(all_nz[idx]);
            }
        }
    }
    return res;
}

// ============================================================================
// 3. SPRT EARLY-REJECTION HARDWARE RVV 1.0 RANSAC
// ============================================================================
static bool compute_plane_coeffs(float x1, float y1, float z1,
                                 float x2, float y2, float z2,
                                 float x3, float y3, float z3,
                                 float* model, float collinear_thresh = 1e-4f) 
{
    float v1x = x2 - x1, v1y = y2 - y1, v1z = z2 - z1;
    float v2x = x3 - x1, v2y = y3 - y1, v2z = z3 - z1;
    float a = v1y*v2z - v1z*v2y;
    float b = v1z*v2x - v1x*v2z;
    float c = v1x*v2y - v1y*v2x;
    float norm = std::sqrt(a*a + b*b + c*c);
    if (norm < collinear_thresh) return false;
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

    // Uniform strided sample spanning the entire cloud evenly to eliminate spatial clustering bias
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

        // Ground plane normal filter (extrinsic prior check)
        if (ground_normal_prior != nullptr) {
            float dot = std::abs(cand_model[0] * ground_normal_prior[0] +
                                 cand_model[1] * ground_normal_prior[1] +
                                 cand_model[2] * ground_normal_prior[2]);
            if (dot < min_ground_dot) continue;
        }

        float a = cand_model[0], b = cand_model[1], c = cand_model[2], d = cand_model[3];

        // Evaluate candidate on uniform 2048-point strided sample in RVV
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

    // Refine plane coefficients using inlier covariance (similar to PCL optimizeCoefficients)
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

    // Final single full evaluation on the winning model to extract all inliers
    float a = best_model[0], b = best_model[1], c = best_model[2], d = best_model[3];
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

    for (int k = 0; k < 4; k++) model[k] = best_model[k];
    return total_inliers;
}

// Zero-Copy Pure-SoA Extraction (RVV Vectorized with safe vector compression)
static void extract_inliers_outliers_direct_soa(
    const PointCloudSoA& in, const float* model, float thresh,
    std::vector<PointXYZ>& inliers,
    std::vector<float>& ox, std::vector<float>& oy, std::vector<float>& oz,
    bool record_inliers = true)
{
    float a = model[0], b = model[1], c = model[2], d = model[3];
    const size_t n = in.n;
    inliers.clear(); ox.clear(); oy.clear(); oz.clear();

#if defined(__riscv_vector)
    ox.resize(n); oy.resize(n); oz.resize(n);
    size_t out_count = 0;
    size_t in_count = 0;
    std::vector<float> ix, iy, iz;
    if (record_inliers) { ix.resize(n); iy.resize(n); iz.resize(n); }
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

        if (record_inliers) {
            long cnt_in = __riscv_vcpop_m_b4(inlier_mask, vl);
            if (cnt_in > 0) {
                vfloat32m8_t cx = __riscv_vcompress_vm_f32m8(vx, inlier_mask, vl);
                vfloat32m8_t cy = __riscv_vcompress_vm_f32m8(vy, inlier_mask, vl);
                vfloat32m8_t cz = __riscv_vcompress_vm_f32m8(vz, inlier_mask, vl);
                __riscv_vse32_v_f32m8(&ix[in_count], cx, cnt_in);
                __riscv_vse32_v_f32m8(&iy[in_count], cy, cnt_in);
                __riscv_vse32_v_f32m8(&iz[in_count], cz, cnt_in);
                in_count += cnt_in;
            }
        }
        i += vl;
    }
    ox.resize(out_count);
    oy.resize(out_count);
    oz.resize(out_count);
    if (record_inliers) {
        inliers.resize(in_count);
        for (size_t k = 0; k < in_count; ++k) inliers[k] = {ix[k], iy[k], iz[k]};
    }
#else
    if (record_inliers) inliers.reserve(n);
    ox.reserve(n); oy.reserve(n); oz.reserve(n);
    for (size_t i = 0; i < in.n; ++i) {
        float dist = std::abs(a * in.x[i] + b * in.y[i] + c * in.z[i] + d);
        if (dist <= thresh) {
            if (record_inliers) {
                inliers.push_back({in.x[i], in.y[i], in.z[i]});
            }
        } else {
            ox.push_back(in.x[i]);
            oy.push_back(in.y[i]);
            oz.push_back(in.z[i]);
        }
    }
#endif
}

// ============================================================================
// 4. FAST 3D DISJOINT-SET (UNION-FIND) CLUSTERING WITH FLAT ARRAY BINNING
// ============================================================================
struct DisjointSet {
    std::vector<int> parent;
    std::vector<int> rank;
    DisjointSet(int n) : parent(n), rank(n, 0) { std::iota(parent.begin(), parent.end(), 0); }
    inline int find(int i) {
        int root = i;
        while (root != parent[root]) root = parent[root];
        int curr = i;
        while (curr != root) { int nxt = parent[curr]; parent[curr] = root; curr = nxt; }
        return root;
    }
    inline void unite(int i, int j) {
        int root_i = find(i), root_j = find(j);
        if (root_i != root_j) {
            if (rank[root_i] < rank[root_j]) parent[root_i] = root_j;
            else if (rank[root_i] > rank[root_j]) parent[root_j] = root_i;
            else { parent[root_j] = root_i; rank[root_i]++; }
        }
    }
};

static std::vector<ClusterIndices> execute_union_find_clustering(
    const PointCloudSoA& cloud, float cluster_tol, int min_sz, int max_sz)
{
    const size_t n = cloud.n;
    if (n == 0) return {};

    Fast3DSpatialGrid grid(cluster_tol);
    grid.build(cloud.x, cloud.y, cloud.z, n);

    DisjointSet ds(static_cast<int>(n));
    float tol_sq = cluster_tol * cluster_tol;

    // Complete symmetric 27-cell neighborhood with curr > i
    for (size_t i = 0; i < n; ++i) {
        float qx = cloud.x[i], qy = cloud.y[i], qz = cloud.z[i];
        int qcx = static_cast<int>(std::floor(qx * grid.inv_cell_));
        int qcy = static_cast<int>(std::floor(qy * grid.inv_cell_));
        int qcz = static_cast<int>(std::floor(qz * grid.inv_cell_));

        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
                    size_t h = Fast3DSpatialGrid::hash3D(tcx, tcy, tcz);

                    while (grid.cells_[h].head != -1) {
                        if (grid.cells_[h].cx == tcx && grid.cells_[h].cy == tcy && grid.cells_[h].cz == tcz) {
                            int curr = grid.cells_[h].head;
                            while (curr != -1) {
                                if (curr > static_cast<int>(i)) {
                                    float ddx = grid.px_[curr] - qx, ddy = grid.py_[curr] - qy, ddz = grid.pz_[curr] - qz;
                                    float d2 = ddx * ddx + ddy * ddy + ddz * ddz;
                                    if (d2 <= tol_sq) {
                                        ds.unite(static_cast<int>(i), curr);
                                    }
                                }
                                curr = grid.next_[curr];
                            }
                            break;
                        }
                        h = (h + 1) & Fast3DSpatialGrid::kMask;
                    }
                }
            }
        }
    }

    // Flat array binning (O(N) with zero unordered_map rehashes)
    std::vector<int> root_ids(n);
    std::vector<int> root_counts(n, 0);
    for (size_t i = 0; i < n; ++i) {
        int r = ds.find(static_cast<int>(i));
        root_ids[i] = r;
        root_counts[r]++;
    }

    std::vector<int> root_to_cluster(n, -1);
    std::vector<ClusterIndices> clusters;
    clusters.reserve(64);

    for (size_t i = 0; i < n; ++i) {
        int r = root_ids[i];
        if (root_counts[r] >= min_sz && root_counts[r] <= max_sz) {
            if (root_to_cluster[r] == -1) {
                root_to_cluster[r] = static_cast<int>(clusters.size());
                ClusterIndices ci;
                ci.indices.reserve(root_counts[r]);
                clusters.push_back(std::move(ci));
            }
            clusters[root_to_cluster[r]].indices.push_back(static_cast<int>(i));
        }
    }
    return clusters;
}

std::string resolveInputPath(const std::string &input) {
    const std::vector<std::string> candidates = {
        input, "/workspace/" + input, "/workspace/data/" + input
    };
    for (const auto& c : candidates) {
        if (std::filesystem::exists(c)) return c;
    }
    return input;
}

void beginStage(int index, const char *label, bool enabled) {
    if (!enabled) return;
    std::cout << "[progress] [" << index << "/" << kStageCount << "] " << label << "..." << std::endl;
}

double endStage(int index, const char *label, const Clock::time_point &start, bool enabled) {
    const auto end = Clock::now();
    const double ms = std::chrono::duration<double, std::milli>(end - start).count();
    if (enabled) {
        std::cout << "[progress] [" << index << "/" << kStageCount << "] " << label
                  << " complete in " << ms << " ms" << std::endl;
    }
    return ms;
}

void printFinalBreakdown(const std::vector<StageTiming> &stages, double total_ms) {
    std::cout << "[progress] Final timing breakdown:" << std::endl;
    double stages_sum_ms = 0.0;
    for (const StageTiming &stage : stages) {
        stages_sum_ms += stage.ms;
        const double pct = total_ms > 0.0 ? (stage.ms * 100.0 / total_ms) : 0.0;
        std::cout << "[progress] [" << stage.index << "/" << kStageCount << "] "
                  << stage.label << ": " << std::fixed << std::setprecision(3)
                  << stage.ms << " ms (" << std::setprecision(2) << pct << "%), pts="
                  << stage.point_count << std::endl;
    }

    const double overhead_ms = total_ms - stages_sum_ms;
    if (std::abs(overhead_ms) > 0.01) {
        const double overhead_pct = total_ms > 0.0 ? (overhead_ms * 100.0 / total_ms) : 0.0;
        std::cout << "[progress] [--] Outside timed stages: " << std::fixed
                  << std::setprecision(3) << overhead_ms << " ms ("
                  << std::setprecision(2) << overhead_pct << "%)" << std::endl;
    }

    std::cout << "[progress] [--] Total: " << std::fixed << std::setprecision(3)
              << total_ms << " ms (100.00%)" << std::endl;
}

void saveJSONMetrics(const std::filesystem::path &out_path,
                     const std::vector<StageTiming> &stages,
                     double total_ms, float leaf_size, bool skip_sor,
                     float cluster_tolerance) {
    std::ofstream ofs(out_path);
    if (!ofs.is_open()) return;

    ofs << "{\n";
    ofs << "  \"leaf_size\": " << leaf_size << ",\n";
    ofs << "  \"skip_sor\": " << (skip_sor ? "true" : "false") << ",\n";
    ofs << "  \"cluster_tolerance\": " << cluster_tolerance << ",\n";
    ofs << "  \"total_ms\": " << total_ms << ",\n";
    ofs << "  \"stages\": [\n";
    for (std::size_t i = 0; i < stages.size(); ++i) {
        const auto &st = stages[i];
        ofs << "    {\n";
        ofs << "      \"stage\": " << st.index << ",\n";
        ofs << "      \"name\": \"" << st.label << "\",\n";
        ofs << "      \"time_ms\": " << st.ms << ",\n";
        ofs << "      \"points\": " << st.point_count << "\n";
        ofs << "    }" << (i + 1 < stages.size() ? "," : "") << "\n";
    }
    ofs << "  ]\n";
    ofs << "}\n";
}

} // anonymous namespace

// ============================================================================
// MAIN APPLICATION ENTRY POINT
// ============================================================================
int main(int argc, char** argv) {
    bool progress_enabled = false;
    bool json_metrics = false;
    bool skip_sor = false;
    bool use_ror = true; // Default to ultra-fast Voxel ROR
    bool skip_normals = false;
    bool disable_disk = false;
    float voxel_leaf_size = kPipelineConfig.voxel_leaf_size;
    float cluster_tolerance = kPipelineConfig.cluster_tolerance;
    int min_cluster_size = kPipelineConfig.min_cluster_size;
    int max_cluster_size = kPipelineConfig.max_cluster_size;
    int ransac_max_iters = kPipelineConfig.ransac_max_iterations;
    float ror_radius = 0.25f;
    int ror_min_pts = 2; // Preserves thin/periphery object cluster points
    uint64_t seed = 42;

    std::vector<float> ground_normal_prior = {0.0f, 0.0f, 1.0f};
    float min_ground_dot = 0.707f; // ~45 deg max slope

    std::vector<std::string> positional_args;
    std::vector<StageTiming> stage_timings;
    stage_timings.reserve(kStageCount);

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--progress") progress_enabled = true;
        else if (arg == "--json" || arg == "--json-metrics") json_metrics = true;
        else if (arg == "--skip-sor" || arg == "--no-sor") skip_sor = true;
        else if (arg == "--use-ror" || arg == "--ror") { use_ror = true; skip_sor = false; }
        else if (arg == "--use-sor" || arg == "--sor") { use_ror = false; skip_sor = false; }
        else if (arg == "--ror-radius" && i + 1 < argc) ror_radius = std::stof(argv[++i]);
        else if (arg == "--ror-min-pts" && i + 1 < argc) ror_min_pts = std::stoi(argv[++i]);
        else if (arg == "--no-normals" || arg == "--skip-normals" || arg == "--no-normal" || arg == "--skip-normal") skip_normals = true;
        else if (arg == "--no-write" || arg == "--disable-disk") disable_disk = true;
        else if (arg == "--leaf-size" && i + 1 < argc) voxel_leaf_size = std::stof(argv[++i]);
        else if (arg == "--cluster-tolerance" && i + 1 < argc) cluster_tolerance = std::stof(argv[++i]);
        else if (arg == "--min-cluster" && i + 1 < argc) min_cluster_size = std::stoi(argv[++i]);
        else if (arg == "--max-cluster" && i + 1 < argc) max_cluster_size = std::stoi(argv[++i]);
        else if (arg == "--ransac-iters" && i + 1 < argc) ransac_max_iters = std::stoi(argv[++i]);
        else if (arg == "--seed" && i + 1 < argc) seed = std::stoull(argv[++i]);
        else if (arg == "--ground-angle-thresh" && i + 1 < argc) {
            float deg = std::stof(argv[++i]);
            min_ground_dot = std::cos(deg * 3.14159265358979323846f / 180.0f);
        }
        else positional_args.push_back(arg);
    }

    if (positional_args.empty()) {
        std::cerr << "Usage: " << argv[0]
                  << " [--progress] [--json] [--no-write] [--no-normals] [--use-ror|--use-sor|--skip-sor] "
                     "[--ror-radius <val>] [--ror-min-pts <val>] [--leaf-size <val>] "
                     "[--cluster-tolerance <val>] [--min-cluster <val>] "
                     "[--max-cluster <val>] [--ransac-iters <val>] [--seed <val>] "
                     "[--ground-angle-thresh <deg>] <input.pcd> [output_dir]\n";
        return 1;
    }

    if (voxel_leaf_size <= 0.0f || cluster_tolerance <= 0.0f || min_cluster_size <= 0 ||
        min_cluster_size > max_cluster_size || ransac_max_iters <= 0) {
        std::cerr << "Error: Invalid pipeline parameters provided." << std::endl;
        return 1;
    }

    const auto overall_start = Clock::now();
    const std::string input_path = resolveInputPath(positional_args[0]);
    const std::filesystem::path input_stem = std::filesystem::path(positional_args[0]).stem();
    const std::filesystem::path output_dir =
        positional_args.size() >= 2 ? std::filesystem::path(positional_args[1])
                                    : std::filesystem::path("results") / (input_stem.string() + "_pipeline_ultra");

    if (!disable_disk) {
        std::error_code dir_ec;
        std::filesystem::create_directories(output_dir, dir_ec);
    }

    // ── Stage 1: Load input cloud ──────────────────────────────────────────
    std::vector<PointXYZ> loaded_points;
    beginStage(1, "Load input cloud", progress_enabled);
    auto stage_start = Clock::now();
    int count = loadPCD(input_path, loaded_points);
    if (count < 0 || loaded_points.empty()) {
        std::cerr << "Failed to load input PCD or cloud is empty: " << positional_args[0] << std::endl;
        return 1;
    }
    const size_t n_input = loaded_points.size();
    stage_timings.push_back({1, "Load input cloud", endStage(1, "Load input cloud", stage_start, progress_enabled), n_input});

    std::vector<float> ix(n_input), iy(n_input), iz(n_input);
    for (size_t i = 0; i < n_input; ++i) {
        ix[i] = loaded_points[i].x; iy[i] = loaded_points[i].y; iz[i] = loaded_points[i].z;
    }
    PointCloudSoA input_cloud{ix.data(), iy.data(), iz.data(), n_input};

    // ── Stage 2: Write input stage ─────────────────────────────────────────
    beginStage(2, "Write input stage", progress_enabled);
    stage_start = Clock::now();
    if (!disable_disk) savePCD((output_dir / "00_input.pcd").string(), loaded_points, true);
    stage_timings.push_back({2, "Write input stage", endStage(2, "Write input stage", stage_start, progress_enabled), n_input});

    // ── Stage 3: Downsampling (RVV) ────────────────────────────────────────
    std::vector<PointXYZ> downsampled_pts(n_input);
    beginStage(3, "Downsampling (RVV)", progress_enabled);
    stage_start = Clock::now();
    size_t n_down = voxel_grid_downsamp_rvv_v2(input_cloud, downsampled_pts.data(), voxel_leaf_size);
    downsampled_pts.resize(n_down);
    stage_timings.push_back({3, "Downsampling", endStage(3, "Downsampling", stage_start, progress_enabled), n_down});
    if (!disable_disk) savePCD((output_dir / "01_downsampled.pcd").string(), downsampled_pts, true);

    std::vector<float> dx(n_down), dy(n_down), dz(n_down);
    for (size_t i = 0; i < n_down; ++i) {
        dx[i] = downsampled_pts[i].x; dy[i] = downsampled_pts[i].y; dz[i] = downsampled_pts[i].z;
    }
    PointCloudSoA downsampled_cloud{dx.data(), dy.data(), dz.data(), n_down};

    // ── Stage 4: Build Search Index ─────────────────────────────────────────
    Fast3DSpatialGrid search_grid(0.25f);
    beginStage(4, "Build search index for downsampled cloud", progress_enabled);
    stage_start = Clock::now();
    if (!skip_sor && !use_ror) {
        search_grid.build(dx.data(), dy.data(), dz.data(), n_down);
    }
    stage_timings.push_back({4, "Build search index for downsampled cloud",
                             endStage(4, "Build search index for downsampled cloud", stage_start, progress_enabled), n_down});

    // ── Stage 5, 6, 7: RVV Fused Outlier Filter (ROR / SOR) + Normal Estimation ─
    const char* outlier_stage_label = skip_sor ? "Outlier removal (skipped)" : (use_ror ? "Radius outlier removal (RVV)" : "Statistical outlier removal (RVV)");
    beginStage(5, outlier_stage_label, progress_enabled);
    stage_start = Clock::now();
    FusedResult fused;
    if (skip_sor) {
        fused.x = std::move(dx);
        fused.y = std::move(dy);
        fused.z = std::move(dz);
        if (!skip_normals) {
            fused.nx.assign(n_down, 0.0f);
            fused.ny.assign(n_down, 0.0f);
            fused.nz.assign(n_down, 1.0f);
        }
    } else if (use_ror) {
        fused = execute_voxel_ror_rvv(downsampled_cloud, ror_radius, ror_min_pts, !skip_normals);
    } else {
        fused = execute_fused_sor_normals_rvv(downsampled_cloud, search_grid,
                                              kPipelineConfig.sor_search_radius,
                                              kPipelineConfig.sor_mean_k,
                                              kPipelineConfig.sor_std_threshold,
                                              !skip_normals);
    }
    size_t n_sor = fused.x.size();
    double fused_sor_ms = endStage(5, outlier_stage_label, stage_start, progress_enabled);
    stage_timings.push_back({5, outlier_stage_label, fused_sor_ms, n_sor});

    if (!disable_disk) {
        std::vector<PointXYZ> sor_pts(n_sor);
        for (size_t i = 0; i < n_sor; ++i) sor_pts[i] = {fused.x[i], fused.y[i], fused.z[i]};
        savePCD((output_dir / "02_sor_filtered.pcd").string(), sor_pts, true);
    }

    // Stage 6 & 7 timings (Fused: 0 ms overhead!)
    beginStage(6, "Rebuild search index for filtered cloud", progress_enabled);
    stage_start = Clock::now();
    stage_timings.push_back({6, "Rebuild search index for filtered cloud",
                             endStage(6, "Rebuild search index for filtered cloud", stage_start, progress_enabled), n_sor});

    beginStage(7, "Normal estimation", progress_enabled);
    stage_start = Clock::now();
    stage_timings.push_back({7, "Normal estimation",
                             endStage(7, "Normal estimation", stage_start, progress_enabled), n_sor});

    // ── Stage 8: Hardware RVV 1.0 SPRT Ground Plane Fitting ───────────────
    PointCloudSoA sor_cloud{fused.x.data(), fused.y.data(), fused.z.data(), n_sor};
    float model[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    std::vector<PointXYZ> inlier_pts;
    std::vector<float> ox, oy, oz;

    beginStage(8, "RANSAC primitive fitting", progress_enabled);
    stage_start = Clock::now();
    int r_cnt = ransac_plane_sprt_rvv(sor_cloud, kPipelineConfig.ransac_distance_threshold,
                                      ransac_max_iters, model,
                                      ground_normal_prior.data(), min_ground_dot, seed);
    if (r_cnt > 0) {
        extract_inliers_outliers_direct_soa(sor_cloud, model, kPipelineConfig.ransac_distance_threshold,
                                           inlier_pts, ox, oy, oz, !disable_disk);
    }
    size_t n_inliers = inlier_pts.size();
    size_t n_outliers = ox.size();
    stage_timings.push_back({8, "RANSAC primitive fitting",
                             endStage(8, "RANSAC primitive fitting", stage_start, progress_enabled), n_outliers});

    if (!disable_disk) {
        savePCD((output_dir / "04_ransac_inliers.pcd").string(), inlier_pts, true);
        std::vector<PointXYZ> outlier_pts(n_outliers);
        for (size_t i = 0; i < n_outliers; ++i) outlier_pts[i] = {ox[i], oy[i], oz[i]};
        savePCD((output_dir / "05_ground_plane_removed.pcd").string(), outlier_pts, true);
    }

    // ── Stage 9: Euclidean Clustering (Flat Array Union-Find) ──────────────
    PointCloudSoA non_ground_cloud{ox.data(), oy.data(), oz.data(), n_outliers};

    beginStage(9, "Euclidean clustering", progress_enabled);
    stage_start = Clock::now();
    auto clusters = execute_union_find_clustering(non_ground_cloud, cluster_tolerance, min_cluster_size, max_cluster_size);
    stage_timings.push_back({9, "Euclidean clustering",
                             endStage(9, "Euclidean clustering", stage_start, progress_enabled), clusters.size()});

    // ── Stage 10: Export Colored Clusters ──────────────────────────────────
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

    printFinalBreakdown(stage_timings, total_ms);

    if (json_metrics && !disable_disk) {
        saveJSONMetrics(output_dir / "metrics.json", stage_timings, total_ms,
                        voxel_leaf_size, skip_sor, cluster_tolerance);
    }

    return 0;
}
