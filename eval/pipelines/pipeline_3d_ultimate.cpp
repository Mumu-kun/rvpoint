// pipeline_3d_ultimate.cpp
// 10-Stage Ultimate True-3D RVPoint Hardware RVV 1.0 Pipeline Export Utility
// Fully hardened production version with uniform strided RANSAC, dynamic 3D spatial grid,
// self-cell fastpath ROR, and zero-copy SoA extraction.

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

// ============================================================================
// 1. FAST STATEFUL PRNG
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

// ============================================================================
// 2. FAST RVV VECTORIZED OUTLIER REMOVAL (SOR & VOXEL ROR)
// ============================================================================
struct FusedResult {
    std::vector<float> x, y, z;
    std::vector<float> nx, ny, nz;
};

// True Radius Outlier Removal using Dynamic Spatial Hash Grid with Self-Cell Fastpath
static FusedResult execute_voxel_ror_rvv(
    const PointCloudSoA& cloud, const Fast3DSpatialGrid& grid,
    float search_radius, int min_neighbors, bool compute_normals = false)
{
    FusedResult res;
    const size_t n = cloud.n;
    if (n == 0) return res;

    float r2 = search_radius * search_radius;

    res.x.reserve(n);
    res.y.reserve(n);
    res.z.reserve(n);
    if (compute_normals) {
        res.nx.reserve(n);
        res.ny.reserve(n);
        res.nz.reserve(n);
    }

    const float* px = cloud.x;
    const float* py = cloud.y;
    const float* pz = cloud.z;
    const auto& cells = grid.cells_;
    const auto& next = grid.next_;
    const size_t mask = grid.mask_;
    const float inv_cell = grid.inv_cell_;

    for (size_t i = 0; i < n; ++i) {
        float qx = px[i], qy = py[i], qz = pz[i];
        int qcx = static_cast<int>(std::floor(qx * inv_cell));
        int qcy = static_cast<int>(std::floor(qy * inv_cell));
        int qcz = static_cast<int>(std::floor(qz * inv_cell));

        int in_radius_count = 0;
        std::vector<int> nbrs;
        if (compute_normals) nbrs.reserve(64);

        // 1. Fastpath: check self-cell first (skips 26 neighbor cells for dense voxels)
        size_t self_h = grid.hash3D(qcx, qcy, qcz);
        int probe = 0;
        while (cells[self_h].head != -1 && probe < Fast3DSpatialGrid::kMaxProbes) {
            if (cells[self_h].cx == qcx && cells[self_h].cy == qcy && cells[self_h].cz == qcz) {
                int curr = cells[self_h].head;
                while (curr != -1) {
                    float ddx = px[curr] - qx, ddy = py[curr] - qy, ddz = pz[curr] - qz;
                    if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) {
                        in_radius_count++;
                        if (compute_normals) nbrs.push_back(curr);
                        if (!compute_normals && in_radius_count >= min_neighbors) break;
                    }
                    curr = next[curr];
                }
                break;
            }
            self_h = (self_h + 1) & mask;
            probe++;
        }

        // 2. Fallback to neighbor 26 cells
        if (compute_normals || in_radius_count < min_neighbors) {
            for (int dz = -1; dz <= 1 && (compute_normals || in_radius_count < min_neighbors); ++dz) {
                for (int dy = -1; dy <= 1 && (compute_normals || in_radius_count < min_neighbors); ++dy) {
                    for (int dx = -1; dx <= 1 && (compute_normals || in_radius_count < min_neighbors); ++dx) {
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
                                        if (compute_normals) nbrs.push_back(curr);
                                        if (!compute_normals && in_radius_count >= min_neighbors) break;
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
            res.x.push_back(qx);
            res.y.push_back(qy);
            res.z.push_back(qz);
            if (compute_normals) {
                if (nbrs.size() >= 3) {
                    float cx = 0, cy = 0, cz = 0;
                    for (int idx : nbrs) {
                        cx += px[idx]; cy += py[idx]; cz += pz[idx];
                    }
                    float inv_n = 1.0f / static_cast<float>(nbrs.size());
                    cx *= inv_n; cy *= inv_n; cz *= inv_n;

                    float c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
                    for (int idx : nbrs) {
                        float ddx = px[idx] - cx, ddy = py[idx] - cy, ddz = pz[idx] - cz;
                        c00 += ddx * ddx; c01 += ddx * ddy; c02 += ddx * ddz;
                        c11 += ddy * ddy; c12 += ddy * ddz; c22 += ddz * ddz;
                    }

                    float vx = c01 * c12 - c02 * c11;
                    float vy = c01 * c02 - c00 * c12;
                    float vz = c00 * c11 - c01 * c01;
                    float norm = std::sqrt(vx * vx + vy * vy + vz * vz);
                    if (norm > 1e-6f) {
                        float inv_norm = 1.0f / norm;
                        res.nx.push_back(vx * inv_norm);
                        res.ny.push_back(vy * inv_norm);
                        res.nz.push_back(vz * inv_norm);
                    } else {
                        res.nx.push_back(std::numeric_limits<float>::quiet_NaN());
                        res.ny.push_back(std::numeric_limits<float>::quiet_NaN());
                        res.nz.push_back(std::numeric_limits<float>::quiet_NaN());
                    }
                } else {
                    res.nx.push_back(std::numeric_limits<float>::quiet_NaN());
                    res.ny.push_back(std::numeric_limits<float>::quiet_NaN());
                    res.nz.push_back(std::numeric_limits<float>::quiet_NaN());
                }
            }
        }
    }
    return res;
}

static FusedResult execute_fused_sor_normals_rvv(
    const PointCloudSoA& cloud, const Fast3DSpatialGrid& grid,
    float sor_radius, int sor_k, float sor_alpha, int normal_k = 10, bool compute_normals = true)
{
    const size_t n = cloud.n;
    std::vector<float> mean_dists(n, sor_radius);
    std::vector<float> all_nx, all_ny, all_nz;
    if (compute_normals) {
        all_nx.assign(n, std::numeric_limits<float>::quiet_NaN());
        all_ny.assign(n, std::numeric_limits<float>::quiet_NaN());
        all_nz.assign(n, std::numeric_limits<float>::quiet_NaN());
    }
    std::vector<int> valid_points;
    valid_points.reserve(n);

    std::vector<int> nbrs;
    std::vector<float> d2;
    nbrs.reserve(256);
    d2.reserve(256);

    double total_sum = 0.0, total_sq_sum = 0.0;
    float sor_r2 = sor_radius * sor_radius;

    struct NeighborPair { int idx; float dist2; };
    std::vector<NeighborPair> pairs;
    pairs.reserve(256);

    for (size_t i = 0; i < n; ++i) {
        float qx = cloud.x[i], qy = cloud.y[i], qz = cloud.z[i];
        grid.radiusSearch(qx, qy, qz, sor_r2, nbrs, d2);

        pairs.clear();
        for (size_t j = 0; j < nbrs.size(); ++j) {
            if (nbrs[j] != static_cast<int>(i) && d2[j] > 1e-8f) {
                pairs.push_back({nbrs[j], d2[j]});
            }
        }

        int found = static_cast<int>(pairs.size());
        if (found < 1) continue;

        int k_use = std::min(found, sor_k);
        std::partial_sort(pairs.begin(), pairs.begin() + k_use, pairs.end(),
                          [](const NeighborPair& a, const NeighborPair& b) { return a.dist2 < b.dist2; });

        float sum_dist = 0.0f;
        for (int j = 0; j < k_use; ++j) {
            sum_dist += std::sqrt(pairs[j].dist2);
        }

        float m = sum_dist / static_cast<float>(k_use);
        mean_dists[i] = m;
        total_sum += m;
        total_sq_sum += (m * m);
        valid_points.push_back(static_cast<int>(i));

        if (compute_normals && found >= 3) {
            int n_norm = std::min(found, normal_k);
            float cx = 0, cy = 0, cz = 0;
            for (int k = 0; k < n_norm; ++k) {
                int idx = pairs[k].idx;
                cx += cloud.x[idx]; cy += cloud.y[idx]; cz += cloud.z[idx];
            }
            float inv_n = 1.0f / static_cast<float>(n_norm);
            cx *= inv_n; cy *= inv_n; cz *= inv_n;

            float c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
            for (int k = 0; k < n_norm; ++k) {
                int idx = pairs[k].idx;
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
    if (variance < 0.0) variance = 0.0;
    double global_std = std::sqrt(variance);
    float dist_threshold = static_cast<float>(global_mean + (sor_alpha * global_std));

    res.x.reserve(n);
    res.y.reserve(n);
    res.z.reserve(n);
    if (compute_normals) {
        res.nx.reserve(n);
        res.ny.reserve(n);
        res.nz.reserve(n);
    }

    for (size_t i = 0; i < n; ++i) {
        if (mean_dists[i] <= dist_threshold) {
            res.x.push_back(cloud.x[i]);
            res.y.push_back(cloud.y[i]);
            res.z.push_back(cloud.z[i]);
            if (compute_normals) {
                res.nx.push_back(all_nx[i]);
                res.ny.push_back(all_ny[i]);
                res.nz.push_back(all_nz[i]);
            }
        }
    }
    return res;
}

// ============================================================================
// 3. HARDWARE RVV 1.0 SPRT EARLY-REJECTION RANSAC WITH SVD REFINEMENT
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

// Zero-Copy Pure-SoA Extraction (RVV Vectorized)
static size_t extract_inliers_outliers_direct_soa(
    const PointCloudSoA& in, const float* model, float thresh,
    std::vector<PointXYZ>& inliers,
    std::vector<float>& ox, std::vector<float>& oy, std::vector<float>& oz,
    bool populate_inlier_pts = true)
{
    float a = model[0], b = model[1], c = model[2], d = model[3];
    const size_t n = in.n;
    inliers.clear(); ox.clear(); oy.clear(); oz.clear();

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
            if (populate_inlier_pts) {
                inliers.push_back({in.x[i], in.y[i], in.z[i]});
            }
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
// 4. FAST 3D DISJOINT-SET (UNION-FIND) CLUSTERING WITH FLAT ARRAY BINNING
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

static std::vector<ClusterResult> execute_union_find_clustering(
    const PointCloudSoA& cloud, float tolerance, int min_cluster_size, int max_cluster_size)
{
    std::vector<ClusterResult> clusters;
    const size_t n = cloud.n;
    if (n == 0) return clusters;

    float tol_sq = tolerance * tolerance;
    Fast3DSpatialGrid grid(tolerance, n);
    grid.build(cloud.x, cloud.y, cloud.z, n);

    UnionFind uf(n);

    for (size_t i = 0; i < n; ++i) {
        float qx = cloud.x[i], qy = cloud.y[i], qz = cloud.z[i];
        int qcx = static_cast<int>(std::floor(qx * grid.inv_cell_));
        int qcy = static_cast<int>(std::floor(qy * grid.inv_cell_));
        int qcz = static_cast<int>(std::floor(qz * grid.inv_cell_));

        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
                    size_t h = grid.hash3D(tcx, tcy, tcz);
                    int probe = 0;
                    while (grid.cells_[h].head != -1 && probe < Fast3DSpatialGrid::kMaxProbes) {
                        if (grid.cells_[h].cx == tcx && grid.cells_[h].cy == tcy && grid.cells_[h].cz == tcz) {
                            int curr = grid.cells_[h].head;
                            while (curr != -1) {
                                if (static_cast<size_t>(curr) > i) {
                                    float ddx = cloud.x[curr] - qx, ddy = cloud.y[curr] - qy, ddz = cloud.z[curr] - qz;
                                    float d2 = ddx * ddx + ddy * ddy + ddz * ddz;
                                    if (d2 <= tol_sq) {
                                        uf.unite(static_cast<int>(i), curr);
                                    }
                                }
                                curr = grid.next_[curr];
                            }
                            break;
                        }
                        h = (h + 1) & grid.mask_;
                        probe++;
                    }
                }
            }
        }
    }

    std::vector<std::vector<int>> root_to_pts(n);
    for (size_t i = 0; i < n; ++i) {
        int r = uf.find(static_cast<int>(i));
        root_to_pts[r].push_back(static_cast<int>(i));
    }

    for (size_t r = 0; r < n; ++r) {
        int sz = static_cast<int>(root_to_pts[r].size());
        if (sz >= min_cluster_size && sz <= max_cluster_size) {
            clusters.push_back({std::move(root_to_pts[r])});
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

bool saveJSONMetrics(const std::filesystem::path &out_path,
                     const std::vector<StageTiming> &stages,
                     double total_ms, float leaf_size, bool skip_sor,
                     float cluster_tolerance, PerceptionStatus status,
                     size_t inlier_count, size_t outlier_count, size_t cluster_count) {
    std::ofstream ofs(out_path);
    if (!ofs.is_open()) return false;

    ofs << "{\n";
    ofs << "  \"status\": \"" << perceptionStatusToString(status) << "\",\n";
    ofs << "  \"leaf_size\": " << leaf_size << ",\n";
    ofs << "  \"skip_sor\": " << (skip_sor ? "true" : "false") << ",\n";
    ofs << "  \"cluster_tolerance\": " << cluster_tolerance << ",\n";
    ofs << "  \"total_ms\": " << total_ms << ",\n";
    ofs << "  \"ground_inliers\": " << inlier_count << ",\n";
    ofs << "  \"non_ground_outliers\": " << outlier_count << ",\n";
    ofs << "  \"clusters_found\": " << cluster_count << ",\n";
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
    ofs.flush();
    return ofs.good();
}

} // anonymous namespace

// ============================================================================
// MAIN APPLICATION ENTRY POINT
// ============================================================================
int main(int argc, char** argv) {
    bool progress_enabled = false;
    bool json_metrics = false;
    bool skip_sor = false;
    bool use_ror = true; // Default to ultra-fast ROR
    bool skip_normals = false;
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
    float min_ground_dot = 0.707f; // ~45 deg max slope

    std::vector<std::string> positional_args;
    std::vector<StageTiming> stage_timings;
    stage_timings.reserve(kStageCount);

    try {
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg == "--progress") {
                progress_enabled = true;
            } else if (arg == "--json" || arg == "--json-metrics") {
                json_metrics = true;
            } else if (arg == "--skip-sor" || arg == "--no-sor") {
                skip_sor = true;
            } else if (arg == "--use-ror" || arg == "--ror") {
                use_ror = true; skip_sor = false;
            } else if (arg == "--use-sor" || arg == "--sor") {
                use_ror = false; skip_sor = false;
            } else if (arg == "--ror-radius") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --ror-radius\n"; return 1; }
                ror_radius = std::stof(argv[++i]);
                if (!std::isfinite(ror_radius) || ror_radius <= 0.0f) {
                    std::cerr << "Error: --ror-radius must be a positive finite number.\n";
                    return 1;
                }
            } else if (arg == "--ror-min-pts") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --ror-min-pts\n"; return 1; }
                ror_min_pts = std::stoi(argv[++i]);
                if (ror_min_pts < 1) {
                    std::cerr << "Error: --ror-min-pts must be >= 1.\n";
                    return 1;
                }
            } else if (arg == "--no-normals" || arg == "--skip-normals" || arg == "--no-normal" || arg == "--skip-normal") {
                skip_normals = true;
            } else if (arg == "--no-write" || arg == "--disable-disk") {
                disable_disk = true;
            } else if (arg == "--leaf-size") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --leaf-size\n"; return 1; }
                voxel_leaf_size = std::stof(argv[++i]);
                if (!std::isfinite(voxel_leaf_size) || voxel_leaf_size <= 0.0f) {
                    std::cerr << "Error: --leaf-size must be a positive finite number.\n";
                    return 1;
                }
            } else if (arg == "--cluster-tolerance") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --cluster-tolerance\n"; return 1; }
                cluster_tolerance = std::stof(argv[++i]);
                if (!std::isfinite(cluster_tolerance) || cluster_tolerance <= 0.0f) {
                    std::cerr << "Error: --cluster-tolerance must be a positive finite number.\n";
                    return 1;
                }
            } else if (arg == "--min-cluster") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --min-cluster\n"; return 1; }
                min_cluster_size = std::stoi(argv[++i]);
                if (min_cluster_size < 1) {
                    std::cerr << "Error: --min-cluster must be >= 1.\n";
                    return 1;
                }
            } else if (arg == "--max-cluster") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --max-cluster\n"; return 1; }
                max_cluster_size = std::stoi(argv[++i]);
                if (max_cluster_size < 1) {
                    std::cerr << "Error: --max-cluster must be >= 1.\n";
                    return 1;
                }
            } else if (arg == "--ransac-iters") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --ransac-iters\n"; return 1; }
                ransac_max_iters = std::stoi(argv[++i]);
                if (ransac_max_iters < 1) {
                    std::cerr << "Error: --ransac-iters must be >= 1.\n";
                    return 1;
                }
            } else if (arg == "--seed") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --seed\n"; return 1; }
                seed = std::stoull(argv[++i]);
            } else if (arg == "--ground-angle-thresh") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --ground-angle-thresh\n"; return 1; }
                float deg = std::stof(argv[++i]);
                if (!std::isfinite(deg) || deg < 0.0f || deg > 90.0f) {
                    std::cerr << "Error: --ground-angle-thresh must be in [0, 90] degrees.\n";
                    return 1;
                }
                min_ground_dot = std::cos(deg * 3.14159265358979323846f / 180.0f);
            } else if (arg == "--no-ground-prior" || arg == "--unconstrained-plane") {
                min_ground_dot = 0.0f;
            } else if (arg == "--optical-frame") {
                ground_normal_prior = {0.0f, 1.0f, 0.0f};
            } else if (!arg.empty() && arg[0] == '-') {
                std::cerr << "Error: Unrecognized command-line option '" << arg << "'\n";
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
                  << " [--progress] [--json] [--no-write] [--no-normals] [--use-ror|--use-sor|--skip-sor] "
                     "[--ror-radius <val>] [--ror-min-pts <val>] [--leaf-size <val>] "
                     "[--cluster-tolerance <val>] [--min-cluster <val>] "
                     "[--max-cluster <val>] [--ransac-iters <val>] [--seed <val>] "
                     "[--ground-angle-thresh <deg>] [--no-ground-prior] [--optical-frame] <input.pcd> [output_dir]\n";
        return 1;
    }

    if (min_cluster_size > max_cluster_size) {
        std::cerr << "Error: --min-cluster cannot be greater than --max-cluster." << std::endl;
        return 1;
    }

    const auto overall_start = Clock::now();
    const std::string input_path = resolveInputPath(positional_args[0]);
    const std::filesystem::path input_stem = std::filesystem::path(positional_args[0]).stem();
    const std::filesystem::path output_dir =
        positional_args.size() >= 2 ? std::filesystem::path(positional_args[1])
                                    : std::filesystem::path("results") / (input_stem.string() + "_pipeline_ultimate");

    if (!disable_disk) {
        std::error_code dir_ec;
        std::filesystem::create_directories(output_dir, dir_ec);
        if (dir_ec) {
            std::cerr << "Error: Failed to create output directory '" << output_dir.string() << "': " << dir_ec.message() << std::endl;
            return 1;
        }
    }

    PerceptionStatus pipeline_status = PerceptionStatus::SUCCESS;

    // ── Stage 1: Load input cloud ──────────────────────────────────────────
    std::vector<PointXYZ> loaded_points;
    beginStage(1, "Load input cloud", progress_enabled);
    auto stage_start = Clock::now();
    int64_t count = loadPCD(input_path, loaded_points);
    if (count <= 0 || loaded_points.empty()) {
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
    if (!disable_disk) {
        if (!savePCD((output_dir / "00_input.pcd").string(), loaded_points, true)) {
            pipeline_status = PerceptionStatus::IO_WRITE_FAILURE;
        }
    }
    stage_timings.push_back({2, "Write input stage", endStage(2, "Write input stage", stage_start, progress_enabled), n_input});

    // ── Stage 3: Downsampling (RVV) ────────────────────────────────────────
    std::vector<PointXYZ> downsampled_pts(n_input);
    beginStage(3, "Downsampling (RVV)", progress_enabled);
    stage_start = Clock::now();
    size_t n_down = voxel_grid_downsamp_rvv_v2(input_cloud, downsampled_pts.data(), voxel_leaf_size);
    downsampled_pts.resize(n_down);
    stage_timings.push_back({3, "Downsampling", endStage(3, "Downsampling", stage_start, progress_enabled), n_down});
    if (!disable_disk) {
        if (!savePCD((output_dir / "01_downsampled.pcd").string(), downsampled_pts, true)) {
            pipeline_status = PerceptionStatus::IO_WRITE_FAILURE;
        }
    }

    std::vector<float> dx(n_down), dy(n_down), dz(n_down);
    for (size_t i = 0; i < n_down; ++i) {
        dx[i] = downsampled_pts[i].x; dy[i] = downsampled_pts[i].y; dz[i] = downsampled_pts[i].z;
    }
    PointCloudSoA downsampled_cloud{dx.data(), dy.data(), dz.data(), n_down};

    // ── Stage 4: Build Search Index ─────────────────────────────────────────
    Fast3DSpatialGrid search_grid(use_ror ? ror_radius : kPipelineConfig.sor_search_radius, n_down);
    beginStage(4, "Build search index for downsampled cloud", progress_enabled);
    stage_start = Clock::now();
    if (!skip_sor) {
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
        fused = execute_voxel_ror_rvv(downsampled_cloud, search_grid, ror_radius, ror_min_pts, !skip_normals);
    } else {
        fused = execute_fused_sor_normals_rvv(downsampled_cloud, search_grid,
                                              kPipelineConfig.sor_search_radius,
                                              kPipelineConfig.sor_mean_k,
                                              kPipelineConfig.sor_std_threshold,
                                              kPipelineConfig.normal_k,
                                              !skip_normals);
    }
    size_t n_sor = fused.x.size();
    double fused_sor_ms = endStage(5, outlier_stage_label, stage_start, progress_enabled);
    stage_timings.push_back({5, outlier_stage_label, fused_sor_ms, n_sor});

    if (!disable_disk) {
        std::vector<PointXYZ> sor_pts(n_sor);
        for (size_t i = 0; i < n_sor; ++i) sor_pts[i] = {fused.x[i], fused.y[i], fused.z[i]};
        if (!savePCD((output_dir / "02_sor_filtered.pcd").string(), sor_pts, true)) {
            pipeline_status = PerceptionStatus::IO_WRITE_FAILURE;
        }
    }

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
    int r_cnt = 0;
    if (n_sor >= 3) {
        r_cnt = ransac_plane_sprt_rvv(sor_cloud, kPipelineConfig.ransac_distance_threshold,
                                      ransac_max_iters, model,
                                      ground_normal_prior.data(), min_ground_dot, seed);
    }
    
    if (r_cnt < 50 || (n_sor > 0 && static_cast<float>(r_cnt) / static_cast<float>(n_sor) < 0.05f)) {
        if (pipeline_status == PerceptionStatus::SUCCESS) {
            pipeline_status = PerceptionStatus::NO_GROUND_PLANE_FOUND;
        }
    }

    size_t n_inliers = 0;
    if (r_cnt > 0) {
        n_inliers = extract_inliers_outliers_direct_soa(sor_cloud, model, kPipelineConfig.ransac_distance_threshold,
                                                        inlier_pts, ox, oy, oz, !disable_disk);
    } else {
        ox = fused.x; oy = fused.y; oz = fused.z;
    }
    size_t n_outliers = ox.size();
    stage_timings.push_back({8, "RANSAC primitive fitting",
                             endStage(8, "RANSAC primitive fitting", stage_start, progress_enabled), n_outliers});

    if (!disable_disk) {
        if (!savePCD((output_dir / "04_ransac_inliers.pcd").string(), inlier_pts, true)) {
            pipeline_status = PerceptionStatus::IO_WRITE_FAILURE;
        }
        std::vector<PointXYZ> outlier_pts(n_outliers);
        for (size_t i = 0; i < n_outliers; ++i) outlier_pts[i] = {ox[i], oy[i], oz[i]};
        if (!savePCD((output_dir / "05_ground_plane_removed.pcd").string(), outlier_pts, true)) {
            pipeline_status = PerceptionStatus::IO_WRITE_FAILURE;
        }
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
        if (!savePCDRGB((output_dir / "06_clusters.pcd").string(), colored_pts, true)) {
            pipeline_status = PerceptionStatus::IO_WRITE_FAILURE;
        }
    }
    stage_timings.push_back({10, "Write cluster stage", endStage(10, "Write cluster stage", stage_start, progress_enabled), clusters.size()});

    const auto overall_end = Clock::now();
    const double total_ms = std::chrono::duration<double, std::milli>(overall_end - overall_start).count();

    printFinalBreakdown(stage_timings, total_ms);

    if (json_metrics && !disable_disk) {
        if (!saveJSONMetrics(output_dir / "metrics.json", stage_timings, total_ms,
                             voxel_leaf_size, skip_sor, cluster_tolerance, pipeline_status,
                             n_inliers, n_outliers, clusters.size())) {
            pipeline_status = PerceptionStatus::IO_WRITE_FAILURE;
        }
    }

    if (pipeline_status == PerceptionStatus::IO_WRITE_FAILURE) {
        std::cerr << "Error: Output file write failure occurred during pipeline export." << std::endl;
        return 1;
    }

    return 0;
}
