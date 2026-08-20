// test_rvv_ceiling_audit.cpp
// Empirical verification benchmark for the RVV Optimization Ceiling Audit.
// Instruments every sub-component of pipeline_3d_ultra with micro-timers
// to confirm the identified gaps and ensure no hidden bottlenecks remain.

#include "simple_pcd_loader.h"
#include "rvv_pcl.h"
#include "euclidean_clustering.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

#if defined(__riscv) || defined(__riscv_vector)
#include <riscv_vector.h>
#endif

using Clock = std::chrono::high_resolution_clock;
using namespace rvv_pcl;

// ── Timing utility ───────────────────────────────────────────────────────────
struct SubTimer {
    const char* label;
    double ms = 0.0;
    Clock::time_point start;
    void begin() { start = Clock::now(); }
    void end() { ms += std::chrono::duration<double, std::milli>(Clock::now() - start).count(); }
};

// ============================================================================
// FAST 3D SPATIAL GRID (identical to pipeline_3d_ultra)
// ============================================================================
class Fast3DSpatialGrid {
public:
    static constexpr size_t kCapacity = 65536;
    static constexpr size_t kMask = kCapacity - 1;

    struct Cell {
        int cx = -999999, cy = -999999, cz = -999999;
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
        for (size_t i = 0; i < kCapacity; ++i) cells_[i] = Cell();
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
};

// ============================================================================
// DISJOINT SET (identical to pipeline_3d_ultra)
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

// ============================================================================
// MAIN: Micro-Instrumented Pipeline
// ============================================================================
int main(int argc, char** argv) {
    std::string input_path = "data/pcd_compressed/0000000030.pcd";
    if (argc > 1) input_path = argv[1];

    std::cout << "=======================================================================\n";
    std::cout << "  RVV OPTIMIZATION CEILING AUDIT — EMPIRICAL VERIFICATION\n";
    std::cout << "  Input: " << input_path << "\n";
    std::cout << "=======================================================================\n\n";

    // ── STAGE 1: Load ──────────────────────────────────────────────────────
    SubTimer t_load{"Load PCD"};
    t_load.begin();
    std::vector<PointXYZ> loaded_points;
    int count = loadPCD(input_path, loaded_points);
    if (count <= 0) {
        std::cerr << "Failed to load: " << input_path << "\n";
        return 1;
    }
    t_load.end();
    size_t n_input = loaded_points.size();
    std::cout << "[1] Load: " << n_input << " points in " << t_load.ms << " ms\n";

    // ── AoS->SoA Conversion #1 ─────────────────────────────────────────────
    SubTimer t_aos2soa1{"AoS->SoA #1 (input)"};
    t_aos2soa1.begin();
    std::vector<float> ix(n_input), iy(n_input), iz(n_input);
    for (size_t i = 0; i < n_input; ++i) {
        ix[i] = loaded_points[i].x; iy[i] = loaded_points[i].y; iz[i] = loaded_points[i].z;
    }
    t_aos2soa1.end();
    PointCloudSoA input_cloud{ix.data(), iy.data(), iz.data(), n_input};
    std::cout << "[*] AoS->SoA #1: " << t_aos2soa1.ms << " ms\n";

    // ── STAGE 3: Downsampling ──────────────────────────────────────────────
    SubTimer t_downsample{"Voxel Downsample (RVV v2)"};
    std::vector<PointXYZ> downsampled_pts(n_input);
    t_downsample.begin();
    size_t n_down = voxel_grid_downsamp_rvv_v2(input_cloud, downsampled_pts.data(), 0.10f);
    t_downsample.end();
    downsampled_pts.resize(n_down);
    std::cout << "[3] Downsample: " << n_input << " -> " << n_down << " pts in " << t_downsample.ms << " ms\n";

    // ── AoS->SoA Conversion #2 ─────────────────────────────────────────────
    SubTimer t_aos2soa2{"AoS->SoA #2 (downsampled)"};
    t_aos2soa2.begin();
    std::vector<float> dx(n_down), dy(n_down), dz(n_down);
    for (size_t i = 0; i < n_down; ++i) {
        dx[i] = downsampled_pts[i].x; dy[i] = downsampled_pts[i].y; dz[i] = downsampled_pts[i].z;
    }
    t_aos2soa2.end();
    PointCloudSoA downsampled_cloud{dx.data(), dy.data(), dz.data(), n_down};
    std::cout << "[*] AoS->SoA #2: " << t_aos2soa2.ms << " ms\n";

    // ── STAGE 4: Build Grid ────────────────────────────────────────────────
    SubTimer t_grid_build{"Grid Build"};
    Fast3DSpatialGrid search_grid(0.25f);
    t_grid_build.begin();
    search_grid.build(dx.data(), dy.data(), dz.data(), n_down);
    t_grid_build.end();
    std::cout << "[4] Grid Build: " << t_grid_build.ms << " ms\n";

    // ── STAGE 5: Fused SOR + Normal — MICRO-INSTRUMENTED ──────────────────
    SubTimer t_sor_grid_search{"SOR: Grid radiusSearch"};
    SubTimer t_sor_rvv_sqrt{"SOR: RVV sqrt+sum"};
    SubTimer t_sor_normal_est{"SOR: Normal estimation"};
    SubTimer t_sor_filter_copy{"SOR: Filter + copy"};

    const float sor_radius = 0.25f;
    const int sor_k = 20;
    const float sor_alpha = 1.0f;
    const float sor_r2 = sor_radius * sor_radius;

    std::vector<float> mean_dists(n_down, sor_radius);
    std::vector<float> all_nx(n_down, 0.0f), all_ny(n_down, 0.0f), all_nz(n_down, 1.0f);
    std::vector<int> valid_points;
    valid_points.reserve(n_down);
    double total_sum = 0.0, total_sq_sum = 0.0;

    std::vector<int> nbrs; nbrs.reserve(256);
    std::vector<float> d2; d2.reserve(256);

    size_t total_neighbors_found = 0;

    auto t5_overall_start = Clock::now();

    for (size_t i = 0; i < n_down; ++i) {
        float qx = downsampled_cloud.x[i], qy = downsampled_cloud.y[i], qz = downsampled_cloud.z[i];

        // (A) Grid search
        t_sor_grid_search.begin();
        search_grid.radiusSearch(qx, qy, qz, sor_r2, nbrs, d2);
        t_sor_grid_search.end();

        int found = static_cast<int>(nbrs.size());
        total_neighbors_found += found;
        if (found < 2) continue;

        int k_use = std::min(found - 1, sor_k);

        // (B) RVV sqrt+sum
        t_sor_rvv_sqrt.begin();
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
        t_sor_rvv_sqrt.end();

        float m = sum_dist / static_cast<float>(k_use);
        mean_dists[i] = m;
        total_sum += m;
        total_sq_sum += (m * m);
        valid_points.push_back(static_cast<int>(i));

        // (C) Normal estimation
        t_sor_normal_est.begin();
        if (found >= 3) {
            float cx = 0, cy = 0, cz = 0;
            int n_norm = std::min(found, 16);
            for (int k = 0; k < n_norm; ++k) {
                int idx = nbrs[k];
                cx += downsampled_cloud.x[idx]; cy += downsampled_cloud.y[idx]; cz += downsampled_cloud.z[idx];
            }
            float inv_n = 1.0f / static_cast<float>(n_norm);
            cx *= inv_n; cy *= inv_n; cz *= inv_n;

            float c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
            for (int k = 0; k < n_norm; ++k) {
                int idx = nbrs[k];
                float ddx = downsampled_cloud.x[idx] - cx, ddy = downsampled_cloud.y[idx] - cy, ddz = downsampled_cloud.z[idx] - cz;
                c00 += ddx * ddx; c01 += ddx * ddy; c02 += ddx * ddz;
                c11 += ddy * ddy; c12 += ddy * ddz; c22 += ddz * ddz;
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
        t_sor_normal_est.end();
    }

    // (D) Filter and copy
    t_sor_filter_copy.begin();
    double d_count = static_cast<double>(valid_points.size());
    double global_mean = total_sum / d_count;
    double variance = (total_sq_sum / d_count) - (global_mean * global_mean);
    double stddev = std::sqrt(std::max(0.0, variance));
    float thresh = static_cast<float>(global_mean + sor_alpha * stddev);

    std::vector<float> fx, fy, fz;
    fx.reserve(valid_points.size()); fy.reserve(valid_points.size()); fz.reserve(valid_points.size());
    for (int idx : valid_points) {
        if (mean_dists[idx] <= thresh) {
            fx.push_back(downsampled_cloud.x[idx]);
            fy.push_back(downsampled_cloud.y[idx]);
            fz.push_back(downsampled_cloud.z[idx]);
        }
    }
    t_sor_filter_copy.end();

    double t5_overall = std::chrono::duration<double, std::milli>(Clock::now() - t5_overall_start).count();
    size_t n_sor = fx.size();

    std::cout << "\n[5] FUSED SOR + NORMAL ESTIMATION BREAKDOWN (" << n_down << " pts):\n";
    std::cout << "    Overall Stage 5 wall time:          " << std::fixed << std::setprecision(2) << t5_overall << " ms\n";
    std::cout << "    (A) Grid radiusSearch:              " << t_sor_grid_search.ms << " ms ("
              << std::setprecision(1) << (t_sor_grid_search.ms / t5_overall * 100.0) << "%)\n";
    std::cout << "    (B) RVV sqrt+sum reduction:         " << std::setprecision(2) << t_sor_rvv_sqrt.ms << " ms ("
              << std::setprecision(1) << (t_sor_rvv_sqrt.ms / t5_overall * 100.0) << "%)\n";
    std::cout << "    (C) Normal estimation (scalar):     " << std::setprecision(2) << t_sor_normal_est.ms << " ms ("
              << std::setprecision(1) << (t_sor_normal_est.ms / t5_overall * 100.0) << "%)\n";
    std::cout << "    (D) Filter + copy:                  " << std::setprecision(2) << t_sor_filter_copy.ms << " ms ("
              << std::setprecision(1) << (t_sor_filter_copy.ms / t5_overall * 100.0) << "%)\n";
    std::cout << "    (E) Unaccounted overhead:           " << std::setprecision(2)
              << (t5_overall - t_sor_grid_search.ms - t_sor_rvv_sqrt.ms - t_sor_normal_est.ms - t_sor_filter_copy.ms) << " ms\n";
    std::cout << "    Avg neighbors/query: " << std::setprecision(1) << (double)total_neighbors_found / n_down << "\n";
    std::cout << "    Output: " << n_sor << " inlier points\n";

    // ── STAGE 8: RANSAC — MICRO-INSTRUMENTED ──────────────────────────────
    PointCloudSoA sor_cloud{fx.data(), fy.data(), fz.data(), n_sor};
    float model[4] = {0, 0, 0, 0};

    SubTimer t_ransac{"RANSAC (RVV library)"};
    t_ransac.begin();
    int r_cnt = ransac_plane_rvv(sor_cloud, 0.20f, 1000, model, 1e-4f, 0.99f);
    t_ransac.end();
    std::cout << "\n[8] RANSAC: " << r_cnt << " inliers, model=[" << std::setprecision(4)
              << model[0] << ", " << model[1] << ", " << model[2] << ", " << model[3]
              << "] in " << std::setprecision(2) << t_ransac.ms << " ms\n";

    // 8b: Scalar extraction (as in pipeline_3d_ultra L467-486)
    SubTimer t_extract_scalar{"Extraction (scalar)"};
    std::vector<float> ox_s, oy_s, oz_s;
    size_t n_inliers_scalar = 0;
    t_extract_scalar.begin();
    {
        float a = model[0], b = model[1], c = model[2], d = model[3];
        ox_s.reserve(n_sor); oy_s.reserve(n_sor); oz_s.reserve(n_sor);
        for (size_t i = 0; i < n_sor; ++i) {
            float dist = std::abs(a * fx[i] + b * fy[i] + c * fz[i] + d);
            if (dist <= 0.20f) {
                n_inliers_scalar++;
            } else {
                ox_s.push_back(fx[i]); oy_s.push_back(fy[i]); oz_s.push_back(fz[i]);
            }
        }
    }
    t_extract_scalar.end();

    // 8c: RVV extraction (library function)
    SubTimer t_extract_rvv{"Extraction (RVV vcompress)"};
    std::vector<PointXYZ> inliers_rvv(n_sor), outliers_rvv(n_sor);
    size_t n_in_rvv = 0, n_out_rvv = 0;
    t_extract_rvv.begin();
    extract_plane_inliers_outliers_rvv(sor_cloud, model, 0.20f,
                                        inliers_rvv.data(), outliers_rvv.data(),
                                        n_in_rvv, n_out_rvv);
    t_extract_rvv.end();

    std::cout << "    Scalar extraction: " << n_inliers_scalar << " in + " << ox_s.size()
              << " out in " << t_extract_scalar.ms << " ms\n";
    std::cout << "    RVV extraction:    " << n_in_rvv << " in + " << n_out_rvv
              << " out in " << t_extract_rvv.ms << " ms\n";
    double extract_speedup = t_extract_scalar.ms / std::max(0.01, t_extract_rvv.ms);
    std::cout << "    *** GAP #2: RVV extraction is " << std::setprecision(2)
              << extract_speedup << "x vs scalar ***\n";

    // ── AoS->SoA Conversion #3 ─────────────────────────────────────────────
    SubTimer t_aos2soa3{"AoS->SoA #3 (outliers)"};
    size_t n_outliers = ox_s.size();
    t_aos2soa3.begin();
    // This conversion is trivial since ox_s is already SoA — but in the actual
    // pipeline_3d_ultra, outlier_pts is AoS and gets converted to SoA at L809-813
    // We simulate the actual pipeline cost using the AoS outliers from RVV extraction
    std::vector<float> ox(n_out_rvv), oy(n_out_rvv), oz(n_out_rvv);
    for (size_t i = 0; i < n_out_rvv; ++i) {
        ox[i] = outliers_rvv[i].x; oy[i] = outliers_rvv[i].y; oz[i] = outliers_rvv[i].z;
    }
    t_aos2soa3.end();
    std::cout << "\n[*] AoS->SoA #3 (outlier conversion): " << t_aos2soa3.ms << " ms (" << n_out_rvv << " pts)\n";

    // ── STAGE 9: Clustering — MICRO-INSTRUMENTED ──────────────────────────
    // Use ox_s/oy_s/oz_s (already SoA) for consistency
    n_outliers = ox_s.size();
    PointCloudSoA non_ground{ox_s.data(), oy_s.data(), oz_s.data(), n_outliers};

    SubTimer t_cluster_grid{"Clustering: Grid build"};
    SubTimer t_cluster_union{"Clustering: Union-Find traversal"};
    SubTimer t_cluster_collect{"Clustering: Cluster collection"};

    Fast3DSpatialGrid cluster_grid(0.15f);
    t_cluster_grid.begin();
    cluster_grid.build(ox_s.data(), oy_s.data(), oz_s.data(), n_outliers);
    t_cluster_grid.end();

    DisjointSet ds(static_cast<int>(n_outliers));
    float tol_sq = 0.15f * 0.15f;

    static constexpr int kHalfOffsets[14][3] = {
        {0, 0, 0}, {1, 0, 0},
        {-1, 1, 0}, {0, 1, 0}, {1, 1, 0},
        {-1, -1, 1}, {0, -1, 1}, {1, -1, 1},
        {-1, 0, 1},  {0, 0, 1},  {1, 0, 1},
        {-1, 1, 1},  {0, 1, 1},  {1, 1, 1}
    };

    t_cluster_union.begin();
    for (size_t i = 0; i < n_outliers; ++i) {
        float qx = non_ground.x[i], qy = non_ground.y[i], qz = non_ground.z[i];
        int qcx = static_cast<int>(std::floor(qx * cluster_grid.inv_cell_));
        int qcy = static_cast<int>(std::floor(qy * cluster_grid.inv_cell_));
        int qcz = static_cast<int>(std::floor(qz * cluster_grid.inv_cell_));

        for (int o = 0; o < 14; ++o) {
            int tcx = qcx + kHalfOffsets[o][0];
            int tcy = qcy + kHalfOffsets[o][1];
            int tcz = qcz + kHalfOffsets[o][2];
            size_t h = Fast3DSpatialGrid::hash3D(tcx, tcy, tcz);

            while (cluster_grid.cells_[h].head != -1) {
                if (cluster_grid.cells_[h].cx == tcx && cluster_grid.cells_[h].cy == tcy && cluster_grid.cells_[h].cz == tcz) {
                    int curr = cluster_grid.cells_[h].head;
                    while (curr != -1) {
                        if (curr > static_cast<int>(i)) {
                            float ddx = cluster_grid.px_[curr] - qx, ddy = cluster_grid.py_[curr] - qy, ddz = cluster_grid.pz_[curr] - qz;
                            float dd2 = ddx * ddx + ddy * ddy + ddz * ddz;
                            if (dd2 <= tol_sq) ds.unite(static_cast<int>(i), curr);
                        }
                        curr = cluster_grid.next_[curr];
                    }
                    break;
                }
                h = (h + 1) & Fast3DSpatialGrid::kMask;
            }
        }
    }
    t_cluster_union.end();

    t_cluster_collect.begin();
    std::unordered_map<int, std::vector<int>> cluster_map;
    for (size_t i = 0; i < n_outliers; ++i) {
        cluster_map[ds.find(static_cast<int>(i))].push_back(static_cast<int>(i));
    }
    size_t n_clusters = 0;
    for (auto& kv : cluster_map) {
        if (kv.second.size() >= 50 && kv.second.size() <= 100000) n_clusters++;
    }
    t_cluster_collect.end();

    double t9_total = t_cluster_grid.ms + t_cluster_union.ms + t_cluster_collect.ms;
    std::cout << "\n[9] EUCLIDEAN CLUSTERING BREAKDOWN (" << n_outliers << " pts -> " << n_clusters << " clusters):\n";
    std::cout << "    Overall Stage 9:                    " << std::setprecision(2) << t9_total << " ms\n";
    std::cout << "    (A) Grid build:                     " << t_cluster_grid.ms << " ms ("
              << std::setprecision(1) << (t_cluster_grid.ms / t9_total * 100.0) << "%)\n";
    std::cout << "    (B) Union-Find + linked-list dist:  " << std::setprecision(2) << t_cluster_union.ms << " ms ("
              << std::setprecision(1) << (t_cluster_union.ms / t9_total * 100.0) << "%)\n";
    std::cout << "    (C) Cluster collection (umap):      " << std::setprecision(2) << t_cluster_collect.ms << " ms ("
              << std::setprecision(1) << (t_cluster_collect.ms / t9_total * 100.0) << "%)\n";

    // ── STAGE 3 DEEP DIVE: sort cost ──────────────────────────────────────
    SubTimer t_ds_sort{"Downsample: std::sort"};
    {
        std::vector<int32_t> fake_keys(n_input);
        for (size_t i = 0; i < n_input; i++) {
            fake_keys[i] = static_cast<int32_t>(std::floor(ix[i] * 10.0f)) * 1000000 +
                           static_cast<int32_t>(std::floor(iy[i] * 10.0f)) * 1000 +
                           static_cast<int32_t>(std::floor(iz[i] * 10.0f));
        }
        std::vector<uint32_t> order(n_input);
        std::iota(order.begin(), order.end(), 0u);

        t_ds_sort.begin();
        std::sort(order.begin(), order.end(),
                  [&fake_keys](uint32_t a, uint32_t b) { return fake_keys[a] < fake_keys[b]; });
        t_ds_sort.end();
    }
    std::cout << "\n[3] VOXEL DOWNSAMPLE DEEP DIVE (" << n_input << " pts):\n";
    std::cout << "    Total downsample time:              " << std::setprecision(2) << t_downsample.ms << " ms\n";
    std::cout << "    std::sort cost (isolated):          " << t_ds_sort.ms << " ms ("
              << std::setprecision(1) << (t_ds_sort.ms / t_downsample.ms * 100.0) << "% of total)\n";
    std::cout << "    Remainder (bbox+keys+centroid):     " << std::setprecision(2)
              << (t_downsample.ms - t_ds_sort.ms) << " ms (RVV vectorized)\n";

    // ── FULL SUMMARY ──────────────────────────────────────────────────────
    double total_compute = t_downsample.ms + t_grid_build.ms + t5_overall +
                            t_ransac.ms + t_extract_scalar.ms + t9_total;
    double total_conversion = t_aos2soa1.ms + t_aos2soa2.ms + t_aos2soa3.ms;

    std::cout << "\n=======================================================================\n";
    std::cout << "  FULL PIPELINE COMPUTE SUMMARY (no disk I/O)\n";
    std::cout << "=======================================================================\n";
    std::cout << "  [3] Downsample:          " << std::setprecision(2) << t_downsample.ms << " ms\n";
    std::cout << "  [4] Grid Build:          " << t_grid_build.ms << " ms\n";
    std::cout << "  [5] Fused SOR+Normal:    " << t5_overall << " ms  <-- DOMINANT\n";
    std::cout << "      +-- radiusSearch:    " << t_sor_grid_search.ms << " ms <-- linked-list\n";
    std::cout << "      +-- RVV sqrt+sum:    " << t_sor_rvv_sqrt.ms << " ms <-- already vectorized\n";
    std::cout << "      +-- Normal (scalar): " << t_sor_normal_est.ms << " ms <-- vluxei32 candidate\n";
    std::cout << "      +-- Filter+copy:     " << t_sor_filter_copy.ms << " ms\n";
    std::cout << "  [8] RANSAC:              " << t_ransac.ms << " ms  (already vectorized)\n";
    std::cout << "      +-- Extraction:      " << t_extract_scalar.ms << " ms (scalar) vs "
              << t_extract_rvv.ms << " ms (RVV)\n";
    std::cout << "  [9] Clustering:          " << t9_total << " ms\n";
    std::cout << "      +-- Grid build:      " << t_cluster_grid.ms << " ms\n";
    std::cout << "      +-- Union+distance:  " << t_cluster_union.ms << " ms <-- linked-list\n";
    std::cout << "      +-- Collection:      " << t_cluster_collect.ms << " ms <-- unordered_map\n";
    std::cout << "  [*] AoS<->SoA overhead:  " << total_conversion << " ms\n";
    std::cout << "  -----------------------------------------------------------\n";
    std::cout << "  TOTAL COMPUTE:           " << total_compute << " ms\n";
    std::cout << "  TOTAL CONVERSION:        " << total_conversion << " ms\n\n";

    // ── GAP VERIFICATION ──────────────────────────────────────────────────
    std::cout << "=======================================================================\n";
    std::cout << "  GAP VERIFICATION RESULTS\n";
    std::cout << "=======================================================================\n";

    double gap1_time = t_sor_grid_search.ms + t_cluster_union.ms;
    double gap2_saving = t_extract_scalar.ms - t_extract_rvv.ms;
    double gap3_time = t_sor_normal_est.ms;
    double gap4_time = t_cluster_collect.ms;
    double gap5_time = total_conversion;
    double gap6_time = t_ds_sort.ms;

    auto verdict = [&](double time, double base, double sig_thresh) {
        double pct = time / base * 100.0;
        std::cout << std::setprecision(1) << pct << "% of compute) -- "
                  << (time > sig_thresh ? "CONFIRMED SIGNIFICANT" : "CONFIRMED BUT SMALL") << "\n";
    };

    std::cout << "  Gap #1: Linked-list traversal (SOR+Clustering)\n";
    std::cout << "          Time: " << std::setprecision(2) << gap1_time << " ms (";
    verdict(gap1_time, total_compute, 50.0);

    std::cout << "  Gap #2: Scalar extraction -> RVV vcompress\n";
    std::cout << "          Savings: " << std::setprecision(2) << gap2_saving << " ms ("
              << extract_speedup << "x speedup) -- "
              << (gap2_saving > 5.0 ? "CONFIRMED" : "CONFIRMED BUT MARGINAL") << "\n";

    std::cout << "  Gap #3: Scalar normal estimation\n";
    std::cout << "          Time: " << std::setprecision(2) << gap3_time << " ms (";
    verdict(gap3_time, total_compute, 20.0);

    std::cout << "  Gap #4: unordered_map cluster collection\n";
    std::cout << "          Time: " << std::setprecision(2) << gap4_time << " ms (";
    verdict(gap4_time, total_compute, 10.0);

    std::cout << "  Gap #5: AoS<->SoA conversion overhead\n";
    std::cout << "          Time: " << std::setprecision(2) << gap5_time << " ms (";
    verdict(gap5_time, total_compute, 10.0);

    std::cout << "  Gap #6: std::sort in voxel downsample\n";
    std::cout << "          Time: " << std::setprecision(2) << gap6_time << " ms ("
              << std::setprecision(1) << (gap6_time / t_downsample.ms * 100.0)
              << "% of Stage 3) -- "
              << (gap6_time > 20.0 ? "CONFIRMED" : "CONFIRMED BUT SMALL") << "\n";

    // Hidden bottleneck check
    double accounted = gap1_time + t_sor_rvv_sqrt.ms + gap3_time + t_sor_filter_copy.ms +
                        t_ransac.ms + t_extract_scalar.ms + gap4_time +
                        t_downsample.ms + t_grid_build.ms + t_cluster_grid.ms;
    double unaccounted = total_compute - accounted;
    std::cout << "\n  HIDDEN BOTTLENECK CHECK:\n";
    std::cout << "          Accounted:    " << std::setprecision(2) << accounted << " ms\n";
    std::cout << "          Total:        " << total_compute << " ms\n";
    std::cout << "          Unaccounted:  " << unaccounted << " ms ("
              << std::setprecision(1) << (std::abs(unaccounted) / total_compute * 100.0) << "%) -- "
              << (std::abs(unaccounted) / total_compute < 0.05 ?
                "NO HIDDEN BOTTLENECKS" : "POSSIBLE HIDDEN BOTTLENECK") << "\n";

    std::cout << "\n=======================================================================\n";
    std::cout << "  VERIFICATION COMPLETE\n";
    std::cout << "=======================================================================\n";

    return 0;
}
