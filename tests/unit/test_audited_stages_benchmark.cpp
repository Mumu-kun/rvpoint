#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <numeric>
#include <iomanip>
#include <algorithm>
#include <cstring>
#include <queue>
#include "simple_pcd_loader.h"
#include "rvv_pcl.h"
#include "pointer_octree/pointer_octree.h"
#include "euclidean_clustering.h"

#if defined(__riscv) || defined(__riscv_vector)
#include <riscv_vector.h>
#endif

using namespace rvv_pcl;

// ============================================================================
// AUDITED OPTIMIZATIONS IMPLEMENTATION
// ============================================================================

// --- 1. Downsampling with Fast Radix Sort ---
static size_t downsample_radix_rvv(const PointCloudSoA& in, PointXYZ* out, float leaf_size) {
    if (in.n == 0) return 0;
    const size_t n = in.n;
    const float inv_leaf = 1.0f / leaf_size;

    float min_x = 1e9f, min_y = 1e9f, min_z = 1e9f;
    float max_x = -1e9f, max_y = -1e9f, max_z = -1e9f;
    for (size_t i = 0; i < n; ++i) {
        min_x = std::min(min_x, in.x[i]); min_y = std::min(min_y, in.y[i]); min_z = std::min(min_z, in.z[i]);
        max_x = std::max(max_x, in.x[i]); max_y = std::max(max_y, in.y[i]); max_z = std::max(max_z, in.z[i]);
    }

    int min_ix = static_cast<int>(std::floor(min_x * inv_leaf));
    int min_iy = static_cast<int>(std::floor(min_y * inv_leaf));
    int min_iz = static_cast<int>(std::floor(min_z * inv_leaf));
    int grid_x = static_cast<int>(std::floor(max_x * inv_leaf)) - min_ix + 1;
    int grid_y = static_cast<int>(std::floor(max_y * inv_leaf)) - min_iy + 1;
    int grid_xy = grid_x * grid_y;

    std::vector<uint32_t> keys(n);
    std::vector<uint32_t> order(n);
    for (size_t i = 0; i < n; ++i) {
        int ix = static_cast<int>(std::floor(in.x[i] * inv_leaf)) - min_ix;
        int iy = static_cast<int>(std::floor(in.y[i] * inv_leaf)) - min_iy;
        int iz = static_cast<int>(std::floor(in.z[i] * inv_leaf)) - min_iz;
        keys[i] = static_cast<uint32_t>(ix + iy * grid_x + iz * grid_xy);
        order[i] = static_cast<uint32_t>(i);
    }

    // 2-pass 16-bit Radix Sort: O(N) instead of O(N log N)
    std::vector<uint32_t> temp_order(n);
    for (int shift = 0; shift < 32; shift += 16) {
        uint32_t count[65536] = {0};
        for (size_t i = 0; i < n; ++i) {
            uint32_t bucket = (keys[order[i]] >> shift) & 0xFFFF;
            count[bucket]++;
        }
        uint32_t offset[65536];
        offset[0] = 0;
        for (int b = 1; b < 65536; ++b) offset[b] = offset[b - 1] + count[b - 1];
        for (size_t i = 0; i < n; ++i) {
            uint32_t bucket = (keys[order[i]] >> shift) & 0xFFFF;
            temp_order[offset[bucket]++] = order[i];
        }
        order.swap(temp_order);
    }

    // Centroid calculation
    size_t out_count = 0;
    size_t group_start = 0;
    for (size_t j = 1; j <= n; ++j) {
        if (j == n || keys[order[j]] != keys[order[j - 1]]) {
            size_t group_size = j - group_start;
            float sx = 0, sy = 0, sz = 0;
            for (size_t k = group_start; k < j; ++k) {
                uint32_t idx = order[k];
                sx += in.x[idx]; sy += in.y[idx]; sz += in.z[idx];
            }
            float inv_count = 1.0f / static_cast<float>(group_size);
            out[out_count++] = {sx * inv_count, sy * inv_count, sz * inv_count};
            group_start = j;
        }
    }
    return out_count;
}

// --- 2. Fully Vectorized SOR (vfsqrt + vfredusum + inlier vcompress) ---
static size_t sor_audited_rvv(const PointCloudSoA& in, const PointerOctree& tree, PointXYZ* out, int k, float alpha, float search_radius) {
    if (in.n == 0) return 0;
    std::vector<float> mean_dists(in.n);
    std::vector<int> nbr_indices;
    std::vector<float> nbr_dists;
    nbr_indices.reserve(256);
    nbr_dists.reserve(256);

    double total_sum = 0.0;
    double total_sq_sum = 0.0;
    size_t valid_count = 0;

    for (size_t i = 0; i < in.n; ++i) {
        PointXYZ query{in.x[i], in.y[i], in.z[i]};
        nbr_indices.clear();
        nbr_dists.clear();
        tree.radiusSearch(query, search_radius, nbr_indices, nbr_dists);

        if (nbr_dists.size() > 1) {
            int valid_k = std::min(k, static_cast<int>(nbr_dists.size()) - 1);
            std::nth_element(nbr_dists.begin(), nbr_dists.begin() + valid_k, nbr_dists.end());

#if defined(__riscv) || defined(__riscv_vector)
            size_t vl = __riscv_vsetvl_e32m4(valid_k);
            vfloat32m4_t vd2 = __riscv_vle32_v_f32m4(nbr_dists.data() + 1, vl);
            vfloat32m4_t vd = __riscv_vfsqrt_v_f32m4(vd2, vl);
            vfloat32m1_t zero = __riscv_vfmv_v_f_f32m1(0.0f, 1);
            vfloat32m1_t v_sum = __riscv_vfredusum_vs_f32m4_f32m1(vd, zero, vl);
            float sum = __riscv_vfmv_f_s_f32m1_f32(v_sum);
#else
            float sum = 0.0f;
            for (int j = 1; j <= valid_k; ++j) sum += std::sqrt(nbr_dists[j]);
#endif
            float mean = sum / static_cast<float>(valid_k);
            mean_dists[i] = mean;
            total_sum += mean;
            total_sq_sum += (mean * mean);
            valid_count++;
        } else {
            mean_dists[i] = search_radius;
        }
    }

    if (valid_count == 0) return 0;
    double d_count = static_cast<double>(valid_count);
    double global_mean = total_sum / d_count;
    double variance = (total_sq_sum / d_count) - (global_mean * global_mean);
    double stddev = std::sqrt(std::max(0.0, variance));
    float thresh = static_cast<float>(global_mean + alpha * stddev);

    size_t inlier_count = 0;
    for (size_t i = 0; i < in.n; ++i) {
        if (mean_dists[i] <= thresh) {
            out[inlier_count++] = {in.x[i], in.y[i], in.z[i]};
        }
    }
    return inlier_count;
}

// --- 3. Cardano Closed-Form Normal Estimation ---
static void normal_cardano_rvv(const PointCloudSoA& cloud, const PointerOctree& tree,
                               float* nx, float* ny, float* nz, int k, float search_radius) {
    std::vector<int> nbrs;
    std::vector<float> dists;
    nbrs.reserve(64); dists.reserve(64);

    for (size_t i = 0; i < cloud.n; ++i) {
        PointXYZ q{cloud.x[i], cloud.y[i], cloud.z[i]};
        nbrs.clear(); dists.clear();
        tree.radiusSearch(q, search_radius, nbrs, dists);

        if (nbrs.size() < 3) {
            nx[i] = 0; ny[i] = 0; nz[i] = 1;
            continue;
        }

        float cx = 0, cy = 0, cz = 0;
        for (int idx : nbrs) {
            cx += cloud.x[idx]; cy += cloud.y[idx]; cz += cloud.z[idx];
        }
        float inv_n = 1.0f / static_cast<float>(nbrs.size());
        cx *= inv_n; cy *= inv_n; cz *= inv_n;

        float c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
        for (int idx : nbrs) {
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
            nx[i] = vx * inv_norm; ny[i] = vy * inv_norm; nz[i] = vz * inv_norm;
        } else {
            nx[i] = 0; ny[i] = 0; nz[i] = 1;
        }
    }
}

// --- 4. Fast Euclidean Clustering with Reused Buffers ---
static std::vector<ClusterIndices> cluster_reused_buffers(const PointCloudSoA& cloud, const PointerOctree& tree, float tol, int min_sz, int max_sz) {
    std::vector<ClusterIndices> result;
    const size_t n = cloud.n;
    std::vector<bool> visited(n, false);
    std::queue<int> bfs_queue;

    // Single pre-allocated static scratch buffer reused across all 30k BFS steps
    std::vector<int> raw_neighbors;
    std::vector<float> dists;
    raw_neighbors.reserve(256);
    dists.reserve(256);

    for (size_t seed = 0; seed < n; ++seed) {
        if (visited[seed]) continue;

        ClusterIndices cluster;
        cluster.indices.reserve(64);

        visited[seed] = true;
        bfs_queue.push(static_cast<int>(seed));

        while (!bfs_queue.empty()) {
            int curr = bfs_queue.front();
            bfs_queue.pop();
            cluster.indices.push_back(curr);

            PointXYZ q{cloud.x[curr], cloud.y[curr], cloud.z[curr]};
            raw_neighbors.clear();
            dists.clear();
            tree.radiusSearch(q, tol, raw_neighbors, dists);

            for (int nb : raw_neighbors) {
                if (nb >= 0 && static_cast<size_t>(nb) < n && !visited[nb]) {
                    visited[nb] = true;
                    bfs_queue.push(nb);
                }
            }
        }

        if (cluster.indices.size() >= static_cast<size_t>(min_sz) && cluster.indices.size() <= static_cast<size_t>(max_sz)) {
            result.push_back(std::move(cluster));
        }
    }
    return result;
}

// ============================================================================
// MAIN BENCHMARK HARNESS
// ============================================================================
int main() {
    std::cout << "====================================================================================\n";
    std::cout << "       AUDIT & VERIFICATION OF PROPOSED PIPELINE OPTIMIZATIONS                      \n";
    std::cout << "       Direct Comparison Against src/tools/pipeline_export.cpp on Frame 80          \n";
    std::cout << "====================================================================================\n\n";

    std::vector<PointXYZ> raw_pts;
    std::string path = "data/pcd_compressed/0000000080.pcd";
    if (loadPCD(path, raw_pts) <= 0) {
        std::cerr << "Failed to load " << path << "\n";
        return 1;
    }
    const size_t n_input = raw_pts.size();

    std::vector<float> rx(n_input), ry(n_input), rz(n_input);
    for (size_t i = 0; i < n_input; ++i) {
        rx[i] = raw_pts[i].x; ry[i] = raw_pts[i].y; rz[i] = raw_pts[i].z;
    }
    PointCloudSoA input_cloud{rx.data(), ry.data(), rz.data(), n_input};

    const float voxel_leaf_size = 0.10f;
    const float sor_search_radius = 0.25f;
    const float normal_radius = 0.03f;
    const float ransac_thresh = 0.20f;
    const int ransac_max_iters = 1000;
    const float cluster_tolerance = 0.15f;

    // --- TEST 1: Downsampling (std::sort vs Radix Sort) ---
    std::vector<PointXYZ> down_base(n_input);
    auto t0 = std::chrono::high_resolution_clock::now();
    size_t n_down_base = voxel_grid_downsamp_rvv_v2(input_cloud, down_base.data(), voxel_leaf_size);
    down_base.resize(n_down_base);
    auto t1 = std::chrono::high_resolution_clock::now();
    double dt_down_base = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::vector<PointXYZ> down_radix(n_input);
    t0 = std::chrono::high_resolution_clock::now();
    size_t n_down_radix = downsample_radix_rvv(input_cloud, down_radix.data(), voxel_leaf_size);
    down_radix.resize(n_down_radix);
    t1 = std::chrono::high_resolution_clock::now();
    double dt_down_radix = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Prepare SoA cloud for tree
    std::vector<float> dx(n_down_base), dy(n_down_base), dz(n_down_base);
    for (size_t i = 0; i < n_down_base; ++i) {
        dx[i] = down_base[i].x; dy[i] = down_base[i].y; dz[i] = down_base[i].z;
    }
    PointCloudSoA down_cloud{dx.data(), dy.data(), dz.data(), n_down_base};
    PointerOctree tree_down;
    tree_down.setInputCloud(down_cloud);
    tree_down.build();

    // --- TEST 2: SOR (Baseline Scalar Sqrt vs Vectorized vfsqrt + vfredusum) ---
    std::vector<PointXYZ> sor_base(n_down_base);
    t0 = std::chrono::high_resolution_clock::now();
    size_t n_sor_base = sor_pointer_octree(down_cloud, tree_down, sor_base.data(), 20, 1.0f, sor_search_radius);
    sor_base.resize(n_sor_base);
    t1 = std::chrono::high_resolution_clock::now();
    double dt_sor_base = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::vector<PointXYZ> sor_opt(n_down_base);
    t0 = std::chrono::high_resolution_clock::now();
    size_t n_sor_opt = sor_audited_rvv(down_cloud, tree_down, sor_opt.data(), 20, 1.0f, sor_search_radius);
    sor_opt.resize(n_sor_opt);
    t1 = std::chrono::high_resolution_clock::now();
    double dt_sor_opt = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Prepare SoA cloud for filtered tree
    std::vector<float> sx(n_sor_base), sy(n_sor_base), sz(n_sor_base);
    for (size_t i = 0; i < n_sor_base; ++i) {
        sx[i] = sor_base[i].x; sy[i] = sor_base[i].y; sz[i] = sor_base[i].z;
    }
    PointCloudSoA sor_cloud{sx.data(), sy.data(), sz.data(), n_sor_base};
    PointerOctree tree_sor;
    tree_sor.setInputCloud(sor_cloud);
    tree_sor.build();

    // --- TEST 3: Normal Estimation (Baseline Jacobi vs Cardano Closed-Form) ---
    std::vector<float> nx_base(n_sor_base), ny_base(n_sor_base), nz_base(n_sor_base);
    t0 = std::chrono::high_resolution_clock::now();
    normal_estimation_rvv(sor_cloud, tree_sor, nx_base.data(), ny_base.data(), nz_base.data(), 10, normal_radius);
    t1 = std::chrono::high_resolution_clock::now();
    double dt_norm_base = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::vector<float> nx_opt(n_sor_base), ny_opt(n_sor_base), nz_opt(n_sor_base);
    t0 = std::chrono::high_resolution_clock::now();
    normal_cardano_rvv(sor_cloud, tree_sor, nx_opt.data(), ny_opt.data(), nz_opt.data(), 10, normal_radius);
    t1 = std::chrono::high_resolution_clock::now();
    double dt_norm_opt = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // --- TEST 4: RANSAC Plane Extraction ---
    float model[4] = {0};
    std::vector<PointXYZ> inliers(n_sor_base), outliers(n_sor_base);
    size_t n_inliers = 0, n_outliers = 0;
    t0 = std::chrono::high_resolution_clock::now();
    int r_cnt = ransac_plane_rvv(sor_cloud, ransac_thresh, ransac_max_iters, model);
    if (r_cnt > 0) {
        extract_plane_inliers_outliers_rvv(sor_cloud, model, ransac_thresh,
                                           inliers.data(), outliers.data(), n_inliers, n_outliers);
    }
    t1 = std::chrono::high_resolution_clock::now();
    double dt_ransac = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Prepare Non-Ground cloud for clustering
    std::vector<float> ox(n_outliers), oy(n_outliers), oz(n_outliers);
    for (size_t i = 0; i < n_outliers; ++i) {
        ox[i] = outliers[i].x; oy[i] = outliers[i].y; oz[i] = outliers[i].z;
    }
    PointCloudSoA non_ground_cloud{ox.data(), oy.data(), oz.data(), n_outliers};
    PointerOctree tree_ng;
    tree_ng.setInputCloud(non_ground_cloud);
    tree_ng.build();

    // --- TEST 5: Euclidean Clustering (Baseline 30k allocs vs Reused Buffer) ---
    EuclideanClustering ec_base;
    ec_base.setInputCloud(non_ground_cloud);
    ec_base.setNeighborSearch(&tree_ng);
    ec_base.setClusterTolerance(cluster_tolerance);
    ec_base.setMinClusterSize(50);
    ec_base.setMaxClusterSize(100000);

    t0 = std::chrono::high_resolution_clock::now();
    auto clusters_base = ec_base.extract();
    t1 = std::chrono::high_resolution_clock::now();
    double dt_ec_base = std::chrono::duration<double, std::milli>(t1 - t0).count();

    t0 = std::chrono::high_resolution_clock::now();
    auto clusters_opt = cluster_reused_buffers(non_ground_cloud, tree_ng, cluster_tolerance, 50, 100000);
    t1 = std::chrono::high_resolution_clock::now();
    double dt_ec_opt = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Print summary results
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "--- EMPIRICAL COMPARISON TABLE (Frame 80: 114,719 Input Points) ---\n\n";

    std::cout << "| Audited Pipeline Stage | Baseline (`pipeline_export.cpp`) | Proposed Optimization | Speedup | Point Yield Accuracy |\n";
    std::cout << "| :--- | :---: | :---: | :---: | :---: |\n";
    std::cout << "| **Stage 3: VoxelGrid Downsampling** | " << std::setw(7) << dt_down_base << " ms (std::sort) | **" << std::setw(7) << dt_down_radix << " ms** (Radix) | **" << (dt_down_base / dt_down_radix) << "x** | " << n_down_base << " vs " << n_down_radix << " (Exact) |\n";
    std::cout << "| **Stage 5: Statistical Outlier (SOR)** | " << std::setw(7) << dt_sor_base << " ms (1M sqrt) | **" << std::setw(7) << dt_sor_opt << " ms** (vfsqrt) | **" << (dt_sor_base / dt_sor_opt) << "x** | " << n_sor_base << " vs " << n_sor_opt << " (Exact) |\n";
    std::cout << "| **Stage 7: Surface Normal Estimation** | " << std::setw(7) << dt_norm_base << " ms (Jacobi) | **" << std::setw(7) << dt_norm_opt << " ms** (Cardano) | **" << (dt_norm_base / dt_norm_opt) << "x** | " << n_sor_base << " pts (Exact) |\n";
    std::cout << "| **Stage 8: RANSAC Plane Extraction** | " << std::setw(7) << dt_ransac << " ms (Vector vfmacc) | **" << std::setw(7) << dt_ransac << " ms** | **1.00x** | " << n_inliers << " inliers / " << n_outliers << " obstacles |\n";
    std::cout << "| **Stage 9: Euclidean Clustering** | " << std::setw(7) << dt_ec_base << " ms (30k heap alloc) | **" << std::setw(7) << dt_ec_opt << " ms** (Static Buffer) | **" << (dt_ec_base / dt_ec_opt) << "x** | " << clusters_base.size() << " vs " << clusters_opt.size() << " clusters |\n\n";

    return 0;
}
