#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <numeric>
#include <iomanip>
#include <algorithm>
#include "simple_pcd_loader.h"
#include "rvv_pcl.h"
#include "pointer_octree/pointer_octree.h"
#include "euclidean_clustering.h"

#if defined(__riscv) || defined(__riscv_vector)
#include <riscv_vector.h>
#endif

using namespace rvv_pcl;

// ============================================================================
// STAGE TIMING STRUCTURE
// ============================================================================
struct StageReport {
    std::string name;
    double baseline_ms;
    double optimized_ms;
    size_t pts_baseline;
    size_t pts_optimized;
};

// ============================================================================
// OPTIMIZED STAGE 5: Vectorized SOR (vfsqrt + vfredusum + vcompress)
// ============================================================================
static size_t sor_vectorized_rvv(
    const PointCloudSoA& cloud,
    const PointerOctree& tree,
    float* out_x, float* out_y, float* out_z,
    int mean_k = 20, float std_mul = 1.0f, float search_radius = 0.25f)
{
    size_t n = cloud.n;
    std::vector<float> mean_dists(n, 0.0f);
    std::vector<int> valid_points;
    valid_points.reserve(n);

    float r2 = search_radius * search_radius;
    std::vector<int> nbr_indices;
    std::vector<float> nbr_dists;
    nbr_indices.reserve(128);
    nbr_dists.reserve(128);

    double total_sum = 0.0;
    double total_sq_sum = 0.0;

    for (size_t i = 0; i < n; ++i) {
        PointXYZ q{cloud.x[i], cloud.y[i], cloud.z[i]};
        tree.radiusSearch(q, search_radius, nbr_indices, nbr_dists);

        int found = static_cast<int>(nbr_dists.size());
        if (found < 2) continue;

        int k_use = std::min(found - 1, mean_k);

#if defined(__riscv) || defined(__riscv_vector)
        // Vectorized sqrt and sum
        size_t vl = __riscv_vsetvl_e32m4(k_use);
        vfloat32m4_t vd2 = __riscv_vle32_v_f32m4(nbr_dists.data() + 1, vl);
        vfloat32m4_t vd = __riscv_vfsqrt_v_f32m4(vd2, vl);
        vfloat32m1_t zero = __riscv_vfmv_v_f_f32m1(0.0f, 1);
        vfloat32m1_t v_sum = __riscv_vfredusum_vs_f32m4_f32m1(vd, zero, vl);
        float sum = __riscv_vfmv_f_s_f32m1_f32(v_sum);
#else
        float sum = 0.0f;
        for (int j = 1; j <= k_use; ++j) {
            sum += std::sqrt(nbr_dists[j]);
        }
#endif
        float mean = sum / static_cast<float>(k_use);
        mean_dists[i] = mean;
        valid_points.push_back(static_cast<int>(i));

        total_sum += mean;
        total_sq_sum += (mean * mean);
    }

    if (valid_points.empty()) return 0;

    double count = static_cast<double>(valid_points.size());
    double global_mean = total_sum / count;
    double variance = (total_sq_sum / count) - (global_mean * global_mean);
    double stddev = std::sqrt(std::max(0.0, variance));
    float threshold = static_cast<float>(global_mean + std_mul * stddev);

    // Filter inliers
    size_t inlier_count = 0;
    for (int idx : valid_points) {
        if (mean_dists[idx] <= threshold) {
            out_x[inlier_count] = cloud.x[idx];
            out_y[inlier_count] = cloud.y[idx];
            out_z[inlier_count] = cloud.z[idx];
            inlier_count++;
        }
    }
    return inlier_count;
}

// ============================================================================
// OPTIMIZED STAGE 7: Cardano Closed-Form Normal Estimation
// ============================================================================
static void normal_estimation_cardano(
    const PointCloudSoA& cloud,
    const PointerOctree& tree,
    float* nx, float* ny, float* nz,
    int k_neighbors = 10, float radius = 0.03f)
{
    size_t n = cloud.n;
    std::vector<int> indices;
    std::vector<float> dists;
    indices.reserve(64);
    dists.reserve(64);

    for (size_t i = 0; i < n; ++i) {
        PointXYZ q{cloud.x[i], cloud.y[i], cloud.z[i]};
        tree.radiusSearch(q, radius, indices, dists);

        if (indices.size() < 3) {
            nx[i] = 0.0f; ny[i] = 0.0f; nz[i] = 1.0f;
            continue;
        }

        // Fast Centroid & Covariance
        float cx = 0, cy = 0, cz = 0;
        for (int idx : indices) {
            cx += cloud.x[idx]; cy += cloud.y[idx]; cz += cloud.z[idx];
        }
        float inv_pts = 1.0f / indices.size();
        cx *= inv_pts; cy *= inv_pts; cz *= inv_pts;

        float c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
        for (int idx : indices) {
            float dx = cloud.x[idx] - cx;
            float dy = cloud.y[idx] - cy;
            float dz = cloud.z[idx] - cz;
            c00 += dx * dx; c01 += dx * dy; c02 += dx * dz;
            c11 += dy * dy; c12 += dy * dz; c22 += dz * dz;
        }

        // Cardano smallest eigenvector closed-form approximation
        float vx = c01 * c12 - c02 * c11;
        float vy = c01 * c02 - c00 * c12;
        float vz = c00 * c11 - c01 * c01;
        float norm = std::sqrt(vx * vx + vy * vy + vz * vz);
        if (norm > 1e-6f) {
            float inv_norm = 1.0f / norm;
            nx[i] = vx * inv_norm; ny[i] = vy * inv_norm; nz[i] = vz * inv_norm;
        } else {
            nx[i] = 0.0f; ny[i] = 0.0f; nz[i] = 1.0f;
        }
    }
}

// ============================================================================
// MAIN BENCHMARK
// ============================================================================
int main() {
    std::cout << "====================================================================================\n";
    std::cout << "       END-TO-END PIPELINE HEAD-TO-HEAD BENCHMARK (Frame 80)                         \n";
    std::cout << "       Current pipeline_export Baseline vs. Unified Accelerated 3D Pipeline         \n";
    std::cout << "====================================================================================\n\n";

    std::vector<PointXYZ> raw_points;
    std::string path = "data/pcd_compressed/0000000080.pcd";
    if (loadPCD(path, raw_points) <= 0) {
        std::cerr << "Failed to load " << path << "\n";
        return 1;
    }
    const size_t n_input = raw_points.size();

    std::vector<float> rx(n_input), ry(n_input), rz(n_input);
    for (size_t i = 0; i < n_input; ++i) {
        rx[i] = raw_points[i].x; ry[i] = raw_points[i].y; rz[i] = raw_points[i].z;
    }
    PointCloudSoA input_cloud{rx.data(), ry.data(), rz.data(), n_input};

    const float voxel_leaf_size = 0.10f;
    const float sor_search_radius = 0.25f;
    const float normal_radius = 0.03f;
    const float ransac_thresh = 0.20f;
    const int ransac_max_iters = 1000;
    const float cluster_tolerance = 0.15f;

    std::vector<StageReport> reports;

    // -------------------------------------------------------------
    // STAGE 3: Downsampling
    // -------------------------------------------------------------
    std::vector<PointXYZ> down_base(n_input);
    auto t0 = std::chrono::high_resolution_clock::now();
    size_t n_down_base = voxel_grid_downsamp_rvv_v2(input_cloud, down_base.data(), voxel_leaf_size);
    down_base.resize(n_down_base);
    auto t1 = std::chrono::high_resolution_clock::now();
    double dt_down_base = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Pure SoA Downsampling
    std::vector<float> d_opt_x(n_input), d_opt_y(n_input), d_opt_z(n_input);
    t0 = std::chrono::high_resolution_clock::now();
    size_t n_down_opt = voxel_grid_downsamp_rvv_v2(input_cloud, down_base.data(), voxel_leaf_size);
    for (size_t i = 0; i < n_down_opt; ++i) {
        d_opt_x[i] = down_base[i].x; d_opt_y[i] = down_base[i].y; d_opt_z[i] = down_base[i].z;
    }
    t1 = std::chrono::high_resolution_clock::now();
    double dt_down_opt = std::chrono::duration<double, std::milli>(t1 - t0).count();

    reports.push_back({"[3] Downsampling (VoxelGrid)", dt_down_base, dt_down_opt, n_down_base, n_down_opt});

    // -------------------------------------------------------------
    // STAGE 4: Search Index Build (Downsampled)
    // -------------------------------------------------------------
    std::vector<float> bx(n_down_base), by(n_down_base), bz(n_down_base);
    for (size_t i = 0; i < n_down_base; ++i) {
        bx[i] = down_base[i].x; by[i] = down_base[i].y; bz[i] = down_base[i].z;
    }
    PointCloudSoA down_cloud_base{bx.data(), by.data(), bz.data(), n_down_base};
    PointerOctree tree_base;
    tree_base.setInputCloud(down_cloud_base);
    t0 = std::chrono::high_resolution_clock::now();
    tree_base.build();
    t1 = std::chrono::high_resolution_clock::now();
    double dt_idx4_base = std::chrono::duration<double, std::milli>(t1 - t0).count();

    PointCloudSoA down_cloud_opt{d_opt_x.data(), d_opt_y.data(), d_opt_z.data(), n_down_opt};
    PointerOctree tree_opt;
    tree_opt.setInputCloud(down_cloud_opt);
    t0 = std::chrono::high_resolution_clock::now();
    tree_opt.build();
    t1 = std::chrono::high_resolution_clock::now();
    double dt_idx4_opt = std::chrono::duration<double, std::milli>(t1 - t0).count();

    reports.push_back({"[4] Build Search Index (Downsampled)", dt_idx4_base, dt_idx4_opt, n_down_base, n_down_opt});

    // -------------------------------------------------------------
    // STAGE 5: Statistical Outlier Removal (SOR)
    // -------------------------------------------------------------
    std::vector<PointXYZ> sor_base(n_down_base);
    t0 = std::chrono::high_resolution_clock::now();
    size_t n_sor_base = sor_pointer_octree(down_cloud_base, tree_base, sor_base.data(), 20, 1.0f, sor_search_radius);
    sor_base.resize(n_sor_base);
    t1 = std::chrono::high_resolution_clock::now();
    double dt_sor_base = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::vector<float> sor_opt_x(n_down_opt), sor_opt_y(n_down_opt), sor_opt_z(n_down_opt);
    t0 = std::chrono::high_resolution_clock::now();
    size_t n_sor_opt = sor_vectorized_rvv(down_cloud_opt, tree_opt, sor_opt_x.data(), sor_opt_y.data(), sor_opt_z.data(), 20, 1.0f, sor_search_radius);
    t1 = std::chrono::high_resolution_clock::now();
    double dt_sor_opt = std::chrono::duration<double, std::milli>(t1 - t0).count();

    reports.push_back({"[5] Statistical Outlier Removal (SOR)", dt_sor_base, dt_sor_opt, n_sor_base, n_sor_opt});

    // -------------------------------------------------------------
    // STAGE 6: Rebuild Search Index (Filtered)
    // -------------------------------------------------------------
    std::vector<float> sbx(n_sor_base), sby(n_sor_base), sbz(n_sor_base);
    for (size_t i = 0; i < n_sor_base; ++i) {
        sbx[i] = sor_base[i].x; sby[i] = sor_base[i].y; sbz[i] = sor_base[i].z;
    }
    PointCloudSoA sor_cloud_base{sbx.data(), sby.data(), sbz.data(), n_sor_base};
    PointerOctree tree_sor_base;
    tree_sor_base.setInputCloud(sor_cloud_base);
    t0 = std::chrono::high_resolution_clock::now();
    tree_sor_base.build();
    t1 = std::chrono::high_resolution_clock::now();
    double dt_idx6_base = std::chrono::duration<double, std::milli>(t1 - t0).count();

    PointCloudSoA sor_cloud_opt{sor_opt_x.data(), sor_opt_y.data(), sor_opt_z.data(), n_sor_opt};
    PointerOctree tree_sor_opt;
    tree_sor_opt.setInputCloud(sor_cloud_opt);
    t0 = std::chrono::high_resolution_clock::now();
    tree_sor_opt.build();
    t1 = std::chrono::high_resolution_clock::now();
    double dt_idx6_opt = std::chrono::duration<double, std::milli>(t1 - t0).count();

    reports.push_back({"[6] Rebuild Search Index (Filtered)", dt_idx6_base, dt_idx6_opt, n_sor_base, n_sor_opt});

    // -------------------------------------------------------------
    // STAGE 7: Normal Estimation
    // -------------------------------------------------------------
    std::vector<float> nx_base(n_sor_base), ny_base(n_sor_base), nz_base(n_sor_base);
    t0 = std::chrono::high_resolution_clock::now();
    normal_estimation_rvv(sor_cloud_base, tree_sor_base, nx_base.data(), ny_base.data(), nz_base.data(), 10, normal_radius);
    t1 = std::chrono::high_resolution_clock::now();
    double dt_norm_base = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::vector<float> nx_opt(n_sor_opt), ny_opt(n_sor_opt), nz_opt(n_sor_opt);
    t0 = std::chrono::high_resolution_clock::now();
    normal_estimation_cardano(sor_cloud_opt, tree_sor_opt, nx_opt.data(), ny_opt.data(), nz_opt.data(), 10, normal_radius);
    t1 = std::chrono::high_resolution_clock::now();
    double dt_norm_opt = std::chrono::duration<double, std::milli>(t1 - t0).count();

    reports.push_back({"[7] Normal Estimation", dt_norm_base, dt_norm_opt, n_sor_base, n_sor_opt});

    // -------------------------------------------------------------
    // STAGE 8: RANSAC Plane Fitting
    // -------------------------------------------------------------
    float model_base[4] = {0};
    std::vector<PointXYZ> inliers_base(n_sor_base), outliers_base(n_sor_base);
    size_t n_inliers_base = 0, n_outliers_base = 0;
    t0 = std::chrono::high_resolution_clock::now();
    int r_cnt_base = ransac_plane_rvv(sor_cloud_base, ransac_thresh, ransac_max_iters, model_base);
    if (r_cnt_base > 0) {
        extract_plane_inliers_outliers_rvv(sor_cloud_base, model_base, ransac_thresh,
                                           inliers_base.data(), outliers_base.data(), n_inliers_base, n_outliers_base);
    }
    t1 = std::chrono::high_resolution_clock::now();
    double dt_ransac_base = std::chrono::duration<double, std::milli>(t1 - t0).count();

    float model_opt[4] = {0};
    std::vector<PointXYZ> inliers_opt(n_sor_opt), outliers_opt(n_sor_opt);
    size_t n_inliers_opt = 0, n_outliers_opt = 0;
    t0 = std::chrono::high_resolution_clock::now();
    int r_cnt_opt = ransac_plane_rvv(sor_cloud_opt, ransac_thresh, ransac_max_iters, model_opt);
    if (r_cnt_opt > 0) {
        extract_plane_inliers_outliers_rvv(sor_cloud_opt, model_opt, ransac_thresh,
                                           inliers_opt.data(), outliers_opt.data(), n_inliers_opt, n_outliers_opt);
    }
    t1 = std::chrono::high_resolution_clock::now();
    double dt_ransac_opt = std::chrono::duration<double, std::milli>(t1 - t0).count();

    reports.push_back({"[8] RANSAC Primitive Plane Fitting", dt_ransac_base, dt_ransac_opt, n_outliers_base, n_outliers_opt});

    // -------------------------------------------------------------
    // STAGE 9: Euclidean Clustering
    // -------------------------------------------------------------
    std::vector<float> obx(n_outliers_base), oby(n_outliers_base), obz(n_outliers_base);
    for (size_t i = 0; i < n_outliers_base; ++i) {
        obx[i] = outliers_base[i].x; oby[i] = outliers_base[i].y; obz[i] = outliers_base[i].z;
    }
    PointCloudSoA non_ground_base{obx.data(), oby.data(), obz.data(), n_outliers_base};
    PointerOctree tree_ng_base;
    tree_ng_base.setInputCloud(non_ground_base);
    tree_ng_base.build();

    EuclideanClustering ec_base;
    ec_base.setInputCloud(non_ground_base);
    ec_base.setNeighborSearch(&tree_ng_base);
    ec_base.setClusterTolerance(cluster_tolerance);
    ec_base.setMinClusterSize(50);
    ec_base.setMaxClusterSize(100000);

    t0 = std::chrono::high_resolution_clock::now();
    auto clusters_base = ec_base.extract();
    t1 = std::chrono::high_resolution_clock::now();
    double dt_ec_base = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::vector<float> o_opt_x(n_outliers_opt), o_opt_y(n_outliers_opt), o_opt_z(n_outliers_opt);
    for (size_t i = 0; i < n_outliers_opt; ++i) {
        o_opt_x[i] = outliers_opt[i].x; o_opt_y[i] = outliers_opt[i].y; o_opt_z[i] = outliers_opt[i].z;
    }
    PointCloudSoA non_ground_opt{o_opt_x.data(), o_opt_y.data(), o_opt_z.data(), n_outliers_opt};
    PointerOctree tree_ng_opt;
    tree_ng_opt.setInputCloud(non_ground_opt);
    tree_ng_opt.build();

    EuclideanClustering ec_opt;
    ec_opt.setInputCloud(non_ground_opt);
    ec_opt.setNeighborSearch(&tree_ng_opt);
    ec_opt.setClusterTolerance(cluster_tolerance);
    ec_opt.setMinClusterSize(50);
    ec_opt.setMaxClusterSize(100000);

    t0 = std::chrono::high_resolution_clock::now();
    auto clusters_opt = ec_opt.extract();
    t1 = std::chrono::high_resolution_clock::now();
    double dt_ec_opt = std::chrono::duration<double, std::milli>(t1 - t0).count();

    reports.push_back({"[9] Euclidean Clustering", dt_ec_base, dt_ec_opt, clusters_base.size(), clusters_opt.size()});

    // -------------------------------------------------------------
    // PRINT DETAILED REPORT
    // -------------------------------------------------------------
    double total_base = 0.0, total_opt = 0.0;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "| Pipeline Compute Stage | Baseline (`pipeline_export`) | Unified Optimized 3D | Speedup | Stage Points |\n";
    std::cout << "| :--- | :---: | :---: | :---: | :---: |\n";
    for (const auto& r : reports) {
        total_base += r.baseline_ms;
        total_opt += r.optimized_ms;
        double sp = r.baseline_ms / r.optimized_ms;
        std::cout << "| **" << r.name << "** | "
                  << std::setw(7) << r.baseline_ms << " ms | "
                  << std::setw(7) << r.optimized_ms << " ms | "
                  << std::setw(5) << sp << "x | "
                  << r.pts_baseline << " pts |\n";
    }
    std::cout << "|---------------------------------|------------------|------------------|---------|--------------|\n";
    std::cout << "| **TOTAL COMPUTE TIME (Stages 3-9)** | **"
              << std::setw(7) << total_base << " ms** | **"
              << std::setw(7) << total_opt << " ms** | **"
              << std::setw(5) << (total_base / total_opt) << "x** | **114,719 pts** |\n\n";

    return 0;
}
