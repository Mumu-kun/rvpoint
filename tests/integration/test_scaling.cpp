#include "rvv_pcl.h"
#include "simple_pcd_loader.h"
#include "euclidean_clustering.h"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>
#include <algorithm>

using namespace rvv_pcl;

int main() {
    std::cout << "==========================================================================" << std::endl;
    std::cout << "  RVPoint Pipeline Scaling Benchmark Across Cloud Sizes (Real Sample)" << std::endl;
    std::cout << "==========================================================================" << std::endl;

    std::vector<PointXYZ> full_points;
    int count = loadPCD("data/pcd_compressed/0000000010.pcd", full_points);
    if (count <= 0) {
        std::cerr << "Error: Could not load data/pcd_compressed/0000000010.pcd" << std::endl;
        return 1;
    }

    std::cout << "Loaded base cloud with " << full_points.size() << " points." << std::endl << std::endl;

    std::vector<std::size_t> sizes = {10000, 25000, 50000, 75000, 100000, full_points.size()};

    std::cout << std::left << std::setw(10) << "N (points)"
              << std::setw(15) << "Voxel (ms)"
              << std::setw(15) << "SOR (ms)"
              << std::setw(15) << "Normal (ms)"
              << std::setw(15) << "RANSAC (ms)"
              << std::setw(15) << "Cluster (ms)"
              << std::setw(15) << "Total (ms)" << std::endl;
    std::cout << std::string(100, '-') << std::endl;

    for (std::size_t n : sizes) {
        if (n > full_points.size()) n = full_points.size();

        std::vector<PointXYZ> sub_pts(full_points.begin(), full_points.begin() + n);

        std::vector<float> x(n), y(n), z(n);
        for (std::size_t i = 0; i < n; ++i) {
            x[i] = sub_pts[i].x;
            y[i] = sub_pts[i].y;
            z[i] = sub_pts[i].z;
        }
        PointCloudSoA in_soa = {x.data(), y.data(), z.data(), n};

        // 1. Voxel Grid Downsampling
        std::vector<PointXYZ> vg_out(n);
        auto t0 = std::chrono::high_resolution_clock::now();
        std::size_t n_vg = voxel_grid_downsamp_rvv_v2(in_soa, vg_out.data(), 0.05f);
        auto t1 = std::chrono::high_resolution_clock::now();
        double ms_vg = std::chrono::duration<double, std::milli>(t1 - t0).count();
        vg_out.resize(n_vg);

        std::vector<float> vx(n_vg), vy(n_vg), vz(n_vg);
        for (std::size_t i = 0; i < n_vg; ++i) {
            vx[i] = vg_out[i].x; vy[i] = vg_out[i].y; vz[i] = vg_out[i].z;
        }
        PointCloudSoA vg_soa = {vx.data(), vy.data(), vz.data(), n_vg};

        // 2. Statistical Outlier Removal
        Octree vg_octree;
        vg_octree.setInputCloud(vg_soa);
        vg_octree.build();

        std::vector<PointXYZ> sor_out(n_vg);
        t0 = std::chrono::high_resolution_clock::now();
        std::size_t n_sor = sor_octree(vg_soa, vg_octree, sor_out.data(), 30, 1.0f, 0.5f);
        t1 = std::chrono::high_resolution_clock::now();
        double ms_sor = std::chrono::duration<double, std::milli>(t1 - t0).count();
        sor_out.resize(n_sor);

        std::vector<float> sx(n_sor), sy(n_sor), sz(n_sor);
        for (std::size_t i = 0; i < n_sor; ++i) {
            sx[i] = sor_out[i].x; sy[i] = sor_out[i].y; sz[i] = sor_out[i].z;
        }
        PointCloudSoA sor_soa = {sx.data(), sy.data(), sz.data(), n_sor};

        // 3. Normal Estimation
        Octree sor_octree;
        sor_octree.setInputCloud(sor_soa);
        sor_octree.build();

        std::vector<float> nx(n_sor), ny(n_sor), nz(n_sor);
        t0 = std::chrono::high_resolution_clock::now();
        normal_estimation_rvv(sor_soa, sor_octree, nx.data(), ny.data(), nz.data(), 10, 0.5f);
        t1 = std::chrono::high_resolution_clock::now();
        double ms_normal = std::chrono::duration<double, std::milli>(t1 - t0).count();

        // 4. RANSAC Plane Fitting
        float model[4];
        std::vector<PointXYZ> inliers(n_sor), outliers(n_sor);
        std::size_t n_inliers = 0, n_outliers = 0;
        t0 = std::chrono::high_resolution_clock::now();
        int inlier_cnt = ransac_plane_rvv(sor_soa, 0.2f, 100, model);
        if (inlier_cnt > 0) {
            extract_plane_inliers_outliers_rvv(sor_soa, model, 0.2f, inliers.data(), outliers.data(), n_inliers, n_outliers);
        }
        t1 = std::chrono::high_resolution_clock::now();
        double ms_ransac = std::chrono::duration<double, std::milli>(t1 - t0).count();
        outliers.resize(n_outliers);

        // 5. Euclidean Clustering
        std::vector<float> ox(n_outliers), oy(n_outliers), oz(n_outliers);
        for (std::size_t i = 0; i < n_outliers; ++i) {
            ox[i] = outliers[i].x; oy[i] = outliers[i].y; oz[i] = outliers[i].z;
        }
        PointCloudSoA non_ground_soa = {ox.data(), oy.data(), oz.data(), n_outliers};

        Octree non_ground_octree;
        non_ground_soa = {ox.data(), oy.data(), oz.data(), n_outliers};
        non_ground_octree.setInputCloud(non_ground_soa);
        non_ground_octree.build();

        EuclideanClustering ec;
        ec.setInputCloud(non_ground_soa);
        ec.setNeighborSearch(&non_ground_octree);
        ec.setClusterTolerance(0.5f);
        ec.setMinClusterSize(10);
        ec.setMaxClusterSize(25000);

        t0 = std::chrono::high_resolution_clock::now();
        auto clusters = ec.extract();
        t1 = std::chrono::high_resolution_clock::now();
        double ms_cluster = std::chrono::duration<double, std::milli>(t1 - t0).count();

        double ms_total = ms_vg + ms_sor + ms_normal + ms_ransac + ms_cluster;

        std::cout << std::left << std::setw(10) << n
                  << std::setw(15) << std::fixed << std::setprecision(2) << ms_vg
                  << std::setw(15) << ms_sor
                  << std::setw(15) << ms_normal
                  << std::setw(15) << ms_ransac
                  << std::setw(15) << ms_cluster
                  << std::setw(15) << ms_total << std::endl;
    }
    return 0;
}
