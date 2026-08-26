// ==============================================================================
// PIPELINE: eval/pipelines/pcl_qemu_pipeline.cpp
// PURPOSE: Upstream Point Cloud Library (PCL 1.14.0) Perception Pipeline with
//          Comprehensive Stage-by-Stage PCD Output Exports (for QEMU / Hardware).
// ==============================================================================

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Upstream PCL 1.14 Template Implementations
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/impl/filter_indices.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/impl/statistical_outlier_removal.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/impl/feature.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>

// Direct PCL RANSAC Plane Model (Standard PCL Template)
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/impl/ransac.hpp>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>

#include <cstdarg>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "core/pipeline_params.h"
#include "io/simple_pcd_loader.h"

// PCL Console Logging Support for Static Builds
namespace pcl::console {
void print(VERBOSITY_LEVEL, const char* format, ...) {
    va_list args;
    va_start(args, format);
    vfprintf(stdout, format, args);
    va_end(args);
}
void change_text_color(FILE*, int, int) {}
void reset_text_color(FILE*) {}
bool isVerbosityLevelEnabled(VERBOSITY_LEVEL) { return true; }
void setVerbosityLevel(VERBOSITY_LEVEL) {}
VERBOSITY_LEVEL getVerbosityLevel() { return L_INFO; }
}

using Clock = std::chrono::high_resolution_clock;

// ── Helper: Load PCD into PCL PointCloud ─────────────────────────────────────
static bool loadPCDToPCL(const std::string &path, pcl::PointCloud<pcl::PointXYZ> &cloud) {
    std::vector<rvpoint::PointXYZ> pts;
    if (!rvpoint::loadPCD(path, pts) || pts.empty()) {
        return false;
    }
    cloud.clear();
    cloud.reserve(pts.size());
    for (const auto &p : pts) {
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
            cloud.push_back(pcl::PointXYZ(p.x, p.y, p.z));
        }
    }
    cloud.width = cloud.size();
    cloud.height = 1;
    cloud.is_dense = true;
    return !cloud.empty();
}

// ── Helper: Save PCL PointCloud to Disk ──────────────────────────────────────
template <typename PointT>
static bool savePCLToPCD(const std::string &path, const pcl::PointCloud<PointT> &cloud, bool binary = true) {
    std::vector<rvpoint::PointXYZ> pts;
    pts.reserve(cloud.size());
    for (const auto &p : cloud) {
        pts.push_back({p.x, p.y, p.z});
    }
    return rvpoint::savePCD(path, pts, binary);
}

// ── Helper: Save Multi-Colored Clusters to PCD ──────────────────────────────
static bool saveClustersColored(const std::string &path,
                                const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                const std::vector<pcl::PointIndices> &clusters) {
    static const uint32_t palette[] = {
        0xFF3333, 0x33FF33, 0x3333FF, 0xFFFF33, 0xFF33FF, 0x33FFFF,
        0xFFA500, 0x800080, 0x008000, 0x000080, 0xFFC0CB, 0x7FFFD4,
        0xA52A2A, 0x5F9EA0, 0xD2691E, 0xFF7F50, 0x6495ED, 0xDC143C
    };
    const size_t num_colors = sizeof(palette) / sizeof(palette[0]);

    std::vector<rvpoint::PointXYZRGB> rgb_pts;
    for (size_t c_idx = 0; c_idx < clusters.size(); ++c_idx) {
        uint32_t color = palette[c_idx % num_colors];
        uint8_t r = static_cast<uint8_t>((color >> 16) & 0xFF);
        uint8_t g = static_cast<uint8_t>((color >> 8) & 0xFF);
        uint8_t b = static_cast<uint8_t>(color & 0xFF);
        for (int idx : clusters[c_idx].indices) {
            const auto &p = cloud[idx];
            rgb_pts.push_back({p.x, p.y, p.z, r, g, b});
        }
    }
    return rvpoint::savePCDRGB(path, rgb_pts, false);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input.pcd> [output_dir] [--preset <dense|tabletop|sparse>] [--leaf-size|--delta <val>] [--cluster-tolerance <val>] [--min-cluster <val>] [--max-cluster <val>] [--ransac-iters <val>] [--no-write] [--progress]\n";
        return 1;
    }

    std::string input_path = "";
    std::string output_dir_str = "";
    rvpoint::PipelineConfig cfg = rvpoint::PipelineConfig::fromDelta(0.02f);
    bool disable_disk = false;
    bool progress_enabled = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--preset" && i + 1 < argc) {
            cfg = rvpoint::PipelineConfig::fromPreset(argv[++i]);
        } else if ((arg == "--leaf-size" || arg == "--delta") && i + 1 < argc) {
            cfg = rvpoint::PipelineConfig::fromDelta(std::stof(argv[++i]));
        } else if (arg == "--cluster-tolerance" && i + 1 < argc) {
            cfg.cluster_tolerance = std::stof(argv[++i]);
        } else if (arg == "--min-cluster" && i + 1 < argc) {
            cfg.min_cluster_size = std::stoi(argv[++i]);
        } else if (arg == "--max-cluster" && i + 1 < argc) {
            cfg.max_cluster_size = std::stoi(argv[++i]);
        } else if (arg == "--ransac-iters" && i + 1 < argc) {
            cfg.ransac_max_iterations = std::stoi(argv[++i]);
        } else if (arg == "--no-write" || arg == "--disable-disk") {
            disable_disk = true;
        } else if (arg == "--progress") {
            progress_enabled = true;
        } else if (arg[0] != '-') {
            if (input_path.empty()) {
                input_path = arg;
            } else if (output_dir_str.empty()) {
                output_dir_str = arg;
            }
        }
    }

    if (input_path.empty()) {
        std::cerr << "Error: No input PCD path specified.\n";
        return 1;
    }

    // Set up output destination directory
    std::filesystem::path input_stem = std::filesystem::path(input_path).stem();
    std::filesystem::path out_dir;
    if (!output_dir_str.empty()) {
        out_dir = output_dir_str;
    } else {
        out_dir = std::filesystem::path("output") / (input_stem.string() + "_pcl_qemu");
    }

    if (!disable_disk) {
        std::error_code ec;
        std::filesystem::create_directories(out_dir, ec);
        std::filesystem::create_directories(out_dir / "05_clusters", ec);
    }

    std::cout << "[pcl-qemu] Loading: " << input_path << " (leaf = " << cfg.voxel_leaf_size << " m)\n";
    std::cout << "[pcl-qemu] Output Destination: " << out_dir.string() << "\n";

    // ── Pre-Pipeline Stage: Load PCD (Disk I/O) ──────────────────────────
    const auto t_io_load_start = Clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (!loadPCDToPCL(input_path, *cloud) || cloud->empty()) {
        std::cerr << "[pcl-qemu] Error: Failed to load " << input_path << "\n";
        return 1;
    }
    const double io_load_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_io_load_start).count();
    std::cout << "[pcl-qemu] [IO] Loaded Points: " << cloud->size() << " in " << io_load_ms << " ms\n";

    if (!disable_disk) {
        savePCLToPCD((out_dir / "00_input.pcd").string(), *cloud, true);
    }

    // ── PURE COMPUTE PIPELINE START ──────────────────────────────────────
    const auto t_compute_start = Clock::now();

    // Stage 1: Voxel Grid Downsampling
    const auto t1_start = Clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr down_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(cfg.voxel_leaf_size, cfg.voxel_leaf_size, cfg.voxel_leaf_size);
    vg.filter(*down_cloud);
    if (down_cloud->empty()) {
        *down_cloud = *cloud;
    }
    const double t1_ms = std::chrono::duration<double, std::milli>(Clock::now() - t1_start).count();
    std::cout << "[pcl-qemu] [1/5] Downsampled: " << down_cloud->size() << " pts (" << t1_ms << " ms)\n";

    if (!disable_disk) {
        savePCLToPCD((out_dir / "01_downsampled.pcd").string(), *down_cloud, true);
    }

    // Stage 2: Statistical Outlier Removal (SOR)
    const auto t2_start = Clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(down_cloud);
    int valid_k = std::min(cfg.sor_mean_k, static_cast<int>(down_cloud->size()) - 1);
    if (valid_k > 0) {
        sor.setMeanK(valid_k);
        sor.setStddevMulThresh(cfg.sor_std_threshold);
        sor.filter(*sor_cloud);
    }
    if (sor_cloud->empty()) {
        *sor_cloud = *down_cloud;
    }
    const double t2_ms = std::chrono::duration<double, std::milli>(Clock::now() - t2_start).count();
    std::cout << "[pcl-qemu] [2/5] SOR Filtered: " << sor_cloud->size() << " pts (" << t2_ms << " ms)\n";

    if (!disable_disk) {
        savePCLToPCD((out_dir / "02_sor_filtered.pcd").string(), *sor_cloud, true);
    }

    // Stage 3: Surface Normal Estimation (KdTree)
    const auto t3_start = Clock::now();
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(sor_cloud);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(sor_cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(cfg.search_radius);
    ne.compute(*normals);
    const double t3_ms = std::chrono::duration<double, std::milli>(Clock::now() - t3_start).count();
    std::cout << "[pcl-qemu] [3/5] Normals Computed: " << normals->size() << " pts (" << t3_ms << " ms)\n";

    // Stage 4: RANSAC Ground Plane Segmentation
    const auto t4_start = Clock::now();
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(
        new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(sor_cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);
    ransac.setDistanceThreshold(cfg.ransac_distance_threshold);
    ransac.setMaxIterations(cfg.ransac_max_iterations);
    ransac.computeModel();

    std::vector<int> inliers;
    ransac.getInliers(inliers);

    pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices());
    inliers_ptr->indices = inliers;

    // Extract Ground Plane Inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_inliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ExtractIndices<pcl::PointXYZ> extract_inliers;
    extract_inliers.setInputCloud(sor_cloud);
    extract_inliers.setIndices(inliers_ptr);
    extract_inliers.setNegative(false);
    extract_inliers.filter(*plane_inliers);

    // Extract Non-Ground Obstacles
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ExtractIndices<pcl::PointXYZ> extract_obstacles;
    extract_obstacles.setInputCloud(sor_cloud);
    extract_obstacles.setIndices(inliers_ptr);
    extract_obstacles.setNegative(true);
    extract_obstacles.filter(*non_ground);
    const double t4_ms = std::chrono::duration<double, std::milli>(Clock::now() - t4_start).count();
    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);
    std::cout << "[pcl-qemu] [4/5] Non-Ground Points: " << non_ground->size() 
              << " (Plane inliers: " << inliers.size() << ") (" << t4_ms << " ms)\n";
    if (coeff.size() >= 4) {
        std::cout << "       [RANSAC] Model: (" << coeff[0] << ", " << coeff[1] << ", " << coeff[2] << ", " << coeff[3] << ")\n";
    }

    if (!disable_disk) {
        savePCLToPCD((out_dir / "04_plane_inliers.pcd").string(), *plane_inliers, true);
        savePCLToPCD((out_dir / "04_non_ground.pcd").string(), *non_ground, true);
    }

    // Stage 5: Euclidean Clustering (KdTree)
    const auto t5_start = Clock::now();
    std::vector<pcl::PointIndices> clusters;
    if (!non_ground->empty()) {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr cluster_tree(new pcl::search::KdTree<pcl::PointXYZ>());
        cluster_tree->setInputCloud(non_ground);

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cfg.cluster_tolerance);
        ec.setMinClusterSize(cfg.min_cluster_size);
        ec.setMaxClusterSize(cfg.max_cluster_size);
        ec.setSearchMethod(cluster_tree);
        ec.setInputCloud(non_ground);
        ec.extract(clusters);
    }
    const double t5_ms = std::chrono::duration<double, std::milli>(Clock::now() - t5_start).count();
    std::cout << "[pcl-qemu] [5/5] Clusters Extracted: " << clusters.size() << " (" << t5_ms << " ms)\n";

    if (!disable_disk && !clusters.empty()) {
        saveClustersColored((out_dir / "05_clusters_colored.pcd").string(), *non_ground, clusters);
        for (size_t c_idx = 0; c_idx < clusters.size(); ++c_idx) {
            pcl::PointCloud<pcl::PointXYZ> cluster_cloud;
            for (int idx : clusters[c_idx].indices) {
                cluster_cloud.push_back((*non_ground)[idx]);
            }
            std::ostringstream oss;
            oss << "cluster_" << std::setw(3) << std::setfill('0') << c_idx << ".pcd";
            savePCLToPCD((out_dir / "05_clusters" / oss.str()).string(), cluster_cloud, true);
        }
    }

    const auto t_compute_end = Clock::now();
    const double compute_total_ms = std::chrono::duration<double, std::milli>(t_compute_end - t_compute_start).count();

    // ── Final Compute Timing Breakdown ───────────────────────────────────
    std::cout << "\n============================================================\n";
    std::cout << " [pcl-qemu] PCL 1.14 QEMU Stage Export Timing Breakdown\n";
    std::cout << "============================================================\n";
    std::cout << "  [1/5] Voxel Grid Downsampling  : " << std::fixed << std::setprecision(3) << t1_ms << " ms ("
              << (compute_total_ms > 0 ? t1_ms / compute_total_ms * 100.0 : 0.0) << "%)\n";
    std::cout << "  [2/5] Statistical Outlier (SOR): " << std::fixed << std::setprecision(3) << t2_ms << " ms ("
              << (compute_total_ms > 0 ? t2_ms / compute_total_ms * 100.0 : 0.0) << "%)\n";
    std::cout << "  [3/5] Normal Estimation (KdTree): " << std::fixed << std::setprecision(3) << t3_ms << " ms ("
              << (compute_total_ms > 0 ? t3_ms / compute_total_ms * 100.0 : 0.0) << "%)\n";
    std::cout << "  [4/5] RANSAC Plane Segmentation: " << std::fixed << std::setprecision(3) << t4_ms << " ms ("
              << (compute_total_ms > 0 ? t4_ms / compute_total_ms * 100.0 : 0.0) << "%)\n";
    std::cout << "  [5/5] Euclidean Clustering     : " << std::fixed << std::setprecision(3) << t5_ms << " ms ("
              << (compute_total_ms > 0 ? t5_ms / compute_total_ms * 100.0 : 0.0) << "%)\n";
    std::cout << "------------------------------------------------------------\n";
    std::cout << "  Total Pipeline Time (Compute Only): " << std::fixed << std::setprecision(3) << compute_total_ms << " ms\n";
    std::cout << "  Pre-Pipeline File Load (Disk I/O) : " << std::fixed << std::setprecision(3) << io_load_ms << " ms\n";
    if (!disable_disk) {
        std::cout << "  Saved Stage Artifacts Directory   : " << out_dir.string() << "\n";
    }
    std::cout << "============================================================\n\n";

    return 0;
}
