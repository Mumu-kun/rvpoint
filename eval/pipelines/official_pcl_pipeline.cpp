// eval/pipelines/official_pcl_pipeline.cpp
// Official Debian PCL 1.14 (Point Cloud Library) 10-Stage Evaluation Pipeline
// Directly uses official PCL algorithms (pcl::VoxelGrid, pcl::StatisticalOutlierRemoval,
// pcl::NormalEstimation, pcl::SACSegmentation, pcl::EuclideanClusterExtraction).

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

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/octree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using Clock = std::chrono::high_resolution_clock;

namespace {

constexpr int kStageCount = 10;

struct StageTiming {
    int index;
    std::string label;
    double ms;
    std::size_t point_count;
};

struct PipelineConfig {
    float voxel_leaf_size = 0.10f;
    float sor_search_radius = 0.25f;
    int sor_mean_k = 20;
    float sor_std_threshold = 1.0f;
    int normal_k = 10;
    float normal_radius = 0.25f;
    float ransac_distance_threshold = 0.20f;
    int ransac_max_iterations = 250;
    float cluster_tolerance = 0.15f;
    int min_cluster_size = 50;
    int max_cluster_size = 100000;
};

void beginStage(int stage_idx, const std::string &label, bool enabled) {
    if (enabled) {
        std::cout << "[pcl-official] [" << stage_idx << "/" << kStageCount << "] "
                  << label << "..." << std::endl;
    }
}

double endStage(int stage_idx, const std::string &label,
                const Clock::time_point &start_time, bool enabled) {
    const auto end_time = Clock::now();
    const double ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    if (enabled) {
        std::cout << "[pcl-official] [" << stage_idx << "/" << kStageCount << "] "
                  << label << " complete in " << std::fixed << std::setprecision(3) << ms << " ms" << std::endl;
    }
    return ms;
}

void printFinalBreakdown(const std::vector<StageTiming> &stages, double total_ms) {
    std::cout << "\n==========================================================================" << std::endl;
    std::cout << " OFFICIAL PCL 1.14 PIPELINE (LIBPCL-DEV RISC-V QEMU RUNTIME)" << std::endl;
    std::cout << "==========================================================================" << std::endl;
    double measured_sum = 0.0;
    for (const auto &st : stages) {
        measured_sum += st.ms;
        const double pct = total_ms > 0.0 ? (st.ms / total_ms) * 100.0 : 0.0;
        std::cout << " [" << std::setw(2) << st.index << "/" << kStageCount << "] "
                  << std::left << std::setw(42) << st.label << ": "
                  << std::right << std::fixed << std::setw(10) << std::setprecision(3) << st.ms << " ms "
                  << "(" << std::setw(6) << std::setprecision(2) << pct << "%), pts="
                  << st.point_count << std::endl;
    }
    const double outside_ms = total_ms - measured_sum;
    const double outside_pct = total_ms > 0.0 ? (outside_ms / total_ms) * 100.0 : 0.0;
    std::cout << " [--] " << std::left << std::setw(42) << "Outside timed stages" << ": "
              << std::right << std::fixed << std::setw(10) << std::setprecision(3) << outside_ms << " ms "
              << "(" << std::setw(6) << std::setprecision(2) << outside_pct << "%)" << std::endl;
    std::cout << "--------------------------------------------------------------------------" << std::endl;
    std::cout << " [--] " << std::left << std::setw(42) << "Total Official PCL Pipeline Time" << ": "
              << std::right << std::fixed << std::setw(10) << std::setprecision(3) << total_ms << " ms (100.00%)" << std::endl;
    std::cout << "==========================================================================\n" << std::endl;
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

} // namespace

int main(int argc, char** argv) {
    bool progress_enabled = false;
    bool json_metrics = false;
    bool skip_sor = false;
    bool skip_normals = false;
    bool disable_disk = false;
    PipelineConfig cfg;

    std::vector<std::string> positional_args;
    std::vector<StageTiming> stage_timings;
    stage_timings.reserve(kStageCount);

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--progress") {
            progress_enabled = true;
        } else if (arg == "--json" || arg == "--json-metrics") {
            json_metrics = true;
        } else if (arg == "--skip-sor" || arg == "--no-sor") {
            skip_sor = true;
        } else if (arg == "--no-normals" || arg == "--skip-normals") {
            skip_normals = true;
        } else if (arg == "--no-write" || arg == "--disable-disk") {
            disable_disk = true;
        } else if (arg == "--leaf-size") {
            if (i + 1 < argc) cfg.voxel_leaf_size = std::stof(argv[++i]);
        } else if (arg == "--cluster-tolerance") {
            if (i + 1 < argc) cfg.cluster_tolerance = std::stof(argv[++i]);
        } else if (arg == "--min-cluster") {
            if (i + 1 < argc) cfg.min_cluster_size = std::stoi(argv[++i]);
        } else if (arg == "--max-cluster") {
            if (i + 1 < argc) cfg.max_cluster_size = std::stoi(argv[++i]);
        } else if (arg == "--ransac-iters") {
            if (i + 1 < argc) cfg.ransac_max_iterations = std::stoi(argv[++i]);
        } else if (!arg.empty() && arg[0] == '-') {
            std::cerr << "Warning: Unrecognized option '" << arg << "'\n";
        } else {
            positional_args.push_back(arg);
        }
    }

    if (positional_args.empty()) {
        std::cerr << "Usage: " << argv[0]
                  << " [--progress] [--json] [--no-write] [--no-normals] [--skip-sor] "
                     "[--leaf-size <val>] [--cluster-tolerance <val>] [--min-cluster <val>] "
                     "[--max-cluster <val>] [--ransac-iters <val>] <input.pcd> [output_dir]\n";
        return 1;
    }

    const auto overall_start = Clock::now();
    const std::string input_path = resolveInputPath(positional_args[0]);
    const std::filesystem::path input_stem = std::filesystem::path(positional_args[0]).stem();
    const std::filesystem::path output_dir =
        positional_args.size() >= 2 ? std::filesystem::path(positional_args[1])
                                    : std::filesystem::path("results") / (input_stem.string() + "_official_pcl");

    if (!disable_disk) {
        std::error_code dir_ec;
        std::filesystem::create_directories(output_dir, dir_ec);
    }

    // ── Stage 1: Load input cloud ──────────────────────────────────────────
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    beginStage(1, "Load input cloud (PCL)", progress_enabled);
    auto stage_start = Clock::now();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_path, *input_cloud) == -1 || input_cloud->empty()) {
        std::cerr << "Failed to load input PCD via PCL: " << input_path << std::endl;
        return 1;
    }
    const size_t n_input = input_cloud->size();
    stage_timings.push_back({1, "Load input cloud", endStage(1, "Load input cloud", stage_start, progress_enabled), n_input});

    // ── Stage 2: Write input stage ─────────────────────────────────────────
    beginStage(2, "Write input stage", progress_enabled);
    stage_start = Clock::now();
    if (!disable_disk) {
        pcl::io::savePCDFileBinary((output_dir / "00_input.pcd").string(), *input_cloud);
    }
    stage_timings.push_back({2, "Write input stage", endStage(2, "Write input stage", stage_start, progress_enabled), n_input});

    // ── Stage 3: Downsampling (pcl::VoxelGrid) ──────────────────────────────
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    beginStage(3, "Downsampling (pcl::VoxelGrid)", progress_enabled);
    stage_start = Clock::now();
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(input_cloud);
    vg.setLeafSize(cfg.voxel_leaf_size, cfg.voxel_leaf_size, cfg.voxel_leaf_size);
    vg.filter(*downsampled_cloud);
    const size_t n_down = downsampled_cloud->size();
    stage_timings.push_back({3, "Downsampling", endStage(3, "Downsampling", stage_start, progress_enabled), n_down});
    if (!disable_disk) {
        pcl::io::savePCDFileBinary((output_dir / "01_downsampled.pcd").string(), *downsampled_cloud);
    }

    // ── Stage 4: Build Search Index (pcl::search::KdTree) ───────────────────
    beginStage(4, "Build search index for downsampled cloud", progress_enabled);
    stage_start = Clock::now();
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_down(new pcl::search::KdTree<pcl::PointXYZ>);
    if (!skip_sor) {
        kdtree_down->setInputCloud(downsampled_cloud);
    }
    stage_timings.push_back({4, "Build search index for downsampled cloud",
                             endStage(4, "Build search index for downsampled cloud", stage_start, progress_enabled), n_down});

    // ── Stage 5: Statistical Outlier Removal (pcl::StatisticalOutlierRemoval) ─
    pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    beginStage(5, "Statistical outlier removal (pcl::SOR)", progress_enabled);
    stage_start = Clock::now();
    if (skip_sor) {
        *sor_cloud = *downsampled_cloud;
    } else {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(downsampled_cloud);
        sor.setSearchMethod(kdtree_down);
        sor.setMeanK(cfg.sor_mean_k);
        sor.setStddevMulThresh(cfg.sor_std_threshold);
        sor.filter(*sor_cloud);
    }
    const size_t n_sor = sor_cloud->size();
    stage_timings.push_back({5, "Statistical outlier removal", endStage(5, "Statistical outlier removal", stage_start, progress_enabled), n_sor});
    if (!disable_disk) {
        pcl::io::savePCDFileBinary((output_dir / "02_sor_filtered.pcd").string(), *sor_cloud);
    }

    // ── Stage 6: Rebuild Search Index for Filtered Cloud ────────────────────
    beginStage(6, "Rebuild search index for filtered cloud", progress_enabled);
    stage_start = Clock::now();
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_filt(new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree_filt->setInputCloud(sor_cloud);
    stage_timings.push_back({6, "Rebuild search index for filtered cloud",
                             endStage(6, "Rebuild search index for filtered cloud", stage_start, progress_enabled), n_sor});

    // ── Stage 7: Normal Estimation (pcl::NormalEstimation) ──────────────────
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    beginStage(7, "Normal estimation (pcl::NormalEstimation)", progress_enabled);
    stage_start = Clock::now();
    if (!skip_normals) {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(sor_cloud);
        ne.setSearchMethod(kdtree_filt);
        ne.setKSearch(cfg.normal_k);
        ne.compute(*cloud_normals);
    }
    stage_timings.push_back({7, "Normal estimation", endStage(7, "Normal estimation", stage_start, progress_enabled), n_sor});

    // ── Stage 8: RANSAC Plane Fitting (pcl::SACSegmentation) ────────────────
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_inliers(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    beginStage(8, "RANSAC primitive fitting (pcl::SACSegmentation)", progress_enabled);
    stage_start = Clock::now();
    if (n_sor >= 3) {
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(cfg.ransac_max_iterations);
        seg.setDistanceThreshold(cfg.ransac_distance_threshold);
        seg.setInputCloud(sor_cloud);
        seg.segment(*inliers, *coefficients);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(sor_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*plane_inliers);

        extract.setNegative(true);
        extract.filter(*non_ground_cloud);
    } else {
        *non_ground_cloud = *sor_cloud;
    }
    const size_t n_non_ground = non_ground_cloud->size();
    stage_timings.push_back({8, "RANSAC primitive fitting", endStage(8, "RANSAC primitive fitting", stage_start, progress_enabled), n_non_ground});

    if (!disable_disk) {
        pcl::io::savePCDFileBinary((output_dir / "04_ransac_inliers.pcd").string(), *plane_inliers);
        pcl::io::savePCDFileBinary((output_dir / "05_ground_plane_removed.pcd").string(), *non_ground_cloud);
    }

    // ── Stage 9: Euclidean Clustering (pcl::EuclideanClusterExtraction) ─────
    std::vector<pcl::PointIndices> cluster_indices;
    beginStage(9, "Euclidean clustering (pcl::EuclideanClusterExtraction)", progress_enabled);
    stage_start = Clock::now();
    if (n_non_ground > 0) {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_clusters(new pcl::search::KdTree<pcl::PointXYZ>);
        kdtree_clusters->setInputCloud(non_ground_cloud);

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cfg.cluster_tolerance);
        ec.setMinClusterSize(cfg.min_cluster_size);
        ec.setMaxClusterSize(cfg.max_cluster_size);
        ec.setSearchMethod(kdtree_clusters);
        ec.setInputCloud(non_ground_cloud);
        ec.extract(cluster_indices);
    }
    stage_timings.push_back({9, "Euclidean clustering", endStage(9, "Euclidean clustering", stage_start, progress_enabled), cluster_indices.size()});

    // ── Stage 10: Write Cluster Stage ───────────────────────────────────────
    beginStage(10, "Write cluster stage", progress_enabled);
    stage_start = Clock::now();
    size_t total_clustered_points = 0;
    if (!disable_disk) {
        int cluster_id = 0;
        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& idx : indices.indices) {
                cluster->push_back((*non_ground_cloud)[idx]);
            }
            total_clustered_points += cluster->size();
            std::string cname = "cluster_" + std::to_string(cluster_id++) + ".pcd";
            pcl::io::savePCDFileBinary((output_dir / cname).string(), *cluster);
        }
    } else {
        for (const auto& indices : cluster_indices) {
            total_clustered_points += indices.indices.size();
        }
    }
    stage_timings.push_back({10, "Write cluster stage", endStage(10, "Write cluster stage", stage_start, progress_enabled), total_clustered_points});

    const auto overall_end = Clock::now();
    const double total_ms = std::chrono::duration<double, std::milli>(overall_end - overall_start).count();

    printFinalBreakdown(stage_timings, total_ms);
    return 0;
}
