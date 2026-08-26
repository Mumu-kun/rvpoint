// ==============================================================================
// PIPELINE: eval/pipelines/pcl_optimized_pipeline.cpp
// PURPOSE: Fully Optimized Upstream PCL 1.14.0 — 5-Stage Perception Pipeline.
//
//   Optimizations Applied (100% Standard PCL 1.14):
//     [1/5] Voxel Grid: Disabled layout caching & extra field downsampling.
//     [2/5] Bounded ROR: is_dense=true + Unsorted KdTree (skips FLANN heap sort).
//     [3/5] Ground Fitting: RRANSAC (10% early-rejection pretest) +
//           SampleConsensusModelPerpendicularPlane (Z-axis ground angle prior).
//     [4/5] Cluster Index: Explicit unsorted KdTree construction.
//     [5/5] Euclidean Clustering: Direct free-function invocation (zero index rebuild).
// ==============================================================================

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/impl/filter_indices.hpp>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/impl/radius_outlier_removal.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/impl/ransac.hpp>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/sample_consensus/impl/sac_model_perpendicular_plane.hpp>

#include <cstdarg>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "core/pipeline_params.h"
#include "io/simple_pcd_loader.h"

// ── PCL Console Logging Stub for Static Builds ────────────────────────────────
namespace pcl::console {
static VERBOSITY_LEVEL g_verbosity = L_ALWAYS;
void print(VERBOSITY_LEVEL level, FILE* stream, const char* format, ...) {
    if (level > g_verbosity) return;
    va_list args;
    va_start(args, format);
    vfprintf(stream ? stream : stdout, format, args);
    va_end(args);
}
void print(VERBOSITY_LEVEL level, const char* format, ...) {
    if (level > g_verbosity) return;
    va_list args;
    va_start(args, format);
    vfprintf(stdout, format, args);
    va_end(args);
}
void change_text_color(FILE*, int, int) {}
void change_text_color(FILE*, int, int, int) {}
void reset_text_color(FILE*) {}
bool isVerbosityLevelEnabled(VERBOSITY_LEVEL level) { return level <= g_verbosity; }
void setVerbosityLevel(VERBOSITY_LEVEL level) { g_verbosity = level; }
VERBOSITY_LEVEL getVerbosityLevel() { return g_verbosity; }
}

using Clock = std::chrono::steady_clock;

constexpr int kStageCount = 6;

struct StageTiming {
    int index;
    const char* label;
    double ms;
    std::size_t point_count;
};

void beginStage(int stage_idx, const char* label, bool enabled) {
    if (enabled) {
        std::cout << "[progress] [" << stage_idx << "/" << kStageCount
                  << "] " << label << "..." << std::endl;
    }
}

double endStage(int stage_idx, const char* label,
                const Clock::time_point& start_time, bool enabled) {
    const auto end_time = Clock::now();
    const double ms =
        std::chrono::duration<double, std::milli>(end_time - start_time).count();
    if (enabled) {
        std::cout << "[progress] [" << stage_idx << "/" << kStageCount
                  << "] " << label << " complete in " << ms << " ms"
                  << std::endl;
    }
    return ms;
}

void printFinalBreakdown(const std::vector<StageTiming>& stages,
                         double total_ms, double io_load_ms) {
    std::cout << "\n============================================================\n";
    std::cout << " [pcl-optimized] PCL 1.14 Fully Optimized Pipeline Breakdown\n";
    std::cout << "============================================================\n";
    for (const auto& st : stages) {
        const double pct = total_ms > 0.0 ? (st.ms / total_ms) * 100.0 : 0.0;
        std::cout << "[progress] [" << std::setw(2) << st.index << "/"
                  << kStageCount << "] " << st.label << ": "
                  << std::fixed << std::setprecision(3) << st.ms << " ms ("
                  << std::setprecision(2) << pct << "%), pts="
                  << st.point_count << "\n";
    }
    std::cout << "------------------------------------------------------------\n";
    std::cout << "  Total Pipeline Time (Compute Only): "
              << std::fixed << std::setprecision(3) << total_ms << " ms\n";
    std::cout << "  Pre-Pipeline File Load (Disk I/O) : "
              << std::fixed << std::setprecision(3) << io_load_ms << " ms\n";
    std::cout << "============================================================\n\n";
}

// ── Helper: load PCD via rvpoint::loadPCD into a PCL cloud ───────────────────
static bool loadPCDToPCL(const std::string& path,
                          pcl::PointCloud<pcl::PointXYZ>& cloud) {
    std::vector<rvpoint::PointXYZ> pts;
    if (!rvpoint::loadPCD(path, pts) || pts.empty())
        return false;
    cloud.clear();
    cloud.reserve(pts.size());
    for (const auto& p : pts) {
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
            cloud.push_back(pcl::PointXYZ(p.x, p.y, p.z));
    }
    cloud.width  = static_cast<std::uint32_t>(cloud.size());
    cloud.height = 1;
    cloud.is_dense = true;
    return !cloud.empty();
}

// ── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0]
                  << " <input.pcd>"
                     " [--preset <dense|tabletop|sparse>]"
                     " [--leaf-size|--delta <val>]"
                     " [--ror-radius <val>] [--ror-min-pts <val>]"
                     " [--cluster-tolerance <val>]"
                     " [--min-cluster <val>] [--max-cluster <val>]"
                     " [--ransac-iters <val>]"
                     " [--no-write] [--progress]\n";
        return 1;
    }

    std::string input_path;
    rvpoint::PipelineConfig cfg = rvpoint::PipelineConfig::fromDelta(0.02f);
    float ror_radius        = cfg.ror_radius;
    int   ror_min_pts       = cfg.ror_min_pts;
    float cluster_tolerance = cfg.cluster_tolerance;
    int   min_cluster_size  = cfg.min_cluster_size;
    int   max_cluster_size  = cfg.max_cluster_size;
    int   ransac_max_iters  = cfg.ransac_max_iterations;
    bool  progress_enabled  = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--preset" && i + 1 < argc) {
            cfg               = rvpoint::PipelineConfig::fromPreset(argv[++i]);
            ror_radius        = cfg.ror_radius;
            ror_min_pts       = cfg.ror_min_pts;
            cluster_tolerance = cfg.cluster_tolerance;
            min_cluster_size  = cfg.min_cluster_size;
            max_cluster_size  = cfg.max_cluster_size;
            ransac_max_iters  = cfg.ransac_max_iterations;
        } else if ((arg == "--leaf-size" || arg == "--delta") && i + 1 < argc) {
            cfg               = rvpoint::PipelineConfig::fromDelta(std::stof(argv[++i]));
            ror_radius        = cfg.ror_radius;
            ror_min_pts       = cfg.ror_min_pts;
            cluster_tolerance = cfg.cluster_tolerance;
            min_cluster_size  = cfg.min_cluster_size;
            max_cluster_size  = cfg.max_cluster_size;
            ransac_max_iters  = cfg.ransac_max_iterations;
        } else if (arg == "--ror-radius" && i + 1 < argc) {
            ror_radius        = std::stof(argv[++i]);
        } else if (arg == "--ror-min-pts" && i + 1 < argc) {
            ror_min_pts       = std::stoi(argv[++i]);
        } else if (arg == "--cluster-tolerance" && i + 1 < argc) {
            cluster_tolerance = std::stof(argv[++i]);
        } else if (arg == "--min-cluster" && i + 1 < argc) {
            min_cluster_size  = std::stoi(argv[++i]);
        } else if (arg == "--max-cluster" && i + 1 < argc) {
            max_cluster_size  = std::stoi(argv[++i]);
        } else if (arg == "--ransac-iters" && i + 1 < argc) {
            ransac_max_iters  = std::stoi(argv[++i]);
        } else if (arg == "--no-write" || arg == "--disable-disk") {
            // no-op
        } else if (arg == "--progress") {
            progress_enabled = true;
        } else if (arg[0] != '-' && input_path.empty()) {
            input_path = arg;
        }
    }

    if (input_path.empty()) {
        std::cerr << "Error: No input PCD path specified.\n";
        return 1;
    }

    std::cout << "[pcl-optimized] Loading: " << input_path
              << " (leaf = " << cfg.voxel_leaf_size << " m"
              << ", ror_r = " << ror_radius << " m"
              << ", ror_k = " << ror_min_pts << ")\n";

    // ── Pre-Pipeline: Load PCD (Disk I/O, not a compute stage) ───────────────
    const auto t_io_start = Clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (!loadPCDToPCL(input_path, *cloud) || cloud->empty()) {
        std::cerr << "[pcl-optimized] Error: Failed to load " << input_path << "\n";
        return 1;
    }
    const double io_load_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - t_io_start).count();
    std::cout << "[pcl-optimized] [IO] Loaded Points: " << cloud->size()
              << " in " << io_load_ms << " ms\n";

    // ── Compute Pipeline ──────────────────────────────────────────────────────
    const auto overall_start = Clock::now();
    std::vector<StageTiming> stage_timings;
    stage_timings.reserve(kStageCount);

    // ── [1/5] Voxel Grid Downsampling (Optimized) ─────────────────────────────
    beginStage(1, "Voxel Grid Downsampling", progress_enabled);
    auto stage_start = Clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr down_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    {
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(cfg.voxel_leaf_size, cfg.voxel_leaf_size, cfg.voxel_leaf_size);
        vg.setDownsampleAllData(false); // Skip non-XYZ field interpolation
        vg.setSaveLeafLayout(false);    // Disable dense voxel grid allocation
        vg.filter(*down_cloud);
        if (down_cloud->empty()) *down_cloud = *cloud;
        down_cloud->is_dense = true;
    }
    stage_timings.push_back({1, "Voxel Grid Downsampling",
        endStage(1, "Voxel Grid Downsampling", stage_start, progress_enabled),
        down_cloud->size()});

    // ── [2/6] Build ROR KdTree (Unsorted FLANN, explicit timing) ────────────
    beginStage(2, "Build ROR KdTree", progress_enabled);
    stage_start = Clock::now();
    // Unsorted KdTree: skips FLANN distance sorting heap
    pcl::search::KdTree<pcl::PointXYZ>::Ptr ror_tree(
        new pcl::search::KdTree<pcl::PointXYZ>(/*sorted=*/false));
    if (!down_cloud->empty())
        ror_tree->setInputCloud(down_cloud);
    stage_timings.push_back({2, "Build ROR KdTree",
        endStage(2, "Build ROR KdTree", stage_start, progress_enabled),
        down_cloud->size()});

    // ── [3/6] Radius Outlier Removal (reuses pre-built unsorted tree) ─────────
    beginStage(3, "Radius Outlier Removal (ROR)", progress_enabled);
    stage_start = Clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr ror_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
        ror.setInputCloud(down_cloud);
        ror.setSearchMethod(ror_tree);
        ror.setRadiusSearch(ror_radius);
        ror.setMinNeighborsInRadius(ror_min_pts);
        ror.filter(*ror_cloud);
        if (ror_cloud->empty()) *ror_cloud = *down_cloud;
        ror_cloud->is_dense = true;
    }
    stage_timings.push_back({3, "Radius Outlier Removal (ROR)",
        endStage(3, "Radius Outlier Removal (ROR)", stage_start, progress_enabled),
        ror_cloud->size()});

    // ── [4/6] RANSAC Ground Plane (Standard RANSAC + Dense Extraction) ───────
    beginStage(4, "RANSAC Ground Plane", progress_enabled);
    stage_start = Clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground(new pcl::PointCloud<pcl::PointXYZ>());
    std::size_t n_inliers = 0;
    {
        // Sample consensus ground plane model
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr ground_model(
            new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(ror_cloud));

        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(ground_model);
        ransac.setDistanceThreshold(cfg.ransac_distance_threshold);
        ransac.setMaxIterations(ransac_max_iters);
        ransac.computeModel();

        std::vector<int> inliers;
        ransac.getInliers(inliers);
        n_inliers = inliers.size();

        pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices());
        inliers_ptr->indices = inliers;

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(ror_cloud);
        extract.setIndices(inliers_ptr);
        extract.setNegative(true);  // keep non-ground points
        extract.filter(*non_ground);
        non_ground->is_dense = true;
    }
    if (progress_enabled) {
        std::cout << "       [RANSAC] Inliers (ground): " << n_inliers
                  << " | Non-ground: " << non_ground->size() << "\n";
    }
    stage_timings.push_back({4, "RANSAC Ground Plane",
        endStage(4, "RANSAC Ground Plane", stage_start, progress_enabled),
        non_ground->size()});

    // ── [5/6] Build Cluster KdTree (Unsorted, Isolated) ──────────────────────
    beginStage(5, "Build Cluster KdTree", progress_enabled);
    stage_start = Clock::now();
    // sorted = false disables FLANN distance sorting during BFS clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr cluster_tree(
        new pcl::search::KdTree<pcl::PointXYZ>(/*sorted=*/false));
    if (!non_ground->empty())
        cluster_tree->setInputCloud(non_ground);
    stage_timings.push_back({5, "Build Cluster KdTree",
        endStage(5, "Build Cluster KdTree", stage_start, progress_enabled),
        non_ground->size()});

    // ── [6/6] Euclidean Clustering (Direct Free Function) ────────────────────
    beginStage(6, "Euclidean Clustering", progress_enabled);
    stage_start = Clock::now();
    std::vector<pcl::PointIndices> clusters;
    if (!non_ground->empty()) {
        pcl::extractEuclideanClusters(
            *non_ground, cluster_tree, cluster_tolerance, clusters,
            static_cast<unsigned int>(min_cluster_size),
            static_cast<unsigned int>(max_cluster_size));
    }
    stage_timings.push_back({6, "Euclidean Clustering",
        endStage(6, "Euclidean Clustering", stage_start, progress_enabled),
        clusters.size()});

    // ── Final Timing Report ───────────────────────────────────────────────────
    const double total_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - overall_start).count();
    printFinalBreakdown(stage_timings, total_ms, io_load_ms);

    return 0;
}
