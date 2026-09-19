// eval/pipelines/official_pcl_stream.cpp
// Multi-Threaded Continuous Stream Pipeline using Official Debian PCL 1.14
// Direct 1:1 Parallel Stream Benchmark against pipeline_3d_stream.cpp
// Target Architecture: SpacemiT K1 / Orange Pi RV2 (RV64GCV Octa-Core)

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
#include <sstream>
#include <string>
#include <vector>

#if defined(_OPENMP)
#include <omp.h>
#endif

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using Clock = std::chrono::high_resolution_clock;

namespace {

struct StreamConfig {
    std::string input_path = "data/pcd_compressed";
    int max_frames = 20;
    int num_threads = 8;
    bool progress = true;
    bool disable_disk = true;
    float voxel_leaf_size = 0.10f;
    float ror_radius = 0.25f;
    int ror_min_pts = 2;
    float ransac_distance_threshold = 0.20f;
    int ransac_max_iterations = 250;
    float cluster_tolerance = 0.15f;
    int min_cluster_size = 50;
    int max_cluster_size = 100000;
};

struct FrameMetrics {
    int frame_id = 0;
    std::string filename;
    double io_read_ms = 0.0;
    double compute_ms = 0.0;

    double voxel_ms = 0.0;
    double ror_ms = 0.0;
    double ransac_ms = 0.0;
    double cluster_ms = 0.0;

    size_t raw_points = 0;
    size_t downsampled_points = 0;
    size_t ror_points = 0;
    size_t obstacle_points = 0;
    size_t cluster_count = 0;
};

} // anonymous namespace

int main(int argc, char** argv) {
    StreamConfig cfg;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--max-frames" && i + 1 < argc) {
            cfg.max_frames = std::atoi(argv[++i]);
        } else if (arg == "--threads" && i + 1 < argc) {
            cfg.num_threads = std::atoi(argv[++i]);
        } else if (arg == "--leaf-size" && i + 1 < argc) {
            cfg.voxel_leaf_size = std::atof(argv[++i]);
        } else if (arg == "--cluster-tolerance" && i + 1 < argc) {
            cfg.cluster_tolerance = std::atof(argv[++i]);
        } else if (arg == "--min-cluster" && i + 1 < argc) {
            cfg.min_cluster_size = std::atoi(argv[++i]);
        } else if (arg == "--max-cluster" && i + 1 < argc) {
            cfg.max_cluster_size = std::atoi(argv[++i]);
        } else if (arg == "--no-write") {
            cfg.disable_disk = true;
        } else if (arg == "--progress") {
            cfg.progress = true;
        } else if (arg == "--no-progress") {
            cfg.progress = false;
        } else if (arg[0] != '-') {
            cfg.input_path = arg;
        }
    }

#if defined(_OPENMP)
    omp_set_num_threads(cfg.num_threads);
#endif

    // Resolve list of PCD files
    std::vector<std::string> pcd_files;
    if (std::filesystem::is_directory(cfg.input_path)) {
        for (const auto& entry : std::filesystem::directory_iterator(cfg.input_path)) {
            if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
                pcd_files.push_back(entry.path().string());
            }
        }
        std::sort(pcd_files.begin(), pcd_files.end());
    } else if (std::filesystem::is_regular_file(cfg.input_path)) {
        pcd_files.push_back(cfg.input_path);
    } else {
        std::cerr << "Error: Invalid input path: " << cfg.input_path << std::endl;
        return 1;
    }

    if (pcd_files.empty()) {
        std::cerr << "Error: No .pcd files found in: " << cfg.input_path << std::endl;
        return 1;
    }

    const size_t unique_pcd_count = pcd_files.size();
    if (cfg.max_frames > 0) {
        if (pcd_files.size() > static_cast<size_t>(cfg.max_frames)) {
            pcd_files.resize(cfg.max_frames);
        } else if (pcd_files.size() < static_cast<size_t>(cfg.max_frames)) {
            size_t orig_sz = pcd_files.size();
            pcd_files.reserve(cfg.max_frames);
            for (size_t i = orig_sz; i < static_cast<size_t>(cfg.max_frames); ++i) {
                pcd_files.push_back(pcd_files[i % orig_sz]);
            }
        }
    }

    std::cout << "========================================================================\n"
              << "  OFFICIAL PCL 1.14 Multi-Threaded Continuous Stream Benchmark\n"
              << "  Target Architecture: SpacemiT K1 / Orange Pi RV2 (RV64GCV Octa-Core)\n"
              << "========================================================================\n"
              << "  Input Directory : " << cfg.input_path << " (" << pcd_files.size() << " frames";
    if (pcd_files.size() > unique_pcd_count) {
        std::cout << " [looped " << unique_pcd_count << " unique files]";
    }
    std::cout << ")\n"
              << "  Streaming Mode  : INTER-FRAME (Asynchronous Multi-Core Stream Pool)\n"
              << "  Worker Threads  : " << cfg.num_threads << "\n"
              << "  Voxel Leaf Size : " << cfg.voxel_leaf_size << " m\n"
              << "  Cluster Tol     : " << cfg.cluster_tolerance << " m (min: " << cfg.min_cluster_size << ")\n"
              << "  Disk I/O        : DISABLED (Zero Disk I/O)\n"
              << "========================================================================\n\n";

    std::vector<FrameMetrics> all_metrics(pcd_files.size());
    const Eigen::Vector3f ground_normal_axis(0.0f, 0.0f, 1.0f);
    const float eps_angle = 45.0f * 3.14159265358979323846f / 180.0f; // ~45 deg max slope

    auto stream_start_wall = Clock::now();

#if defined(_OPENMP)
    #pragma omp parallel for schedule(dynamic, 1)
#endif
    for (size_t f_idx = 0; f_idx < pcd_files.size(); ++f_idx) {
        int tid = 0;
#if defined(_OPENMP)
        tid = omp_get_thread_num();
#endif
        FrameMetrics& m = all_metrics[f_idx];
        m.frame_id = static_cast<int>(f_idx);
        m.filename = std::filesystem::path(pcd_files[f_idx]).filename().string();

        // --------------------------------------------------------------------
        // Stage 0: Load PCD
        // --------------------------------------------------------------------
        auto t_read_start = Clock::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_files[f_idx], *input_cloud) == -1 || input_cloud->empty()) {
            std::cerr << "[PCL Warning] Frame " << f_idx << " failed to load: " << pcd_files[f_idx] << std::endl;
            continue;
        }
        auto t_read_end = Clock::now();
        m.io_read_ms = std::chrono::duration<double, std::milli>(t_read_end - t_read_start).count();
        m.raw_points = input_cloud->size();

        // --------------------------------------------------------------------
        // Pure Compute Pipeline
        // --------------------------------------------------------------------
        auto t_comp_start = Clock::now();

        // 1. Voxel Downsample
        auto t1 = Clock::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(input_cloud);
        vg.setLeafSize(cfg.voxel_leaf_size, cfg.voxel_leaf_size, cfg.voxel_leaf_size);
        vg.filter(*downsampled_cloud);
        auto t2 = Clock::now();
        m.voxel_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
        m.downsampled_points = downsampled_cloud->size();

        // 2. Radius Outlier Removal
        auto t3 = Clock::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr ror_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
        ror.setInputCloud(downsampled_cloud);
        ror.setRadiusSearch(cfg.ror_radius);
        ror.setMinNeighborsInRadius(cfg.ror_min_pts);
        ror.filter(*ror_cloud);
        auto t4 = Clock::now();
        m.ror_ms = std::chrono::duration<double, std::milli>(t4 - t3).count();
        m.ror_points = ror_cloud->size();

        // 3. RANSAC Ground Segmentation
        auto t5 = Clock::now();
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(cfg.ransac_max_iterations);
        seg.setDistanceThreshold(cfg.ransac_distance_threshold);
        seg.setAxis(ground_normal_axis);
        seg.setEpsAngle(eps_angle);
        seg.setInputCloud(ror_cloud);
        seg.segment(*inliers, *coefficients);

        pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (!inliers->indices.empty()) {
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(ror_cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*non_ground_cloud);
        } else {
            *non_ground_cloud = *ror_cloud;
        }
        auto t6 = Clock::now();
        m.ransac_ms = std::chrono::duration<double, std::milli>(t6 - t5).count();
        m.obstacle_points = non_ground_cloud->size();

        // 4. Euclidean Clustering
        auto t7 = Clock::now();
        std::vector<pcl::PointIndices> cluster_indices;
        if (!non_ground_cloud->empty()) {
            pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_ec(new pcl::search::KdTree<pcl::PointXYZ>);
            kdtree_ec->setInputCloud(non_ground_cloud);
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(cfg.cluster_tolerance);
            ec.setMinClusterSize(cfg.min_cluster_size);
            ec.setMaxClusterSize(cfg.max_cluster_size);
            ec.setSearchMethod(kdtree_ec);
            ec.setInputCloud(non_ground_cloud);
            ec.extract(cluster_indices);
        }
        auto t8 = Clock::now();
        m.cluster_ms = std::chrono::duration<double, std::milli>(t8 - t7).count();
        m.cluster_count = cluster_indices.size();

        auto t_comp_end = Clock::now();
        m.compute_ms = std::chrono::duration<double, std::milli>(t_comp_end - t_comp_start).count();

        if (cfg.progress) {
            double fps = (m.compute_ms > 0.0) ? (1000.0 / m.compute_ms) : 0.0;
            #pragma omp critical
            {
                std::cout << "[pcl-stream] [Core " << tid << "] Frame " << std::setw(4) << f_idx << " (" << m.filename << ") | "
                          << "Compute: " << std::fixed << std::setprecision(2) << std::setw(6) << m.compute_ms << " ms ("
                          << std::setprecision(1) << std::setw(4) << fps << " FPS) | "
                          << "I/O Read: " << std::setprecision(2) << std::setw(5) << m.io_read_ms << " ms | "
                          << "Pts: " << std::setw(6) << m.raw_points << " -> " << std::setw(5) << m.downsampled_points << " | "
                          << "Obstacles: " << std::setw(5) << m.obstacle_points << " | "
                          << "Clusters: " << std::setw(2) << m.cluster_count << std::endl;
            }
        }
    }

    auto stream_end_wall = Clock::now();
    double total_wall_ms = std::chrono::duration<double, std::milli>(stream_end_wall - stream_start_wall).count();

    if (all_metrics.empty()) {
        std::cerr << "Error: No frames processed." << std::endl;
        return 1;
    }

    double sum_comp = 0, min_comp = 1e9, max_comp = 0;
    double sum_read = 0;
    double sum_vox = 0, sum_ror = 0, sum_ransac = 0, sum_clust = 0;

    for (const auto& m : all_metrics) {
        sum_comp += m.compute_ms;
        min_comp = std::min(min_comp, m.compute_ms);
        max_comp = std::max(max_comp, m.compute_ms);
        sum_read += m.io_read_ms;
        sum_vox += m.voxel_ms;
        sum_ror += m.ror_ms;
        sum_ransac += m.ransac_ms;
        sum_clust += m.cluster_ms;
    }

    size_t count = all_metrics.size();
    double avg_comp = sum_comp / count;
    double avg_read = sum_read / count;
    double effective_fps = (avg_comp > 0.0) ? (1000.0 / avg_comp) : 0.0;
    double stream_wall_fps = (total_wall_ms > 0.0) ? (count * 1000.0 / total_wall_ms) : 0.0;

    std::cout << "\n========================================================================\n"
              << "  OFFICIAL PCL CONTINUOUS STREAM BENCHMARK SUMMARY (" << count << " frames)\n"
              << "========================================================================\n"
              << "  [Overall Stream Throughput & Wall-Clock Runtime]:\n"
              << "    • Total Wall-Clock Time   : " << std::fixed << std::setprecision(2) << total_wall_ms << " ms\n"
              << "    • Sustained Stream Rate   : " << std::setprecision(2) << stream_wall_fps << " FPS (Asynchronous Worker Pool)\n"
              << "------------------------------------------------------------------------\n"
              << "  [Pure Compute Pipeline Latency] (Excluding all Disk I/O):\n"
              << "    • Average Compute Latency : " << std::fixed << std::setprecision(2) << avg_comp << " ms\n"
              << "    • Min Compute Latency     : " << min_comp << " ms\n"
              << "    • Max Compute Latency     : " << max_comp << " ms\n"
              << "    • Per-Frame Compute Rate  : " << std::setprecision(2) << effective_fps << " FPS\n"
              << "------------------------------------------------------------------------\n"
              << "  [Compute Stage Breakdown (Average per Frame)]:\n"
              << "    1. Voxel Downsample       : " << std::setw(6) << (sum_vox / count) << " ms ("
              << std::setprecision(1) << ((sum_vox / sum_comp) * 100.0) << "%)\n"
              << "    2. Radius Outlier Removal : " << std::setw(6) << (sum_ror / count) << " ms ("
              << std::setprecision(1) << ((sum_ror / sum_comp) * 100.0) << "%)\n"
              << "    3. RANSAC Ground Fit      : " << std::setw(6) << (sum_ransac / count) << " ms ("
              << std::setprecision(1) << ((sum_ransac / sum_comp) * 100.0) << "%)\n"
              << "    4. Euclidean Clustering   : " << std::setw(6) << (sum_clust / count) << " ms ("
              << std::setprecision(1) << ((sum_clust / sum_comp) * 100.0) << "%)\n"
              << "========================================================================\n";

    return 0;
}
