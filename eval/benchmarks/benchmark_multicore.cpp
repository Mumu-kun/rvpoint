#include "include/rvpoint.h"
#include "io/simple_pcd_loader.h"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <numeric>

#if defined(_OPENMP)
#include <omp.h>
#endif

using namespace rvpoint;
using Clock = std::chrono::high_resolution_clock;

struct ModeMetrics {
    std::string name;
    double latency_ms = 0.0;
    double fps = 0.0;
    size_t filtered_points = 0;
    size_t clusters_count = 0;
    size_t clustered_points = 0;
};

int main(int argc, char** argv) {
    std::cout << "==============================================================\n";
    std::cout << " RVPoint Multi-Core Perception Architecture Benchmark (ADR-0013)\n";
    std::cout << " Target: RISC-V 64 rv64gcv (SpacemiT K1 Dual-Cluster Topology)\n";
    std::cout << "==============================================================\n\n";

    // 1. Locate and load dataset
    std::string pcd_path = "data/pcd_compressed/0000000000.pcd";
    if (argc > 1) {
        pcd_path = argv[1];
    } else if (!std::filesystem::exists(pcd_path)) {
        pcd_path = "data/0000000000.pcd";
    }

    PointCloud input_cloud;
    if (std::filesystem::exists(pcd_path)) {
        std::cout << "Loading dataset point cloud: " << pcd_path << " ...\n";
        if (!loadPCD(pcd_path, input_cloud)) {
            std::cerr << "Failed to load PCD file: " << pcd_path << "\n";
            return 1;
        }
    } else {
        std::cout << "PCD dataset not found. Generating synthetic automotive scene (15,000 points)...\n";
        input_cloud.reserve(15000);
        // Ground plane
        for (int i = 0; i < 10000; ++i) {
            float x = -20.0f + 40.0f * (i % 100) / 100.0f;
            float y = -20.0f + 40.0f * (i / 100) / 100.0f;
            input_cloud.push_back(x, y, 0.0f);
        }
        // Obstacle 1 (Car at x=5, y=5)
        for (int i = 0; i < 1500; ++i) {
            input_cloud.push_back(5.0f + 0.02f * (i % 30), 5.0f + 0.02f * (i / 30), 0.5f + 0.01f * (i % 20));
        }
        // Obstacle 2 (Pedestrian at x=-3, y=8)
        for (int i = 0; i < 500; ++i) {
            input_cloud.push_back(-3.0f + 0.01f * (i % 15), 8.0f + 0.01f * (i / 15), 0.8f + 0.01f * (i % 25));
        }
        // Obstacle 3 (Vehicle at x=0, y=15)
        for (int i = 0; i < 2000; ++i) {
            input_cloud.push_back(0.0f + 0.02f * (i % 40), 15.0f + 0.02f * (i / 40), 0.5f + 0.01f * (i % 20));
        }
    }

    std::cout << "Input Cloud: " << input_cloud.size() << " points.\n\n";

    // Common perception parameters
    const float voxel_leaf = 0.20f;
    const float ransac_dist_thresh = 0.20f;
    const int ransac_max_iter = 100;
    const float ror_radius = 0.40f;
    const int ror_min_pts = 4;
    const float cluster_tol = 0.50f;
    const int min_cluster_size = 15;
    const int max_cluster_size = 20000;

    int num_threads = 4;
#if defined(_OPENMP)
    num_threads = omp_get_max_threads();
    if (num_threads > 8) num_threads = 8;
#endif

    std::cout << "Active Hardware Threads: " << num_threads << "\n\n";

    std::vector<ModeMetrics> all_metrics;

    std::cout << "Starting Preprocessing..." << std::endl;
    VoxelGrid vg(voxel_leaf);
    vg.reserve(input_cloud.size());
    PointCloud downsampled;
    vg(input_cloud.view(), downsampled);
    std::cout << "Downsampled to " << downsampled.size() << " pts." << std::endl;

    RansacPlane ground_seg(ransac_dist_thresh, ransac_max_iter);
    ground_seg.reserve(downsampled.size());
    PointCloud inliers, obstacles;
    PlaneModel ground_model;
    ground_seg(downsampled.view(), ground_model, inliers, obstacles);
    std::cout << "Obstacles extracted: " << obstacles.size() << " pts." << std::endl;

    // =========================================================================
    // Mode 1: Sequential (Baseline 1 Core)
    // =========================================================================
    {
        std::cout << "Starting Mode 1 (Sequential)..." << std::endl;
        ModeMetrics m;
        m.name = "Sequential (1-Core Baseline)";

        RadiusOutlierRemoval ror(ror_radius, ror_min_pts);
        ror.threads(1);
        ror.reserve(obstacles.size());

        EuclideanClustering ec(cluster_tol, min_cluster_size, max_cluster_size);
        ec.reserve(obstacles.size());

        PointCloud filtered;
        ClusterResult clusters;

        // Warmup
        ror(obstacles.view(), filtered);
        ec(filtered.view(), clusters);

        // Benchmark
        auto t0 = Clock::now();
        ror(obstacles.view(), filtered);
        ec(filtered.view(), clusters);
        auto t1 = Clock::now();

        m.latency_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        m.fps = 1000.0 / m.latency_ms;
        m.filtered_points = filtered.size();
        m.clusters_count = clusters.num_clusters();
        m.clustered_points = clusters.indices.size();

        all_metrics.push_back(m);
        std::cout << "[Mode 1] " << m.name << ": " << std::fixed << std::setprecision(2)
                  << m.latency_ms << " ms (" << m.fps << " FPS) | Clusters: " << m.clusters_count
                  << " (" << m.clustered_points << " pts)\n";
    }

    // =========================================================================
    // Mode 2: IntraFrame KernelParallel (Multi-Threaded Point Loops)
    // =========================================================================
    {
        std::cout << "Starting Mode 2 (KernelParallel)..." << std::endl;
        ModeMetrics m;
        m.name = "IntraFrame KernelParallel (" + std::to_string(num_threads) + "-Core OpenMP)";

        RadiusOutlierRemoval ror(ror_radius, ror_min_pts);
        ror.threads(num_threads);
        ror.reserve(obstacles.size());

        EuclideanClustering ec(cluster_tol, min_cluster_size, max_cluster_size);
        ec.reserve(obstacles.size());

        PointCloud filtered;
        ClusterResult clusters;

        // Warmup
        ror(obstacles.view(), filtered);
        ec(filtered.view(), clusters);

        // Benchmark
        auto t0 = Clock::now();
        ror(obstacles.view(), filtered);
        ec(filtered.view(), clusters);
        auto t1 = Clock::now();

        m.latency_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        m.fps = 1000.0 / m.latency_ms;
        m.filtered_points = filtered.size();
        m.clusters_count = clusters.num_clusters();
        m.clustered_points = clusters.indices.size();

        all_metrics.push_back(m);
        std::cout << "[Mode 2] " << m.name << ": " << std::fixed << std::setprecision(2)
                  << m.latency_ms << " ms (" << m.fps << " FPS) | Clusters: " << m.clusters_count
                  << " (" << m.clustered_points << " pts)\n";
    }

    // =========================================================================
    // Mode 3: IntraFrame SpatialSlabEngine (Quantile Domain Decomposition)
    // =========================================================================
    {
        std::cout << "Starting Mode 3 (SpatialSlabEngine)..." << std::endl;
        ModeMetrics m;
        m.name = "IntraFrame SpatialSlabEngine (" + std::to_string(num_threads) + " Slabs)";

        SpatialSlabEngine slab_engine(ror_radius, ror_min_pts, cluster_tol,
                                      min_cluster_size, max_cluster_size,
                                      num_threads, SlabAxis::X);
        slab_engine.reserve(obstacles.size());

        PointCloud filtered;
        ClusterResult clusters;

        // Warmup
        try {
            slab_engine(obstacles.view(), filtered, clusters);
        } catch (const std::exception& ex) {
            std::cerr << "EXCEPTION in Mode 3 warmup: " << ex.what() << std::endl;
            return 1;
        }

        // Benchmark
        auto t0 = Clock::now();
        try {
            slab_engine(obstacles.view(), filtered, clusters);
        } catch (const std::exception& ex) {
            std::cerr << "EXCEPTION in Mode 3 benchmark: " << ex.what() << std::endl;
            return 1;
        }
        auto t1 = Clock::now();

        m.latency_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        m.fps = 1000.0 / m.latency_ms;
        m.filtered_points = filtered.size();
        m.clusters_count = clusters.num_clusters();
        m.clustered_points = clusters.indices.size();

        all_metrics.push_back(m);
        std::cout << "[Mode 3] " << m.name << ": " << std::fixed << std::setprecision(2)
                  << m.latency_ms << " ms (" << m.fps << " FPS) | Clusters: " << m.clusters_count
                  << " (" << m.clustered_points << " pts)" << std::endl;
    }

    // =========================================================================
    // Mode 4: FrameWorkerPool (Throughput Streaming across 8 Frames)
    // =========================================================================
    {
        std::cout << "Starting Mode 4 (FrameWorkerPool)..." << std::endl;
        ModeMetrics m;
        m.name = "FrameWorkerPool (" + std::to_string(num_threads) + " Workers Streaming)";

        // Construct perception pipeline (obstacles -> ROR -> EuclideanClustering) in PipelineManager
        PipelineManager pm;
        pm.set_execution_mode(ExecutionMode::FrameWorkerPool);

        pm.add_node("ror_filter",
            in<PointCloud>("obstacles"),
            out<PointCloud>("filtered"),
            param<float>("radius", ror_radius),
            param<int>("min_pts", ror_min_pts)
        ).kernel(RadiusOutlierRemoval{});

        pm.add_node("clustering",
            in<PointCloud>("filtered"),
            out<ClusterResult>("clusters"),
            param<float>("tol", cluster_tol),
            param<int>("min_sz", min_cluster_size),
            param<int>("max_sz", max_cluster_size)
        ).kernel(EuclideanClustering{});

        pm.set_primary_input("obstacles");
        pm.initialize(obstacles.size());

        const size_t num_stream_frames = 8;
        std::vector<PointCloud> stream_frames(num_stream_frames);
        for (size_t i = 0; i < num_stream_frames; ++i) {
            stream_frames[i] = obstacles;
        }

        // Benchmark batch streaming
        auto t0 = Clock::now();
        size_t processed_clusters = 0;
        size_t processed_clustered_pts = 0;
        size_t processed_filtered_pts = 0;

        RegisterId clust_id = pm.get_id("clusters");
        RegisterId filt_id = pm.get_id("filtered");

        struct FrameRes {
            size_t n_clusters;
            size_t n_clustered_pts;
            size_t n_filtered_pts;
        };

        pm.run_worker_pool_resequenced(
            stream_frames,
            [clust_id, filt_id](const FrameContext& fc) -> FrameRes {
                const auto& cl = fc.get<ClusterResult>(clust_id);
                const auto& fl = fc.get<PointCloud>(filt_id);
                return {cl.num_clusters(), cl.indices.size(), fl.size()};
            },
            [&](uint64_t /*seq*/, FrameRes res) {
                processed_clusters += res.n_clusters;
                processed_clustered_pts += res.n_clustered_pts;
                processed_filtered_pts += res.n_filtered_pts;
            },
            num_threads
        );
        auto t1 = Clock::now();

        double total_stream_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        m.latency_ms = total_stream_ms / num_stream_frames;
        m.fps = (num_stream_frames * 1000.0) / total_stream_ms;
        m.filtered_points = processed_filtered_pts / num_stream_frames;
        m.clusters_count = processed_clusters / num_stream_frames;
        m.clustered_points = processed_clustered_pts / num_stream_frames;

        all_metrics.push_back(m);
        std::cout << "[Mode 4] " << m.name << ": " << std::fixed << std::setprecision(2)
                  << total_stream_ms << " ms total (" << m.latency_ms << " ms/frame, "
                  << m.fps << " FPS sustained) | Clusters: " << m.clusters_count
                  << " (" << m.clustered_points << " pts)\n";
    }

    // --- Performance Summary Table ---
    std::cout << "\n======================= BENCHMARK SUMMARY =======================\n";
    std::cout << std::left << std::setw(42) << "Execution Mode"
              << std::setw(15) << "Latency (ms)"
              << std::setw(12) << "Throughput"
              << std::setw(12) << "Clusters"
              << "Speedup\n";
    std::cout << "-----------------------------------------------------------------\n";

    double base_latency = all_metrics[0].latency_ms;
    for (const auto& met : all_metrics) {
        double speedup = (met.latency_ms > 0) ? (base_latency / met.latency_ms) : 1.0;
        std::cout << std::left << std::setw(42) << met.name
                  << std::fixed << std::setprecision(2)
                  << std::setw(15) << met.latency_ms
                  << std::setw(5) << met.fps << " FPS    "
                  << std::setw(12) << met.clusters_count
                  << std::setprecision(2) << speedup << "x\n";
    }
    std::cout << "=================================================================\n\n";

    // Write output telemetry JSON to output/benchmark_multicore.json
    std::filesystem::create_directories("output");
    std::ofstream out_json("output/benchmark_multicore.json");
    if (out_json.is_open()) {
        out_json << "{\n";
        out_json << "  \"dataset\": \"" << pcd_path << "\",\n";
        out_json << "  \"input_points\": " << input_cloud.size() << ",\n";
        out_json << "  \"num_threads\": " << num_threads << ",\n";
        out_json << "  \"results\": [\n";
        for (size_t i = 0; i < all_metrics.size(); ++i) {
            const auto& met = all_metrics[i];
            out_json << "    {\n";
            out_json << "      \"mode\": \"" << met.name << "\",\n";
            out_json << "      \"latency_ms\": " << met.latency_ms << ",\n";
            out_json << "      \"fps\": " << met.fps << ",\n";
            out_json << "      \"filtered_points\": " << met.filtered_points << ",\n";
            out_json << "      \"clusters_count\": " << met.clusters_count << ",\n";
            out_json << "      \"clustered_points\": " << met.clustered_points << "\n";
            out_json << "    }" << (i + 1 < all_metrics.size() ? "," : "") << "\n";
        }
        out_json << "  ]\n";
        out_json << "}\n";
        std::cout << "Telemetry written to: output/benchmark_multicore.json\n";
    }

    return 0;
}
