#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "core/point_types.h"
#include "features/bounding_box/bounding_box.h"
#include "features/bounding_disc/bounding_disc.h"
#include "features/convex_hull/convex_hull.h"
#include "filters/camera_alignment/camera_alignment.h"
#include "filters/passthrough_filter/passthrough_filter.h"
#include "io/simple_pcd_loader.h"
#include "segmentation/forward_cell_clustering.h"

using namespace rvpoint;

int main(int argc, char** argv) {
    std::string pcd_path = "data/pcd_compressed/0000000000.pcd";
    if (argc > 1) {
        pcd_path = argv[1];
    }

    std::cout << "============================================================" << std::endl;
    std::cout << " RVPoint Geometry Strategies Empirical Benchmark" << std::endl;
    std::cout << " Dataset: " << pcd_path << std::endl;
    std::cout << "============================================================" << std::endl;

    PointCloud raw_cloud;
    if (!loadPCD(pcd_path, raw_cloud)) {
        pcd_path = "data/0000000000.pcd";
        if (!loadPCD(pcd_path, raw_cloud)) {
            std::cerr << "[ERROR] Could not load PCD from: " << pcd_path << std::endl;
            return 1;
        }
    }

    // Step 1: Pre-process point cloud
    PointCloud cam_cloud;
    cam_cloud.resize(raw_cloud.size());
    for (size_t i = 0; i < raw_cloud.size(); ++i) {
        cam_cloud.x[i] = -raw_cloud.y[i];
        cam_cloud.y[i] = -raw_cloud.z[i];
        cam_cloud.z[i] = raw_cloud.x[i];
    }

    CameraAlignmentParams align_params;
    align_params.mount_height_m = 1.73f;
    CameraAlignment align(align_params);
    float g[3] = {0.0f, 9.81f, 0.0f};
    PointCloud body_cloud;
    align.transform_to_body(cam_cloud, g, body_cloud);

    PassThroughFilter filter(0.15f, 2.50f);
    PointCloud obstacles_cloud;
    filter.filter(body_cloud, obstacles_cloud);

    ForwardCellClustering clusterer(0.35f, 15, 25000);
    ClusterResult clusters;
    clusterer(obstacles_cloud, clusters);

    const size_t num_clusters = clusters.num_clusters();
    std::cout << "[*] Segmented " << num_clusters << " real obstacle clusters from "
              << obstacles_cloud.size() << " non-ground points.\n" << std::endl;

    constexpr int kWarmup = 2;
    constexpr int kRuns = 5;

    // =========================================================================
    // 1. Benchmark ConvexHull2D Strategies
    // =========================================================================
    std::cout << "------------------------------------------------------------" << std::endl;
    std::cout << " 1. ConvexHull2D Strategy Benchmarks (" << num_clusters << " clusters)" << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;

    auto bench_hull = [&](ConvexHullStrategy strat, const std::string& name) {
        ConvexHull2D hull_ext(4096, strat);
        PointCloud2D out_hull;

        // Warmup
        for (int w = 0; w < kWarmup; ++w) {
            for (size_t c = 0; c < num_clusters; ++c) {
                hull_ext(obstacles_cloud, clusters.cluster_indices(c), clusters.cluster_size(c), out_hull);
            }
        }

        double total_us = 0.0;
        size_t total_vertices = 0;
        for (int r = 0; r < kRuns; ++r) {
            auto t0 = std::chrono::high_resolution_clock::now();
            for (size_t c = 0; c < num_clusters; ++c) {
                hull_ext(obstacles_cloud, clusters.cluster_indices(c), clusters.cluster_size(c), out_hull);
                if (r == 0) total_vertices += out_hull.size();
            }
            auto t1 = std::chrono::high_resolution_clock::now();
            total_us += std::chrono::duration<double, std::micro>(t1 - t0).count();
        }

        double avg_frame_ms = (total_us / kRuns) / 1000.0;
        double avg_cluster_us = (total_us / kRuns) / num_clusters;
        double avg_m = static_cast<double>(total_vertices) / num_clusters;

        std::cout << "  " << std::left << std::setw(20) << name
                  << " | Frame: " << std::right << std::setw(8) << std::fixed << std::setprecision(2) << avg_frame_ms << " ms"
                  << " | Per Cluster: " << std::setw(7) << std::fixed << std::setprecision(1) << avg_cluster_us << " us"
                  << " | Avg Vertices M: " << std::setw(4) << std::fixed << std::setprecision(1) << avg_m
                  << std::endl;
    };

    bench_hull(ConvexHullStrategy::MONOTONE_CHAIN, "Monotone Chain (RVV)");
    bench_hull(ConvexHullStrategy::JARVIS_MARCH,   "Jarvis March (RVV)");
    bench_hull(ConvexHullStrategy::ANGULAR_BINNING, "Angular Binning (16)");

    // Precompute hulls using Monotone Chain for bounding box benchmarks
    ConvexHull2D pre_hull(4096, ConvexHullStrategy::MONOTONE_CHAIN);
    std::vector<PointCloud2D> cluster_hulls(num_clusters);
    for (size_t c = 0; c < num_clusters; ++c) {
        pre_hull(obstacles_cloud, clusters.cluster_indices(c), clusters.cluster_size(c), cluster_hulls[c]);
    }

    // =========================================================================
    // 2. Benchmark BoundingBoxExtractor Strategies
    // =========================================================================
    std::cout << "\n------------------------------------------------------------" << std::endl;
    std::cout << " 2. BoundingBoxExtractor Strategy Benchmarks (" << num_clusters << " clusters)" << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;

    auto bench_bbox = [&](BoundingBoxStrategy strat, const std::string& name) {
        BoundingBoxParams p;
        p.strategy = strat;
        BoundingBoxExtractor bbox_ext(p, 4096);
        OrientedBoundingBox obb;

        // Warmup
        for (int w = 0; w < kWarmup; ++w) {
            for (size_t c = 0; c < num_clusters; ++c) {
                bbox_ext(obstacles_cloud, clusters.cluster_indices(c), clusters.cluster_size(c), cluster_hulls[c], obb);
            }
        }

        double total_us = 0.0;
        for (int r = 0; r < kRuns; ++r) {
            auto t0 = std::chrono::high_resolution_clock::now();
            for (size_t c = 0; c < num_clusters; ++c) {
                bbox_ext(obstacles_cloud, clusters.cluster_indices(c), clusters.cluster_size(c), cluster_hulls[c], obb);
            }
            auto t1 = std::chrono::high_resolution_clock::now();
            total_us += std::chrono::duration<double, std::micro>(t1 - t0).count();
        }

        double avg_frame_ms = (total_us / kRuns) / 1000.0;
        double avg_cluster_us = (total_us / kRuns) / num_clusters;

        std::cout << "  " << std::left << std::setw(20) << name
                  << " | Frame: " << std::right << std::setw(8) << std::fixed << std::setprecision(2) << avg_frame_ms << " ms"
                  << " | Per Cluster: " << std::setw(7) << std::fixed << std::setprecision(1) << avg_cluster_us << " us"
                  << std::endl;
    };

    bench_bbox(BoundingBoxStrategy::MIN_AREA,      "Min-Area Calipers");
    bench_bbox(BoundingBoxStrategy::WIREFRAME_PCA, "Wireframe PCA");
    bench_bbox(BoundingBoxStrategy::EDGE_ALIGN,    "Edge-Perimeter (EPA)");

    // L-Shape with stratified decimation (N <= 64)
    {
        BoundingBoxParams p;
        p.strategy = BoundingBoxStrategy::L_SHAPE_ALIGN;
        p.decimation_threshold = 64;
        BoundingBoxExtractor bbox_ext(p, 4096);
        OrientedBoundingBox obb;

        for (int w = 0; w < kWarmup; ++w) {
            for (size_t c = 0; c < num_clusters; ++c) {
                bbox_ext(obstacles_cloud, clusters.cluster_indices(c), clusters.cluster_size(c), cluster_hulls[c], obb);
            }
        }
        double total_us = 0.0;
        for (int r = 0; r < kRuns; ++r) {
            auto t0 = std::chrono::high_resolution_clock::now();
            for (size_t c = 0; c < num_clusters; ++c) {
                bbox_ext(obstacles_cloud, clusters.cluster_indices(c), clusters.cluster_size(c), cluster_hulls[c], obb);
            }
            auto t1 = std::chrono::high_resolution_clock::now();
            total_us += std::chrono::duration<double, std::micro>(t1 - t0).count();
        }
        double avg_frame_ms = (total_us / kRuns) / 1000.0;
        double avg_cluster_us = (total_us / kRuns) / num_clusters;
        std::cout << "  " << std::left << std::setw(20) << "L-Shape (Decim N<=64)"
                  << " | Frame: " << std::right << std::setw(8) << std::fixed << std::setprecision(2) << avg_frame_ms << " ms"
                  << " | Per Cluster: " << std::setw(7) << std::fixed << std::setprecision(1) << avg_cluster_us << " us"
                  << std::endl;
    }

    // L-Shape full points (No decimation)
    {
        BoundingBoxParams p;
        p.strategy = BoundingBoxStrategy::L_SHAPE_ALIGN;
        p.decimation_threshold = 999999;
        BoundingBoxExtractor bbox_ext(p, 4096);
        OrientedBoundingBox obb;

        for (int w = 0; w < kWarmup; ++w) {
            for (size_t c = 0; c < num_clusters; ++c) {
                bbox_ext(obstacles_cloud, clusters.cluster_indices(c), clusters.cluster_size(c), cluster_hulls[c], obb);
            }
        }
        double total_us = 0.0;
        for (int r = 0; r < kRuns; ++r) {
            auto t0 = std::chrono::high_resolution_clock::now();
            for (size_t c = 0; c < num_clusters; ++c) {
                bbox_ext(obstacles_cloud, clusters.cluster_indices(c), clusters.cluster_size(c), cluster_hulls[c], obb);
            }
            auto t1 = std::chrono::high_resolution_clock::now();
            total_us += std::chrono::duration<double, std::micro>(t1 - t0).count();
        }
        double avg_frame_ms = (total_us / kRuns) / 1000.0;
        double avg_cluster_us = (total_us / kRuns) / num_clusters;
        std::cout << "  " << std::left << std::setw(20) << "L-Shape (Full RVV)"
                  << " | Frame: " << std::right << std::setw(8) << std::fixed << std::setprecision(2) << avg_frame_ms << " ms"
                  << " | Per Cluster: " << std::setw(7) << std::fixed << std::setprecision(1) << avg_cluster_us << " us"
                  << std::endl;
    }

    // Precompute boxes using EDGE_ALIGN for disc benchmarks
    BoundingBoxParams default_p;
    default_p.strategy = BoundingBoxStrategy::EDGE_ALIGN;
    BoundingBoxExtractor default_bbox(default_p, 4096);
    std::vector<OrientedBoundingBox> cluster_boxes(num_clusters);
    for (size_t c = 0; c < num_clusters; ++c) {
        default_bbox(obstacles_cloud, clusters.cluster_indices(c), clusters.cluster_size(c), cluster_hulls[c], cluster_boxes[c]);
    }

    // =========================================================================
    // 3. Benchmark BoundingDiscExtractor Paths
    // =========================================================================
    std::cout << "\n------------------------------------------------------------" << std::endl;
    std::cout << " 3. BoundingDiscExtractor Path Benchmarks (" << num_clusters << " clusters)" << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;

    BoundingDiscExtractor disc_ext(4096);
    BoundingDisc disc;

    // Path 1: Point-Centroid Disc
    {
        for (int w = 0; w < kWarmup; ++w) {
            for (size_t c = 0; c < num_clusters; ++c) {
                disc_ext.compute_from_points(obstacles_cloud, clusters.cluster_indices(c), clusters.cluster_size(c), disc);
            }
        }
        double total_us = 0.0;
        for (int r = 0; r < kRuns; ++r) {
            auto t0 = std::chrono::high_resolution_clock::now();
            for (size_t c = 0; c < num_clusters; ++c) {
                disc_ext.compute_from_points(obstacles_cloud, clusters.cluster_indices(c), clusters.cluster_size(c), disc);
            }
            auto t1 = std::chrono::high_resolution_clock::now();
            total_us += std::chrono::duration<double, std::micro>(t1 - t0).count();
        }
        double avg_frame_ms = (total_us / kRuns) / 1000.0;
        double avg_cluster_us = (total_us / kRuns) / num_clusters;
        std::cout << "  " << std::left << std::setw(20) << "Point-Centroid (RVV)"
                  << " | Frame: " << std::right << std::setw(8) << std::fixed << std::setprecision(2) << avg_frame_ms << " ms"
                  << " | Per Cluster: " << std::setw(7) << std::fixed << std::setprecision(1) << avg_cluster_us << " us"
                  << std::endl;
    }

    // Path 2: Concentric Disc from OBB
    {
        for (int w = 0; w < kWarmup; ++w) {
            for (size_t c = 0; c < num_clusters; ++c) {
                disc_ext.compute_concentric(cluster_boxes[c], disc);
            }
        }
        double total_us = 0.0;
        for (int r = 0; r < kRuns; ++r) {
            auto t0 = std::chrono::high_resolution_clock::now();
            for (size_t c = 0; c < num_clusters; ++c) {
                disc_ext.compute_concentric(cluster_boxes[c], disc);
            }
            auto t1 = std::chrono::high_resolution_clock::now();
            total_us += std::chrono::duration<double, std::micro>(t1 - t0).count();
        }
        double avg_frame_ms = (total_us / kRuns) / 1000.0;
        double avg_cluster_us = (total_us / kRuns) / num_clusters;
        std::cout << "  " << std::left << std::setw(20) << "Concentric OBB (O1)"
                  << " | Frame: " << std::right << std::setw(8) << std::fixed << std::setprecision(2) << avg_frame_ms << " ms"
                  << " | Per Cluster: " << std::setw(7) << std::fixed << std::setprecision(1) << avg_cluster_us << " us"
                  << std::endl;
    }

    std::cout << "============================================================" << std::endl;
    std::cout << " Benchmark completed successfully." << std::endl;
    std::cout << "============================================================" << std::endl;

    return 0;
}

