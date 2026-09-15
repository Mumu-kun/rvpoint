#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <chrono>

#include "features/convex_hull/convex_hull.h"
#include "features/bounding_disc/bounding_disc.h"
#include "features/bounding_box/bounding_box.h"
#include "filters/voxel_grid/voxel_grid.h"
#include "filters/camera_alignment/camera_alignment.h"
#include "filters/passthrough_filter/passthrough_filter.h"
#include "segmentation/forward_cell_clustering.h"
#include "io/simple_pcd_loader.h"
#include "core/point_types.h"

using namespace rvpoint;

static void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " <input_pcd> [output_json] [output_obstacles_pcd] [options]\n"
              << "Options:\n"
              << "  --optical             Input is camera optical frame (X right, Y down, Z forward). Aligns with gravity.\n"
              << "  --body                Input is already vehicle body frame (X forward, Y left, Z up). Default.\n"
              << "  --strategy <strat>    Bounding box strategy: min_area | l_shape | edge_align | pca | adaptive. Default: edge_align.\n"
              << "  --voxel <size>        Optional pre-clustering voxelization filter (meters, e.g. 0.08).\n"
              << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    std::string input_pcd = argv[1];
    std::string output_json = (argc > 2 && argv[2][0] != '-') ? argv[2] : "output/ticket_06_real_data.json";
    std::string output_pcd = (argc > 3 && argv[3][0] != '-') ? argv[3] : "";

    bool is_optical = false;
    std::string strat_str = "edge_align";
    float voxel_size = 0.0f;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--optical") is_optical = true;
        if (arg == "--body") is_optical = false;
        if (arg == "--strategy" && i + 1 < argc) {
            strat_str = argv[++i];
        }
        if (arg == "--voxel" && i + 1 < argc) {
            voxel_size = std::stof(argv[++i]);
        }
    }

    std::cout << "============================================================" << std::endl;
    std::cout << " RVPoint Perception Pipeline: Obstacle & 3D Bounding Box Extractor" << std::endl;
    std::cout << " Input PCD:    " << input_pcd << std::endl;
    std::cout << " Output JSON:  " << output_json << std::endl;
    if (!output_pcd.empty()) {
        std::cout << " Output PCD:   " << output_pcd << std::endl;
    }
    std::cout << " Coordinate:   " << (is_optical ? "Camera Optical (Transforming to Body)" : "Vehicle Body (ISO 8855)") << std::endl;
    std::cout << " Strategy:     " << strat_str << std::endl;
    if (voxel_size > 0.0f) {
        std::cout << " Voxel Size:   " << voxel_size << " m (Pre-clustering downsampler with extent margin)" << std::endl;
    }
    std::cout << "============================================================" << std::endl;

    auto t0 = std::chrono::high_resolution_clock::now();

    // 1. Ingestion
    PointCloud raw_cloud;
    if (!loadPCD(input_pcd, raw_cloud)) {
        std::cerr << "[ERROR] Failed to load PCD from: " << input_pcd << std::endl;
        return 1;
    }
    std::cout << "[1] Ingested " << raw_cloud.size() << " raw points." << std::endl;

    // 2. Alignment to vehicle body frame (+X forward, +Y left, +Z up, road at Z=0)
    PointCloud body_cloud;
    if (is_optical) {
        CameraAlignmentParams align_params;
        align_params.mount_height_m = 1.73f;
        CameraAlignment align(align_params);
        float g[3] = {0.0f, 9.81f, 0.0f}; // Y is down in optical frame
        align.transform_to_body(raw_cloud, g, body_cloud);
    } else {
        // If raw points have Z ground around -1.73m (like KITTI velodyne coordinate), level Z so ground is ~0
        float min_z = 1e9f;
        for (size_t i = 0; i < raw_cloud.size(); ++i) {
            if (raw_cloud.z[i] < min_z) min_z = raw_cloud.z[i];
        }
        float z_offset = (min_z < -1.0f) ? 1.73f : 0.0f;

        body_cloud.resize(raw_cloud.size());
        for (size_t i = 0; i < raw_cloud.size(); ++i) {
            body_cloud.x[i] = raw_cloud.x[i];
            body_cloud.y[i] = raw_cloud.y[i];
            body_cloud.z[i] = raw_cloud.z[i] + z_offset;
        }
    }

    // 3. Ground & Canopy Elevation Slicing (Whole 360-degree scene, no X/Y corridor crop)
    PassThroughFilter filter(0.15f, 2.50f);
    PointCloud raw_obstacles_cloud;
    filter.filter(body_cloud, raw_obstacles_cloud);

    std::cout << "[2] Isolated " << raw_obstacles_cloud.size() << " non-ground obstacle points across the whole scene." << std::endl;
    if (raw_obstacles_cloud.empty()) {
        std::cerr << "[WARN] Zero obstacle points in scene. Check coordinate frame." << std::endl;
        return 0;
    }

    // Optional Level 1 Scene Voxelization
    PointCloud obstacles_cloud;
    if (voxel_size > 0.0f) {
        VoxelGrid voxel_filter(voxel_size);
        voxel_filter(raw_obstacles_cloud, obstacles_cloud, voxel_size);
        std::cout << "[2b] Voxel downsampled to " << obstacles_cloud.size() << " points (cell: " << voxel_size << "m)." << std::endl;
    } else {
        obstacles_cloud = std::move(raw_obstacles_cloud);
    }

    // 4. ForwardCellClustering (RVV 1.0 accelerated spatial grouping)
    auto t_clust_start = std::chrono::high_resolution_clock::now();
    ForwardCellClustering clusterer(0.35f, 15, 25000);
    ClusterResult clusters;
    clusterer(obstacles_cloud, clusters);
    auto t_clust_end = std::chrono::high_resolution_clock::now();
    double clust_ms = std::chrono::duration<double, std::milli>(t_clust_end - t_clust_start).count();
    std::cout << "[3] Segmented " << clusters.num_clusters() << " obstacle clusters in " << clust_ms << " ms." << std::endl;

    // 5. Decoupled Leaf Kernels: ConvexHull2D -> BoundingBoxExtractor -> BoundingDiscExtractor
    ConvexHull2D hull_extractor(2048);

    BoundingBoxParams bbox_params;
    bool is_adaptive = (strat_str == "adaptive");
    if (strat_str == "min_area") bbox_params.strategy = BoundingBoxStrategy::MIN_AREA;
    else if (strat_str == "l_shape") bbox_params.strategy = BoundingBoxStrategy::L_SHAPE_ALIGN;
    else if (strat_str == "pca") bbox_params.strategy = BoundingBoxStrategy::WIREFRAME_PCA;
    else bbox_params.strategy = BoundingBoxStrategy::EDGE_ALIGN; // Default

    BoundingBoxExtractor bbox_extractor(bbox_params, 2048);
    BoundingDiscExtractor disc_extractor;

    PointCloud2D hull_scratch;
    std::vector<OrientedBoundingBox> obstacle_boxes(clusters.num_clusters());
    std::vector<BoundingDisc> obstacle_discs(clusters.num_clusters());

    double hull_us_total = 0.0;
    double bbox_us_total = 0.0;
    double disc_us_total = 0.0;

    auto t_geom_start = std::chrono::high_resolution_clock::now();

    for (size_t c = 0; c < clusters.num_clusters(); ++c) {
        const uint32_t* c_idx = clusters.cluster_indices(c);
        const size_t c_size = clusters.cluster_size(c);

        // Stage A: 2D Convex Hull
        auto ta0 = std::chrono::high_resolution_clock::now();
        hull_extractor(obstacles_cloud, c_idx, c_size, hull_scratch);
        auto ta1 = std::chrono::high_resolution_clock::now();
        hull_us_total += std::chrono::duration<double, std::micro>(ta1 - ta0).count();

        // Stage B: 3D Bounding Box (Adaptive policy or direct strategy)
        auto tb0 = std::chrono::high_resolution_clock::now();
        if (is_adaptive) {
            // Pipeline-level policy: small/sparse clusters -> MIN_AREA; substantial clusters -> EDGE_ALIGN
            if (c_size < 20 || hull_scratch.size() < 4) {
                bbox_extractor.compute_min_area(obstacles_cloud, c_idx, c_size, hull_scratch, obstacle_boxes[c]);
            } else {
                bbox_extractor.compute_edge_align(obstacles_cloud, c_idx, c_size, hull_scratch, obstacle_boxes[c]);
            }
        } else {
            bbox_extractor(obstacles_cloud, c_idx, c_size, hull_scratch, obstacle_boxes[c]);
        }

        // Bounding margin padding if voxel downsampling was active (prevents under-bounding)
        if (voxel_size > 0.0f) {
            obstacle_boxes[c].extent_x += voxel_size;
            obstacle_boxes[c].extent_y += voxel_size;
        }
        auto tb1 = std::chrono::high_resolution_clock::now();
        bbox_us_total += std::chrono::duration<double, std::micro>(tb1 - tb0).count();

        // Stage C: Concentric Circumscribed Bounding Disc
        auto tc0 = std::chrono::high_resolution_clock::now();
        disc_extractor.compute_concentric(obstacle_boxes[c], obstacle_discs[c]);
        auto tc1 = std::chrono::high_resolution_clock::now();
        disc_us_total += std::chrono::duration<double, std::micro>(tc1 - tc0).count();
    }

    auto t_geom_end = std::chrono::high_resolution_clock::now();
    double geom_total_ms = std::chrono::duration<double, std::milli>(t_geom_end - t_geom_start).count();

    auto t1 = std::chrono::high_resolution_clock::now();
    double total_pipeline_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::cout << "\n============================================================" << std::endl;
    std::cout << " Obstacle Perception Timing Profile (QEMU Emulation)" << std::endl;
    std::cout << "============================================================" << std::endl;
    std::cout << "  [1] PassThrough & Voxel Filter:    " << std::chrono::duration<double, std::milli>(t_clust_start - t0).count() << " ms" << std::endl;
    std::cout << "  [2] ForwardCellClustering:         " << clust_ms << " ms (" << clusters.num_clusters() << " clusters)" << std::endl;
    std::cout << "  [3] Geometry Extraction (Total):   " << geom_total_ms << " ms" << std::endl;
    std::cout << "      ├─ ConvexHull2D:               " << hull_us_total << " us (" << (hull_us_total / 1000.0) << " ms, " << (hull_us_total / clusters.num_clusters()) << " us/cluster)" << std::endl;
    std::cout << "      ├─ BoundingBox (" << strat_str << "): " << bbox_us_total << " us (" << (bbox_us_total / 1000.0) << " ms, " << (bbox_us_total / clusters.num_clusters()) << " us/cluster)" << std::endl;
    std::cout << "      └─ BoundingDisc (Concentric):  " << disc_us_total << " us (" << (disc_us_total / 1000.0) << " ms, " << (disc_us_total / clusters.num_clusters()) << " us/cluster)" << std::endl;
    std::cout << "  ----------------------------------------------------------" << std::endl;
    std::cout << "  End-to-End Frame Processing:       " << total_pipeline_ms << " ms" << std::endl;
    std::cout << "============================================================\n" << std::endl;

    // 6. Export to JSON
    std::ofstream json_file(output_json);
    if (!json_file.is_open()) {
        std::cerr << "[ERROR] Could not open output file: " << output_json << std::endl;
        return 1;
    }

    constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;

    json_file << "{\n";
    json_file << "  \"input_pcd\": \"" << input_pcd << "\",\n";
    json_file << "  \"raw_point_count\": " << raw_cloud.size() << ",\n";
    json_file << "  \"obstacle_point_count\": " << obstacles_cloud.size() << ",\n";
    json_file << "  \"num_clusters\": " << obstacle_boxes.size() << ",\n";
    json_file << "  \"clusters\": [\n";

    for (size_t i = 0; i < obstacle_boxes.size(); ++i) {
        const auto& box = obstacle_boxes[i];
        const auto& disc = obstacle_discs[i];
        json_file << "    {\n";
        json_file << "      \"id\": " << i << ",\n";
        json_file << "      \"point_count\": " << box.point_count << ",\n";
        json_file << "      \"disc\": {\"cx\": " << disc.cx << ", \"cy\": " << disc.cy
                  << ", \"radius\": " << disc.radius << ", \"z_min\": " << disc.z_min
                  << ", \"z_max\": " << disc.z_max << "},\n";
        json_file << "      \"obb\": {\"cx\": " << box.cx << ", \"cy\": " << box.cy
                  << ", \"cz\": " << box.cz << ", \"extent_x\": " << box.extent_x
                  << ", \"extent_y\": " << box.extent_y << ", \"extent_z\": " << box.extent_z
                  << ", \"yaw_deg\": " << (box.yaw_rad / kDegToRad) << ",\n";
        json_file << "        \"corners\": [";
        for (int k = 0; k < 4; ++k) {
            json_file << "{\"x\": " << box.corners[k].x << ", \"y\": " << box.corners[k].y << "}"
                      << (k < 3 ? ", " : "");
        }
        json_file << "]},\n";

        // Sample points for 3D visualizer display
        json_file << "      \"sample_points\": [";
        auto [idx_ptr, idx_count] = clusters.cluster(i);
        size_t step = std::max<size_t>(1, idx_count / 120);
        bool first = true;
        for (size_t s = 0; s < idx_count; s += step) {
            uint32_t p_idx = idx_ptr[s];
            if (!first) json_file << ", ";
            json_file << "{\"x\": " << obstacles_cloud.x[p_idx]
                      << ", \"y\": " << obstacles_cloud.y[p_idx]
                      << ", \"z\": " << obstacles_cloud.z[p_idx] << "}";
            first = false;
        }
        json_file << "]\n";
        json_file << "    }" << (i + 1 < obstacle_boxes.size() ? "," : "") << "\n";
    }
    json_file << "  ]\n}\n";
    json_file.close();

    std::cout << "[5] Exported telemetry JSON -> " << output_json << std::endl;

    // Optional PCD export
    if (!output_pcd.empty()) {
        savePCD(output_pcd, obstacles_cloud, true);
        std::cout << "[6] Exported segmented obstacle PCD -> " << output_pcd << std::endl;
    }

    std::cout << "==> Obstacle extraction completed successfully." << std::endl;
    return 0;
}

