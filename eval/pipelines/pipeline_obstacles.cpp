#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <chrono>

#include "features/bounding_box/bounding_box.h"
#include "filters/camera_alignment/camera_alignment.h"
#include "filters/passthrough_filter/passthrough_filter.h"
#include "segmentation/forward_cell_clustering.h"
#include "io/simple_pcd_loader.h"
#include "core/point_types.h"

using namespace rvpoint;

static void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " <input_pcd> [output_json] [output_obstacles_pcd] [--optical|--body]\n"
              << "Options:\n"
              << "  --optical   Input is camera optical frame (X right, Y down, Z forward). Aligns with gravity.\n"
              << "  --body      Input is already vehicle body frame (X forward, Y left, Z up). Default.\n"
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
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--optical") is_optical = true;
        if (arg == "--body") is_optical = false;
    }

    std::cout << "============================================================" << std::endl;
    std::cout << " RVPoint Perception Pipeline: Obstacle & 3D Bounding Box Extractor" << std::endl;
    std::cout << " Input PCD:    " << input_pcd << std::endl;
    std::cout << " Output JSON:  " << output_json << std::endl;
    if (!output_pcd.empty()) {
        std::cout << " Output PCD:   " << output_pcd << std::endl;
    }
    std::cout << " Coordinate:   " << (is_optical ? "Camera Optical (Transforming to Body)" : "Vehicle Body (ISO 8855)") << std::endl;
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

    // 3. PassThroughFilter (Corridor cropping: isolate obstacles from road ground and overhead)
    PassThroughFilter filter(0.15f, 2.50f);
    filter.set_limits_x(1.0f, 25.0f);
    filter.set_limits_y(-5.5f, 5.5f);
    PointCloud obstacles_cloud;
    filter.filter(body_cloud, obstacles_cloud);

    std::cout << "[2] Isolated " << obstacles_cloud.size() << " non-ground driving corridor points." << std::endl;
    if (obstacles_cloud.empty()) {
        std::cerr << "[WARN] Zero obstacle points in driving corridor. Check coordinate frame." << std::endl;
        return 0;
    }

    // 4. ForwardCellClustering (RVV 1.0 accelerated spatial grouping)
    ForwardCellClustering clusterer(0.35f, 15, 3000);
    ClusterResult clusters;
    clusterer(obstacles_cloud, clusters);
    std::cout << "[3] Segmented " << clusters.num_clusters() << " obstacle clusters." << std::endl;

    // 5. ObstacleGeometryExtractor (ADR-0009: Bounding Discs + Rotating Calipers 3D OBBs)
    ObstacleGeometryExtractor extractor(4096);
    std::vector<ObstacleGeometry> obstacle_geoms;
    extractor.extract_all(obstacles_cloud, clusters, obstacle_geoms);

    auto t1 = std::chrono::high_resolution_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::cout << "[4] Extracted dual geometries in " << total_ms << " ms total." << std::endl;

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
    json_file << "  \"num_clusters\": " << obstacle_geoms.size() << ",\n";
    json_file << "  \"clusters\": [\n";

    for (size_t i = 0; i < obstacle_geoms.size(); ++i) {
        const auto& o = obstacle_geoms[i];
        json_file << "    {\n";
        json_file << "      \"id\": " << i << ",\n";
        json_file << "      \"point_count\": " << o.obb.point_count << ",\n";
        json_file << "      \"disc\": {\"cx\": " << o.disc.cx << ", \"cy\": " << o.disc.cy
                  << ", \"radius\": " << o.disc.radius << ", \"z_min\": " << o.disc.z_min
                  << ", \"z_max\": " << o.disc.z_max << "},\n";
        json_file << "      \"obb\": {\"cx\": " << o.obb.cx << ", \"cy\": " << o.obb.cy
                  << ", \"cz\": " << o.obb.cz << ", \"extent_x\": " << o.obb.extent_x
                  << ", \"extent_y\": " << o.obb.extent_y << ", \"extent_z\": " << o.obb.extent_z
                  << ", \"yaw_deg\": " << (o.obb.yaw_rad / kDegToRad) << ",\n";
        json_file << "        \"corners\": [";
        for (int k = 0; k < 4; ++k) {
            json_file << "{\"x\": " << o.obb.corners_x[k] << ", \"y\": " << o.obb.corners_y[k] << "}"
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
        json_file << "    }" << (i + 1 < obstacle_geoms.size() ? "," : "") << "\n";
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
