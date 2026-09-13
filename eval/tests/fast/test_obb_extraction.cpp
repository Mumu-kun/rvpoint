#include <cmath>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <algorithm>

#include "features/bounding_box/bounding_box.h"
#include "segmentation/forward_cell_clustering.h"
#include "core/point_types.h"

using namespace rvpoint;

#define TEST_CHECK(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "    [FAIL] " << msg << " (" #cond ") at line " << __LINE__ << std::endl; \
            std::exit(1); \
        } \
    } while (0)

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;

static void test_convex_hull_2d() {
    std::cout << "[1] Verifying Andrew's Monotone Chain 2D convex hull..." << std::endl;
    ObstacleGeometryExtractor extractor(128);

    PointCloud cloud;
    // Known pentagon vertices:
    // P0(-1, -1), P1(1, -1), P2(2, 0), P3(1, 1), P4(-1, 1)
    cloud.push_back(-1.0f, -1.0f, 0.0f);
    cloud.push_back( 1.0f, -1.0f, 0.0f);
    cloud.push_back( 2.0f,  0.0f, 0.0f);
    cloud.push_back( 1.0f,  1.0f, 0.0f);
    cloud.push_back(-1.0f,  1.0f, 0.0f);

    // Add 20 interior points inside the pentagon
    for (int i = 0; i < 20; ++i) {
        float x = -0.5f + (i % 5) * 0.2f;
        float y = -0.5f + (i / 5) * 0.2f;
        cloud.push_back(x, y, 0.0f);
    }

    std::vector<uint32_t> indices(cloud.size());
    for (size_t i = 0; i < indices.size(); ++i) indices[i] = static_cast<uint32_t>(i);

    std::vector<Point2D> hull;
    extractor.compute_convex_hull_2d(cloud, indices.data(), indices.size(), hull);

    TEST_CHECK(hull.size() == 5, "Convex hull has exactly 5 vertices");

    // Verify all hull points are exterior points (orig_idx < 5)
    for (const auto& pt : hull) {
        TEST_CHECK(pt.orig_idx < 5, "Hull vertex is an exterior point");
    }

    std::cout << "    [PASS] 2D Convex hull verified: 5 outer vertices retained, interior points eliminated." << std::endl;
}

static void test_bounding_disc() {
    std::cout << "[2] Verifying Bounding Disc extraction (centroid + max radius)..." << std::endl;
    ObstacleGeometryExtractor extractor(256);

    PointCloud cloud;
    const float true_cx = 1.50f;
    const float true_cy = -0.80f;
    const float true_radius = 0.45f;

    // Generate 64 points around circle + interior
    for (int i = 0; i < 64; ++i) {
        float angle = (i * 2.0f * kPi) / 64.0f;
        float r = (i % 4 == 0) ? true_radius : (true_radius * (0.3f + 0.7f * (i % 5) / 5.0f));
        float x = true_cx + r * std::cos(angle);
        float y = true_cy + r * std::sin(angle);
        float z = 0.05f + 0.25f * (i % 7) / 7.0f;
        cloud.push_back(x, y, z);
    }

    std::vector<uint32_t> indices(cloud.size());
    for (size_t i = 0; i < indices.size(); ++i) indices[i] = static_cast<uint32_t>(i);

    BoundingDisc disc;
    extractor.compute_disc(cloud, indices.data(), indices.size(), disc);

    TEST_CHECK(disc.point_count == cloud.size(), "Disc point count");
    TEST_CHECK(std::abs(disc.cx - true_cx) < 0.02f, "Centroid X match");
    TEST_CHECK(std::abs(disc.cy - true_cy) < 0.02f, "Centroid Y match");
    TEST_CHECK(std::abs(disc.radius - true_radius) < 0.02f, "Bounding radius match");
    TEST_CHECK(disc.z_min >= 0.049f && disc.z_min <= 0.06f, "Elevation z_min match");
    TEST_CHECK(disc.z_max <= 0.31f && disc.z_max >= 0.25f, "Elevation z_max match");

    std::cout << "    [PASS] Bounding disc verified: Centroid (" << disc.cx << ", " << disc.cy
              << "), Radius: " << disc.radius << "m, Z: [" << disc.z_min << ", " << disc.z_max << "]m." << std::endl;
}

static void test_synthetic_rotated_obb() {
    std::cout << "[3] Verifying 3D Oriented Bounding Box (Rotating Calipers)..." << std::endl;
    ObstacleGeometryExtractor extractor(512);

    const float true_cx = 2.50f;
    const float true_cy = 1.20f;
    const float true_cz = 0.40f;
    const float true_length = 1.60f; // along heading
    const float true_width  = 0.70f; // across heading
    const float true_height = 0.60f; // Z
    const float true_yaw_deg = 35.0f;
    const float yaw_rad = true_yaw_deg * kDegToRad;

    const float cos_y = std::cos(yaw_rad);
    const float sin_y = std::sin(yaw_rad);

    PointCloud cloud;
    // Sample points densely along the perimeter and corners of the rotated box
    const float half_l = true_length * 0.5f;
    const float half_w = true_width * 0.5f;
    const float half_h = true_height * 0.5f;

    // 8 box corners
    float corner_u[4] = {-half_l,  half_l, half_l, -half_l};
    float corner_v[4] = {-half_w, -half_w, half_w,  half_w};
    for (int i = 0; i < 4; ++i) {
        float x = true_cx + corner_u[i] * cos_y - corner_v[i] * sin_y;
        float y = true_cy + corner_u[i] * sin_y + corner_v[i] * cos_y;
        cloud.push_back(x, y, true_cz - half_h);
        cloud.push_back(x, y, true_cz + half_h);
    }

    // Points along edges
    for (int s = 0; s <= 20; ++s) {
        float alpha = -half_l + (true_length * s) / 20.0f;
        for (int side : {-1, 1}) {
            float v = side * half_w;
            float x = true_cx + alpha * cos_y - v * sin_y;
            float y = true_cy + alpha * sin_y + v * cos_y;
            cloud.push_back(x, y, true_cz);
        }
    }
    for (int s = 0; s <= 10; ++s) {
        float beta = -half_w + (true_width * s) / 10.0f;
        for (int side : {-1, 1}) {
            float u = side * half_l;
            float x = true_cx + u * cos_y - beta * sin_y;
            float y = true_cy + u * sin_y + beta * cos_y;
            cloud.push_back(x, y, true_cz);
        }
    }

    std::vector<uint32_t> indices(cloud.size());
    for (size_t i = 0; i < indices.size(); ++i) indices[i] = static_cast<uint32_t>(i);

    OrientedBoundingBox obb;
    extractor.compute_obb(cloud, indices.data(), indices.size(), obb);

    TEST_CHECK(std::abs(obb.cx - true_cx) < 0.02f, "OBB Centroid X match");
    TEST_CHECK(std::abs(obb.cy - true_cy) < 0.02f, "OBB Centroid Y match");
    TEST_CHECK(std::abs(obb.cz - true_cz) < 0.02f, "OBB Centroid Z match");
    TEST_CHECK(std::abs(obb.extent_x - true_length) < 0.03f, "OBB Extent X (length) match");
    TEST_CHECK(std::abs(obb.extent_y - true_width) < 0.03f, "OBB Extent Y (width) match");
    TEST_CHECK(std::abs(obb.extent_z - true_height) < 0.02f, "OBB Extent Z (height) match");

    // Yaw comparison (accounting for 180 degree box symmetry)
    float yaw_diff = std::abs(obb.yaw_rad - yaw_rad);
    if (yaw_diff > kPi * 0.5f) yaw_diff = std::abs(yaw_diff - kPi);
    TEST_CHECK(yaw_diff < 2.0f * kDegToRad, "OBB Yaw heading matches within 2 degrees");

    std::cout << "    [PASS] 3D OBB verified: Centroid (" << obb.cx << ", " << obb.cy << ", " << obb.cz
              << "), Extents (" << obb.extent_x << " x " << obb.extent_y << " x " << obb.extent_z
              << ")m, Yaw: " << (obb.yaw_rad / kDegToRad) << " deg (target: " << true_yaw_deg << " deg)." << std::endl;
}

static void test_end_to_end_clustering_and_extraction() {
    std::cout << "[4] Verifying end-to-end clustering -> dual obstacle extraction..." << std::endl;
    PointCloud cloud;

    // Obstacle 1: Box at (1.5, -0.6), 50 points
    for (int i = 0; i < 50; ++i) {
        float x = 1.4f + (i % 5) * 0.05f;
        float y = -0.7f + ((i / 5) % 5) * 0.05f;
        float z = 0.05f + (i / 25) * 0.10f;
        cloud.push_back(x, y, z);
    }

    // Obstacle 2: Cylindrical pole at (3.0, 0.8), 40 points
    for (int i = 0; i < 40; ++i) {
        float theta = (i * 2.0f * kPi) / 20.0f;
        float x = 3.0f + 0.15f * std::cos(theta);
        float y = 0.8f + 0.15f * std::sin(theta);
        float z = 0.02f + (i / 20) * 0.10f;
        cloud.push_back(x, y, z);
    }

    ForwardCellClustering clusterer(0.15f, 10, 500);
    ClusterResult clusters;
    clusterer(cloud, clusters);

    TEST_CHECK(clusters.num_clusters() == 2, "Extracted exactly 2 obstacle clusters");

    ObstacleGeometryExtractor extractor;
    std::vector<ObstacleGeometry> obstacles;
    extractor.extract_all(cloud, clusters, obstacles);

    TEST_CHECK(obstacles.size() == 2, "Extracted 2 dual obstacle geometries");

    for (size_t i = 0; i < obstacles.size(); ++i) {
        const auto& obs = obstacles[i];
        TEST_CHECK(obs.disc.radius > 0.05f, "Valid disc radius");
        TEST_CHECK(obs.obb.extent_x > 0.05f, "Valid OBB extent_x");
        TEST_CHECK(obs.obb.extent_y > 0.05f, "Valid OBB extent_y");
        TEST_CHECK(obs.obb.extent_z >= 0.05f, "Valid OBB extent_z");
    }

    std::cout << "    [PASS] End-to-end integration verified: 2 clusters segmented and transformed to dual geometric bounds." << std::endl;
}

int main() {
    std::cout << "============================================================" << std::endl;
    std::cout << " Running test_obb_extraction (ADR-0009 Dual Obstacle Extractor)" << std::endl;
    std::cout << "============================================================" << std::endl;

    test_convex_hull_2d();
    test_bounding_disc();
    test_synthetic_rotated_obb();
    test_end_to_end_clustering_and_extraction();

    std::cout << "============================================================" << std::endl;
    std::cout << " test_obb_extraction PASSED ALL CHECKS" << std::endl;
    std::cout << "============================================================" << std::endl;
    return 0;
}
