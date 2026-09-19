#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <algorithm>

#include "core/point_types.h"
#include "features/convex_hull/convex_hull.h"
#include "features/bounding_disc/bounding_disc.h"
#include "features/bounding_box/bounding_box.h"
#include "filters/camera_alignment/camera_alignment.h"
#include "filters/passthrough_filter/passthrough_filter.h"
#include "segmentation/forward_cell_clustering.h"
#include "io/simple_pcd_loader.h"

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

static void test_convex_hull_strategies() {
    std::cout << "[1] Verifying 2D convex hull strategies (Monotone Chain, Jarvis March, Angular Binning)..." << std::endl;
    ConvexHull2D hull_mono(128, ConvexHullStrategy::MONOTONE_CHAIN);
    ConvexHull2D hull_jarvis(128, ConvexHullStrategy::JARVIS_MARCH);
    ConvexHull2D hull_bin(128, ConvexHullStrategy::ANGULAR_BINNING);

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

    PointCloud2D h_mono, h_jarvis, h_bin;
    hull_mono.compute(cloud, indices.data(), indices.size(), h_mono);
    hull_jarvis.compute(cloud, indices.data(), indices.size(), h_jarvis);
    hull_bin.compute(cloud, indices.data(), indices.size(), h_bin);

    TEST_CHECK(h_mono.size() == 5, "Monotone chain extracts exactly 5 pentagon vertices");
    TEST_CHECK(h_jarvis.size() == 5, "Jarvis March extracts exactly 5 pentagon vertices");
    TEST_CHECK(h_bin.size() >= 4, "Angular binning captures polygon bounds");

    std::cout << "    [PASS] Convex hull strategies verified (Monotone Chain: "
              << h_mono.size() << " pts, Jarvis: " << h_jarvis.size() << " pts, Binning: " << h_bin.size() << " pts)." << std::endl;
}

static void test_bounding_disc_dual_paths() {
    std::cout << "[2] Verifying Bounding Disc dual paths (Point-Centroid & Concentric)..." << std::endl;
    BoundingDiscExtractor disc_extractor(256);

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

    // Path 1: Point-Centroid Disc
    BoundingDisc disc_pts;
    disc_extractor.compute_from_points(cloud, indices.data(), indices.size(), disc_pts);

    TEST_CHECK(disc_pts.point_count == cloud.size(), "Disc point count");
    TEST_CHECK(std::abs(disc_pts.cx - true_cx) < 0.02f, "Centroid X match");
    TEST_CHECK(std::abs(disc_pts.cy - true_cy) < 0.02f, "Centroid Y match");
    TEST_CHECK(std::abs(disc_pts.radius - true_radius) < 0.02f, "Bounding radius match");

    // Path 2: Concentric Disc from synthetic OBB
    OrientedBoundingBox mock_box;
    mock_box.cx = 4.0f;
    mock_box.cy = 2.0f;
    mock_box.cz = 0.5f;
    mock_box.extent_x = 4.0f; // length
    mock_box.extent_y = 2.0f; // width
    mock_box.extent_z = 1.5f; // height
    mock_box.point_count = 100;

    BoundingDisc disc_concentric;
    disc_extractor.compute_concentric(mock_box, disc_concentric);

    const float expected_r = 0.5f * std::sqrt(4.0f * 4.0f + 2.0f * 2.0f); // 0.5 * sqrt(20) = ~2.236m
    TEST_CHECK(std::abs(disc_concentric.cx - 4.0f) < 1e-4f, "Concentric disc CX matches OBB");
    TEST_CHECK(std::abs(disc_concentric.cy - 2.0f) < 1e-4f, "Concentric disc CY matches OBB");
    TEST_CHECK(std::abs(disc_concentric.radius - expected_r) < 1e-3f, "Concentric disc radius circumscribes OBB exactly");

    std::cout << "    [PASS] Both Point-Centroid and Concentric disc paths verified." << std::endl;
}

static void test_synthetic_rotated_obb() {
    std::cout << "[3] Verifying 3D Oriented Bounding Box (Rotating Calipers MIN_AREA & Wireframe PCA)..." << std::endl;
    ConvexHull2D hull_extractor(512);

    BoundingBoxParams params;
    params.strategy = BoundingBoxStrategy::MIN_AREA;
    BoundingBoxExtractor bbox_extractor(params, 512);

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

    PointCloud2D hull;
    hull_extractor.compute(cloud, indices.data(), indices.size(), hull);

    OrientedBoundingBox obb;
    bbox_extractor.compute(cloud, indices.data(), indices.size(), hull, obb);

    TEST_CHECK(std::abs(obb.cx - true_cx) < 0.02f, "OBB Centroid X match");
    TEST_CHECK(std::abs(obb.cy - true_cy) < 0.02f, "OBB Centroid Y match");
    TEST_CHECK(std::abs(obb.cz - true_cz) < 0.02f, "OBB Centroid Z match");
    TEST_CHECK(std::abs(obb.extent_x - true_length) < 0.03f, "OBB Extent X (length) match");
    TEST_CHECK(std::abs(obb.extent_y - true_width) < 0.03f, "OBB Extent Y (width) match");
    TEST_CHECK(std::abs(obb.extent_z - true_height) < 0.02f, "OBB Extent Z (height) match");

    float yaw_diff = std::abs(obb.yaw_rad - yaw_rad);
    if (yaw_diff > kPi * 0.5f) yaw_diff = std::abs(yaw_diff - kPi);
    TEST_CHECK(yaw_diff < 2.0f * kDegToRad, "OBB Yaw heading matches within 2 degrees");

    // Verify synchronized footprint corners
    for (int k = 0; k < 4; ++k) {
        float dx = obb.corners[k].x - obb.cx;
        float dy = obb.corners[k].y - obb.cy;
        float dist = std::sqrt(dx * dx + dy * dy);
        float expected_diag = std::sqrt(half_l * half_l + half_w * half_w);
        TEST_CHECK(std::abs(dist - expected_diag) < 0.03f, "Corner vertex diagonal distance matches extents");
    }

    // Verify Strategy WIREFRAME_PCA
    BoundingBoxExtractor pca_extractor(BoundingBoxParams{BoundingBoxStrategy::WIREFRAME_PCA}, 512);
    OrientedBoundingBox obb_pca;
    pca_extractor.compute(cloud, indices.data(), indices.size(), hull, obb_pca);
    TEST_CHECK(std::abs(obb_pca.extent_x - true_length) < 0.05f, "Wireframe PCA extent X");
    TEST_CHECK(std::abs(obb_pca.extent_y - true_width) < 0.05f, "Wireframe PCA extent Y");

    std::cout << "    [PASS] 3D OBB verified: Centroid (" << obb.cx << ", " << obb.cy
              << "), Extents (" << obb.extent_x << " x " << obb.extent_y << ")m, Yaw: "
              << (obb.yaw_rad / kDegToRad) << " deg." << std::endl;
}

static void test_l_shape_face_alignment() {
    std::cout << "[4] Verifying L-Shape Face Alignment (Snapping to Vehicle Straight Faces)..." << std::endl;

    const float true_heading_deg = 25.0f;
    const float heading_rad = true_heading_deg * kDegToRad;
    const float cos_h = std::cos(heading_rad);
    const float sin_h = std::sin(heading_rad);

    const float center_x = 5.0f;
    const float center_y = -1.0f;
    const float center_z = 0.5f;

    const float L = 4.0f;
    const float W = 2.0f;

    PointCloud cloud;

    // Visible Face 1: Side panel (-L/2 to +L/2 at lateral offset -W/2)
    for (int i = 0; i <= 40; ++i) {
        float u = -0.5f * L + (L * i) / 40.0f;
        float v = -0.5f * W;
        float x = center_x + u * cos_h - v * sin_h;
        float y = center_y + u * sin_h + v * cos_h;
        cloud.push_back(x, y, center_z);
    }

    // Visible Face 2: Front bumper (-W/2 to +W/2 at longitudinal offset +L/2)
    for (int i = 0; i <= 20; ++i) {
        float u = 0.5f * L;
        float v = -0.5f * W + (W * i) / 20.0f;
        float x = center_x + u * cos_h - v * sin_h;
        float y = center_y + u * sin_h + v * cos_h;
        cloud.push_back(x, y, center_z);
    }

    // Add slight corner rounding (indented points at the corner)
    for (int i = 0; i < 5; ++i) {
        float u = 0.5f * L - 0.15f;
        float v = -0.5f * W + 0.15f;
        float x = center_x + u * cos_h - v * sin_h;
        float y = center_y + u * sin_h + v * cos_h;
        cloud.push_back(x, y, center_z);
    }

    std::vector<uint32_t> indices(cloud.size());
    for (size_t i = 0; i < indices.size(); ++i) indices[i] = static_cast<uint32_t>(i);

    ConvexHull2D hull_extractor(256);
    PointCloud2D hull;
    hull_extractor.compute(cloud, indices.data(), indices.size(), hull);

    BoundingBoxParams lshape_params;
    lshape_params.strategy = BoundingBoxStrategy::L_SHAPE_ALIGN;
    lshape_params.truncation_dist = 0.20f;
    lshape_params.area_constraint_ratio = 1.25f;
    BoundingBoxExtractor lshape_extractor(lshape_params, 256);

    OrientedBoundingBox obb_lshape;
    lshape_extractor.compute(cloud, indices.data(), indices.size(), hull, obb_lshape);

    float yaw_diff = std::abs(obb_lshape.yaw_rad - heading_rad);
    if (yaw_diff > kPi * 0.5f) yaw_diff = std::abs(yaw_diff - kPi);

    std::cout << "    L_SHAPE_ALIGN detected yaw: " << (obb_lshape.yaw_rad / kDegToRad)
              << " deg (target: " << true_heading_deg << " deg, error: " << (yaw_diff / kDegToRad) << " deg)" << std::endl;

    TEST_CHECK(yaw_diff < 3.0f * kDegToRad, "L_SHAPE_ALIGN snaps accurately to natural vehicle faces within 3 degrees");

    // Also verify EDGE_ALIGN (Hull Edge-Perimeter Alignment)
    BoundingBoxParams edge_params;
    edge_params.strategy = BoundingBoxStrategy::EDGE_ALIGN;
    BoundingBoxExtractor edge_extractor(edge_params, 256);

    OrientedBoundingBox obb_edge;
    edge_extractor.compute(cloud, indices.data(), indices.size(), hull, obb_edge);

    float edge_yaw_diff = std::abs(obb_edge.yaw_rad - heading_rad);
    if (edge_yaw_diff > kPi * 0.5f) edge_yaw_diff = std::abs(edge_yaw_diff - kPi);

    std::cout << "    EDGE_ALIGN detected yaw:    " << (obb_edge.yaw_rad / kDegToRad)
              << " deg (target: " << true_heading_deg << " deg, error: " << (edge_yaw_diff / kDegToRad) << " deg)" << std::endl;

    TEST_CHECK(edge_yaw_diff < 3.0f * kDegToRad, "EDGE_ALIGN snaps accurately to vehicle sheet metal within 3 degrees");

    // Also evaluate MIN_AREA and WIREFRAME_PCA to measure their failure modes on vehicle L-shapes
    OrientedBoundingBox obb_min_area, obb_pca;
    edge_extractor.compute_min_area(cloud, indices.data(), indices.size(), hull, obb_min_area);
    edge_extractor.compute_wireframe_pca(hull, 0.0f, 1.0f, static_cast<uint32_t>(cloud.size()), obb_pca);

    float min_area_yaw_diff = std::abs(obb_min_area.yaw_rad - heading_rad);
    if (min_area_yaw_diff > kPi * 0.5f) min_area_yaw_diff = std::abs(min_area_yaw_diff - kPi);

    float pca_yaw_diff = std::abs(obb_pca.yaw_rad - heading_rad);
    if (pca_yaw_diff > kPi * 0.5f) pca_yaw_diff = std::abs(pca_yaw_diff - kPi);

    std::cout << "    MIN_AREA detected yaw:      " << (obb_min_area.yaw_rad / kDegToRad)
              << " deg (target: " << true_heading_deg << " deg, error: " << (min_area_yaw_diff / kDegToRad) << " deg)" << std::endl;
    std::cout << "    WIREFRAME_PCA detected yaw: " << (obb_pca.yaw_rad / kDegToRad)
              << " deg (target: " << true_heading_deg << " deg, error: " << (pca_yaw_diff / kDegToRad) << " deg)" << std::endl;

    // Verify direct unconditional methods produce identical results
    OrientedBoundingBox direct_box;
    edge_extractor.compute_edge_align(cloud, indices.data(), indices.size(), hull, direct_box);
    TEST_CHECK(std::abs(direct_box.yaw_rad - obb_edge.yaw_rad) < 1e-4f, "Direct compute_edge_align produces identical box");

    edge_extractor.compute_l_shape(cloud, indices.data(), indices.size(), hull, direct_box);
    TEST_CHECK(std::abs(direct_box.yaw_rad - obb_lshape.yaw_rad) < 1e-4f, "Direct compute_l_shape produces identical box");

    std::cout << "    [PASS] L-Shape and Edge-Perimeter Alignment snapped successfully to vehicle sheet metal orientation." << std::endl;
}

static void test_end_to_end_clustering_and_extraction() {
    std::cout << "[5] Verifying end-to-end clustering -> decoupled geometry extraction..." << std::endl;
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

    ConvexHull2D hull_extractor(256);
    BoundingBoxExtractor bbox_extractor;
    BoundingDiscExtractor disc_extractor;

    PointCloud2D hull;
    std::vector<OrientedBoundingBox> boxes(clusters.num_clusters());
    std::vector<BoundingDisc> discs(clusters.num_clusters());

    for (size_t c = 0; c < clusters.num_clusters(); ++c) {
        const uint32_t* c_idx = clusters.cluster_indices(c);
        const size_t c_size = clusters.cluster_size(c);

        hull_extractor(cloud, c_idx, c_size, hull);
        bbox_extractor(cloud, c_idx, c_size, hull, boxes[c]);
        disc_extractor.compute_concentric(boxes[c], discs[c]);

        TEST_CHECK(discs[c].radius > 0.05f, "Valid disc radius");
        TEST_CHECK(boxes[c].extent_x > 0.05f, "Valid OBB extent_x");
        TEST_CHECK(boxes[c].extent_y > 0.05f, "Valid OBB extent_y");
    }

    std::cout << "    [PASS] End-to-end integration verified: 2 clusters transformed via decoupled leaf kernels." << std::endl;
}

static void test_real_pcd_obstacle_extraction() {
    std::cout << "[6] Running decoupled obstacle extraction on whole real dataset (data/pcd_compressed/0000000000.pcd)..." << std::endl;
    PointCloud raw_cloud;
    std::string path = "data/pcd_compressed/0000000000.pcd";
    if (!loadPCD(path, raw_cloud)) {
        path = "data/0000000000.pcd";
        if (!loadPCD(path, raw_cloud)) {
            std::cout << "    [SKIP] Sample PCD file not found." << std::endl;
            return;
        }
    }

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

    // Whole scene: no artificial X/Y corridor limits
    PassThroughFilter filter(0.15f, 2.50f);
    PointCloud obstacles_cloud;
    filter.filter(body_cloud, obstacles_cloud);

    TEST_CHECK(!obstacles_cloud.empty(), "Obstacle points isolated from real PCD");

    ForwardCellClustering clusterer(0.35f, 15, 25000);
    ClusterResult clusters;
    clusterer(obstacles_cloud, clusters);

    TEST_CHECK(clusters.num_clusters() > 0, "At least one obstacle cluster segmented across the whole scene");

    ConvexHull2D hull_extractor(4096);
    BoundingBoxParams bbox_params;
    bbox_params.strategy = BoundingBoxStrategy::L_SHAPE_ALIGN;
    BoundingBoxExtractor bbox_extractor(bbox_params, 4096);
    BoundingDiscExtractor disc_extractor;

    PointCloud2D hull;
    std::vector<OrientedBoundingBox> boxes(clusters.num_clusters());
    std::vector<BoundingDisc> discs(clusters.num_clusters());

    for (size_t c = 0; c < clusters.num_clusters(); ++c) {
        const uint32_t* c_idx = clusters.cluster_indices(c);
        const size_t c_size = clusters.cluster_size(c);

        hull_extractor(obstacles_cloud, c_idx, c_size, hull);
        bbox_extractor(obstacles_cloud, c_idx, c_size, hull, boxes[c]);
        disc_extractor.compute_concentric(boxes[c], discs[c]);
    }

    std::cout << "    Segmented " << clusters.num_clusters() << " obstacle clusters across whole scene from "
              << obstacles_cloud.size() << " non-ground points." << std::endl;

    TEST_CHECK(clusters.num_clusters() >= 50, "Extracted >= 50 whole-scene obstacle clusters");

    std::cout << "    [PASS] Real PCD whole-scene decoupled obstacle geometry pipeline executed cleanly." << std::endl;
}

int main() {
    std::cout << "============================================================" << std::endl;
    std::cout << " Running test_obb_extraction (Decoupled Geometry Stack)" << std::endl;
    std::cout << "============================================================" << std::endl;

    test_convex_hull_strategies();
    test_bounding_disc_dual_paths();
    test_synthetic_rotated_obb();
    test_l_shape_face_alignment();
    test_end_to_end_clustering_and_extraction();
    test_real_pcd_obstacle_extraction();

    std::cout << "============================================================" << std::endl;
    std::cout << " test_obb_extraction PASSED ALL CHECKS" << std::endl;
    std::cout << "============================================================" << std::endl;
    return 0;
}
