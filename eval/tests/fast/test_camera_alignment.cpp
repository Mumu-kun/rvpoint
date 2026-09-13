#include <cmath>
#include <iostream>
#include <vector>
#include <cstdlib>

#include "filters/camera_alignment/camera_alignment.h"
#include "filters/passthrough_filter/passthrough_filter.h"
#include "io/simple_pcd_loader.h"

using namespace rvpoint;

#define TEST_CHECK(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "    [FAIL] " << msg << " (" #cond ") at line " << __LINE__ << std::endl; \
            std::exit(1); \
        } \
    } while (0)

static void test_camera_alignment_level() {
    std::cout << "[1] Verifying level extrinsics alignment (0 pitch)..." << std::endl;
    CameraAlignmentParams params;
    params.mount_height_m = 0.15f;
    params.mount_x_offset_m = 0.05f;
    params.mount_y_offset_m = 0.0f;
    CameraAlignment align(params);

    float g[3] = {0.0f, 9.81f, 0.0f};
    float R[9], t[3];
    align.compute_extrinsics(g, R, t);

    // Optical forward [0, 0, 1] -> Body forward [1, 0, 0]
    float fwd_x = R[0]*0.0f + R[1]*0.0f + R[2]*1.0f;
    float fwd_y = R[3]*0.0f + R[4]*0.0f + R[5]*1.0f;
    float fwd_z = R[6]*0.0f + R[7]*0.0f + R[8]*1.0f;

    TEST_CHECK(std::abs(fwd_x - 1.0f) < 1e-4f, "Forward X match");
    TEST_CHECK(std::abs(fwd_y - 0.0f) < 1e-4f, "Forward Y match");
    TEST_CHECK(std::abs(fwd_z - 0.0f) < 1e-4f, "Forward Z match");

    // Optical up [0, -1, 0] -> Body up [0, 0, 1]
    float up_x = R[0]*0.0f + R[1]*(-1.0f) + R[2]*0.0f;
    float up_y = R[3]*0.0f + R[4]*(-1.0f) + R[5]*0.0f;
    float up_z = R[6]*0.0f + R[7]*(-1.0f) + R[8]*0.0f;

    TEST_CHECK(std::abs(up_x - 0.0f) < 1e-4f, "Up X match");
    TEST_CHECK(std::abs(up_y - 0.0f) < 1e-4f, "Up Y match");
    TEST_CHECK(std::abs(up_z - 1.0f) < 1e-4f, "Up Z match");

    TEST_CHECK(std::abs(t[0] - 0.05f) < 1e-4f, "Translation X match");
    TEST_CHECK(std::abs(t[2] - 0.15f) < 1e-4f, "Translation Z match");

    std::cout << "    [PASS] Level optical-to-body extrinsics verified." << std::endl;
}

static void test_camera_alignment_gravity_leveling() {
    std::cout << "[2] Verifying dynamic pitch leveling via CoreMotion gravity vector..." << std::endl;
    CameraAlignmentParams params;
    params.mount_height_m = 0.12f;
    CameraAlignment align(params);

    float pitch = 20.0f * (3.14159265f / 180.0f);
    float g[3] = {0.0f, 9.81f * std::cos(pitch), 9.81f * std::sin(pitch)};

    PointCloud cam_cloud;
    cam_cloud.push_back(0.0f, 0.0f, 1.0f); // 1m along optical axis
    PointCloud body_cloud;
    align.transform_to_body(cam_cloud, g, body_cloud);

    TEST_CHECK(body_cloud.size() == 1, "Body cloud size");
    float expected_z = 0.12f - 1.0f * std::sin(pitch);
    TEST_CHECK(std::abs(body_cloud.z[0] - expected_z) < 1e-3f, "Pitch leveled elevation");

    std::cout << "    [PASS] Dynamic pitch leveling verified." << std::endl;
}

static void test_camera_alignment_plane_model() {
    std::cout << "[3] Verifying camera alignment via optical PlaneModel..." << std::endl;
    CameraAlignmentParams params;
    params.camera_pitch_deg = 15.0f;
    CameraAlignment align(params);

    // Ground plane in camera optical frame:
    // Camera is mounted at height h = 0.15m and tilted down 15 degrees.
    // Ground normal in camera coordinates:
    // When tilted down, ground normal tilts backwards in camera frame:
    // UP has negative Y and negative Z in camera frame.
    float pitch = 15.0f * (3.14159265f / 180.0f);
    // Let normal point UP towards camera: [0, -cos(pitch), -sin(pitch)]
    // Point on ground directly below camera: [0, h / cos(pitch), 0] in camera coords...
    // Or more directly: plane equation a*x + b*y + c*z + d = 0.
    // Origin (camera) is at distance d = 0.15m from plane along normal.
    PlaneModel ground_plane;
    ground_plane.a = 0.0f;
    ground_plane.b = std::cos(pitch);   // Pointing downward initially (will be flipped to UP by align)
    ground_plane.c = std::sin(pitch);
    ground_plane.d = -0.15f;            // ax + by + cz - 0.15 = 0

    PointCloud cam_cloud;
    // Generate synthetic ground points on this plane at various depths z = 0.5 to 3.0m
    for (int i = 0; i < 20; ++i) {
        float cz = 0.5f + i * 0.1f;
        float cx = (i % 5 - 2) * 0.1f;
        // cz*sin(pitch) + cy*cos(pitch) - 0.15 = 0 => cy = (0.15 - cz*sin(pitch)) / cos(pitch)
        float cy = (0.15f - cz * std::sin(pitch)) / std::cos(pitch);
        cam_cloud.push_back(cx, cy, cz);
    }
    // Add 5 obstacle points hovering 0.10m above the ground
    for (int i = 0; i < 5; ++i) {
        float cz = 1.0f + i * 0.2f;
        float cx = 0.0f;
        // In optical frame, "above ground" (towards sky) means lower y value
        float cy_ground = (0.15f - cz * std::sin(pitch)) / std::cos(pitch);
        float cy_obs = cy_ground - 0.10f / std::cos(pitch); // 10cm above ground
        cam_cloud.push_back(cx, cy_obs, cz);
    }

    PointCloud body_cloud;
    align.transform_to_body(cam_cloud, ground_plane, body_cloud);

    TEST_CHECK(body_cloud.size() == 25, "Transformed cloud size");

    // Check that all 20 ground points have Z_body ~ 0.0
    for (size_t i = 0; i < 20; ++i) {
        TEST_CHECK(std::abs(body_cloud.z[i]) < 1e-3f, "Ground point at Z ~ 0");
    }

    // Check that obstacle points have Z_body ~ 0.10m
    for (size_t i = 20; i < 25; ++i) {
        TEST_CHECK(std::abs(body_cloud.z[i] - 0.10f) < 5e-3f, "Obstacle point at Z ~ 0.10m");
    }

    // Also test hybrid: compute_extrinsics with gravity + plane
    float g[3] = {0.0f, 9.81f * std::cos(pitch), 9.81f * std::sin(pitch)};
    PointCloud hybrid_body_cloud;
    align.transform_to_body(cam_cloud, g, ground_plane, hybrid_body_cloud);
    for (size_t i = 0; i < 20; ++i) {
        TEST_CHECK(std::abs(hybrid_body_cloud.z[i]) < 1e-3f, "Hybrid ground point at Z ~ 0");
    }

    std::cout << "    [PASS] Optical PlaneModel alignment verified (ground leveled to Z=0, height calibrated)." << std::endl;
}

static void test_camera_alignment_gravity_plane_disagreement() {
    std::cout << "[4] Verifying handling when gravity and plane disagree..." << std::endl;
    CameraAlignmentParams params;
    params.mount_height_m = 0.12f;
    params.max_ground_angle_deg = 15.0f;
    CameraAlignment align(params);

    // Gravity vector: nominal pitch 10 degrees down
    float pitch_rad = 10.0f * (3.14159265f / 180.0f);
    float g[3] = {0.0f, 9.81f * std::cos(pitch_rad), 9.81f * std::sin(pitch_rad)};

    // Case A: Ground plane tilted by 5 degrees relative to gravity horizon (e.g. 5 deg ramp)
    // Within 15 deg threshold -> ACCEPTED!
    float ramp_pitch = pitch_rad + 5.0f * (3.14159265f / 180.0f);
    PlaneModel ramp_plane;
    ramp_plane.a = 0.0f;
    ramp_plane.b = std::cos(ramp_pitch);
    ramp_plane.c = std::sin(ramp_pitch);
    ramp_plane.d = -0.15f; // perpendicular distance is 0.15m

    float h_vert = 0.0f;
    bool accepted = align.is_ground_plane_valid(g, ramp_plane, &h_vert);
    TEST_CHECK(accepted, "5 degree ramp plane accepted");
    // Vertical height should be d / cos(5 deg) = 0.15 / 0.99619 = 0.15057m
    float expected_h_vert = 0.15f / std::cos(5.0f * (3.14159265f / 180.0f));
    TEST_CHECK(std::abs(h_vert - expected_h_vert) < 1e-3f, "Vertical height along gravity axis match");

    float R[9], t[3];
    align.compute_extrinsics(g, ramp_plane, R, t);
    TEST_CHECK(std::abs(t[2] - expected_h_vert) < 1e-3f, "Extrinsics t_z calibrated to vertical height");

    // Case B: Wall plane with normal perpendicular to gravity (90 degrees away from UP)
    // Exceeds 15 deg threshold -> REJECTED!
    PlaneModel wall_plane;
    wall_plane.a = 1.0f; // Wall normal in X direction
    wall_plane.b = 0.0f;
    wall_plane.c = 0.0f;
    wall_plane.d = -1.50f;

    accepted = align.is_ground_plane_valid(g, wall_plane, &h_vert);
    TEST_CHECK(!accepted, "Vertical wall plane rejected as non-ground");

    align.compute_extrinsics(g, wall_plane, R, t);
    // When rejected, t[2] MUST safely fall back to nominal mount_height_m (0.12m)
    TEST_CHECK(std::abs(t[2] - params.mount_height_m) < 1e-4f, "Wall fallback to nominal mount height");

    // Verify Z-axis ALWAYS aligns with gravity (R[6], R[7], R[8] == -g / ||g||)
    float g_norm = std::sqrt(g[0]*g[0] + g[1]*g[1] + g[2]*g[2]);
    TEST_CHECK(std::abs(R[6] - (-g[0] / g_norm)) < 1e-4f, "Z-axis strictly aligns with gravity X");
    TEST_CHECK(std::abs(R[7] - (-g[1] / g_norm)) < 1e-4f, "Z-axis strictly aligns with gravity Y");
    TEST_CHECK(std::abs(R[8] - (-g[2] / g_norm)) < 1e-4f, "Z-axis strictly aligns with gravity Z");

    std::cout << "    [PASS] Gravity-gated ground validation verified (ramps accepted, walls rejected, Z-axis always aligned to gravity)." << std::endl;
}

static void test_passthrough_elevation_slicing() {
    std::cout << "[3] Verifying PassThroughFilter ground & ceiling extraction..." << std::endl;
    PassThroughFilter filter(0.03f, 0.60f); // Ground is Z < 0.03m, Ceiling is Z > 0.60m
    filter.set_limits_x(0.10f, 4.00f);
    filter.set_limits_y(-2.00f, 2.00f);

    PointCloud cloud;
    // 100 ground points (Z = 0)
    for (int i = 0; i < 100; ++i) {
        cloud.push_back(0.5f + (i % 10) * 0.2f, -0.5f + (i / 10) * 0.1f, 0.0f);
    }
    // 25 obstacle points (Z = 0.15m)
    for (int i = 0; i < 25; ++i) {
        cloud.push_back(1.0f + (i % 5) * 0.05f, -0.2f + (i / 5) * 0.1f, 0.15f);
    }
    // 15 ceiling points (Z = 1.5m)
    for (int i = 0; i < 15; ++i) {
        cloud.push_back(1.0f, 0.0f, 1.5f);
    }

    PointCloud obstacles;
    PointCloud non_obstacles;
    filter.filter(cloud, obstacles, &non_obstacles);

    TEST_CHECK(obstacles.size() == 25, "Obstacles count");
    TEST_CHECK(non_obstacles.size() == 115, "Rejected (ground + ceiling) count");

    for (size_t i = 0; i < obstacles.size(); ++i) {
        TEST_CHECK(obstacles.z[i] >= 0.03f && obstacles.z[i] <= 0.60f, "Obstacle in elevation bounds");
    }

    std::cout << "    [PASS] Elevation slicing correctly isolated obstacles from ground and ceiling." << std::endl;
}

static void test_pipeline_alignment_and_passthrough_real_pcd() {
    std::cout << "[4] Testing CameraAlignment + PassThroughFilter pipeline on real PCD..." << std::endl;
    PointCloud raw_cloud;
    std::string path = "data/pcd_compressed/0000000000.pcd";
    if (!loadPCD(path, raw_cloud)) {
        path = "data/0000000000.pcd";
        if (!loadPCD(path, raw_cloud)) {
            std::cout << "    [SKIP] Sample PCD file not found." << std::endl;
            return;
        }
    }

    // Convert raw LiDAR points into camera optical frame
    // KITTI: X fwd, Y left, Z up (-1.73m is road)
    // Camera: X right (-Y), Y down (-Z), Z forward (X)
    PointCloud cam_cloud;
    cam_cloud.resize(raw_cloud.size());
    for (size_t i = 0; i < raw_cloud.size(); ++i) {
        cam_cloud.x[i] = -raw_cloud.y[i];
        cam_cloud.y[i] = -raw_cloud.z[i];
        cam_cloud.z[i] = raw_cloud.x[i];
    }

    // Stage 1: Camera Alignment
    CameraAlignmentParams align_params;
    align_params.mount_height_m = 1.73f;
    CameraAlignment align(align_params);

    float g[3] = {0.0f, 9.81f, 0.0f};
    PointCloud body_cloud;
    align.transform_to_body(cam_cloud, g, body_cloud);
    TEST_CHECK(body_cloud.size() == raw_cloud.size(), "Body cloud size match");

    // Stage 2: PassThroughFilter (Removes ground below 0.15m and ceiling above 2.5m)
    PassThroughFilter passthrough(0.15f, 2.50f);
    passthrough.set_limits_x(2.0f, 25.0f);
    passthrough.set_limits_y(-4.0f, 4.0f);

    PointCloud obstacles;
    PointCloud ground_and_ceiling;
    passthrough.filter(body_cloud, obstacles, &ground_and_ceiling);

    TEST_CHECK(!obstacles.empty(), "Obstacles not empty");
    TEST_CHECK(obstacles.size() > 500, "Corridor obstacles isolated");
    TEST_CHECK(ground_and_ceiling.size() > obstacles.size(), "Ground and background extracted");

    std::cout << "    [PASS] Real PCD processed: " << raw_cloud.size() << " raw points -> "
              << obstacles.size() << " driving-corridor obstacle points isolated, "
              << ground_and_ceiling.size() << " ground/ceiling points removed." << std::endl;
}

int main() {
    std::cout << "============================================================" << std::endl;
    std::cout << " Running test_camera_alignment (Camera Alignment & PassThrough)" << std::endl;
    std::cout << "============================================================" << std::endl;

    test_camera_alignment_level();
    test_camera_alignment_gravity_leveling();
    test_camera_alignment_plane_model();
    test_camera_alignment_gravity_plane_disagreement();
    test_passthrough_elevation_slicing();
    test_pipeline_alignment_and_passthrough_real_pcd();

    std::cout << "============================================================" << std::endl;
    std::cout << " test_camera_alignment PASSED ALL CHECKS" << std::endl;
    std::cout << "============================================================" << std::endl;
    return 0;
}
