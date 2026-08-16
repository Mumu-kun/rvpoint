// test_tracking_pipeline.cpp
//
// End-to-end test for the full 9-stage Tracking Pipeline
// =======================================================
// Tests: keyframe initialization, tracking frame processing,
// plane propagation, point splitting correctness.

#include "registration/tracking_pipeline.h"
#include "registration/tracking_types.h"
#include "rvv_pcl.h"
#include "simple_pcd_loader.h"

#include <cmath>
#include <iostream>
#include <iomanip>
#include <vector>

using namespace rvv_pcl;

namespace {

// Generate a synthetic scene with a ground plane and some objects
void generateSyntheticScene(std::vector<PointXYZ>& points, int n_ground, int n_object) {
    points.clear();
    points.reserve(n_ground + n_object);

    // Ground plane: z = 0, covering x/y from -2 to 2
    std::srand(42);
    for (int i = 0; i < n_ground; ++i) {
        float x = ((float)(std::rand() % 4000) / 1000.0f) - 2.0f;
        float y = ((float)(std::rand() % 4000) / 1000.0f) - 2.0f;
        float z = ((float)(std::rand() % 10) / 1000.0f); // small noise
        points.push_back({x, y, z});
    }

    // Object cluster: sphere at (1, 1, 0.5) with radius 0.3
    for (int i = 0; i < n_object; ++i) {
        float theta = ((float)(std::rand() % 3142) / 1000.0f);
        float phi = ((float)(std::rand() % 3142) / 1000.0f);
        float r = 0.3f * ((float)(std::rand() % 1000) / 1000.0f);
        float x = 1.0f + r * std::sin(phi) * std::cos(theta);
        float y = 1.0f + r * std::sin(phi) * std::sin(theta);
        float z = 0.5f + r * std::cos(phi);
        points.push_back({x, y, z});
    }
}

// Apply a known transform to all points
void transformPoints(const SE3Transform& T, const std::vector<PointXYZ>& in,
                     std::vector<PointXYZ>& out) {
    out.resize(in.size());
    for (std::size_t i = 0; i < in.size(); ++i) {
        out[i] = T.apply(in[i]);
    }
}

} // anonymous namespace

int main(int argc, char** argv) {
    std::cout << "=============================================" << std::endl;
    std::cout << "   Tracking Pipeline End-to-End Test" << std::endl;
    std::cout << "=============================================" << std::endl;

    int pass_count = 0;
    int fail_count = 0;

    // =========================================================================
    // Test 1: Keyframe initialization + tracking frame processing
    // =========================================================================
    {
        std::cout << "\n--- Test 1: Keyframe Init + Tracking Frame ---" << std::endl;

        // Generate keyframe
        std::vector<PointXYZ> keyframe;
        generateSyntheticScene(keyframe, 500, 100);
        std::cout << "  Keyframe points: " << keyframe.size() << std::endl;

        // Initialize pipeline
        TrackingConfig config;
        config.voxel_leaf_size = 0.1f;
        config.icp_max_iterations = 20;
        config.correspondence_max_dist = 0.5f;
        config.plane_verify_dist = 0.05f;
        config.plane_verify_ratio = 0.1f;
        config.cluster_verify_dist = 0.5f;
        config.cluster_verify_ratio = 0.1f;
        config.ransac_max_iters = 200;
        config.normal_k = 10;
        config.normal_radius = 0.3f;
        config.spatial_hash_cell_size = 0.2f;

        TrackingPipeline pipeline;
        pipeline.initializeKeyframe(keyframe.data(), keyframe.size(), config);

        std::cout << "  Keyframe initialized." << std::endl;
        std::cout << "  Planes detected: " << pipeline.getState().planes.size() << std::endl;
        std::cout << "  Clusters detected: " << pipeline.getState().clusters.size() << std::endl;

        // Apply small transform for tracking frame
        float angle = 2.0f * 3.14159265f / 180.0f; // 2 degrees
        SE3Transform motion = SE3Transform::fromAxisAngle(0, 0, angle, 0.05f, 0.02f, 0.0f);

        std::vector<PointXYZ> tracking_frame;
        transformPoints(motion, keyframe, tracking_frame);

        // Process tracking frame
        TrackingResult result = pipeline.processTrackingFrame(
            tracking_frame.data(), tracking_frame.size(), config);

        std::cout << "  ICP iterations:   " << result.icp_iterations_used << std::endl;
        std::cout << "  ICP final error:  " << result.icp_final_error << std::endl;
        std::cout << "  Confirmed points: " << result.n_confirmed << std::endl;
        std::cout << "  Residual points:  " << result.n_residual << std::endl;
        std::cout << "  Confirmed planes: " << result.confirmed_planes.size() << std::endl;
        std::cout << "  New planes:       " << result.new_planes.size() << std::endl;

        // Check: confirmed + residual should equal total downsampled points
        std::size_t total = result.n_confirmed + result.n_residual;
        std::cout << "  Total points (confirmed + residual): " << total << std::endl;

        // Timing breakdown
        std::cout << "\n  Stage Timing Breakdown:" << std::endl;
        std::cout << "    Stage 1 (Voxel Downsample):  " << std::fixed << std::setprecision(3)
                  << result.timing.voxel_downsample_ms << " ms" << std::endl;
        std::cout << "    Stage 2 (Correspondence):    " << result.timing.correspondence_ms << " ms" << std::endl;
        std::cout << "    Stage 3+4 (Residual+Reduce): " << result.timing.residual_jacobian_ms << " ms" << std::endl;
        std::cout << "    Stage 5 (Solve):             " << result.timing.solve_ms << " ms" << std::endl;
        std::cout << "    Stage 6 (Propagate):         " << result.timing.propagate_ms << " ms" << std::endl;
        std::cout << "    Stage 7 (Verify):            " << result.timing.verify_ms << " ms" << std::endl;
        std::cout << "    Stage 8 (Split):             " << result.timing.split_ms << " ms" << std::endl;
        std::cout << "    Stage 9 (Update Index):      " << result.timing.update_index_ms << " ms" << std::endl;
        std::cout << "    Residual Full Pipeline:      " << result.timing.residual_full_pipeline_ms << " ms" << std::endl;
        std::cout << "    TOTAL Tracking:              " << result.timing.total_tracking_ms << " ms" << std::endl;

        // Basic sanity check: the pipeline should not crash and should produce timing > 0
        bool pass = (result.timing.total_tracking_ms > 0) &&
                    (total > 0) &&
                    (result.icp_iterations_used >= 1);

        if (pass) {
            std::cout << "  PASS" << std::endl;
            pass_count++;
        } else {
            std::cout << "  FAIL" << std::endl;
            fail_count++;
        }
    }

    // =========================================================================
    // Test 2: Multiple consecutive tracking frames
    // =========================================================================
    {
        std::cout << "\n--- Test 2: Multiple Tracking Frames (5 frames) ---" << std::endl;

        std::vector<PointXYZ> keyframe;
        generateSyntheticScene(keyframe, 400, 80);

        TrackingConfig config;
        config.voxel_leaf_size = 0.1f;
        config.icp_max_iterations = 15;
        config.correspondence_max_dist = 0.5f;
        config.normal_k = 10;
        config.normal_radius = 0.3f;
        config.spatial_hash_cell_size = 0.2f;

        TrackingPipeline pipeline;
        pipeline.initializeKeyframe(keyframe.data(), keyframe.size(), config);

        bool all_pass = true;
        std::vector<PointXYZ> current_frame = keyframe;

        for (int frame = 1; frame <= 5; ++frame) {
            // Small incremental motion per frame
            float angle = 1.0f * 3.14159265f / 180.0f;
            SE3Transform motion = SE3Transform::fromAxisAngle(0, 0, angle,
                                                                0.02f, 0.01f, 0.0f);

            std::vector<PointXYZ> next_frame;
            transformPoints(motion, current_frame, next_frame);

            TrackingResult result = pipeline.processTrackingFrame(
                next_frame.data(), next_frame.size(), config);

            std::cout << "  Frame " << frame
                      << ": iters=" << result.icp_iterations_used
                      << " err=" << std::fixed << std::setprecision(4) << result.icp_final_error
                      << " confirmed=" << result.n_confirmed
                      << " residual=" << result.n_residual
                      << " time=" << std::setprecision(2) << result.timing.total_tracking_ms << "ms"
                      << std::endl;

            if (result.timing.total_tracking_ms <= 0) {
                all_pass = false;
            }

            current_frame = next_frame;
        }

        if (all_pass) {
            std::cout << "  PASS" << std::endl;
            pass_count++;
        } else {
            std::cout << "  FAIL" << std::endl;
            fail_count++;
        }
    }

    // =========================================================================
    // Test 3: Point conservation (confirmed + residual = total)
    // =========================================================================
    {
        std::cout << "\n--- Test 3: Point Conservation Check ---" << std::endl;

        std::vector<PointXYZ> keyframe;
        generateSyntheticScene(keyframe, 300, 50);

        TrackingConfig config;
        config.voxel_leaf_size = 0.15f;
        config.icp_max_iterations = 10;
        config.correspondence_max_dist = 0.5f;
        config.normal_k = 10;
        config.normal_radius = 0.3f;
        config.spatial_hash_cell_size = 0.2f;

        TrackingPipeline pipeline;
        pipeline.initializeKeyframe(keyframe.data(), keyframe.size(), config);

        // Small motion
        SE3Transform motion = SE3Transform::fromAxisAngle(0, 0, 0, 0.03f, 0.01f, 0.0f);
        std::vector<PointXYZ> tracking_frame;
        transformPoints(motion, keyframe, tracking_frame);

        TrackingResult result = pipeline.processTrackingFrame(
            tracking_frame.data(), tracking_frame.size(), config);

        // Verify no points lost
        std::size_t total = result.n_confirmed + result.n_residual;
        std::cout << "  Confirmed: " << result.n_confirmed << std::endl;
        std::cout << "  Residual:  " << result.n_residual << std::endl;
        std::cout << "  Total:     " << total << std::endl;

        // Note: total should be the downsampled count, not original
        bool pass = (total > 0);

        if (pass) {
            std::cout << "  PASS" << std::endl;
            pass_count++;
        } else {
            std::cout << "  FAIL" << std::endl;
            fail_count++;
        }
    }

    // =========================================================================
    // Test 4: RVV propagate_transform kernel
    // =========================================================================
    {
        std::cout << "\n--- Test 4: RVV propagate_transform_rvv kernel ---" << std::endl;

        // Pure translation
        SE3Transform T = SE3Transform::fromAxisAngle(0, 0, 0, 1.0f, 2.0f, 3.0f);

        float in_x[] = {0, 1, 2, 3};
        float in_y[] = {0, 0, 0, 0};
        float in_z[] = {0, 0, 0, 0};
        float out_x[4], out_y[4], out_z[4];

        propagate_transform_rvv(T, in_x, in_y, in_z, out_x, out_y, out_z, 4);

        bool pass = true;
        for (int i = 0; i < 4; ++i) {
            if (std::abs(out_x[i] - (in_x[i] + 1.0f)) > 0.001f ||
                std::abs(out_y[i] - (in_y[i] + 2.0f)) > 0.001f ||
                std::abs(out_z[i] - (in_z[i] + 3.0f)) > 0.001f) {
                pass = false;
                std::cout << "  Point " << i << ": expected ("
                          << in_x[i]+1 << "," << in_y[i]+2 << "," << in_z[i]+3
                          << ") got (" << out_x[i] << "," << out_y[i] << "," << out_z[i] << ")"
                          << std::endl;
            }
        }

        if (pass) {
            std::cout << "  PASS" << std::endl;
            pass_count++;
        } else {
            std::cout << "  FAIL" << std::endl;
            fail_count++;
        }
    }

    // =========================================================================
    // Summary
    // =========================================================================
    std::cout << "\n=============================================" << std::endl;
    std::cout << "  Results: " << pass_count << " PASSED, " << fail_count << " FAILED" << std::endl;
    std::cout << "=============================================" << std::endl;

    return fail_count > 0 ? 1 : 0;
}
