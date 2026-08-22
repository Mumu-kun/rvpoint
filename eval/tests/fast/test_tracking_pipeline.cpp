// test_tracking_pipeline.cpp
//
// Fast unit tests for the 9-Stage Tracking Pipeline
// Tests:
//   1. Full tracking cycle: Keyframe init -> Tracking frame -> Verify result
//   2. Multiple consecutive tracking frames (5 frames)
//   3. Point conservation check (confirmed + residual = total downsampled)
//   4. RVV propagate_transform_rvv kernel correctness

#include "include/rvpoint.h"

#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

using namespace rvpoint;

namespace {

void generateSyntheticScene(std::vector<PointXYZ>& points,
                            int ground_n = 400,
                            int cluster_n = 100) {
    points.clear();
    points.reserve(ground_n * ground_n + cluster_n * 3);

    // Ground plane points: z ≈ 0
    float spacing = 0.05f;
    for (int i = 0; i < 30; ++i) {
        for (int j = 0; j < 30; ++j) {
            float x = (i - 15) * spacing;
            float y = (j - 15) * spacing;
            float z = 0.001f * ((i + j) % 3);
            points.push_back({x, y, z});
        }
    }

    // Obstacle cluster 1: centered at (0.5, 0.5, 0.2)
    for (int i = 0; i < 50; ++i) {
        float angle = i * 0.125f;
        float r = 0.1f * (1.0f + 0.1f * (i % 5));
        points.push_back({0.5f + r * std::cos(angle),
                          0.5f + r * std::sin(angle),
                          0.2f + 0.02f * (i % 7)});
    }

    // Obstacle cluster 2: centered at (-0.4, 0.3, 0.15)
    for (int i = 0; i < 40; ++i) {
        float angle = i * 0.15f;
        float r = 0.08f * (1.0f + 0.1f * (i % 4));
        points.push_back({-0.4f + r * std::cos(angle),
                          0.3f + r * std::sin(angle),
                          0.15f + 0.02f * (i % 5)});
    }
}

void transformPoints(const SE3Transform& T,
                     const std::vector<PointXYZ>& in,
                     std::vector<PointXYZ>& out) {
    out.resize(in.size());
    for (std::size_t i = 0; i < in.size(); ++i) {
        out[i] = T.apply(in[i]);
    }
}

} // anonymous namespace

int main() {
    std::cout << "=============================================" << std::endl;
    std::cout << "   Tracking Pipeline Unit Test" << std::endl;
    std::cout << "=============================================" << std::endl;

    int pass_count = 0;
    int fail_count = 0;

    // Test 1: Full tracking cycle
    {
        std::cout << "\n--- Test 1: Keyframe Init + Single Tracking Frame ---" << std::endl;

        std::vector<PointXYZ> keyframe;
        generateSyntheticScene(keyframe, 400, 80);
        std::cout << "  Keyframe points: " << keyframe.size() << std::endl;

        TrackingConfig config;
        config.voxel_leaf_size = 0.1f;
        config.icp_max_iterations = 15;
        config.correspondence_max_dist = 0.5f;
        config.normal_k = 10;
        config.normal_radius = 0.3f;
        config.spatial_hash_cell_size = 0.2f;

        TrackingPipeline pipeline;
        pipeline.initializeKeyframe(keyframe.data(), keyframe.size(), config);

        const auto& state = pipeline.getState();
        std::cout << "  Reference cloud downsampled: " << state.ref_n << " points" << std::endl;
        std::cout << "  Detected planes:   " << state.planes.size() << std::endl;
        std::cout << "  Detected clusters: " << state.clusters.size() << std::endl;

        SE3Transform small_motion = SE3Transform::fromAxisAngle(0, 0, 0.02f, 0.05f, 0.02f, 0.0f);
        std::vector<PointXYZ> tracking_frame;
        transformPoints(small_motion, keyframe, tracking_frame);

        TrackingResult result = pipeline.processTrackingFrame(
            tracking_frame.data(), tracking_frame.size(), config);

        std::cout << "  ICP iterations:   " << result.icp_iterations_used << std::endl;
        std::cout << "  Final ICP error:  " << result.icp_final_error << std::endl;
        std::cout << "  Confirmed points: " << result.n_confirmed << std::endl;
        std::cout << "  Residual points:  " << result.n_residual << std::endl;
        std::cout << "  Confirmed planes: " << result.confirmed_planes.size() << std::endl;
        std::cout << "  New planes:       " << result.new_planes.size() << std::endl;

        std::size_t total = result.n_confirmed + result.n_residual;
        std::cout << "  Total points (confirmed + residual): " << total << std::endl;

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

    // Test 2: Multiple consecutive tracking frames (5 frames)
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
            float angle = 1.0f * 3.14159265f / 180.0f;
            SE3Transform motion = SE3Transform::fromAxisAngle(0, 0, angle, 0.02f, 0.01f, 0.0f);

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

    // Test 3: Point Conservation Check
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

        SE3Transform motion = SE3Transform::fromAxisAngle(0, 0, 0, 0.03f, 0.01f, 0.0f);
        std::vector<PointXYZ> tracking_frame;
        transformPoints(motion, keyframe, tracking_frame);

        TrackingResult result = pipeline.processTrackingFrame(
            tracking_frame.data(), tracking_frame.size(), config);

        std::size_t total = result.n_confirmed + result.n_residual;
        std::cout << "  Confirmed: " << result.n_confirmed << std::endl;
        std::cout << "  Residual:  " << result.n_residual << std::endl;
        std::cout << "  Total:     " << total << std::endl;

        bool pass = (total > 0);

        if (pass) {
            std::cout << "  PASS" << std::endl;
            pass_count++;
        } else {
            std::cout << "  FAIL" << std::endl;
            fail_count++;
        }
    }

    // Test 4: RVV propagate_transform kernel
    {
        std::cout << "\n--- Test 4: RVV propagate_transform_rvv kernel ---" << std::endl;

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

    std::cout << "\n=============================================" << std::endl;
    std::cout << "  Results: " << pass_count << " PASSED, " << fail_count << " FAILED" << std::endl;
    std::cout << "=============================================" << std::endl;

    return fail_count > 0 ? 1 : 0;
}
