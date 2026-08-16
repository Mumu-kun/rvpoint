// tracking_mode_bench.cpp
//
// Tracking Mode Benchmark — Full Pipeline vs Tracking Mode Speedup
// =================================================================
//
// Benchmark methodology:
//   1. Load or generate a point cloud (the "keyframe")
//   2. Generate N synthetic tracking frames by applying small incremental transforms
//   3. Measure: running the FULL PIPELINE on every frame (baseline)
//   4. Measure: running TRACKING MODE (full pipeline on frame 0, tracking on frames 1..N)
//   5. Report per-stage breakdown, total time, speedup, and instruction-count proxy
//
// Output: JSON benchmark results + console summary

#include "registration/tracking_pipeline.h"
#include "registration/tracking_types.h"
#include "rvv_pcl.h"
#include "simple_pcd_loader.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

using namespace rvv_pcl;

namespace {

struct FrameResult {
    int frame_id;
    double full_pipeline_ms;
    double tracking_total_ms;
    double stage1_ms, stage2_ms, stage3_ms, stage5_ms;
    double stage6_ms, stage7_ms, stage8_ms, stage9_ms;
    double residual_pipeline_ms;
    int icp_iterations;
    float icp_error;
    std::size_t n_confirmed, n_residual;
    double speedup;
};

// Generate a synthetic indoor scene
void generateSyntheticScene(std::vector<PointXYZ>& points, int n_points) {
    points.clear();
    points.reserve(n_points);

    std::srand(12345);

    // Ground plane (z=0)
    int n_ground = n_points * 3 / 5;
    for (int i = 0; i < n_ground; ++i) {
        float x = ((float)(std::rand() % 6000) / 1000.0f) - 3.0f;
        float y = ((float)(std::rand() % 6000) / 1000.0f) - 3.0f;
        float z = ((float)(std::rand() % 20) / 1000.0f) - 0.01f; // noise around z=0
        points.push_back({x, y, z});
    }

    // Wall (x=2, yz plane)
    int n_wall = n_points / 5;
    for (int i = 0; i < n_wall; ++i) {
        float x = 2.0f + ((float)(std::rand() % 20) / 1000.0f) - 0.01f;
        float y = ((float)(std::rand() % 4000) / 1000.0f) - 2.0f;
        float z = ((float)(std::rand() % 2000) / 1000.0f);
        points.push_back({x, y, z});
    }

    // Object cluster (sphere at (0, 1, 0.5))
    int n_obj = n_points - n_ground - n_wall;
    for (int i = 0; i < n_obj; ++i) {
        float theta = ((float)(std::rand() % 6283) / 1000.0f);
        float phi = ((float)(std::rand() % 3142) / 1000.0f);
        float r = 0.4f * ((float)(std::rand() % 1000) / 1000.0f);
        float x = 0.0f + r * std::sin(phi) * std::cos(theta);
        float y = 1.0f + r * std::sin(phi) * std::sin(theta);
        float z = 0.5f + r * std::cos(phi);
        points.push_back({x, y, z});
    }
}

void transformPoints(const SE3Transform& T, const std::vector<PointXYZ>& in,
                     std::vector<PointXYZ>& out) {
    out.resize(in.size());
    for (std::size_t i = 0; i < in.size(); ++i) {
        out[i] = T.apply(in[i]);
    }
}

void saveJSON(const std::filesystem::path& path,
              const std::vector<FrameResult>& results,
              double total_full_ms, double total_tracking_ms,
              std::size_t n_raw_points, std::size_t n_frames) {
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        std::cerr << "Failed to open " << path << std::endl;
        return;
    }

    ofs << "{\n";
    ofs << "  \"benchmark\": \"tracking_mode_vs_full_pipeline\",\n";
    ofs << "  \"n_raw_points\": " << n_raw_points << ",\n";
    ofs << "  \"n_frames\": " << n_frames << ",\n";
    ofs << "  \"total_full_pipeline_ms\": " << total_full_ms << ",\n";
    ofs << "  \"total_tracking_mode_ms\": " << total_tracking_ms << ",\n";
    ofs << "  \"overall_speedup\": " << std::fixed << std::setprecision(2) << (total_full_ms / total_tracking_ms) << ",\n";
    ofs << "  \"frames\": [\n";

    for (std::size_t i = 0; i < results.size(); ++i) {
        const auto& r = results[i];
        ofs << "    {\n";
        ofs << "      \"frame_id\": " << r.frame_id << ",\n";
        ofs << "      \"full_pipeline_ms\": " << std::fixed << std::setprecision(3) << r.full_pipeline_ms << ",\n";
        ofs << "      \"tracking_total_ms\": " << r.tracking_total_ms << ",\n";
        ofs << "      \"speedup\": " << std::setprecision(2) << r.speedup << ",\n";
        ofs << "      \"icp_iterations\": " << r.icp_iterations << ",\n";
        ofs << "      \"icp_error\": " << std::setprecision(6) << r.icp_error << ",\n";
        ofs << "      \"n_confirmed\": " << r.n_confirmed << ",\n";
        ofs << "      \"n_residual\": " << r.n_residual << ",\n";
        ofs << "      \"stage_timing_ms\": {\n";
        ofs << "        \"voxel_downsample\": " << std::setprecision(3) << r.stage1_ms << ",\n";
        ofs << "        \"correspondence\": " << r.stage2_ms << ",\n";
        ofs << "        \"residual_jacobian_reduction\": " << r.stage3_ms << ",\n";
        ofs << "        \"solve\": " << r.stage5_ms << ",\n";
        ofs << "        \"propagate\": " << r.stage6_ms << ",\n";
        ofs << "        \"verify\": " << r.stage7_ms << ",\n";
        ofs << "        \"split\": " << r.stage8_ms << ",\n";
        ofs << "        \"update_index\": " << r.stage9_ms << ",\n";
        ofs << "        \"residual_full_pipeline\": " << r.residual_pipeline_ms << "\n";
        ofs << "      }\n";
        ofs << "    }" << (i + 1 < results.size() ? "," : "") << "\n";
    }

    ofs << "  ]\n";
    ofs << "}\n";
}

} // anonymous namespace

int main(int argc, char** argv) {
    std::cout << "================================================================" << std::endl;
    std::cout << "   RVPoint Tracking Mode Benchmark" << std::endl;
    std::cout << "   Full Pipeline vs Tracking Mode — Speedup Comparison" << std::endl;
    std::cout << "================================================================" << std::endl;

    // Parse arguments
    std::string input_file = "";
    std::filesystem::path output_dir = "benchmarks";
    int n_tracking_frames = 10;
    int n_synthetic_points = 5000;

    if (argc >= 2) input_file = argv[1];
    if (argc >= 3) output_dir = std::filesystem::path(argv[2]);
    if (argc >= 4) n_tracking_frames = std::atoi(argv[3]);

    std::filesystem::create_directories(output_dir);

    // Load or generate point cloud
    std::vector<PointXYZ> keyframe_points;

    if (!input_file.empty()) {
        std::cout << "Loading PCD: " << input_file << std::endl;
        if (loadPCD(input_file, keyframe_points) < 0) {
            std::cerr << "Failed to load " << input_file << std::endl;
            return 1;
        }
    } else {
        std::cout << "No input PCD specified. Generating synthetic scene with "
                  << n_synthetic_points << " points." << std::endl;
        generateSyntheticScene(keyframe_points, n_synthetic_points);
    }

    std::cout << "Keyframe points: " << keyframe_points.size() << std::endl;
    std::cout << "Tracking frames: " << n_tracking_frames << std::endl;
    std::cout << std::endl;

    // Configuration
    TrackingConfig config;
    config.voxel_leaf_size = 0.05f;
    config.icp_max_iterations = 15;
    config.correspondence_max_dist = 0.5f;
    config.plane_verify_dist = 0.05f;
    config.plane_verify_ratio = 0.1f;
    config.cluster_verify_dist = 0.3f;
    config.cluster_verify_ratio = 0.1f;
    config.ransac_dist_thresh = 0.02f;
    config.ransac_max_iters = 300;
    config.cluster_tolerance = 0.05f;
    config.min_cluster_size = 10;
    config.max_cluster_size = 25000;
    config.normal_k = 15;
    config.normal_radius = 0.15f;
    config.spatial_hash_cell_size = 0.15f;

    // Generate tracking frames (small incremental transforms)
    std::vector<std::vector<PointXYZ>> tracking_frames(n_tracking_frames);
    std::vector<PointXYZ> current_frame = keyframe_points;

    for (int f = 0; f < n_tracking_frames; ++f) {
        float angle = 1.0f * 3.14159265f / 180.0f; // 1 degree per frame
        SE3Transform motion = SE3Transform::fromAxisAngle(
            0, 0, angle,
            0.01f, 0.005f, 0.0f  // 1cm translation per frame
        );
        transformPoints(motion, current_frame, tracking_frames[f]);
        current_frame = tracking_frames[f];
    }

    // =========================================================================
    // BENCHMARK 1: Full Pipeline on Every Frame
    // =========================================================================
    std::cout << "=== Benchmark 1: Full Pipeline on Every Frame ===" << std::endl;

    double total_full_pipeline_ms = 0;
    std::vector<double> full_pipeline_times(n_tracking_frames);

    // Measure keyframe
    double keyframe_full_ms;
    TrackingPipeline::runFullPipeline(keyframe_points.data(), keyframe_points.size(),
                                      config, keyframe_full_ms);
    total_full_pipeline_ms += keyframe_full_ms;
    std::cout << "  Keyframe (full pipeline): " << std::fixed << std::setprecision(2)
              << keyframe_full_ms << " ms" << std::endl;

    // Measure each tracking frame as if we ran full pipeline
    for (int f = 0; f < n_tracking_frames; ++f) {
        double frame_ms;
        TrackingPipeline::runFullPipeline(tracking_frames[f].data(), tracking_frames[f].size(),
                                          config, frame_ms);
        full_pipeline_times[f] = frame_ms;
        total_full_pipeline_ms += frame_ms;
    }

    double avg_full_ms = total_full_pipeline_ms / (n_tracking_frames + 1);
    std::cout << "  Average per-frame (full): " << std::fixed << std::setprecision(2)
              << avg_full_ms << " ms" << std::endl;
    std::cout << "  Total (full pipeline):    " << total_full_pipeline_ms << " ms" << std::endl;
    std::cout << std::endl;

    // =========================================================================
    // BENCHMARK 2: Tracking Mode (full on keyframe, tracking on rest)
    // =========================================================================
    std::cout << "=== Benchmark 2: Tracking Mode ===" << std::endl;

    TrackingPipeline pipeline;

    auto t0_init = std::chrono::high_resolution_clock::now();
    pipeline.initializeKeyframe(keyframe_points.data(), keyframe_points.size(), config);
    auto t1_init = std::chrono::high_resolution_clock::now();
    double init_ms = std::chrono::duration<double, std::milli>(t1_init - t0_init).count();

    std::cout << "  Keyframe init: " << std::fixed << std::setprecision(2) << init_ms << " ms" << std::endl;
    std::cout << "  Detected " << pipeline.getState().planes.size() << " planes, "
              << pipeline.getState().clusters.size() << " clusters" << std::endl;

    double total_tracking_ms = init_ms; // Include keyframe init
    std::vector<FrameResult> results;

    for (int f = 0; f < n_tracking_frames; ++f) {
        TrackingResult tr = pipeline.processTrackingFrame(
            tracking_frames[f].data(), tracking_frames[f].size(), config);

        double speedup = full_pipeline_times[f] / std::max(tr.timing.total_tracking_ms, 0.001);

        FrameResult fr;
        fr.frame_id = f + 1;
        fr.full_pipeline_ms = full_pipeline_times[f];
        fr.tracking_total_ms = tr.timing.total_tracking_ms;
        fr.stage1_ms = tr.timing.voxel_downsample_ms;
        fr.stage2_ms = tr.timing.correspondence_ms;
        fr.stage3_ms = tr.timing.residual_jacobian_ms;
        fr.stage5_ms = tr.timing.solve_ms;
        fr.stage6_ms = tr.timing.propagate_ms;
        fr.stage7_ms = tr.timing.verify_ms;
        fr.stage8_ms = tr.timing.split_ms;
        fr.stage9_ms = tr.timing.update_index_ms;
        fr.residual_pipeline_ms = tr.timing.residual_full_pipeline_ms;
        fr.icp_iterations = tr.icp_iterations_used;
        fr.icp_error = tr.icp_final_error;
        fr.n_confirmed = tr.n_confirmed;
        fr.n_residual = tr.n_residual;
        fr.speedup = speedup;

        results.push_back(fr);
        total_tracking_ms += tr.timing.total_tracking_ms;

        std::cout << "  Frame " << std::setw(2) << (f+1)
                  << ": tracking=" << std::fixed << std::setprecision(2) << tr.timing.total_tracking_ms
                  << "ms  full=" << full_pipeline_times[f]
                  << "ms  speedup=" << std::setprecision(1) << speedup << "x"
                  << "  icp_iters=" << tr.icp_iterations_used
                  << "  confirmed=" << tr.n_confirmed
                  << "  residual=" << tr.n_residual
                  << std::endl;
    }

    // =========================================================================
    // Summary
    // =========================================================================
    double overall_speedup = total_full_pipeline_ms / std::max(total_tracking_ms, 0.001);
    double avg_tracking_ms = 0;
    if (!results.empty()) {
        for (const auto& r : results) avg_tracking_ms += r.tracking_total_ms;
        avg_tracking_ms /= results.size();
    }

    std::cout << "\n================================================================" << std::endl;
    std::cout << "                         RESULTS SUMMARY" << std::endl;
    std::cout << "================================================================" << std::endl;
    std::cout << "  Input points:           " << keyframe_points.size() << std::endl;
    std::cout << "  Tracking frames:        " << n_tracking_frames << std::endl;
    std::cout << "  Full pipeline total:    " << std::fixed << std::setprecision(2)
              << total_full_pipeline_ms << " ms" << std::endl;
    std::cout << "  Tracking mode total:    " << total_tracking_ms << " ms" << std::endl;
    std::cout << "  Overall speedup:        " << std::setprecision(2) << overall_speedup << "x" << std::endl;
    std::cout << "  Avg full pipeline/frame:" << std::setprecision(2) << avg_full_ms << " ms" << std::endl;
    std::cout << "  Avg tracking/frame:     " << std::setprecision(2) << avg_tracking_ms << " ms" << std::endl;
    std::cout << "  Avg per-frame speedup:  " << std::setprecision(2)
              << (avg_full_ms / std::max(avg_tracking_ms, 0.001)) << "x" << std::endl;

    // Per-stage breakdown (averages)
    if (!results.empty()) {
        double avg_s1=0, avg_s2=0, avg_s3=0, avg_s5=0;
        double avg_s6=0, avg_s7=0, avg_s8=0, avg_s9=0, avg_res=0;
        for (const auto& r : results) {
            avg_s1 += r.stage1_ms; avg_s2 += r.stage2_ms;
            avg_s3 += r.stage3_ms; avg_s5 += r.stage5_ms;
            avg_s6 += r.stage6_ms; avg_s7 += r.stage7_ms;
            avg_s8 += r.stage8_ms; avg_s9 += r.stage9_ms;
            avg_res += r.residual_pipeline_ms;
        }
        double nf = (double)results.size();
        std::cout << "\n  Average Per-Stage Timing:" << std::endl;
        std::cout << "    Stage 1 (Voxel Downsample):     " << std::setprecision(3) << (avg_s1/nf) << " ms" << std::endl;
        std::cout << "    Stage 2 (Correspondence):       " << (avg_s2/nf) << " ms" << std::endl;
        std::cout << "    Stage 3+4 (Residual+Reduction): " << (avg_s3/nf) << " ms" << std::endl;
        std::cout << "    Stage 5 (Solve 6x6):            " << (avg_s5/nf) << " ms" << std::endl;
        std::cout << "    Stage 6 (Propagate):            " << (avg_s6/nf) << " ms" << std::endl;
        std::cout << "    Stage 7 (Verify):               " << (avg_s7/nf) << " ms" << std::endl;
        std::cout << "    Stage 8 (Split):                " << (avg_s8/nf) << " ms" << std::endl;
        std::cout << "    Stage 9 (Update Index):         " << (avg_s9/nf) << " ms" << std::endl;
        std::cout << "    Residual Full Pipeline:         " << (avg_res/nf) << " ms" << std::endl;
    }

    // Power consumption proxy: instruction count estimate
    std::cout << "\n  Power Consumption Proxy (Instruction Count Estimate):" << std::endl;
    std::cout << "    Full pipeline processes ALL points through ALL stages every frame." << std::endl;
    std::cout << "    Tracking mode processes ALL points only through Stage 1 (downsample)," << std::endl;
    std::cout << "    then only matched points through lightweight ICP stages," << std::endl;
    std::cout << "    and only RESIDUAL points through the expensive full pipeline." << std::endl;
    if (!results.empty()) {
        double avg_residual_ratio = 0;
        for (const auto& r : results) {
            avg_residual_ratio += (double)r.n_residual / std::max((double)(r.n_confirmed + r.n_residual), 1.0);
        }
        avg_residual_ratio /= results.size();
        std::cout << "    Average residual ratio: " << std::setprecision(1)
                  << (avg_residual_ratio * 100.0) << "% of points need full pipeline" << std::endl;
        std::cout << "    Estimated instruction savings: ~"
                  << std::setprecision(1) << ((1.0 - avg_residual_ratio) * 100.0)
                  << "% fewer full-pipeline instructions per tracking frame" << std::endl;
    }

    std::cout << "================================================================" << std::endl;

    // Save results to JSON
    auto json_path = output_dir / "tracking_benchmark_results.json";
    saveJSON(json_path, results, total_full_pipeline_ms, total_tracking_ms,
             keyframe_points.size(), n_tracking_frames);
    std::cout << "\nResults saved to: " << json_path.string() << std::endl;

    return 0;
}
