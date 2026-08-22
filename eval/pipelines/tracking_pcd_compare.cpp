// tracking_pcd_compare.cpp
//
// End-to-End Multi-Frame PCD Comparison Pipeline
// ===============================================
// Compares Full Pipeline execution vs. Tracking Mode across a sequence
// of sequential PCD frames.
//
// Zero-Disk-I/O Compute Timing:
//   - All input PCD files are loaded into RAM before execution timers start.
//   - PCD saving is executed separately outside the compute timing intervals.

#include "include/rvpoint.h"

#include <algorithm>
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

using namespace rvpoint;

namespace {

void runFullPipelineExport(const std::vector<PointXYZ>& input,
                           std::vector<PointXYZ>& ground_removed_out,
                           double& out_compute_ms) {
    auto t0 = std::chrono::high_resolution_clock::now();

    std::size_t n = input.size();
    if (n == 0) {
        ground_removed_out.clear();
        out_compute_ms = 0;
        return;
    }

    std::vector<float> rx(n), ry(n), rz(n);
    for (std::size_t i = 0; i < n; ++i) {
        rx[i] = input[i].x;
        ry[i] = input[i].y;
        rz[i] = input[i].z;
    }
    PointCloudSoA raw_soa = {rx.data(), ry.data(), rz.data(), n};

    // 1. Voxel downsampling (0.10m leaf)
    std::vector<PointXYZ> downsampled(n);
    std::size_t n_down = voxel_grid_downsamp_rvv_v2(raw_soa, downsampled.data(), 0.10f);
    downsampled.resize(n_down);

    std::vector<float> dx(n_down), dy(n_down), dz(n_down);
    for (std::size_t i = 0; i < n_down; ++i) {
        dx[i] = downsampled[i].x;
        dy[i] = downsampled[i].y;
        dz[i] = downsampled[i].z;
    }
    PointCloudSoA ds_soa = {dx.data(), dy.data(), dz.data(), n_down};

    // 2. Statistical Outlier Removal
    std::vector<PointXYZ> sor_out(n_down);
    std::size_t n_sor = sor_rvv(ds_soa, sor_out.data(), 30, 2.0f);
    sor_out.resize(n_sor);

    std::vector<float> sx(n_sor), sy(n_sor), sz(n_sor);
    for (std::size_t i = 0; i < n_sor; ++i) {
        sx[i] = sor_out[i].x;
        sy[i] = sor_out[i].y;
        sz[i] = sor_out[i].z;
    }
    PointCloudSoA sor_soa = {sx.data(), sy.data(), sz.data(), n_sor};

    // 3. RANSAC Ground Plane Detection
    float plane_model[4];
    ransac_plane_rvv(sor_soa, 0.15f, 200, plane_model);

    // Extract non-ground points (outliers to the ground plane)
    std::vector<PointXYZ> non_ground(n_sor);
    std::size_t n_non_ground = extract_plane_outliers_rvv(sor_soa, plane_model, 0.15f, non_ground.data());
    non_ground.resize(n_non_ground);
    ground_removed_out = std::move(non_ground);

    auto t1 = std::chrono::high_resolution_clock::now();
    out_compute_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
}

void runTrackingModeExport(const std::vector<std::vector<PointXYZ>>& all_frames,
                           int keyframe_interval,
                           const std::filesystem::path& output_dir,
                           std::vector<double>& out_tracking_compute_ms) {
    int n_frames = (int)all_frames.size();
    out_tracking_compute_ms.resize(n_frames, 0.0);

    TrackingConfig config;
    config.voxel_leaf_size = 0.10f;
    config.icp_max_iterations = 15;
    config.correspondence_max_dist = 0.5f;
    config.plane_verify_dist = 0.15f;
    config.plane_verify_ratio = 0.4f;
    config.cluster_tolerance = 0.15f;
    config.normal_radius = 0.2f;
    config.normal_k = 15;
    config.spatial_hash_cell_size = 0.2f;

    TrackingPipeline pipeline;
    bool has_keyframe = false;

    for (int f = 0; f < n_frames; ++f) {
        std::ostringstream oss;
        oss << std::setw(10) << std::setfill('0') << f;
        std::string fname = oss.str() + ".pcd";
        std::filesystem::path out_pcd_path = output_dir / fname;

        bool is_keyframe = (f % keyframe_interval == 0) || !has_keyframe;

        if (is_keyframe) {
            auto t0 = std::chrono::high_resolution_clock::now();
            pipeline.initializeKeyframe(all_frames[f].data(), all_frames[f].size(), config);
            auto t1 = std::chrono::high_resolution_clock::now();
            double compute_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

            out_tracking_compute_ms[f] = compute_ms;
            has_keyframe = true;

            // Save non-ground points from keyframe
            std::vector<PointXYZ> ground_removed;
            double dummy_ms;
            runFullPipelineExport(all_frames[f], ground_removed, dummy_ms);
            savePCD(out_pcd_path.string(), ground_removed, true);

            if (f % 10 == 0 || f == n_frames - 1) {
                std::cout << "  Frame " << std::setw(4) << f << " (KEYFRAME): "
                          << std::fixed << std::setprecision(1) << compute_ms << " ms" << std::endl;
            }
        } else {
            auto t0 = std::chrono::high_resolution_clock::now();
            TrackingResult result = pipeline.processTrackingFrame(
                all_frames[f].data(), all_frames[f].size(), config);
            auto t1 = std::chrono::high_resolution_clock::now();
            double compute_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

            out_tracking_compute_ms[f] = compute_ms;

            // Output residual points
            savePCD(out_pcd_path.string(), result.residual_points, true);

            if (f % 10 == 0 || f == n_frames - 1) {
                std::cout << "  Frame " << std::setw(4) << f << " (tracking): "
                          << std::fixed << std::setprecision(1) << compute_ms << " ms, "
                          << "confirmed=" << result.n_confirmed
                          << ", residual=" << result.n_residual << std::endl;
            }
        }
    }
}

void saveComparisonJSON(const std::filesystem::path& path,
                        const std::vector<double>& full_ms,
                        const std::vector<double>& tracking_ms,
                        int n_frames, int keyframe_interval) {
    std::ofstream ofs(path);
    if (!ofs.is_open()) return;

    double total_full = 0, total_tracking = 0;
    for (int i = 0; i < n_frames; ++i) {
        total_full += full_ms[i];
        total_tracking += tracking_ms[i];
    }

    ofs << "{\n";
    ofs << "  \"comparison\": \"full_pipeline_vs_tracking_mode\",\n";
    ofs << "  \"n_frames\": " << n_frames << ",\n";
    ofs << "  \"keyframe_interval\": " << keyframe_interval << ",\n";
    ofs << "  \"total_full_pipeline_ms\": " << std::fixed << std::setprecision(2) << total_full << ",\n";
    ofs << "  \"total_tracking_mode_ms\": " << total_tracking << ",\n";
    ofs << "  \"overall_speedup\": " << (total_full / std::max(total_tracking, 0.001)) << ",\n";
    ofs << "  \"frames\": [\n";

    for (int i = 0; i < n_frames; ++i) {
        ofs << "    {\"frame\": " << i
            << ", \"full_ms\": " << full_ms[i]
            << ", \"tracking_ms\": " << tracking_ms[i]
            << ", \"speedup\": " << (full_ms[i] / std::max(tracking_ms[i], 0.001))
            << "}" << (i + 1 < n_frames ? "," : "") << "\n";
    }

    ofs << "  ]\n";
    ofs << "}\n";
}

} // anonymous namespace

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <pcd_dir> <output_dir> <max_frames> [keyframe_interval]" << std::endl;
        std::cerr << std::endl;
        std::cerr << "  pcd_dir           Directory containing sequential PCD files" << std::endl;
        std::cerr << "  output_dir        Output directory for processed PCDs & JSON" << std::endl;
        std::cerr << "  max_frames        Number of frames to process (0 = all)" << std::endl;
        std::cerr << "  keyframe_interval Tracking mode keyframe interval (default: 5)" << std::endl;
        std::cerr << std::endl;
        std::cerr << "Example:" << std::endl;
        std::cerr << "  " << argv[0] << " data/pcd_compressed output/pcd_compare 20 5" << std::endl;
        return 1;
    }

    std::filesystem::path pcd_dir = argv[1];
    std::filesystem::path output_dir = argv[2];
    int max_frames = std::atoi(argv[3]);
    int keyframe_interval = (argc >= 5) ? std::atoi(argv[4]) : 5;

    if (keyframe_interval < 1) keyframe_interval = 1;

    std::cout << "================================================================" << std::endl;
    std::cout << "   RVPoint — Tracking Mode Multi-Frame PCD Comparison" << std::endl;
    std::cout << "================================================================" << std::endl;
    std::cout << "  PCD directory:      " << pcd_dir.string() << std::endl;
    std::cout << "  Output directory:   " << output_dir.string() << std::endl;
    std::cout << "  Max frames:         " << max_frames << std::endl;
    std::cout << "  Keyframe interval:  " << keyframe_interval << std::endl;
    std::cout << "================================================================" << std::endl;

    std::vector<std::filesystem::path> pcd_files;
    for (const auto& entry : std::filesystem::directory_iterator(pcd_dir)) {
        if (entry.path().extension() == ".pcd") {
            pcd_files.push_back(entry.path());
        }
    }
    std::sort(pcd_files.begin(), pcd_files.end());

    if (pcd_files.empty()) {
        std::cerr << "Error: No PCD files found in " << pcd_dir.string() << std::endl;
        return 1;
    }

    int n_available = (int)pcd_files.size();
    int n_frames = (max_frames > 0 && max_frames < n_available) ? max_frames : n_available;
    pcd_files.resize(n_frames);

    std::cout << "\nFound " << n_available << " PCD files, processing " << n_frames << " frames.\n" << std::endl;

    // Load all frames into RAM before timing
    std::cout << "Pre-loading all frames into RAM..." << std::endl;
    std::vector<std::vector<PointXYZ>> all_frames(n_frames);
    for (int f = 0; f < n_frames; ++f) {
        if (loadPCD(pcd_files[f].string(), all_frames[f]) < 0) {
            std::cerr << "Failed to load: " << pcd_files[f].string() << std::endl;
            return 1;
        }
    }
    std::cout << "All " << n_frames << " frames loaded. Starting benchmarks.\n" << std::endl;

    std::filesystem::path full_dir = output_dir / "full_pipeline";
    std::filesystem::path track_dir = output_dir / "tracking_mode";
    std::filesystem::create_directories(full_dir);
    std::filesystem::create_directories(track_dir);

    // Mode A: Full Pipeline
    std::cout << "=== Mode A: Full Pipeline (per-frame baseline) ===" << std::endl;
    std::vector<double> full_pipeline_ms(n_frames, 0.0);

    for (int f = 0; f < n_frames; ++f) {
        std::ostringstream oss;
        oss << std::setw(10) << std::setfill('0') << f;
        std::string fname = oss.str() + ".pcd";

        std::vector<PointXYZ> ground_removed;
        double ms = 0;
        runFullPipelineExport(all_frames[f], ground_removed, ms);
        full_pipeline_ms[f] = ms;

        savePCD((full_dir / fname).string(), ground_removed, true);

        if (f % 10 == 0 || f == n_frames - 1) {
            std::cout << "  Frame " << std::setw(4) << f << ": " << std::fixed
                      << std::setprecision(1) << ms << " ms, "
                      << ground_removed.size() << " output points" << std::endl;
        }
    }

    double total_full = std::accumulate(full_pipeline_ms.begin(), full_pipeline_ms.end(), 0.0);
    std::cout << "  Total full pipeline: " << std::fixed << std::setprecision(1)
              << total_full << " ms\n" << std::endl;

    // Mode B: Tracking Mode
    std::cout << "=== Mode B: Tracking Mode (keyframe every "
              << keyframe_interval << " frames) ===" << std::endl;

    std::vector<double> tracking_ms;
    runTrackingModeExport(all_frames, keyframe_interval, track_dir, tracking_ms);

    double total_tracking = std::accumulate(tracking_ms.begin(), tracking_ms.end(), 0.0);
    std::cout << "  Total tracking mode: " << std::fixed << std::setprecision(1)
              << total_tracking << " ms\n" << std::endl;

    // Summary
    double overall_speedup = total_full / std::max(total_tracking, 0.001);

    std::cout << "================================================================" << std::endl;
    std::cout << "                      COMPARISON SUMMARY" << std::endl;
    std::cout << "================================================================" << std::endl;
    std::cout << "  Frames processed:       " << n_frames << std::endl;
    std::cout << "  Keyframe interval:      " << keyframe_interval << std::endl;
    std::cout << "  Total full pipeline:    " << std::fixed << std::setprecision(1)
              << total_full << " ms" << std::endl;
    std::cout << "  Total tracking mode:    " << total_tracking << " ms" << std::endl;
    std::cout << "  Overall compute speedup:" << std::setprecision(2)
              << overall_speedup << "x" << std::endl;
    std::cout << std::endl;
    std::cout << "  Output directories:" << std::endl;
    std::cout << "    Full pipeline PCDs: " << full_dir.string() << std::endl;
    std::cout << "    Tracking mode PCDs: " << track_dir.string() << std::endl;
    std::cout << "================================================================" << std::endl;

    saveComparisonJSON(output_dir / "comparison_results.json",
                       full_pipeline_ms, tracking_ms, n_frames, keyframe_interval);
    std::cout << "\nJSON results saved to: " << (output_dir / "comparison_results.json").string() << std::endl;

    return 0;
}
