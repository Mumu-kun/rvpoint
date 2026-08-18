// tracking_pcd_compare.cpp
//
// Tracking Mode PCD Comparison Tool
// ===================================
// Processes sequential PCD frames from a directory through two modes:
//   Mode A — Full Pipeline:    Each frame processed independently
//   Mode B — Tracking Mode:    Full pipeline on keyframes, tracking on the rest
//
// Outputs processed (ground-removed) PCD files to separate directories
// so they can be visually compared in a 3D viewer.
//
// Usage:
//   tracking_pcd_compare <pcd_dir> <output_dir> <max_frames> [keyframe_interval]

#include "registration/tracking_pipeline.h"
#include "registration/tracking_types.h"
#include "rvv_pcl.h"
#include "simple_pcd_loader.h"
#include "pointer_octree/pointer_octree.h"

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

using namespace rvv_pcl;

namespace {

// ─── Full pipeline for a single frame (ground-removal only, no clustering) ───
// Returns the ground-removed (RANSAC outlier) points.
void runFullPipelineExport(const std::vector<PointXYZ>& raw_points,
                           std::vector<PointXYZ>& out_ground_removed,
                           double& out_time_ms) {
    auto t0 = std::chrono::high_resolution_clock::now();
    std::size_t n = raw_points.size();

    // AoS → SoA
    std::vector<float> rx(n), ry(n), rz(n);
    for (std::size_t i = 0; i < n; ++i) {
        rx[i] = raw_points[i].x;
        ry[i] = raw_points[i].y;
        rz[i] = raw_points[i].z;
    }
    PointCloudSoA raw_soa = {rx.data(), ry.data(), rz.data(), n};

    // Stage 1: Voxel downsample
    std::vector<PointXYZ> downsampled(n);
    std::size_t n_down = voxel_grid_downsamp_rvv_v2(raw_soa, downsampled.data(), 0.01f);
    downsampled.resize(n_down);

    // Convert to SoA
    std::vector<float> dx(n_down), dy(n_down), dz(n_down);
    for (std::size_t i = 0; i < n_down; ++i) {
        dx[i] = downsampled[i].x;
        dy[i] = downsampled[i].y;
        dz[i] = downsampled[i].z;
    }
    PointCloudSoA ds_soa = {dx.data(), dy.data(), dz.data(), n_down};

    // Stage 2: SOR (using PointerOctree-accelerated variant)
    PointerOctree sor_tree;
    sor_tree.setInputCloud(ds_soa);
    sor_tree.build();
    std::vector<PointXYZ> sor_pts(n_down);
    std::size_t n_sor = sor_pointer_octree(ds_soa, sor_tree, sor_pts.data(), 20, 1.0f);
    sor_pts.resize(n_sor);

    // Convert to SoA
    std::vector<float> sx(n_sor), sy(n_sor), sz(n_sor);
    for (std::size_t i = 0; i < n_sor; ++i) {
        sx[i] = sor_pts[i].x;
        sy[i] = sor_pts[i].y;
        sz[i] = sor_pts[i].z;
    }
    PointCloudSoA sor_soa = {sx.data(), sy.data(), sz.data(), n_sor};

    // Stage 3: Normal estimation
    Octree search;
    search.setInputCloud(sor_soa);
    search.build();
    std::vector<float> nx(n_sor), ny(n_sor), nz(n_sor);
    normal_estimation_rvv(sor_soa, search, nx.data(), ny.data(), nz.data(), 10, 0.03f);

    // Stage 4: RANSAC plane fitting
    float model[4] = {};
    int ransac_inliers = ransac_plane_rvv(sor_soa, 0.2f, 1000, model);

    // Stage 5: Extract inliers/outliers
    if (ransac_inliers > 0) {
        std::vector<PointXYZ> inliers(n_sor), outliers(n_sor);
        std::size_t n_in = 0, n_out = 0;
        extract_plane_inliers_outliers_rvv(sor_soa, model, 0.2f,
                                           inliers.data(), outliers.data(), n_in, n_out);
        outliers.resize(n_out);
        out_ground_removed = std::move(outliers);
    } else {
        // RANSAC failed — return SOR-filtered cloud as fallback
        out_ground_removed = sor_pts;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    out_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
}

// ─── Tracking mode: full pipeline on keyframes, tracking on others ───────────
// For tracking frames, the output is confirmed + residual points
// (residual points go through RANSAC again internally).
// For keyframes, the output is the full-pipeline ground-removed cloud.
void runTrackingModeExport(const std::vector<std::vector<PointXYZ>>& all_frames,
                           int keyframe_interval,
                           const std::filesystem::path& output_dir,
                           std::vector<double>& per_frame_ms) {
    TrackingConfig config;
    config.voxel_leaf_size = 0.01f;
    config.icp_max_iterations = 15;
    config.correspondence_max_dist = 0.5f;
    config.plane_verify_dist = 0.05f;
    config.plane_verify_ratio = 0.1f;
    config.cluster_verify_dist = 0.3f;
    config.cluster_verify_ratio = 0.1f;
    config.ransac_dist_thresh = 0.2f;
    config.ransac_max_iters = 1000;
    config.cluster_tolerance = 0.15f;
    config.min_cluster_size = 50;
    config.max_cluster_size = 100000;
    config.normal_k = 10;
    config.normal_radius = 0.03f;
    config.spatial_hash_cell_size = 0.15f;

    TrackingPipeline pipeline;
    int n_frames = (int)all_frames.size();
    per_frame_ms.resize(n_frames, 0.0);

    for (int f = 0; f < n_frames; ++f) {
        std::ostringstream oss;
        oss << std::setw(10) << std::setfill('0') << f;
        std::string fname = oss.str() + ".pcd";

        bool is_keyframe = (f % keyframe_interval == 0);

        if (is_keyframe) {
            // Run full pipeline independently
            std::vector<PointXYZ> ground_removed;
            double ms = 0;
            runFullPipelineExport(all_frames[f], ground_removed, ms);
            per_frame_ms[f] = ms;

            savePCD((output_dir / fname).string(), ground_removed, true);

            // Re-initialize tracking pipeline from this keyframe
            auto t0 = std::chrono::high_resolution_clock::now();
            pipeline.initializeKeyframe(all_frames[f].data(), all_frames[f].size(), config);
            auto t1 = std::chrono::high_resolution_clock::now();
            // Add keyframe init time to this frame's total
            per_frame_ms[f] += std::chrono::duration<double, std::milli>(t1 - t0).count();

            std::cout << "  [KEYFRAME] Frame " << std::setw(4) << f
                      << ": full pipeline + tracking init (" << std::fixed
                      << std::setprecision(1) << per_frame_ms[f] << " ms), "
                      << ground_removed.size() << " output points" << std::endl;
        } else {
            // Run tracking mode
            auto t0 = std::chrono::high_resolution_clock::now();
            TrackingResult tr = pipeline.processTrackingFrame(
                all_frames[f].data(), all_frames[f].size(), config);
            auto t1 = std::chrono::high_resolution_clock::now();
            per_frame_ms[f] = std::chrono::duration<double, std::milli>(t1 - t0).count();

            // Combine confirmed + residual points for output
            // The confirmed points are the ones that matched existing models
            // The residual points were processed through RANSAC internally
            // We output everything that is NOT the ground plane
            // For simplicity, we take all downsampled frame points that were NOT
            // confirmed as plane inliers — those are in residual_points
            // plus non-plane confirmed points.
            // Since the tracking pipeline already splits points, we can output
            // the residual points (which went through RANSAC again) as a proxy.

            // Output: residual points = points not matched to known ground plane
            // This is the tracking mode's equivalent of ground-removed output
            std::vector<PointXYZ> output_pts;

            // residual_points from tracking already had RANSAC run on them
            // We want to output everything except the ground plane
            // The confirmed clusters + residual outliers = ground-removed cloud
            output_pts = tr.residual_points;

            savePCD((output_dir / fname).string(), output_pts, true);

            std::cout << "  [TRACKING] Frame " << std::setw(4) << f
                      << ": tracking mode (" << std::fixed
                      << std::setprecision(1) << per_frame_ms[f] << " ms), "
                      << "confirmed=" << tr.n_confirmed
                      << " residual=" << tr.n_residual
                      << " icp_iters=" << tr.icp_iterations_used
                      << " output=" << output_pts.size() << " pts" << std::endl;
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
        std::cerr << "  output_dir        Output directory for processed PCDs" << std::endl;
        std::cerr << "  max_frames        Number of frames to process (0 = all)" << std::endl;
        std::cerr << "  keyframe_interval Tracking mode keyframe interval (default: 5)" << std::endl;
        std::cerr << std::endl;
        std::cerr << "Example:" << std::endl;
        std::cerr << "  " << argv[0] << " data/pcd_compressed data/pcd_processed 20 5" << std::endl;
        return 1;
    }

    std::filesystem::path pcd_dir = argv[1];
    std::filesystem::path output_dir = argv[2];
    int max_frames = std::atoi(argv[3]);
    int keyframe_interval = (argc >= 5) ? std::atoi(argv[4]) : 5;

    if (keyframe_interval < 1) keyframe_interval = 1;

    std::cout << "================================================================" << std::endl;
    std::cout << "   RVPoint — Tracking Mode PCD Comparison" << std::endl;
    std::cout << "================================================================" << std::endl;
    std::cout << "  PCD directory:      " << pcd_dir.string() << std::endl;
    std::cout << "  Output directory:   " << output_dir.string() << std::endl;
    std::cout << "  Max frames:         " << max_frames << std::endl;
    std::cout << "  Keyframe interval:  " << keyframe_interval << std::endl;
    std::cout << "================================================================" << std::endl;

    // ── Discover and sort PCD files ──
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

    // ── Load all frames into memory ──
    std::cout << "Loading frames..." << std::endl;
    std::vector<std::vector<PointXYZ>> all_frames(n_frames);
    for (int f = 0; f < n_frames; ++f) {
        if (loadPCD(pcd_files[f].string(), all_frames[f]) < 0) {
            std::cerr << "Failed to load: " << pcd_files[f].string() << std::endl;
            return 1;
        }
        if (f % 20 == 0 || f == n_frames - 1) {
            std::cout << "  Loaded frame " << f << "/" << n_frames
                      << " (" << all_frames[f].size() << " points)" << std::endl;
        }
    }

    // ── Create output directories ──
    std::filesystem::path full_dir = output_dir / "full_pipeline";
    std::filesystem::path track_dir = output_dir / "tracking_mode";
    std::filesystem::create_directories(full_dir);
    std::filesystem::create_directories(track_dir);

    // ======================================================================
    // MODE A: Full Pipeline on every frame
    // ======================================================================
    std::cout << "\n=== Mode A: Full Pipeline (every frame independently) ===" << std::endl;

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
              << total_full << " ms" << std::endl;

    // ======================================================================
    // MODE B: Tracking Mode
    // ======================================================================
    std::cout << "\n=== Mode B: Tracking Mode (keyframe every "
              << keyframe_interval << " frames) ===" << std::endl;

    std::vector<double> tracking_ms;
    runTrackingModeExport(all_frames, keyframe_interval, track_dir, tracking_ms);

    double total_tracking = std::accumulate(tracking_ms.begin(), tracking_ms.end(), 0.0);
    std::cout << "  Total tracking mode: " << std::fixed << std::setprecision(1)
              << total_tracking << " ms" << std::endl;

    // ======================================================================
    // Summary
    // ======================================================================
    double overall_speedup = total_full / std::max(total_tracking, 0.001);

    std::cout << "\n================================================================" << std::endl;
    std::cout << "                      COMPARISON SUMMARY" << std::endl;
    std::cout << "================================================================" << std::endl;
    std::cout << "  Frames processed:       " << n_frames << std::endl;
    std::cout << "  Keyframe interval:      " << keyframe_interval << std::endl;
    std::cout << "  Total full pipeline:    " << std::fixed << std::setprecision(1)
              << total_full << " ms" << std::endl;
    std::cout << "  Total tracking mode:    " << total_tracking << " ms" << std::endl;
    std::cout << "  Overall speedup:        " << std::setprecision(2)
              << overall_speedup << "x" << std::endl;
    std::cout << std::endl;
    std::cout << "  Output directories:" << std::endl;
    std::cout << "    Full pipeline: " << full_dir.string() << std::endl;
    std::cout << "    Tracking mode: " << track_dir.string() << std::endl;
    std::cout << "================================================================" << std::endl;

    // Save JSON summary
    saveComparisonJSON(output_dir / "comparison_results.json",
                       full_pipeline_ms, tracking_ms, n_frames, keyframe_interval);
    std::cout << "\nJSON results: " << (output_dir / "comparison_results.json").string() << std::endl;

    return 0;
}
