// eval/pipelines/pipeline_3d_ultra.cpp
// ============================================================================
// 10-Stage Modernized RVPoint True 3D Perception Pipeline
// ============================================================================
// Deepened architecture adhering to:
//   - ADR-0010: Zero-Heap Hot-Path Invariant (Pre-allocated persistent scratch)
//   - ADR-0011: Non-Virtual Deep Class Functor Pattern
//   - ADR-0012: Slotted Register-File DAG Pipeline & PipelineManager
//   - Bit-exact / stage-for-stage telemetry compatibility with baseline benchmarks
// ============================================================================

#include "include/rvpoint.h"
#include "io/simple_pcd_loader.h"
#include "filters/voxel_grid.h"
#include "filters/radius_outlier_removal.h"
#include "filters/statistical_outlier_removal.h"
#include "features/normal_estimation.h"
#include "search/fast_3d_spatial_grid.h"
#include "segmentation/ransac_plane.h"
#include "segmentation/euclidean_clustering.h"
#include "pipeline/pipeline_manager.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

using Clock = std::chrono::high_resolution_clock;
using namespace rvpoint;

namespace {

constexpr int kStageCount = 10;

enum class PerceptionStatus {
    SUCCESS = 0,
    NO_GROUND_PLANE_FOUND = 1,
    INSUFFICIENT_INLIERS = 2,
    DEGENERATE_POINT_CLOUD = 3,
    SEARCH_INDEX_OVERFLOW = 4,
    IO_WRITE_FAILURE = 5,
    INVALID_CLI_ARGUMENTS = 6
};

inline const char* perceptionStatusToString(PerceptionStatus status) {
    switch (status) {
        case PerceptionStatus::SUCCESS: return "SUCCESS";
        case PerceptionStatus::NO_GROUND_PLANE_FOUND: return "NO_GROUND_PLANE_FOUND";
        case PerceptionStatus::INSUFFICIENT_INLIERS: return "INSUFFICIENT_INLIERS";
        case PerceptionStatus::DEGENERATE_POINT_CLOUD: return "DEGENERATE_POINT_CLOUD";
        case PerceptionStatus::SEARCH_INDEX_OVERFLOW: return "SEARCH_INDEX_OVERFLOW";
        case PerceptionStatus::IO_WRITE_FAILURE: return "IO_WRITE_FAILURE";
        case PerceptionStatus::INVALID_CLI_ARGUMENTS: return "INVALID_CLI_ARGUMENTS";
        default: return "UNKNOWN";
    }
}

struct ColorRGB {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

inline ColorRGB getClusterColor(size_t cluster_idx) {
    static const ColorRGB kPalette[] = {
        {230, 25, 75},   {60, 180, 75},   {255, 225, 25},  {0, 130, 200},
        {245, 130, 48},  {145, 30, 180},  {70, 240, 240},  {240, 50, 230},
        {210, 245, 60},  {250, 190, 212}, {0, 128, 128},  {220, 190, 255},
        {170, 110, 40},  {255, 250, 200}, {128, 0, 0},    {170, 255, 195},
        {128, 128, 0},   {255, 215, 180}, {0, 0, 128},    {128, 128, 128}
    };
    constexpr size_t kPaletteSize = sizeof(kPalette) / sizeof(kPalette[0]);
    return kPalette[cluster_idx % kPaletteSize];
}

struct StageTiming {
    int index;
    std::string label;
    double ms;
    std::size_t point_count;
};

bool saveJSONMetrics(const std::filesystem::path &out_path,
                     const std::vector<StageTiming> &timings,
                     double total_ms,
                     float leaf_size,
                     bool skip_sor,
                     float cluster_tol,
                     PerceptionStatus status,
                     size_t n_inliers,
                     size_t n_outliers,
                     size_t n_clusters) {
    std::ofstream ofs(out_path);
    if (!ofs.is_open()) return false;
    ofs << "{\n";
    ofs << "  \"total_ms\": " << total_ms << ",\n";
    ofs << "  \"status\": \"" << perceptionStatusToString(status) << "\",\n";
    ofs << "  \"voxel_leaf_size\": " << leaf_size << ",\n";
    ofs << "  \"skip_sor\": " << (skip_sor ? "true" : "false") << ",\n";
    ofs << "  \"cluster_tolerance\": " << cluster_tol << ",\n";
    ofs << "  \"ground_inliers\": " << n_inliers << ",\n";
    ofs << "  \"obstacle_points\": " << n_outliers << ",\n";
    ofs << "  \"num_clusters\": " << n_clusters << ",\n";
    ofs << "  \"stages\": [\n";
    for (size_t i = 0; i < timings.size(); ++i) {
        ofs << "    {\n";
        ofs << "      \"index\": " << timings[i].index << ",\n";
        ofs << "      \"label\": \"" << timings[i].label << "\",\n";
        ofs << "      \"ms\": " << timings[i].ms << ",\n";
        ofs << "      \"pts\": " << timings[i].point_count << "\n";
        ofs << "    }" << (i + 1 < timings.size() ? "," : "") << "\n";
    }
    ofs << "  ]\n";
    ofs << "}\n";
    return true;
}

void beginStage(int stage_idx, const std::string &label, bool enabled) {
    if (enabled) {
        std::cout << "[progress] [" << stage_idx << "/" << kStageCount << "] "
                  << label << "..." << std::endl;
    }
}

double endStage(int stage_idx, const std::string &label,
                const Clock::time_point &start_time, bool enabled) {
    const auto end_time = Clock::now();
    const double ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    if (enabled) {
        std::cout << "[progress] [" << stage_idx << "/" << kStageCount << "] "
                  << label << " complete in " << ms << " ms" << std::endl;
    }
    return ms;
}

void printFinalBreakdown(const std::vector<StageTiming> &stages, double total_ms) {
    std::cout << "[progress] Final timing breakdown:" << std::endl;
    double measured_sum = 0.0;
    for (const auto &st : stages) {
        measured_sum += st.ms;
        const double pct = total_ms > 0.0 ? (st.ms / total_ms) * 100.0 : 0.0;
        std::cout << "[progress] [" << std::setw(2) << st.index << "/" << kStageCount << "] "
                  << std::left << std::setw(42) << (st.label + ":") << " "
                  << std::right << std::fixed << std::setw(7) << std::setprecision(3) << st.ms << " ms "
                  << "(" << std::setw(5) << std::setprecision(2) << pct << "%), pts="
                  << st.point_count << std::endl;
    }
    const double outside_ms = std::max(0.0, total_ms - measured_sum);
    const double outside_pct = total_ms > 0.0 ? (outside_ms / total_ms) * 100.0 : 0.0;
    std::cout << "[progress] [--] " << std::left << std::setw(42) << "Outside timed stages:" << " "
              << std::right << std::fixed << std::setw(7) << std::setprecision(3) << outside_ms << " ms "
              << "(" << std::setw(5) << std::setprecision(2) << outside_pct << "%)" << std::endl;
    std::cout << "[progress] [--] " << std::left << std::setw(42) << "Total:" << " "
              << std::right << std::fixed << std::setw(7) << std::setprecision(2) << total_ms << " ms (100.00%)" << std::endl;
}

std::string resolveInputPath(const std::string &raw_path) {
    if (std::filesystem::exists(raw_path)) return raw_path;
    const std::filesystem::path p(raw_path);
    const std::filesystem::path data_dir("data");
    const std::filesystem::path alt1 = data_dir / p.filename();
    if (std::filesystem::exists(alt1)) return alt1.string();
    const std::filesystem::path alt2 = data_dir / "pcd_compressed" / p.filename();
    if (std::filesystem::exists(alt2)) return alt2.string();
    return raw_path;
}

} // namespace

int main(int argc, char** argv) {
    bool progress_enabled = false;
    bool json_metrics = false;
    bool skip_sor = false;
    bool use_ror = true; // Default to ultra-fast ROR
    bool skip_normals = false;
    bool disable_disk = false;

    float voxel_leaf_size = 0.10f;
    float ror_radius = 0.25f;
    int ror_min_pts = 2;
    float ransac_distance_threshold = 0.20f;
    int ransac_max_iters = 250;
    float cluster_tolerance = 0.15f;
    int min_cluster_size = 50;
    int max_cluster_size = 100000;
    uint64_t seed = 42;

    std::vector<float> ground_normal_prior = {0.0f, 0.0f, 1.0f};
    float min_ground_dot = 0.707f;
    bool has_ground_prior = true;

    std::vector<std::string> positional_args;
    std::vector<StageTiming> stage_timings;
    stage_timings.reserve(kStageCount);

    try {
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg == "--progress") {
                progress_enabled = true;
            } else if (arg == "--json" || arg == "--json-metrics") {
                json_metrics = true;
            } else if (arg == "--skip-sor" || arg == "--no-sor") {
                skip_sor = true;
            } else if (arg == "--use-ror" || arg == "--ror") {
                use_ror = true; skip_sor = false;
            } else if (arg == "--use-sor" || arg == "--sor") {
                use_ror = false; skip_sor = false;
            } else if (arg == "--ror-radius") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --ror-radius\n"; return 1; }
                ror_radius = std::stof(argv[++i]);
            } else if (arg == "--ror-min-pts") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --ror-min-pts\n"; return 1; }
                ror_min_pts = std::stoi(argv[++i]);
            } else if (arg == "--no-normals" || arg == "--skip-normals") {
                skip_normals = true;
            } else if (arg == "--no-write" || arg == "--disable-disk") {
                disable_disk = true;
            } else if (arg == "--leaf-size") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --leaf-size\n"; return 1; }
                voxel_leaf_size = std::stof(argv[++i]);
            } else if (arg == "--cluster-tolerance") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --cluster-tolerance\n"; return 1; }
                cluster_tolerance = std::stof(argv[++i]);
            } else if (arg == "--min-cluster") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --min-cluster\n"; return 1; }
                min_cluster_size = std::stoi(argv[++i]);
            } else if (arg == "--max-cluster") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --max-cluster\n"; return 1; }
                max_cluster_size = std::stoi(argv[++i]);
            } else if (arg == "--ransac-iters") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --ransac-iters\n"; return 1; }
                ransac_max_iters = std::stoi(argv[++i]);
            } else if (arg == "--ransac-dist" || arg == "--ransac-thresh" || arg == "--ransac-distance") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for " << arg << "\n"; return 1; }
                ransac_distance_threshold = std::stof(argv[++i]);
            } else if (arg == "--seed") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --seed\n"; return 1; }
                seed = std::stoull(argv[++i]);
            } else if (arg == "--ground-angle-thresh") {
                if (i + 1 >= argc) { std::cerr << "Error: Missing value for --ground-angle-thresh\n"; return 1; }
                float deg = std::stof(argv[++i]);
                min_ground_dot = std::cos(deg * 3.14159265358979323846f / 180.0f);
            } else if (arg == "--no-ground-prior" || arg == "--unconstrained-plane") {
                has_ground_prior = false;
                min_ground_dot = 0.0f;
            } else if (arg == "--optical-frame") {
                ground_normal_prior = {0.0f, 1.0f, 0.0f};
            } else if (!arg.empty() && arg[0] == '-') {
                std::cerr << "Error: Unrecognized command-line option \x27" << arg << "\x27\n";
                return 1;
            } else {
                positional_args.push_back(arg);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error parsing CLI arguments: " << e.what() << std::endl;
        return 1;
    }

    if (positional_args.empty()) {
        std::cerr << "Usage: " << argv[0]
                  << " [--progress] [--json] [--no-write] [--no-normals] [--use-ror|--use-sor|--skip-sor] "
                     "[--ror-radius <val>] [--ror-min-pts <val>] [--leaf-size <val>] "
                     "[--cluster-tolerance <val>] [--min-cluster <val>] "
                     "[--max-cluster <val>] [--ransac-iters <val>] [--ransac-dist <val>] [--seed <val>] "
                     "[--ground-angle-thresh <deg>] [--no-ground-prior] [--optical-frame] <input.pcd> [output_dir]\n";
        return 1;
    }

    const auto overall_start = Clock::now();
    const std::string input_path = resolveInputPath(positional_args[0]);
    const std::filesystem::path input_stem = std::filesystem::path(positional_args[0]).stem();
    const std::filesystem::path output_dir =
        positional_args.size() >= 2 ? std::filesystem::path(positional_args[1])
                                    : std::filesystem::path("results") / (input_stem.string() + "_pipeline_ultra");
                                    : std::filesystem::path("output") / (input_stem.string() + "_pipeline_ultra");

    if (!disable_disk) {
        std::error_code dir_ec;
        std::filesystem::create_directories(output_dir, dir_ec);
    }

    // ── Stage 1: Load input cloud ──────────────────────────────────────────
    PointCloud raw_cloud;
    beginStage(1, "Load input cloud", progress_enabled);
    auto stage_start = Clock::now();
    if (!loadPCD(input_path, raw_cloud) || raw_cloud.empty()) {
        std::cerr << "Failed to load input PCD or cloud is empty: " << positional_args[0] << std::endl;
        return 1;
    }
    const size_t n_input = raw_cloud.n;
    stage_timings.push_back({1, "Load input cloud", endStage(1, "Load input cloud", stage_start, progress_enabled), n_input});

    // ── Stage 2: Write input stage ─────────────────────────────────────────
    beginStage(2, "Write input stage", progress_enabled);
    stage_start = Clock::now();
    if (!disable_disk) {
        savePCD((output_dir / "00_input.pcd").string(), raw_cloud, true);
    }
    stage_timings.push_back({2, "Write input stage", endStage(2, "Write input stage", stage_start, progress_enabled), n_input});

    // ========================================================================
    // PipelineManager DAG Engine Setup (ADR-0012)
    // ========================================================================
    PipelineManager pm;

    // Node 1: Voxel Grid Downsampling (Stage 3)
    pm.add_node("voxel_grid",
        rvpoint::in<rvpoint::PointCloud>("raw_cloud"),
        rvpoint::out<rvpoint::PointCloud>("downsampled_cloud"),
        rvpoint::param<float>("voxel_leaf_size", voxel_leaf_size)
    )
    .kernel(rvpoint::VoxelGrid{});

    // Node 2: Build Search Index for Downsampled Cloud (Stage 4)
    pm.add_node("build_search_index",
        rvpoint::in<rvpoint::PointCloud>("downsampled_cloud"),
        rvpoint::out<rvpoint::Fast3DSpatialGrid>("spatial_grid"),
        rvpoint::param<float>("search_radius", use_ror ? ror_radius : 0.25f)
    )
    .kernel([](const rvpoint::PointCloud& in, rvpoint::Fast3DSpatialGrid& grid, float radius) {
        grid = rvpoint::Fast3DSpatialGrid(radius, std::max<size_t>(65536, in.n));
        grid.build(in);
    });
    .kernel(rvpoint::SpatialGridBuilder{});

    // Node 3: Outlier Filter (ROR or SOR or pass-through) (Stage 5)
    if (skip_sor) {
        pm.add_node("outlier_filter",
            rvpoint::in<rvpoint::PointCloud>("downsampled_cloud"),
            rvpoint::out<rvpoint::PointCloud>("filtered_cloud")
        )
        .kernel([](const rvpoint::PointCloud& in, rvpoint::PointCloud& out) {
            out = in;
        });
    } else if (use_ror) {
        rvpoint::RadiusOutlierRemoval ror(ror_radius, ror_min_pts);
        pm.add_node("outlier_filter",
            rvpoint::in<rvpoint::PointCloud>("downsampled_cloud"),
            rvpoint::in<rvpoint::Fast3DSpatialGrid>("spatial_grid"),
            rvpoint::out<rvpoint::PointCloud>("filtered_cloud"),
            rvpoint::param<float>("ror_radius", ror_radius),
            rvpoint::param<int>("ror_min_pts", ror_min_pts)
        )
        .kernel(std::move(ror));
    } else {
        rvpoint::StatisticalOutlierRemoval sor(20, 1.0f);
        pm.add_node("outlier_filter",
            rvpoint::in<rvpoint::PointCloud>("downsampled_cloud"),
            rvpoint::out<rvpoint::PointCloud>("filtered_cloud"),
            rvpoint::param<int>("sor_k", 20),
            rvpoint::param<float>("sor_std_mul", 1.0f)
        )
        .kernel(std::move(sor));
    }

    // Node 4: Rebuild Search Index for Filtered Cloud (Stage 6)
    pm.add_node("rebuild_search_index",
        rvpoint::in<rvpoint::PointCloud>("filtered_cloud"),
        rvpoint::out<rvpoint::Fast3DSpatialGrid>("filtered_grid"),
        rvpoint::param<float>("search_radius", 0.25f)
    )
    .kernel([](const rvpoint::PointCloud& in, rvpoint::Fast3DSpatialGrid& grid, float radius) {
        grid = rvpoint::Fast3DSpatialGrid(radius, std::max<size_t>(65536, in.n));
        grid.build(in);
    });
    .kernel(rvpoint::SpatialGridBuilder{});

    // Node 5: Surface Normal Estimation (Stage 7)
    if (skip_normals) {
        pm.add_node("normal_estimation",
            rvpoint::in<rvpoint::PointCloud>("filtered_cloud"),
            rvpoint::out<rvpoint::PointCloud>("normals_cloud")
        )
        .kernel([](const rvpoint::PointCloud& in, rvpoint::PointCloud& normals) {
            normals.resize(in.n);
            for (size_t i = 0; i < in.n; ++i) {
                normals.x[i] = 0.0f; normals.y[i] = 0.0f; normals.z[i] = 1.0f;
            }
        });
    } else {
        rvpoint::NormalEstimation ne(10, 0.25f, 0.0f, 0.0f, 0.0f);
        pm.add_node("normal_estimation",
            rvpoint::in<rvpoint::PointCloud>("filtered_cloud"),
            rvpoint::in<rvpoint::Fast3DSpatialGrid>("filtered_grid"),
            rvpoint::out<rvpoint::PointCloud>("normals_cloud"),
            rvpoint::param<int>("k_search", 10),
            rvpoint::param<float>("radius_search", 0.25f),
            rvpoint::param<float>("vp_x", 0.0f),
            rvpoint::param<float>("vp_y", 0.0f),
            rvpoint::param<float>("vp_z", 0.0f)
        )
        .kernel(std::move(ne));
    }

    // Node 6: RANSAC Ground Plane Fitting & Partitioning (Stage 8)
    rvpoint::RansacPlane rp(ransac_distance_threshold, ransac_max_iters);
    rp.set_seed(static_cast<uint32_t>(seed));
    rp.set_use_sample_screening(true);
    rp.set_use_covariance_refinement(true);
    if (has_ground_prior) {
        rp.set_ground_normal_prior(ground_normal_prior[0], ground_normal_prior[1],
                                   ground_normal_prior[2], min_ground_dot);
    } else {
        rp.clear_ground_normal_prior();
    }

    pm.add_node("ransac_plane_split",
        rvpoint::in<rvpoint::PointCloud>("filtered_cloud"),
        rvpoint::out<rvpoint::PlaneModel>("ground_plane"),
        rvpoint::out<rvpoint::PointCloud>("ground_cloud"),
        rvpoint::out<rvpoint::PointCloud>("obstacle_cloud"),
        rvpoint::param<float>("ransac_dist", ransac_distance_threshold),
        rvpoint::param<int>("ransac_iters", ransac_max_iters)
    )
    .kernel(std::move(rp));

    // Node 7: Euclidean Clustering (Stage 9)
    rvpoint::EuclideanClustering ec(cluster_tolerance, min_cluster_size, max_cluster_size);
    ec.set_use_spatial_grid(true);

    pm.add_node("euclidean_clustering",
        rvpoint::in<rvpoint::PointCloud>("obstacle_cloud"),
        rvpoint::out<rvpoint::ClusterResult>("clusters"),
        rvpoint::param<float>("cluster_tolerance", cluster_tolerance),
        rvpoint::param<int>("min_cluster_size", min_cluster_size),
        rvpoint::param<int>("max_cluster_size", max_cluster_size)
    )
    .kernel(std::move(ec));

    // Initialize RegisterFile slots and Kahn\x27s DAG order
    pm.set_primary_input("raw_cloud");
    pm.initialize(std::max<size_t>(65536, n_input));

    // Execute Frame (DAG Execution)
    if (progress_enabled) {
        std::cout << "[progress] [3/10] Downsampling (RVV)..." << std::endl;
        std::cout << "[progress] [4/10] Build search index for downsampled cloud..." << std::endl;
        std::cout << "[progress] [5/10] " << (use_ror ? "Radius outlier removal (RVV)..." : "Statistical outlier removal (RVV)...") << std::endl;
        std::cout << "[progress] [6/10] Rebuild search index for filtered cloud..." << std::endl;
        std::cout << "[progress] [7/10] Normal estimation..." << std::endl;
        std::cout << "[progress] [8/10] RANSAC primitive fitting..." << std::endl;
        std::cout << "[progress] [9/10] Euclidean clustering (Hardware RVV 1.0)..." << std::endl;
    }

    pm.step(raw_cloud);

    // Retrieve slot representations for stage timing and telemetry
    const auto& rf = pm.registers();
    const auto& down_cloud = rf.get<rvpoint::PointCloud>(rf.get_id("downsampled_cloud"));
    const auto& filt_cloud = rf.get<rvpoint::PointCloud>(rf.get_id("filtered_cloud"));
    const auto& obs_cloud  = rf.get<rvpoint::PointCloud>(rf.get_id("obstacle_cloud"));
    const auto& clusters   = rf.get<rvpoint::ClusterResult>(rf.get_id("clusters"));
    const auto& down_cloud   = rf.get<rvpoint::PointCloud>(rf.get_id("downsampled_cloud"));
    const auto& filt_cloud   = rf.get<rvpoint::PointCloud>(rf.get_id("filtered_cloud"));
    const auto& ground_plane = rf.get<rvpoint::PlaneModel>(rf.get_id("ground_plane"));
    const auto& ground_cloud = rf.get<rvpoint::PointCloud>(rf.get_id("ground_cloud"));
    const auto& obs_cloud    = rf.get<rvpoint::PointCloud>(rf.get_id("obstacle_cloud"));
    const auto& clusters     = rf.get<rvpoint::ClusterResult>(rf.get_id("clusters"));

    PerceptionStatus pipeline_status = PerceptionStatus::SUCCESS;
    if (ground_plane.inliers == 0) {
        pipeline_status = PerceptionStatus::NO_GROUND_PLANE_FOUND;
        if (progress_enabled) {
            std::cerr << "[progress] Warning: No ground plane found (inlier count is 0)." << std::endl;
        }
    }

    // Extract execution times from PipelineManager compiled nodes
    double ms_stage3 = 0.0, ms_stage4 = 0.0, ms_stage5 = 0.0;
    double ms_stage6 = 0.0, ms_stage7 = 0.0, ms_stage8 = 0.0, ms_stage9 = 0.0;

    for (const auto& node : pm.nodes()) {
        if (node.name == "voxel_grid") ms_stage3 = node.last_execution_time_ms;
        else if (node.name == "build_search_index") ms_stage4 = node.last_execution_time_ms;
        else if (node.name == "outlier_filter") ms_stage5 = node.last_execution_time_ms;
        else if (node.name == "rebuild_search_index") ms_stage6 = node.last_execution_time_ms;
        else if (node.name == "normal_estimation") ms_stage7 = node.last_execution_time_ms;
        else if (node.name == "ransac_plane_split") ms_stage8 = node.last_execution_time_ms;
        else if (node.name == "euclidean_clustering") ms_stage9 = node.last_execution_time_ms;
    }

    if (progress_enabled) {
        std::cout << "[progress] [3/10] Downsampling complete in " << ms_stage3 << " ms" << std::endl;
        std::cout << "[progress] [4/10] Build search index for downsampled cloud complete in " << ms_stage4 << " ms" << std::endl;
        std::cout << "[progress] [5/10] " << (use_ror ? "Radius outlier removal (RVV)" : "Statistical outlier removal (RVV)") << " complete in " << ms_stage5 << " ms" << std::endl;
        std::cout << "[progress] [6/10] Rebuild search index for filtered cloud complete in " << ms_stage6 << " ms" << std::endl;
        std::cout << "[progress] [7/10] Normal estimation complete in " << ms_stage7 << " ms" << std::endl;
        std::cout << "[progress] [8/10] RANSAC primitive fitting complete in " << ms_stage8 << " ms" << std::endl;
        std::cout << "[progress] [9/10] Euclidean clustering (Hardware RVV 1.0) complete in " << ms_stage9 << " ms" << std::endl;
    }

    stage_timings.push_back({3, "Downsampling", ms_stage3, down_cloud.n});
    stage_timings.push_back({4, "Build search index for downsampled cloud", ms_stage4, down_cloud.n});
    stage_timings.push_back({5, use_ror ? "Radius outlier removal (RVV)" : "Statistical outlier removal (RVV)", ms_stage5, filt_cloud.n});
    stage_timings.push_back({6, "Rebuild search index for filtered cloud", ms_stage6, filt_cloud.n});
    stage_timings.push_back({7, "Normal estimation", ms_stage7, filt_cloud.n});
    stage_timings.push_back({8, "RANSAC primitive fitting", ms_stage8, obs_cloud.n});
    stage_timings.push_back({9, "Euclidean clustering (Hardware RVV 1.0)", ms_stage9, clusters.num_clusters()});

    // ── Stage 10: Write cluster stage ──────────────────────────────────────
    // ── Stage 10: Write pipeline outputs ───────────────────────────────────
    beginStage(10, "Write cluster stage", progress_enabled);
    stage_start = Clock::now();
    if (!disable_disk) {
        if (!savePCD((output_dir / "01_downsampled.pcd").string(), down_cloud, true)) {
            pipeline_status = PerceptionStatus::IO_WRITE_FAILURE;
        }
        if (!savePCD((output_dir / "02_sor_filtered.pcd").string(), filt_cloud, true)) {
            pipeline_status = PerceptionStatus::IO_WRITE_FAILURE;
        }
        if (!skip_normals) {
            const auto& normals_cloud = rf.get<rvpoint::PointCloud>(rf.get_id("normals_cloud"));
            if (!savePCD((output_dir / "03_normals.pcd").string(), normals_cloud, true)) {
                pipeline_status = PerceptionStatus::IO_WRITE_FAILURE;
            }
        }
        if (!savePCD((output_dir / "04_ransac_inliers.pcd").string(), ground_cloud, true)) {
            pipeline_status = PerceptionStatus::IO_WRITE_FAILURE;
        }
        if (!savePCD((output_dir / "05_ground_plane_removed.pcd").string(), obs_cloud, true)) {
            pipeline_status = PerceptionStatus::IO_WRITE_FAILURE;
        }

        std::vector<PointXYZRGB> colored_pts;
        colored_pts.reserve(obs_cloud.n);
        for (size_t c_idx = 0; c_idx < clusters.num_clusters(); ++c_idx) {
            auto col = getClusterColor(c_idx);
            size_t c_start = clusters.offsets[c_idx];
            size_t c_end = clusters.offsets[c_idx + 1];
            for (size_t j = c_start; j < c_end; ++j) {
                int pt_idx = clusters.indices[j];
                if (pt_idx >= 0 && static_cast<size_t>(pt_idx) < obs_cloud.n) {
                    colored_pts.push_back({obs_cloud.x[pt_idx], obs_cloud.y[pt_idx], obs_cloud.z[pt_idx],
                                          col.r, col.g, col.b});
                }
            }
        }
        if (!savePCDRGB((output_dir / "06_clusters.pcd").string(), colored_pts, true)) {
            pipeline_status = PerceptionStatus::IO_WRITE_FAILURE;
        }
    }
    stage_timings.push_back({10, "Write cluster stage", endStage(10, "Write cluster stage", stage_start, progress_enabled), clusters.num_clusters()});

    const auto overall_end = Clock::now();
    const double total_ms = std::chrono::duration<double, std::milli>(overall_end - overall_start).count();

    printFinalBreakdown(stage_timings, total_ms);

    if (json_metrics) {
        if (!disable_disk) {
            if (!saveJSONMetrics(output_dir / "metrics.json", stage_timings, total_ms,
                                 voxel_leaf_size, skip_sor, cluster_tolerance, pipeline_status,
                                 ground_plane.inliers, obs_cloud.n, clusters.num_clusters())) {
                pipeline_status = PerceptionStatus::IO_WRITE_FAILURE;
            }
        }
        std::cout << "{\n"
                  << "  \"total_ms\": " << total_ms << ",\n"
                  << "  \"status\": \"" << perceptionStatusToString(pipeline_status) << "\",\n"
                  << "  \"raw_points\": " << n_input << ",\n"
                  << "  \"downsampled_points\": " << down_cloud.n << ",\n"
                  << "  \"filtered_points\": " << filt_cloud.n << ",\n"
                  << "  \"ground_inliers\": " << ground_plane.inliers << ",\n"
                  << "  \"obstacle_points\": " << obs_cloud.n << ",\n"
                  << "  \"clusters_found\": " << clusters.num_clusters() << "\n"
                  << "}\n";
    }

    if (pipeline_status == PerceptionStatus::NO_GROUND_PLANE_FOUND) {
        return 1;
    }
    if (pipeline_status == PerceptionStatus::IO_WRITE_FAILURE) {
        std::cerr << "Error: Output file write failure occurred during pipeline export." << std::endl;
        return 1;
    }

    return 0;
}
