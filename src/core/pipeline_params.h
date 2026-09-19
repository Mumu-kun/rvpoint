#ifndef RVPOINT_CORE_PIPELINE_PARAMS_H
#define RVPOINT_CORE_PIPELINE_PARAMS_H

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

namespace rvpoint {

/**
 * @brief Single Source of Truth Base Pitch (Δ) Pipeline Parameter Derivation.
 *
 * Auto-derives all downstream perception stage parameters from a single spatial
 * resolution pitch Δ, ensuring consistency across RVPoint accelerated and PCL
 * baseline pipelines.
 */
struct PipelineConfig {
  // ── Base Spatial Resolution ───────────────────────────────────────────
  float delta = 0.02f; ///< Base Voxel Pitch (Δ), default 2cm tabletop

  // ── Stage 1: Voxel Grid Downsampling ──────────────────────────────────
  float voxel_leaf_size = 0.02f; ///< Voxel Pitch = 1.0 * Δ

  // ── Stage 2: Outlier Removal (SOR / ROR) ──────────────────────────────
  int sor_mean_k = 20;             ///< SOR Mean-K neighbors (std: 20)
  float sor_std_threshold = 1.0f;  ///< SOR Outlier threshold α = 1.0σ
  float sor_search_radius = 0.05f; ///< SOR Search Radius = 2.5 * Δ
  float ror_radius = 0.05f;        ///< ROR Radius = 2.5 * Δ
  int ror_min_pts = 5;             ///< ROR Minimum neighbors

  // ── Stage 3: Surface Normal Estimation ────────────────────────────────
  float search_radius = 0.05f; ///< Normal Search Radius = 2.5 * Δ
  int normal_k = 15;           ///< Minimum expected neighbors (≥15)

  // ── Stage 4: RANSAC Ground Plane Segmentation ────────────────────────
  float ransac_distance_threshold =
      0.01f;                       ///< Sub-voxel inlier tolerance ε = 0.5 * Δ
  int ransac_max_iterations = 250; ///< Candidate plane iterations
  float ransac_ground_angle_deg =
      15.0f;             ///< Max ground normal deviation in degrees
  uint64_t seed = 42ULL; ///< Deterministic PRNG seed

  // ── Stage 5: Euclidean Clustering ─────────────────────────────────────
  float cluster_tolerance = 0.025f; ///< Proximity tolerance d = 1.25 * Δ
  int min_cluster_size = 10;        ///< Min cluster point count
  int max_cluster_size = 100000;    ///< Max cluster point count

  /**
   * @brief Derive all pipeline stage parameters from a single Base Pitch Δ.
   * @param d Base resolution pitch in meters (e.g. 0.01 for dense, 0.02 for
   * tabletop, 0.05 for outdoor).
   */
  static PipelineConfig fromDelta(float d) {
    PipelineConfig cfg;
    cfg.delta = d;
    cfg.voxel_leaf_size = d;      // Scale: 1.0 * Δ
    cfg.search_radius = 2.5f * d; // Scale: 2.5 * Δ
    cfg.normal_k = 15;
    cfg.sor_search_radius = 2.5f * d; // Scale: 2.5 * Δ
    cfg.sor_mean_k = 20;
    cfg.sor_std_threshold = 1.0f; // 1.0σ cutoff
    cfg.ror_radius = 2.5f * d;    // Scale: 2.5 * Δ
    cfg.ror_min_pts = 5;
    cfg.ransac_distance_threshold =
        0.5f * d; // Scale: 0.5 * Δ (sub-voxel planar consensus)
    cfg.ransac_max_iterations = 250;
    cfg.ransac_ground_angle_deg = 15.0f;
    cfg.seed = 42ULL;
    cfg.cluster_tolerance =
        1.25f * d; // Scale: 1.25 * Δ (seamless intra-object linking)
    cfg.min_cluster_size = 10;
    cfg.max_cluster_size = 100000;
    return cfg;
  }

  /**
   * @brief Instantiate standardized resolution presets.
   */
  static PipelineConfig fromPreset(const std::string &preset_name) {
    if (preset_name == "dense" || preset_name == "object") {
      return fromDelta(0.01f); // 1cm pitch (Fine object scans)
    } else if (preset_name == "sparse" || preset_name == "outdoor" ||
               preset_name == "lidar") {
      return fromDelta(0.05f); // 5cm pitch (Automotive / terrain sweeps)
    } else {
      // Default: Tabletop benchmark (2cm pitch)
      return fromDelta(0.02f);
    }
  }

  void print() const {
    std::cout << "[PipelineConfig] Base Pitch (Δ): " << std::fixed
              << std::setprecision(4) << delta << " m\n"
              << "  ├─ Voxel Leaf Size (1.0Δ):    " << voxel_leaf_size << " m\n"
              << "  ├─ SOR / ROR Radius (2.5Δ):   " << sor_search_radius
              << " m (k=" << sor_mean_k << ", std=" << sor_std_threshold
              << "σ)\n"
              << "  ├─ Normal Radius (2.5Δ):      " << search_radius
              << " m (min_k=" << normal_k << ")\n"
              << "  ├─ RANSAC Inlier Dist (0.5Δ): " << ransac_distance_threshold
              << " m (iters=" << ransac_max_iterations << ")\n"
              << "  └─ Cluster Tolerance (1.25Δ): " << cluster_tolerance
              << " m (size=[" << min_cluster_size << ".." << max_cluster_size
              << "])\n";
  }
};

inline constexpr PipelineConfig kDefaultPipelineConfig{};

} // namespace rvpoint

#endif // RVPOINT_CORE_PIPELINE_PARAMS_H
