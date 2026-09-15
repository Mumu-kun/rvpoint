#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include <cmath>

#include "core/point_types.h"

namespace rvpoint {

/**
 * @brief Strategy selector for bounding box orientation extraction.
 */
enum class BoundingBoxStrategy : uint8_t {
    MIN_AREA,      ///< Pure Freeman-Shapira Rotating Calipers / Vectorized Hull Edge Projection
    L_SHAPE_ALIGN, ///< Optimized Zhang et al. Face Closeness Alignment (RVV Batched + Midpoint)
    WIREFRAME_PCA, ///< Continuous Perimeter Covariance (Closed-form O(1), density-invariant)
    EDGE_ALIGN     ///< Hull Edge-Perimeter Alignment (EPA: O(M) edge-length weighted orientation snapping)
};

/**
 * @brief Configuration parameters for BoundingBoxExtractor.
 */
struct BoundingBoxParams {
    BoundingBoxStrategy strategy = BoundingBoxStrategy::L_SHAPE_ALIGN;
    float truncation_dist = 0.20f;       ///< Truncation distance d0 for closeness evaluation (meters)
    float area_constraint_ratio = 1.20f; ///< Feasible area window: Area <= 1.20 * A_min
    uint32_t decimation_threshold = 64;  ///< Stratified scoring decimation threshold (points)

    // Optional gating / threshold hints for pipeline orchestrators
    float min_l_shape_extent = 0.80f;    ///< Suggested minimum dimension for L-shape fitting (meters)
    uint32_t min_l_shape_points = 15;    ///< Suggested minimum points for L-shape fitting
    float min_closeness_gain = 1.20f;    ///< Suggested minimum closeness gain over min-area
};

/**
 * @brief Pure leaf kernel extracting 3D Oriented Bounding Boxes from point cloud clusters.
 *
 * Conforms to AGENTS.md Leaf-Kernel Invariant (owns zero child kernels, does not compute BoundingDisc),
 * ADR-0010 (zero heap allocations on steady-state hot paths), and ADR-0011 (backend dispatch).
 *
 * All strategies are pure and unconditional.
 */
class BoundingBoxExtractor {
public:
    explicit BoundingBoxExtractor(const BoundingBoxParams& params = BoundingBoxParams{},
                                  std::size_t max_points = 2048,
                                  Backend backend = Backend::Auto);

    void reserve(std::size_t max_points);

    void set_backend(Backend b) noexcept { backend_ = b; }
    Backend backend() const noexcept { return backend_; }

    void set_strategy(BoundingBoxStrategy s) noexcept { params_.strategy = s; }
    BoundingBoxStrategy strategy() const noexcept { return params_.strategy; }

    const BoundingBoxParams& params() const noexcept { return params_; }
    void set_params(const BoundingBoxParams& p) noexcept { params_ = p; }

    /**
     * @brief Extract 3D Oriented Bounding Box using the configured strategy.
     */
    void compute(const PointCloud& cloud,
                 const uint32_t* indices,
                 std::size_t count,
                 const PointCloud2D& hull,
                 OrientedBoundingBox& out_box);

    /**
     * @brief Unconditional minimum-area bounding box from convex hull.
     */
    void compute_min_area(const PointCloud& cloud,
                          const uint32_t* indices,
                          std::size_t count,
                          const PointCloud2D& hull,
                          OrientedBoundingBox& out_box);

    /**
     * @brief Unconditional Zhang et al. face closeness alignment on feasible candidate window.
     */
    void compute_l_shape(const PointCloud& cloud,
                         const uint32_t* indices,
                         std::size_t count,
                         const PointCloud2D& hull,
                         OrientedBoundingBox& out_box);

    /**
     * @brief Unconditional Hull Edge-Perimeter Alignment (EPA) on feasible candidate window.
     */
    void compute_edge_align(const PointCloud& cloud,
                            const uint32_t* indices,
                            std::size_t count,
                            const PointCloud2D& hull,
                            OrientedBoundingBox& out_box);

    /**
     * @brief Unconditional continuous perimeter line-integral covariance PCA.
     */
    void compute_wireframe_pca(const PointCloud2D& hull,
                               float z_min, float z_max,
                               uint32_t pt_count,
                               OrientedBoundingBox& out_box);

    /**
     * @brief Functor call operator.
     */
    void operator()(const PointCloud& cloud,
                    const uint32_t* indices,
                    std::size_t count,
                    const PointCloud2D& hull,
                    OrientedBoundingBox& out_box) {
        compute(cloud, indices, count, hull, out_box);
    }

private:
    struct CandidateBox {
        float ux = 0.0f, uy = 0.0f;
        float u_min = 0.0f, u_max = 0.0f, v_min = 0.0f, v_max = 0.0f;
        float area = 0.0f;
    };

    struct BatchedCandidate {
        float ux = 0.0f, uy = 0.0f;
        float u_mid = 0.0f, v_mid = 0.0f;
        float thresh_u = 0.0f, thresh_v = 0.0f;
    };

    BoundingBoxParams params_;
    Backend backend_;

    // Stage-owned pre-allocated scratch workspaces (ADR-0010)
    std::vector<float> scratch_x_;
    std::vector<float> scratch_y_;
    std::vector<float> scratch_z_;
    std::vector<float> scratch_dec_x_;
    std::vector<float> scratch_dec_y_;
    std::vector<CandidateBox> candidates_;

    void extract_candidates(const PointCloud2D& hull, std::size_t& candidate_count,
                            std::size_t& min_area_idx, float& min_area);

    void populate_box(const CandidateBox& win, float min_z, float max_z,
                      uint32_t pt_count, OrientedBoundingBox& out_box);

    float evaluate_closeness_single(const CandidateBox& cand, const float* px, const float* py, std::size_t count);
    float evaluate_closeness_single_rvv(const CandidateBox& cand, const float* px, const float* py, std::size_t count);
    float evaluate_closeness_single_scalar(const CandidateBox& cand, const float* px, const float* py, std::size_t count);

#if defined(__riscv_vector)
    void evaluate_closeness_batch3_rvv(const float* px, const float* py, std::size_t count,
                                       const BatchedCandidate& c0, const BatchedCandidate& c1, const BatchedCandidate& c2,
                                       float& s0, float& s1, float& s2);
#endif
};

} // namespace rvpoint
