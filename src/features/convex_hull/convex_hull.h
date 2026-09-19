#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include "core/point_types.h"

namespace rvpoint {

/**
 * @brief Strategy selector for 2D Convex Hull extraction.
 */
enum class ConvexHullStrategy : uint8_t {
    MONOTONE_CHAIN,   ///< Andrew's Monotone Chain with RVV Akl-Toussaint extrema pre-filter
    JARVIS_MARCH,     ///< Vectorized Jarvis March Gift Wrapping (Zero-sort, optimal for M <= 16)
    ANGULAR_BINNING   ///< Polar Angular Support Approximation (Deterministic O(B K), ultra-fast)
};

/**
 * @brief Pure leaf kernel extracting 2D Convex Hull.
 *
 * Conforms to AGENTS.md Leaf-Kernel Invariant (owns zero child kernels),
 * ADR-0010 (zero heap allocations on steady-state hot paths), and ADR-0011.
 * Outputs a PointCloud2D with vertices ordered counter-clockwise.
 */
class ConvexHull2D {
public:
    explicit ConvexHull2D(std::size_t max_points = 2048,
                          ConvexHullStrategy strategy = ConvexHullStrategy::MONOTONE_CHAIN,
                          Backend backend = Backend::Auto);

    /**
     * @brief Pre-allocate scratch capacity for hot-path zero-heap execution (ADR-0010).
     */
    void reserve(std::size_t max_points);

    void set_backend(Backend b) noexcept { backend_ = b; }
    Backend backend() const noexcept { return backend_; }

    void set_strategy(ConvexHullStrategy s) noexcept { strategy_ = s; }
    ConvexHullStrategy strategy() const noexcept { return strategy_; }

    /**
     * @brief Compute 2D convex hull from an indexed subset of a 3D point cloud.
     * @param cloud Source 3D point cloud.
     * @param indices Array of indices belonging to cluster.
     * @param count Number of indices.
     * @param out_hull Output 2D convex polygon vertices in CCW order (SoA).
     */
    void compute(const PointCloud& cloud,
                 const uint32_t* indices,
                 std::size_t count,
                 PointCloud2D& out_hull);

    /**
     * @brief Functor call operator.
     */
    void operator()(const PointCloud& cloud,
                    const uint32_t* indices,
                    std::size_t count,
                    PointCloud2D& out_hull) {
        compute(cloud, indices, count, out_hull);
    }

private:
    ConvexHullStrategy strategy_;
    Backend backend_;

    // Stage-owned pre-allocated scratch workspaces (ADR-0010)
    std::vector<float> pts_x_;
    std::vector<float> pts_y_;
    std::vector<float> res_x_;
    std::vector<float> res_y_;
    std::vector<uint32_t> perm_;

    void compute_monotone_chain(std::size_t count, PointCloud2D& out_hull);
    void compute_jarvis_march(std::size_t count, PointCloud2D& out_hull);
    void compute_angular_binning(std::size_t count, PointCloud2D& out_hull);

#if defined(__riscv_vector)
    void filter_akl_toussaint_rvv(std::size_t count, std::size_t& out_count);
#endif
};

} // namespace rvpoint
