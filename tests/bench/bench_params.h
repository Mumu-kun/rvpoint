#ifndef RVPOINT_TESTS_BENCH_PARAMS_H
#define RVPOINT_TESTS_BENCH_PARAMS_H

#include <cstddef>
#include <cstdint>

namespace rvpoint {
namespace bench {

static constexpr std::size_t N      = 10000;   // Database point count
static constexpr std::size_t Q      = 50;     // Query point count
static constexpr float       RADIUS = 2.0f;    // Search radius
static constexpr uint32_t    SEED   = 42;      // Random seed

} // namespace bench
} // namespace rvpoint

namespace rvv_pcl = rvpoint;

#endif // RVPOINT_TESTS_BENCH_PARAMS_H
