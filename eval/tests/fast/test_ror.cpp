#include "core/point_types.h"
#include "filters/radius_outlier_removal.h"
#include <iostream>
#include <random>
#include <vector>

using namespace rvpoint;

int main() {
    std::cout << "========================================\n";
    std::cout << "   Test: Radius Outlier Removal (ROR)   \n";
    std::cout << "========================================\n";

    const size_t N_CLUSTER = 200;
    const size_t N_OUTLIERS = 10;
    const size_t N_TOTAL = N_CLUSTER + N_OUTLIERS;

    PointCloud cloud(N_TOTAL);
    std::mt19937 gen(42);
    std::uniform_real_distribution<float> cluster_dist(-0.1f, 0.1f);

    // 1. Generate dense cluster centered at origin (all points within ~0.2m of each other)
    for (size_t i = 0; i < N_CLUSTER; ++i) {
        cloud.push_back(cluster_dist(gen), cluster_dist(gen), cluster_dist(gen));
    }

    // 2. Generate isolated outliers far away (distance > 10.0m)
    for (size_t i = 0; i < N_OUTLIERS; ++i) {
        float offset = 10.0f + static_cast<float>(i) * 5.0f;
        cloud.push_back(offset, offset, offset);
    }

    std::cout << "Generated cloud with " << cloud.size() << " points ("
              << N_CLUSTER << " cluster points, " << N_OUTLIERS << " outliers).\n";

    // 3. Instantiate and warmup ROR filter
    RadiusOutlierRemoval ror(0.5f, 5);
    ror.reserve(N_TOTAL);

    PointCloud filtered;
    size_t filtered_count = ror(cloud.view(), filtered);

    std::cout << "Filtered count: " << filtered_count << " (Expected: " << N_CLUSTER << ")\n";

    if (filtered_count != N_CLUSTER) {
        std::cerr << "[FAIL] Expected " << N_CLUSTER << " points, got " << filtered_count << std::endl;
        return 1;
    }

    // Verify all filtered points are from the cluster (bounds check)
    for (size_t i = 0; i < filtered.size(); ++i) {
        if (std::abs(filtered.x[i]) > 1.0f || std::abs(filtered.y[i]) > 1.0f || std::abs(filtered.z[i]) > 1.0f) {
            std::cerr << "[FAIL] Outlier point was not removed: ("
                      << filtered.x[i] << ", " << filtered.y[i] << ", " << filtered.z[i] << ")\n";
            return 1;
        }
    }

    // 4. Test multi-threaded KernelParallel parity
    RadiusOutlierRemoval ror_parallel(0.5f, 5);
    ror_parallel.threads(4);
    ror_parallel.reserve(N_TOTAL);

    PointCloud filtered_parallel;
    size_t parallel_count = ror_parallel(cloud.view(), filtered_parallel);
    if (parallel_count != filtered_count) {
        std::cerr << "[FAIL] Multi-thread parity failed! Single: " << filtered_count
                  << " vs Parallel: " << parallel_count << std::endl;
        return 1;
    }
    for (size_t i = 0; i < filtered_count; ++i) {
        if (filtered.x[i] != filtered_parallel.x[i] ||
            filtered.y[i] != filtered_parallel.y[i] ||
            filtered.z[i] != filtered_parallel.z[i]) {
            std::cerr << "[FAIL] Multi-thread point order mismatch at index " << i << std::endl;
            return 1;
        }
    }
    std::cout << "[PASS] Multi-threaded ROR (4 threads) bit-exact parity verified!\n";

    std::cout << "[PASS] Radius Outlier Removal successfully filtered outliers!\n";
    return 0;
}

