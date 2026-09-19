#include "include/rvpoint.h"
#include <iostream>
#include <vector>
#include <random>
#include <cassert>
#include <cmath>

using namespace rvpoint;

static int g_passed = 0;
static int g_failed = 0;

#define TEST_CHECK(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAILED: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            g_failed++; \
        } else { \
            g_passed++; \
        } \
    } while(0)

int main() {
    std::cout << "=== Running test_spatial_slab ===\n";

    // 1. Empty cloud test
    {
        PointCloud empty;
        PointCloud filtered;
        ClusterResult clusters;
        SpatialSlabEngine engine;
        engine(empty, filtered, clusters);
        TEST_CHECK(filtered.size() == 0, "Empty cloud produces 0 filtered points");
        TEST_CHECK(clusters.num_clusters() == 0, "Empty cloud produces 0 clusters");
    }

    // 2. Separated clusters spanning across slab cut planes
    {
        std::mt19937 rng(42);
        std::normal_distribution<float> noise(0.0f, 0.05f);

        PointCloud cloud;
        cloud.reserve(600);

        // Cluster 0: Centered at x = -5.0
        for (int i = 0; i < 200; ++i) {
            cloud.push_back(-5.0f + noise(rng), noise(rng), noise(rng));
        }

        // Cluster 1: Straddling across x = 0.0 (wide cluster: x in [-0.4, 0.4])
        // With 4 equal slabs, the median cut plane will fall right through this cluster!
        for (int i = 0; i < 200; ++i) {
            cloud.push_back(-0.3f + (0.6f * static_cast<float>(i) / 200.0f), noise(rng), noise(rng));
        }

        // Cluster 2: Centered at x = 5.0
        for (int i = 0; i < 200; ++i) {
            cloud.push_back(5.0f + noise(rng), noise(rng), noise(rng));
        }

        // Run with 1 slab (baseline reference)
        SpatialSlabEngine engine_ref(0.3f, 3, 0.2f, 10, 500, 1, SlabAxis::X);
        PointCloud filt_ref;
        ClusterResult clust_ref;
        engine_ref(cloud, filt_ref, clust_ref);

        // Run with 4 slabs (parallel domain decomposition & boundary stitching)
        SpatialSlabEngine engine_multi(0.3f, 3, 0.2f, 10, 500, 4, SlabAxis::X);
        engine_multi.reserve(1000);
        PointCloud filt_multi;
        ClusterResult clust_multi;
        engine_multi(cloud, filt_multi, clust_multi);

        TEST_CHECK(clust_ref.num_clusters() == 3, "Baseline produces exactly 3 clusters");
        TEST_CHECK(clust_multi.num_clusters() == 3, "SpatialSlabEngine (4 slabs) produces exactly 3 clusters (boundary stitched)");

        std::cout << "Baseline clusters: " << clust_ref.num_clusters()
                  << " (total clustered pts: " << clust_ref.indices.size() << ")\n";
        std::cout << "4-Slab clusters:   " << clust_multi.num_clusters()
                  << " (total clustered pts: " << clust_multi.indices.size() << ")\n";

        TEST_CHECK(std::abs(static_cast<int>(clust_multi.indices.size()) -
                            static_cast<int>(clust_ref.indices.size())) <= 10,
                   "Multi-slab cluster points count closely matches baseline");
    }

    // 3. Multi-axis test (SlabAxis::Y)
    {
        PointCloud cloud;
        for (int i = 0; i < 100; ++i) {
            cloud.push_back(0.0f, -2.0f + 0.01f * i, 0.0f);
        }
        for (int i = 0; i < 100; ++i) {
            cloud.push_back(0.0f, 2.0f + 0.01f * i, 0.0f);
        }

        SpatialSlabEngine engine_y(0.3f, 2, 0.2f, 10, 500, 4, SlabAxis::Y);
        PointCloud filt;
        ClusterResult clust;
        engine_y(cloud, filt, clust);

        TEST_CHECK(clust.num_clusters() == 2, "SpatialSlabEngine on SlabAxis::Y detects 2 clusters");
    }

    // 4. ForwardCellClustering (13-Forward Traversal)
    {
        PointCloud cloud;
        for (int i = 0; i < 50; ++i) {
            cloud.push_back(0.0f, 0.01f * i, 0.0f);
        }
        for (int i = 0; i < 50; ++i) {
            cloud.push_back(5.0f, 0.01f * i, 0.0f);
        }

        ForwardCellClustering fcc(0.2f, 10, 500);
        fcc.reserve(200);
        ClusterResult clust;
        fcc(cloud, clust);

        TEST_CHECK(clust.num_clusters() == 2, "ForwardCellClustering detects 2 separate clusters");
        TEST_CHECK(clust.indices.size() == 100, "ForwardCellClustering clusters all 100 points");
        std::cout << "ForwardCellClustering (13-Forward) clusters: " << clust.num_clusters()
                  << " (pts: " << clust.indices.size() << ")\n";
    }

    std::cout << "\nTest Summary: Passed: " << g_passed << ", Failed: " << g_failed << "\n";
    return (g_failed == 0) ? 0 : 1;
}
