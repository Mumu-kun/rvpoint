#include "include/rvpoint.h"
#include <iostream>
#include <cassert>
#include <random>

using namespace rvpoint;

// ==============================================================================
// Compile-time Static Concept Verification
// ==============================================================================

// Spatial Search Concepts
static_assert(is_radius_search_v<Fast3DSpatialGrid>,
              "Fast3DSpatialGrid must model RadiusSearchable");
static_assert(has_neighbor_counting_v<Fast3DSpatialGrid>,
              "Fast3DSpatialGrid must support count_neighbors");
static_assert(is_radius_search_v<PointerOctree>,
              "PointerOctree must model RadiusSearchable");
static_assert(is_knn_search_v<PointerOctree>,
              "PointerOctree must model KNNSearchable");

// Cloud Filter Concepts
static_assert(is_point_cloud_filter_v<VoxelGrid>,
              "VoxelGrid must model PointCloudFilter");
static_assert(is_point_cloud_filter_v<RadiusOutlierRemoval>,
              "RadiusOutlierRemoval must model PointCloudFilter");
static_assert(is_point_cloud_filter_v<StatisticalOutlierRemoval>,
              "StatisticalOutlierRemoval must model PointCloudFilter");

// Segmentation Sub-Families: Model Fitter & Cluster Extractor
static_assert(is_model_fitter_v<RansacPlane, PlaneModel>,
              "RansacPlane must model ModelFitter<PlaneModel>");
static_assert(is_cluster_extractor_v<EuclideanClustering>,
              "EuclideanClustering must model ClusterExtractor");

// ==============================================================================
// Runtime Verification through Generic Helpers
// ==============================================================================

int main() {
    std::cout << "=== Running Kernel Concept & Abstraction Verification ===" << std::endl;

    // 1. Generate synthetic planar test cloud with cluster
    const size_t N = 1000;
    PointCloud cloud;
    cloud.reserve(N);

    std::mt19937 rng(42);
    std::uniform_real_distribution<float> plane_dist(-2.0f, 2.0f);
    std::normal_distribution<float> noise(0.0f, 0.01f);

    for (size_t i = 0; i < N; ++i) {
        float x = plane_dist(rng);
        float y = plane_dist(rng);
        float z = 0.5f * x - 0.3f * y + 1.0f + noise(rng); // Z = 0.5X - 0.3Y + 1
        cloud.push_back(x, y, z);
    }

    std::cout << "[1] Cloud created with " << cloud.size() << " points." << std::endl;

    // 2. Test Cloud Filter Concept helper: rvpoint::filter
    VoxelGrid vg(0.1f);
    PointCloud filtered;
    size_t filtered_count = rvpoint::filter(cloud, filtered, vg);
    std::cout << "[2] VoxelGrid filter applied via concept helper: "
              << filtered_count << " points retained." << std::endl;
    assert(filtered_count > 0 && filtered_count < N);

    // 3. Test Spatial Search Concept helper: rvpoint::radius_search
    Fast3DSpatialGrid grid(0.2f, filtered.size());
    bool build_ok = grid.build(filtered.view());
    assert(build_ok);

    NeighborQueryResult search_res;
    rvpoint::radius_search(grid, 0.0f, 0.0f, 1.0f, 0.5f, search_res);
    std::cout << "[3] Fast3DSpatialGrid radius_search via concept helper: "
              << search_res.indices.size() << " neighbors found." << std::endl;
    assert(!search_res.indices.empty());

    // 4. Test Spatial Search Concept helper: rvpoint::count_neighbors
    int counted = rvpoint::count_neighbors(grid, 0.0f, 0.0f, 1.0f, 0.5f, 50);
    std::cout << "[4] Fast3DSpatialGrid count_neighbors via concept helper: "
              << counted << std::endl;
    assert(counted > 0);

    // 5. Test Model Fitter Concept helper: rvpoint::fit_model
    RansacPlane ransac(0.05f, 100);
    PlaneModel plane;
    int inliers = rvpoint::fit_model(ransac, cloud.view(), plane);
    std::cout << "[5] RansacPlane fit_model via concept helper: "
              << inliers << " inliers found. Normal: ("
              << plane.a << ", " << plane.b << ", " << plane.c << ")" << std::endl;
    assert(inliers > static_cast<int>(N * 0.7f));

    // 6. Test Cluster Extractor Concept helper: rvpoint::extract_clusters
    EuclideanClustering ec(0.3f, 5, 2000);
    ClusterResult clusters;
    rvpoint::extract_clusters(ec, filtered.view(), clusters);
    std::cout << "[6] EuclideanClustering extract_clusters via concept helper: "
              << clusters.num_clusters() << " clusters identified." << std::endl;
    assert(clusters.num_clusters() >= 1);

    // 7. Test PointerOctree k-NN Search
    PointerOctree octree;
    octree.setInputCloud(filtered.view());
    octree.build();
    NeighborQueryResult knn_res;
    octree.nearest_k_search(0.0f, 0.0f, 1.0f, 10, knn_res);
    std::cout << "[7] PointerOctree nearest_k_search: "
              << knn_res.indices.size() << " nearest neighbors found." << std::endl;
    assert(knn_res.indices.size() == 10);

    std::cout << "[PASS] All kernel concept abstractions verified successfully!" << std::endl;
    return 0;
}

