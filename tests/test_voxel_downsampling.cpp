/**
 * @file test_voxel_downsampling.cpp
 * @brief Unit tests for voxel grid downsampling implementations
 *
 * Tests both scalar and RVV implementations for correctness and equivalence.
 */

#include <algorithm>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <random>
#include <vector>

#include "rvpoint/point3d.hpp"

// External function declarations (implemented in separate files)
std::vector<Point3D> voxel_downsample_scalar(const std::vector<Point3D>& input, float leaf_size);
std::vector<Point3D> voxel_downsample_rvv(const std::vector<Point3D>& input, float leaf_size);

// Helper function to generate random point cloud
std::vector<Point3D> generate_random_cloud(size_t num_points, float min_coord = -100.0f,
                                           float max_coord = 100.0f, unsigned int seed = 42) {
    std::vector<Point3D> cloud;
    cloud.reserve(num_points);

    std::mt19937 gen(seed);
    std::uniform_real_distribution<float> dis(min_coord, max_coord);

    for (size_t i = 0; i < num_points; ++i) {
        cloud.emplace_back(dis(gen), dis(gen), dis(gen));
    }

    return cloud;
}

// Helper to sort point clouds for comparison
void sort_points(std::vector<Point3D>& points) {
    std::sort(points.begin(), points.end(), [](const Point3D& a, const Point3D& b) {
        if (std::abs(a.x - b.x) > 1e-5f)
            return a.x < b.x;
        if (std::abs(a.y - b.y) > 1e-5f)
            return a.y < b.y;
        return a.z < b.z;
    });
}

TEST_CASE("Voxel downsampling - empty input", "[voxel][scalar]") {
    std::vector<Point3D> empty;

    SECTION("Scalar implementation") {
        auto result = voxel_downsample_scalar(empty, 1.0f);
        REQUIRE(result.empty());
    }

    SECTION("RVV implementation") {
        auto result = voxel_downsample_rvv(empty, 1.0f);
        REQUIRE(result.empty());
    }
}

TEST_CASE("Voxel downsampling - single point", "[voxel][scalar]") {
    std::vector<Point3D> input = {Point3D(1.0f, 2.0f, 3.0f)};

    SECTION("Scalar implementation") {
        auto result = voxel_downsample_scalar(input, 1.0f);
        REQUIRE(result.size() == 1);
        REQUIRE(result[0].approx_equal(input[0]));
    }

    SECTION("RVV implementation") {
        auto result = voxel_downsample_rvv(input, 1.0f);
        REQUIRE(result.size() == 1);
        REQUIRE(result[0].approx_equal(input[0]));
    }
}

TEST_CASE("Voxel downsampling - two points in same voxel", "[voxel][scalar]") {
    std::vector<Point3D> input = {Point3D(0.1f, 0.2f, 0.3f), Point3D(0.4f, 0.5f, 0.6f)};
    float leaf_size = 1.0f;

    // Expected centroid
    Point3D expected(0.25f, 0.35f, 0.45f);

    SECTION("Scalar implementation") {
        auto result = voxel_downsample_scalar(input, leaf_size);
        REQUIRE(result.size() == 1);
        REQUIRE(result[0].approx_equal(expected));
    }

    SECTION("RVV implementation") {
        auto result = voxel_downsample_rvv(input, leaf_size);
        REQUIRE(result.size() == 1);
        REQUIRE(result[0].approx_equal(expected));
    }
}

TEST_CASE("Voxel downsampling - two points in different voxels", "[voxel][scalar]") {
    std::vector<Point3D> input = {Point3D(0.5f, 0.5f, 0.5f), Point3D(1.5f, 1.5f, 1.5f)};
    float leaf_size = 1.0f;

    SECTION("Scalar implementation") {
        auto result = voxel_downsample_scalar(input, leaf_size);
        REQUIRE(result.size() == 2);
    }

    SECTION("RVV implementation") {
        auto result = voxel_downsample_rvv(input, leaf_size);
        REQUIRE(result.size() == 2);
    }
}

TEST_CASE("Voxel downsampling - multiple points forming a grid", "[voxel][scalar]") {
    // Create a 3x3x3 grid of points
    std::vector<Point3D> input;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                input.emplace_back(static_cast<float>(i) + 0.5f, static_cast<float>(j) + 0.5f,
                                   static_cast<float>(k) + 0.5f);
            }
        }
    }

    float leaf_size = 1.0f;

    SECTION("Scalar implementation") {
        auto result = voxel_downsample_scalar(input, leaf_size);
        REQUIRE(result.size() == 27); // Each point in different voxel
    }

    SECTION("RVV implementation") {
        auto result = voxel_downsample_rvv(input, leaf_size);
        REQUIRE(result.size() == 27);
    }
}

TEST_CASE("Voxel downsampling - negative coordinates", "[voxel][scalar]") {
    std::vector<Point3D> input = {Point3D(-0.5f, -0.5f, -0.5f), Point3D(-1.5f, -1.5f, -1.5f),
                                  Point3D(-0.3f, -0.3f, -0.3f)};
    float leaf_size = 1.0f;

    SECTION("Scalar implementation") {
        auto result = voxel_downsample_scalar(input, leaf_size);
        // Points at -0.5 and -0.3 are in same voxel [-1, 0)
        // Point at -1.5 is in different voxel [-2, -1)
        REQUIRE(result.size() == 2);
    }

    SECTION("RVV implementation") {
        auto result = voxel_downsample_rvv(input, leaf_size);
        REQUIRE(result.size() == 2);
    }
}

TEST_CASE("Voxel downsampling - larger leaf size", "[voxel][scalar]") {
    std::vector<Point3D> input = {Point3D(0.5f, 0.5f, 0.5f), Point3D(1.5f, 1.5f, 1.5f),
                                  Point3D(2.5f, 2.5f, 2.5f)};

    SECTION("Leaf size 1.0") {
        auto result_scalar = voxel_downsample_scalar(input, 1.0f);
        REQUIRE(result_scalar.size() == 3);
    }

    SECTION("Leaf size 2.0") {
        auto result_scalar = voxel_downsample_scalar(input, 2.0f);
        // Points at 0.5 and 1.5 in same voxel [0, 2)
        // Point at 2.5 in different voxel [2, 4)
        REQUIRE(result_scalar.size() == 2);
    }

    SECTION("Leaf size 5.0") {
        auto result_scalar = voxel_downsample_scalar(input, 5.0f);
        // All points in same voxel [0, 5)
        REQUIRE(result_scalar.size() == 1);
    }
}

TEST_CASE("Voxel downsampling - random cloud comparison", "[voxel][comparison]") {
    // Generate random point cloud
    const size_t num_points = 10000;
    const float leaf_size = 1.0f;

    auto input = generate_random_cloud(num_points, -50.0f, 50.0f);

    auto result_scalar = voxel_downsample_scalar(input, leaf_size);
    auto result_rvv = voxel_downsample_rvv(input, leaf_size);

    // Both should produce same number of output points
    REQUIRE(result_scalar.size() == result_rvv.size());

    // Sort both results for comparison
    sort_points(result_scalar);
    sort_points(result_rvv);

    // Check that all points are approximately equal
    REQUIRE(result_scalar.size() > 0);
    for (size_t i = 0; i < result_scalar.size(); ++i) {
        REQUIRE(result_scalar[i].approx_equal(result_rvv[i], 1e-4f));
    }
}

TEST_CASE("Voxel downsampling - reduction ratio", "[voxel][performance]") {
    const size_t num_points = 100000;
    const float leaf_size = 5.0f; // Larger leaf size for better reduction

    auto input = generate_random_cloud(num_points, -100.0f, 100.0f);

    SECTION("Scalar implementation") {
        auto result = voxel_downsample_scalar(input, leaf_size);
        double reduction_ratio = static_cast<double>(result.size()) / num_points;

        // Should have significant reduction with 5.0 leaf size
        REQUIRE(reduction_ratio < 0.9); // Expect at least 10% reduction
        REQUIRE(result.size() > 0);
        REQUIRE(result.size() < num_points); // Output should be smaller
    }

    SECTION("RVV implementation") {
        auto result = voxel_downsample_rvv(input, leaf_size);
        double reduction_ratio = static_cast<double>(result.size()) / num_points;

        // Should have significant reduction with 5.0 leaf size
        REQUIRE(reduction_ratio < 0.9); // Expect at least 10% reduction
        REQUIRE(result.size() > 0);
        REQUIRE(result.size() < num_points); // Output should be smaller
    }
}

TEST_CASE("Voxel downsampling - centroid accuracy", "[voxel][accuracy]") {
    // Create points that we know the exact centroid for
    std::vector<Point3D> input = {Point3D(0.0f, 0.0f, 0.0f), Point3D(0.2f, 0.2f, 0.2f),
                                  Point3D(0.4f, 0.4f, 0.4f), Point3D(0.6f, 0.6f, 0.6f),
                                  Point3D(0.8f, 0.8f, 0.8f)};
    float leaf_size = 1.0f;

    // Expected centroid: average of all points
    Point3D expected(0.4f, 0.4f, 0.4f);

    SECTION("Scalar implementation") {
        auto result = voxel_downsample_scalar(input, leaf_size);
        REQUIRE(result.size() == 1);
        REQUIRE(result[0].approx_equal(expected, 1e-5f));
    }

    SECTION("RVV implementation") {
        auto result = voxel_downsample_rvv(input, leaf_size);
        REQUIRE(result.size() == 1);
        REQUIRE(result[0].approx_equal(expected, 1e-5f));
    }
}

TEST_CASE("Voxel downsampling - stress test", "[voxel][stress]") {
    const size_t num_points = 1000000; // 1M points
    const float leaf_size = 1.0f;

    auto input = generate_random_cloud(num_points, -100.0f, 100.0f);

    SECTION("Scalar implementation") {
        auto result = voxel_downsample_scalar(input, leaf_size);
        REQUIRE(result.size() > 0);
        REQUIRE(result.size() < num_points);
    }

    SECTION("RVV implementation") {
        auto result = voxel_downsample_rvv(input, leaf_size);
        REQUIRE(result.size() > 0);
        REQUIRE(result.size() < num_points);
    }
}

TEST_CASE("Voxel downsampling - boundary conditions", "[voxel][edge]") {
    SECTION("Very small leaf size") {
        std::vector<Point3D> input = {Point3D(0.0f, 0.0f, 0.0f), Point3D(0.001f, 0.001f, 0.001f)};
        float leaf_size = 0.01f;

        auto result_scalar = voxel_downsample_scalar(input, leaf_size);
        auto result_rvv = voxel_downsample_rvv(input, leaf_size);

        REQUIRE(result_scalar.size() == result_rvv.size());
        REQUIRE(result_scalar.size() == 1); // Both in same voxel
    }

    SECTION("Very large leaf size") {
        auto input = generate_random_cloud(1000, -100.0f, 100.0f);
        float leaf_size = 1000.0f;

        auto result_scalar = voxel_downsample_scalar(input, leaf_size);
        auto result_rvv = voxel_downsample_rvv(input, leaf_size);

        REQUIRE(result_scalar.size() == result_rvv.size());
        REQUIRE(result_scalar.size() == 1); // All in same voxel
    }

    SECTION("Leaf size equals coordinate range") {
        std::vector<Point3D> input = {Point3D(0.0f, 0.0f, 0.0f), Point3D(5.0f, 5.0f, 5.0f),
                                      Point3D(9.9f, 9.9f, 9.9f)};
        float leaf_size = 10.0f;

        auto result_scalar = voxel_downsample_scalar(input, leaf_size);
        auto result_rvv = voxel_downsample_rvv(input, leaf_size);

        REQUIRE(result_scalar.size() == result_rvv.size());
        REQUIRE(result_scalar.size() == 1); // All in voxel [0,10)
    }
}

TEST_CASE("Voxel downsampling - extreme coordinates", "[voxel][edge]") {
    SECTION("Very large positive coordinates") {
        std::vector<Point3D> input = {Point3D(1000.0f, 1000.0f, 1000.0f),
                                      Point3D(1000.5f, 1000.5f, 1000.5f)};
        float leaf_size = 1.0f;

        auto result_scalar = voxel_downsample_scalar(input, leaf_size);
        auto result_rvv = voxel_downsample_rvv(input, leaf_size);

        REQUIRE(result_scalar.size() == result_rvv.size());
        REQUIRE(result_scalar.size() == 1);
    }

    SECTION("Very large negative coordinates") {
        std::vector<Point3D> input = {Point3D(-1000.0f, -1000.0f, -1000.0f),
                                      Point3D(-1000.4f, -1000.4f, -1000.4f)};
        float leaf_size = 1.0f;

        auto result_scalar = voxel_downsample_scalar(input, leaf_size);
        auto result_rvv = voxel_downsample_rvv(input, leaf_size);

        REQUIRE(result_scalar.size() == result_rvv.size());
        REQUIRE(result_scalar.size() == 1); // Both in voxel [-1000,-999)
    }

    SECTION("Mixed extreme coordinates") {
        std::vector<Point3D> input = {Point3D(-1000.0f, -1000.0f, -1000.0f),
                                      Point3D(1000.0f, 1000.0f, 1000.0f),
                                      Point3D(0.0f, 0.0f, 0.0f)};
        float leaf_size = 100.0f;

        auto result_scalar = voxel_downsample_scalar(input, leaf_size);
        auto result_rvv = voxel_downsample_rvv(input, leaf_size);

        REQUIRE(result_scalar.size() == result_rvv.size());
    }
}

TEST_CASE("Voxel downsampling - duplicate points", "[voxel][edge]") {
    SECTION("All identical points") {
        std::vector<Point3D> input(100, Point3D(1.0f, 2.0f, 3.0f));
        float leaf_size = 1.0f;

        auto result_scalar = voxel_downsample_scalar(input, leaf_size);
        auto result_rvv = voxel_downsample_rvv(input, leaf_size);

        REQUIRE(result_scalar.size() == 1);
        REQUIRE(result_rvv.size() == 1);
        REQUIRE(result_scalar[0].approx_equal(Point3D(1.0f, 2.0f, 3.0f)));
        REQUIRE(result_rvv[0].approx_equal(Point3D(1.0f, 2.0f, 3.0f)));
    }

    SECTION("Multiple groups of duplicates") {
        std::vector<Point3D> input;
        for (int i = 0; i < 50; ++i)
            input.push_back(Point3D(0.0f, 0.0f, 0.0f));
        for (int i = 0; i < 50; ++i)
            input.push_back(Point3D(5.0f, 5.0f, 5.0f));
        float leaf_size = 1.0f;

        auto result_scalar = voxel_downsample_scalar(input, leaf_size);
        auto result_rvv = voxel_downsample_rvv(input, leaf_size);

        REQUIRE(result_scalar.size() == 2);
        REQUIRE(result_rvv.size() == 2);
    }
}

TEST_CASE("Voxel downsampling - precision edge cases", "[voxel][edge]") {
    SECTION("Points near voxel boundaries") {
        std::vector<Point3D> input = {
            Point3D(0.9999f, 0.9999f, 0.9999f), // Just below boundary
            Point3D(1.0000f, 1.0000f, 1.0000f), // On boundary
            Point3D(1.0001f, 1.0001f, 1.0001f)  // Just above boundary
        };
        float leaf_size = 1.0f;

        auto result_scalar = voxel_downsample_scalar(input, leaf_size);
        auto result_rvv = voxel_downsample_rvv(input, leaf_size);

        REQUIRE(result_scalar.size() == result_rvv.size());
    }

    SECTION("Very close points within same voxel") {
        std::vector<Point3D> input = {Point3D(0.5f, 0.5f, 0.5f), Point3D(0.5001f, 0.5001f, 0.5001f),
                                      Point3D(0.5002f, 0.5002f, 0.5002f)};
        float leaf_size = 1.0f;

        auto result_scalar = voxel_downsample_scalar(input, leaf_size);
        auto result_rvv = voxel_downsample_rvv(input, leaf_size);

        REQUIRE(result_scalar.size() == 1);
        REQUIRE(result_rvv.size() == 1);
    }
}

TEST_CASE("Voxel downsampling - variable density clouds", "[voxel][realistic]") {
    SECTION("Dense cluster with sparse outliers") {
        std::vector<Point3D> input;

        // Dense cluster around origin
        for (int i = 0; i < 1000; ++i) {
            input.emplace_back(static_cast<float>(rand() % 10) / 10.0f,
                               static_cast<float>(rand() % 10) / 10.0f,
                               static_cast<float>(rand() % 10) / 10.0f);
        }

        // Sparse outliers
        input.push_back(Point3D(100.0f, 100.0f, 100.0f));
        input.push_back(Point3D(-100.0f, -100.0f, -100.0f));

        float leaf_size = 1.0f;

        auto result_scalar = voxel_downsample_scalar(input, leaf_size);
        auto result_rvv = voxel_downsample_rvv(input, leaf_size);

        REQUIRE(result_scalar.size() == result_rvv.size());
        REQUIRE(result_scalar.size() > 2);            // At least the outliers + some from cluster
        REQUIRE(result_scalar.size() < input.size()); // Should have downsampled
    }
}

TEST_CASE("Voxel downsampling - different leaf sizes comparison", "[voxel][comparison]") {
    auto input = generate_random_cloud(10000, -50.0f, 50.0f);

    std::vector<float> leaf_sizes = {0.5f, 1.0f, 2.0f, 5.0f, 10.0f};

    for (float leaf_size : leaf_sizes) {
        auto result_scalar = voxel_downsample_scalar(input, leaf_size);
        auto result_rvv = voxel_downsample_rvv(input, leaf_size);

        // Same number of outputs
        REQUIRE(result_scalar.size() == result_rvv.size());

        // Sort and compare
        sort_points(result_scalar);
        sort_points(result_rvv);

        for (size_t i = 0; i < result_scalar.size(); ++i) {
            REQUIRE(result_scalar[i].approx_equal(result_rvv[i], 1e-4f));
        }
    }
}

TEST_CASE("Voxel downsampling - memory and stability", "[voxel][stability]") {
    SECTION("Multiple consecutive operations") {
        auto input = generate_random_cloud(10000, -50.0f, 50.0f);
        float leaf_size = 1.0f;

        // Run multiple times to check for memory issues
        for (int iter = 0; iter < 10; ++iter) {
            auto result_scalar = voxel_downsample_scalar(input, leaf_size);
            auto result_rvv = voxel_downsample_rvv(input, leaf_size);

            REQUIRE(result_scalar.size() == result_rvv.size());
            REQUIRE(result_scalar.size() > 0);
        }
    }

    SECTION("Chained downsampling") {
        auto input = generate_random_cloud(100000, -100.0f, 100.0f);

        // First pass with large leaf size
        auto result1 = voxel_downsample_scalar(input, 5.0f);
        REQUIRE(result1.size() > 0);
        REQUIRE(result1.size() < input.size());

        // Second pass with smaller leaf size on already downsampled cloud
        auto result2 = voxel_downsample_scalar(result1, 2.0f);
        REQUIRE(result2.size() > 0);
        REQUIRE(result2.size() <= result1.size());
    }
}
