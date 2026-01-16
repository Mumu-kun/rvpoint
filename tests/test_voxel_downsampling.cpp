/**
 * @file test_voxel_downsampling.cpp
 * @brief Unit tests for voxel grid downsampling implementations
 *
 * Tests both scalar and RVV implementations for correctness and equivalence.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

// Simple 3D point structure (matching the implementations)
struct Point3D {
    float x, y, z;

    Point3D() : x(0.0f), y(0.0f), z(0.0f) {}
    Point3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    bool approx_equal(const Point3D& other, float epsilon = 1e-5f) const {
        return std::abs(x - other.x) < epsilon &&
               std::abs(y - other.y) < epsilon &&
               std::abs(z - other.z) < epsilon;
    }
};

// External function declarations (implemented in separate files)
std::vector<Point3D> voxel_downsample_scalar(const std::vector<Point3D>& input, float leaf_size);
std::vector<Point3D> voxel_downsample_rvv(const std::vector<Point3D>& input, float leaf_size);

// Helper function to generate random point cloud
std::vector<Point3D> generate_random_cloud(size_t num_points, float min_coord = -100.0f, float max_coord = 100.0f, unsigned int seed = 42) {
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
    std::sort(points.begin(), points.end(),
        [](const Point3D& a, const Point3D& b) {
            if (std::abs(a.x - b.x) > 1e-5f) return a.x < b.x;
            if (std::abs(a.y - b.y) > 1e-5f) return a.y < b.y;
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
    std::vector<Point3D> input = { Point3D(1.0f, 2.0f, 3.0f) };

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
    std::vector<Point3D> input = {
        Point3D(0.1f, 0.2f, 0.3f),
        Point3D(0.4f, 0.5f, 0.6f)
    };
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
    std::vector<Point3D> input = {
        Point3D(0.5f, 0.5f, 0.5f),
        Point3D(1.5f, 1.5f, 1.5f)
    };
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
                input.emplace_back(
                    static_cast<float>(i) + 0.5f,
                    static_cast<float>(j) + 0.5f,
                    static_cast<float>(k) + 0.5f
                );
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
    std::vector<Point3D> input = {
        Point3D(-0.5f, -0.5f, -0.5f),
        Point3D(-1.5f, -1.5f, -1.5f),
        Point3D(-0.3f, -0.3f, -0.3f)
    };
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
    std::vector<Point3D> input = {
        Point3D(0.5f, 0.5f, 0.5f),
        Point3D(1.5f, 1.5f, 1.5f),
        Point3D(2.5f, 2.5f, 2.5f)
    };

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
    std::vector<Point3D> input = {
        Point3D(0.0f, 0.0f, 0.0f),
        Point3D(0.2f, 0.2f, 0.2f),
        Point3D(0.4f, 0.4f, 0.4f),
        Point3D(0.6f, 0.6f, 0.6f),
        Point3D(0.8f, 0.8f, 0.8f)
    };
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
