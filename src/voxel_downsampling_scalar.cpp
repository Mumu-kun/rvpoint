/**
 * @file voxel_downsampling_scalar.cpp
 * @brief Scalar implementation of voxel grid downsampling
 *
 * This implementation uses a hash-map based approach to group points by voxel
 * and compute centroids for each voxel. This is the baseline scalar version
 * for comparison with the RVV-vectorized implementation.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <unordered_map>
#include <vector>

// Simple 3D point structure
struct Point3D {
    float x, y, z;

    Point3D() : x(0.0f), y(0.0f), z(0.0f) {}
    Point3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

// Voxel key for hash map - represents (i,j,k) coordinates
struct VoxelKey {
    int32_t i, j, k;

    VoxelKey(int32_t i_, int32_t j_, int32_t k_) : i(i_), j(j_), k(k_) {}

    bool operator==(const VoxelKey& other) const {
        return i == other.i && j == other.j && k == other.k;
    }
};

// Hash function for VoxelKey
struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const {
        // Simple hash combining the three coordinates
        std::size_t h1 = std::hash<int32_t>{}(key.i);
        std::size_t h2 = std::hash<int32_t>{}(key.j);
        std::size_t h3 = std::hash<int32_t>{}(key.k);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

// Accumulator for computing centroid
struct VoxelAccumulator {
    float sum_x, sum_y, sum_z;
    uint32_t count;

    VoxelAccumulator() : sum_x(0.0f), sum_y(0.0f), sum_z(0.0f), count(0) {}

    void add_point(const Point3D& p) {
        sum_x += p.x;
        sum_y += p.y;
        sum_z += p.z;
        count++;
    }

    Point3D get_centroid() const {
        if (count == 0) return Point3D();
        float inv_count = 1.0f / static_cast<float>(count);
        return Point3D(sum_x * inv_count, sum_y * inv_count, sum_z * inv_count);
    }
};

/**
 * @brief Voxel grid downsampling - scalar implementation
 *
 * @param input Input point cloud
 * @param leaf_size Voxel size (same for all dimensions)
 * @return Downsampled point cloud
 */
std::vector<Point3D> voxel_downsample_scalar(
    const std::vector<Point3D>& input,
    float leaf_size)
{
    if (input.empty() || leaf_size <= 0.0f) {
        return std::vector<Point3D>();
    }

    // Inverse of leaf size for division → multiplication optimization
    const float inv_leaf = 1.0f / leaf_size;

    // Hash map to group points by voxel
    std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxel_map;
    voxel_map.reserve(input.size() / 4); // Estimate for typical downsampling

    // Step 1: Assign each point to a voxel and accumulate
    for (const auto& point : input) {
        // Compute voxel indices (i, j, k)
        // Using floor division: i = floor(x / leaf)
        int32_t i = static_cast<int32_t>(std::floor(point.x * inv_leaf));
        int32_t j = static_cast<int32_t>(std::floor(point.y * inv_leaf));
        int32_t k = static_cast<int32_t>(std::floor(point.z * inv_leaf));

        VoxelKey key(i, j, k);

        // Add point to voxel accumulator
        voxel_map[key].add_point(point);
    }

    // Step 2: Compute centroid for each voxel
    std::vector<Point3D> output;
    output.reserve(voxel_map.size());

    for (const auto& pair : voxel_map) {
        output.push_back(pair.second.get_centroid());
    }

    return output;
}

/**
 * @brief Print statistics about the downsampling
 */
static void print_stats(const std::vector<Point3D>& input,
                const std::vector<Point3D>& output,
                double time_ms) {
    std::cout << "Scalar Implementation Statistics:\n";
    std::cout << "  Input points:  " << input.size() << "\n";
    std::cout << "  Output points: " << output.size() << "\n";
    std::cout << "  Reduction:     " << (100.0 * (1.0 - static_cast<double>(output.size()) / input.size())) << "%\n";
    std::cout << "  Time:          " << time_ms << " ms\n";
    std::cout << "  Throughput:    " << (input.size() / (time_ms / 1000.0) / 1e6) << " Mpoints/s\n";
}

// Test harness
#ifdef BUILD_STANDALONE
#include <chrono>
#include <random>

std::vector<Point3D> generate_random_cloud(size_t num_points, float min_coord = -100.0f, float max_coord = 100.0f) {
    std::vector<Point3D> cloud;
    cloud.reserve(num_points);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(min_coord, max_coord);

    for (size_t i = 0; i < num_points; ++i) {
        cloud.emplace_back(dis(gen), dis(gen), dis(gen));
    }

    return cloud;
}

int main(int argc, char** argv) {
    // Parse command line arguments
    size_t num_points = 1000000;  // 1M points default
    float leaf_size = 1.0f;
    int num_iterations = 5;

    if (argc > 1) num_points = std::stoull(argv[1]);
    if (argc > 2) leaf_size = std::stof(argv[2]);
    if (argc > 3) num_iterations = std::stoi(argv[3]);

    std::cout << "Voxel Grid Downsampling - Scalar Implementation\n";
    std::cout << "================================================\n\n";
    std::cout << "Configuration:\n";
    std::cout << "  Number of points: " << num_points << "\n";
    std::cout << "  Leaf size:        " << leaf_size << "\n";
    std::cout << "  Iterations:       " << num_iterations << "\n\n";

    // Generate test data
    std::cout << "Generating random point cloud...\n";
    auto input_cloud = generate_random_cloud(num_points);

    // Warm-up run
    std::cout << "Warming up...\n";
    auto result = voxel_downsample_scalar(input_cloud, leaf_size);

    // Benchmark runs
    std::cout << "\nRunning benchmark...\n";
    std::vector<double> times;
    times.reserve(static_cast<size_t>(num_iterations));

    for (int i = 0; i < num_iterations; ++i) {
        auto start = std::chrono::high_resolution_clock::now();
        result = voxel_downsample_scalar(input_cloud, leaf_size);
        auto end = std::chrono::high_resolution_clock::now();

        double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
        times.push_back(elapsed_ms);
        std::cout << "  Run " << (i + 1) << ": " << elapsed_ms << " ms\n";
    }

    // Compute statistics
    double sum = 0.0;
    for (double t : times) sum += t;
    double avg_time = sum / times.size();

    double variance = 0.0;
    for (double t : times) {
        double diff = t - avg_time;
        variance += diff * diff;
    }
    double std_dev = std::sqrt(variance / times.size());

    std::cout << "\n";
    print_stats(input_cloud, result, avg_time);
    std::cout << "  Std dev:       " << std_dev << " ms\n";

    // Verify output sanity
    std::cout << "\nOutput validation:\n";
    if (!result.empty()) {
        std::cout << "  First point:  (" << result[0].x << ", " << result[0].y << ", " << result[0].z << ")\n";
        std::cout << "  Last point:   (" << result.back().x << ", " << result.back().y << ", " << result.back().z << ")\n";
    }

    return 0;
}
#endif
