/**
 * @file benchmark_voxel_downsampling.cpp
 * @brief Benchmarks for voxel grid downsampling implementations
 *
 * Compares scalar vs RVV implementations using Google Benchmark.
 */

#include <benchmark/benchmark.h>
#include <random>
#include <vector>

// Simple 3D point structure
struct Point3D {
    float x, y, z;

    Point3D() : x(0.0f), y(0.0f), z(0.0f) {}
    Point3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

// External function declarations
std::vector<Point3D> voxel_downsample_scalar(const std::vector<Point3D>& input, float leaf_size);
std::vector<Point3D> voxel_downsample_rvv(const std::vector<Point3D>& input, float leaf_size);

// Helper to generate random point cloud
std::vector<Point3D> generate_random_cloud(size_t num_points, float min_coord = -100.0f, float max_coord = 100.0f) {
    static std::mt19937 gen(42); // Fixed seed for reproducibility
    std::uniform_real_distribution<float> dis(min_coord, max_coord);

    std::vector<Point3D> cloud;
    cloud.reserve(num_points);

    for (size_t i = 0; i < num_points; ++i) {
        cloud.emplace_back(dis(gen), dis(gen), dis(gen));
    }

    return cloud;
}

// Benchmark: Scalar implementation with varying point counts
static void BM_VoxelDownsample_Scalar(benchmark::State& state) {
    const size_t num_points = static_cast<size_t>(state.range(0));
    const float leaf_size = 1.0f;

    auto input = generate_random_cloud(num_points);

    for (auto _ : state) {
        auto result = voxel_downsample_scalar(input, leaf_size);
        benchmark::DoNotOptimize(result);
    }

    state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(num_points));
    state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(num_points) * sizeof(Point3D));
}

// Benchmark: RVV implementation with varying point counts
static void BM_VoxelDownsample_RVV(benchmark::State& state) {
    const size_t num_points = static_cast<size_t>(state.range(0));
    const float leaf_size = 1.0f;

    auto input = generate_random_cloud(num_points);

    for (auto _ : state) {
        auto result = voxel_downsample_rvv(input, leaf_size);
        benchmark::DoNotOptimize(result);
    }

    state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(num_points));
    state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(num_points) * sizeof(Point3D));
}

// Benchmark: Scalar implementation with varying leaf sizes
static void BM_VoxelDownsample_Scalar_LeafSize(benchmark::State& state) {
    const size_t num_points = 100000;
    const float leaf_size = static_cast<float>(state.range(0)) / 10.0f;

    auto input = generate_random_cloud(num_points);

    for (auto _ : state) {
        auto result = voxel_downsample_scalar(input, leaf_size);
        benchmark::DoNotOptimize(result);
    }

    state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(num_points));
}

// Benchmark: RVV implementation with varying leaf sizes
static void BM_VoxelDownsample_RVV_LeafSize(benchmark::State& state) {
    const size_t num_points = 100000;
    const float leaf_size = static_cast<float>(state.range(0)) / 10.0f;

    auto input = generate_random_cloud(num_points);

    for (auto _ : state) {
        auto result = voxel_downsample_rvv(input, leaf_size);
        benchmark::DoNotOptimize(result);
    }

    state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(num_points));
}

// Register benchmarks with different point counts
BENCHMARK(BM_VoxelDownsample_Scalar)
    ->Args({1000})
    ->Args({10000})
    ->Args({100000})
    ->Args({1000000})
    ->Unit(benchmark::kMillisecond);

BENCHMARK(BM_VoxelDownsample_RVV)
    ->Args({1000})
    ->Args({10000})
    ->Args({100000})
    ->Args({1000000})
    ->Unit(benchmark::kMillisecond);

// Register benchmarks with different leaf sizes
BENCHMARK(BM_VoxelDownsample_Scalar_LeafSize)
    ->Args({5})   // 0.5
    ->Args({10})  // 1.0
    ->Args({20})  // 2.0
    ->Args({50})  // 5.0
    ->Unit(benchmark::kMillisecond);

BENCHMARK(BM_VoxelDownsample_RVV_LeafSize)
    ->Args({5})   // 0.5
    ->Args({10})  // 1.0
    ->Args({20})  // 2.0
    ->Args({50})  // 5.0
    ->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
