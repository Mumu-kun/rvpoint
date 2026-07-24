#include "benchmark_kernels.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

volatile std::uint64_t benchmark_sink = 0;

bool parseSize(const std::string &token, std::size_t &size) {
  try {
    std::size_t consumed = 0;
    const unsigned long long parsed = std::stoull(token, &consumed, 10);
    if (consumed == token.size()) {
      size = static_cast<std::size_t>(parsed);
      return true;
    }
    if (consumed + 1 == token.size() && (token.back() == 'k' || token.back() == 'K')) {
      size = static_cast<std::size_t>(parsed) * 1000u;
      return true;
    }
  } catch (const std::exception &) {
    return false;
  }
  return false;
}

void printUsage() {
  std::cerr << "Usage: benchmark <mode> <kernel> <size> [iterations]\n"
            << "  mode       : scalar|rvv\n"
            << "  kernel     : l2|reduction|filter|radius|normal|caravan\n"
            << "  size       : integer or k-suffixed value such as 1k, 10k, 100k\n"
            << "  iterations : number of benchmark iterations (default: 10)\n";
}

std::uint64_t executeKernel(rvv_pcl::bench::Kernel kernel, const rvv_pcl::PointCloudSoA &cloud) {
  switch (kernel) {
  case rvv_pcl::bench::Kernel::L2:
    return rvv_pcl::bench::runL2Distance(cloud);
  case rvv_pcl::bench::Kernel::Reduction:
    return rvv_pcl::bench::runReduction(cloud);
  case rvv_pcl::bench::Kernel::Filter:
    return rvv_pcl::bench::runMaskedFilter(cloud);
  case rvv_pcl::bench::Kernel::Radius:
    return rvv_pcl::bench::runRadiusSearch(cloud);
  case rvv_pcl::bench::Kernel::Normal:
    return rvv_pcl::bench::runNormalEstimation(cloud);
  case rvv_pcl::bench::Kernel::Caravan:
    return rvv_pcl::bench::runCaravanRadiusSearch(cloud);
  }
  return 0;
}

} // namespace

int main(int argc, char **argv) {
  if (argc < 4 || argc > 5) {
    printUsage();
    return 1;
  }

  const std::string requested_mode = argv[1];
  const std::string kernel_token = argv[2];
  const std::string size_token = argv[3];
  std::size_t iterations = 10;

  if (argc == 5) {
    try {
      iterations = std::stoull(argv[4]);
      if (iterations == 0) iterations = 1;
    } catch (...) {
      std::cerr << "invalid iterations count: " << argv[4] << "\n";
      return 1;
    }
  }

  if (requested_mode != rvv_pcl::bench::modeName()) {
    std::cerr << "benchmark built for mode '" << rvv_pcl::bench::modeName()
              << "' but requested '" << requested_mode << "'\n";
    return 1;
  }

  rvv_pcl::bench::Kernel kernel;
  if (!rvv_pcl::bench::parseKernel(kernel_token, kernel)) {
    std::cerr << "unknown kernel: " << kernel_token << "\n";
    printUsage();
    return 1;
  }

  std::size_t size = 0;
  if (!parseSize(size_token, size) || size == 0) {
    std::cerr << "invalid size: " << size_token << "\n";
    printUsage();
    return 1;
  }

  const rvv_pcl::PointCloudSoA cloud = rvv_pcl::bench::makeDeterministicCloud(size);

  // Warmup run
  std::uint64_t checksum = executeKernel(kernel, cloud);

  // Timed iterations
  auto start_time = std::chrono::high_resolution_clock::now();
  for (std::size_t iter = 0; iter < iterations; ++iter) {
    checksum ^= executeKernel(kernel, cloud);
  }
  auto end_time = std::chrono::high_resolution_clock::now();

  const auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
  const double avg_us = static_cast<double>(elapsed_us) / static_cast<double>(iterations);

  std::cout << "[BENCHMARK] Mode: " << requested_mode
            << " | Kernel: " << rvv_pcl::bench::kernelName(kernel)
            << " | Size: " << size
            << " | Iterations: " << iterations
            << " | Total: " << elapsed_us << " us"
            << " | Avg: " << std::fixed << std::setprecision(2) << avg_us << " us/iter\n";

  std::cout << "BENCHMARK_RESULT mode=" << requested_mode
            << " kernel=" << rvv_pcl::bench::kernelName(kernel)
            << " size=" << size
            << " iterations=" << iterations
            << " time_us=" << elapsed_us
            << " avg_us=" << std::fixed << std::setprecision(2) << avg_us
            << " checksum=" << checksum << "\n";

  benchmark_sink ^= checksum;
  return 0;
}

