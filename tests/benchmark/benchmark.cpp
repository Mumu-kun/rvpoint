#include "benchmark_kernels.h"

#include <cstddef>
#include <cstdint>
#include <exception>
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
  std::cerr << "Usage: benchmark <mode> <kernel> <size>\n"
            << "  mode   : scalar|rvv\n"
            << "  kernel : l2|reduction|filter|radius|normal\n"
            << "  size   : integer or k-suffixed value such as 1k, 10k, 100k\n";
}

} // namespace

int main(int argc, char **argv) {
  if (argc != 4) {
    printUsage();
    return 1;
  }

  const std::string requested_mode = argv[1];
  const std::string kernel_token = argv[2];
  const std::string size_token = argv[3];

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

  std::uint64_t checksum = 0;
  switch (kernel) {
  case rvv_pcl::bench::Kernel::L2:
    checksum = rvv_pcl::bench::runL2Distance(cloud);
    break;
  case rvv_pcl::bench::Kernel::Reduction:
    checksum = rvv_pcl::bench::runReduction(cloud);
    break;
  case rvv_pcl::bench::Kernel::Filter:
    checksum = rvv_pcl::bench::runMaskedFilter(cloud);
    break;
  case rvv_pcl::bench::Kernel::Radius:
    checksum = rvv_pcl::bench::runRadiusSearch(cloud);
    break;
  case rvv_pcl::bench::Kernel::Normal:
    checksum = rvv_pcl::bench::runNormalEstimation(cloud);
    break;
  }

  benchmark_sink ^= checksum;
  return 0;
}

