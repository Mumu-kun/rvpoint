#pragma once

#include "core/point_types.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rvpoint::bench {

enum class Kernel {
  L2,
  Reduction,
  Filter,
  Radius,
  Normal,
  Caravan,
};

struct BenchmarkCloudHolder {
  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> z;
  PointCloudSoA soa;
};

BenchmarkCloudHolder makeDeterministicCloud(std::size_t size);
bool parseKernel(const std::string &name, Kernel &kernel);
const char *kernelName(Kernel kernel);
const char *modeName();

uint64_t runL2Distance(const PointCloudSoA &cloud);
uint64_t runReduction(const PointCloudSoA &cloud);
uint64_t runMaskedFilter(const PointCloudSoA &cloud);
uint64_t runRadiusSearch(const PointCloudSoA &cloud);
uint64_t runNormalEstimation(const PointCloudSoA &cloud);
uint64_t runCaravanRadiusSearch(const PointCloudSoA &cloud);

} // namespace rvpoint::bench
