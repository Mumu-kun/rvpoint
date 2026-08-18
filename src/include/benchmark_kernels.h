#pragma once

#include "rvv_pcl.h"

#include <cstddef>
#include <cstdint>
#include <string>

namespace rvv_pcl::bench {

enum class Kernel {
  L2,
  Reduction,
  Filter,
  Radius,
  Normal,
  Caravan,
};

PointCloudSoA makeDeterministicCloud(std::size_t size);
bool parseKernel(const std::string &name, Kernel &kernel);
const char *kernelName(Kernel kernel);
const char *modeName();

uint64_t runL2Distance(const PointCloudSoA &cloud);
uint64_t runReduction(const PointCloudSoA &cloud);
uint64_t runMaskedFilter(const PointCloudSoA &cloud);
uint64_t runRadiusSearch(const PointCloudSoA &cloud);
uint64_t runNormalEstimation(const PointCloudSoA &cloud);
uint64_t runCaravanRadiusSearch(const PointCloudSoA &cloud);

} // namespace rvv_pcl::bench