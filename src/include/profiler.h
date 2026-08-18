#pragma once

#include <chrono>
#include <iostream>
#include <string>

namespace rvv_pcl {

#ifdef RVPOINT_ENABLE_PROFILING

class ScopedTimer {
public:
  explicit ScopedTimer(const char *name)
      : name_(name), start_(std::chrono::high_resolution_clock::now()) {}

  ~ScopedTimer() {
    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start_).count();
    std::cout << "[profile] " << name_ << ": " << ms << " ms" << std::endl;
  }

private:
  const char *name_;
  std::chrono::high_resolution_clock::time_point start_;
};

#define RVPOINT_PROFILE_SCOPE(name)                                            \
  ::rvv_pcl::ScopedTimer _timer_##__LINE__(name)

#else

#define RVPOINT_PROFILE_SCOPE(name) ((void)0)

#endif

} // namespace rvv_pcl
