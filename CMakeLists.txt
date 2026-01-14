cmake_minimum_required(VERSION 3.15)
project(rvpoint
    VERSION 0.1.0
    DESCRIPTION "RV Point - Point Cloud Processing for RISC-V"
    LANGUAGES CXX
)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find dependencies via vcpkg
find_package(spdlog CONFIG REQUIRED)
find_package(tl-expected CONFIG REQUIRED)
find_package(Catch2 3 CONFIG REQUIRED)
find_package(benchmark CONFIG REQUIRED)

# Export compile commands for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options
option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_EXAMPLES "Build example applications" ON)
option(BUILD_BENCHMARKS "Build benchmarks" ON)
option(ENABLE_RVV "Enable RISC-V Vector extension" OFF)
option(USE_CUSTOM_ISA "Enable custom PCL instructions (xpcl)" OFF)

# Compiler flags
if(ENABLE_RVV)
    add_compile_options(-march=rv64gcv)
    add_compile_definitions(PCL_HAS_RVV)
    message(STATUS "RISC-V Vector extension enabled")
else()
    add_compile_options(-march=rv64gc)
    message(STATUS "Building scalar version (rv64gc)")
endif()

if(USE_CUSTOM_ISA)
    add_compile_definitions(PCL_HAS_CUSTOM_ISA)
    message(STATUS "Custom PCL instructions enabled (requires modified Spike)")
endif()

# Warning flags
add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion
    -Wsign-conversion
)

# Optimization flags for Release build
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3 -ffast-math)
endif()

# Library target will be added here by students
# add_library(pcl_riscv INTERFACE)
# target_include_directories(pcl_riscv INTERFACE
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
# )

# Tests
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Examples
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Benchmarks
if(BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()

# Installation (uncomment when library is implemented)
# install(TARGETS pcl_riscv EXPORT pcl_riscv-targets)
# install(DIRECTORY include/ DESTINATION include)
# install(EXPORT pcl_riscv-targets
#     FILE pcl_riscv-targets.cmake
#     NAMESPACE pcl_riscv::
#     DESTINATION lib/cmake/pcl_riscv
# )

# Print configuration summary
message(STATUS "")
message(STATUS "PCL-RISC-V Configuration Summary:")
message(STATUS "  Version:          ${PROJECT_VERSION}")
message(STATUS "  Build Type:       ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard:     ${CMAKE_CXX_STANDARD}")
message(STATUS "  RVV Enabled:      ${ENABLE_RVV}")
message(STATUS "  Custom ISA:       ${USE_CUSTOM_ISA}")
message(STATUS "  Build Tests:      ${BUILD_TESTS}")
message(STATUS "  Build Examples:   ${BUILD_EXAMPLES}")
message(STATUS "  Build Benchmarks: ${BUILD_BENCHMARKS}")
message(STATUS "")
