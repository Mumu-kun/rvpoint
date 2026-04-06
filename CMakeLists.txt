cmake_minimum_required(VERSION 3.16)
project(RVVPCL LANGUAGES C CXX)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set output directory for executables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
file(MAKE_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

# Static linking for QEMU compatibility
set(CMAKE_EXE_LINKER_FLAGS "-static")
option(RVV_PCL_USE_RVV "Enable RVV helper backend" ON)
set(RISCV_ARCH "rv64gcv" CACHE STRING "RISC-V ISA string, e.g. rv64gc or rv64gcv")
set(RISCV_ABI "lp64d" CACHE STRING "RISC-V ABI")

# RISC-V Specific Flags
set(RISCV_FLAGS "-march=${RISCV_ARCH} -mabi=${RISCV_ABI}")
set(RVV_PCL_ENABLE_RVV OFF)
if(RVV_PCL_USE_RVV AND RISCV_ARCH MATCHES ".*v.*")
    set(RVV_PCL_ENABLE_RVV ON)
elseif(RVV_PCL_USE_RVV)
    message(WARNING "RVV requested, but RISCV_ARCH='${RISCV_ARCH}' has no vector extension. Building scalar fallback.")
endif()
message(STATUS "Compiler Flags: ${CMAKE_CXX_FLAGS}")
message(STATUS "RISC-V ISA: ${RISCV_ARCH}")
message(STATUS "RISC-V ABI: ${RISCV_ABI}")
message(STATUS "RVV backend enabled: ${RVV_PCL_ENABLE_RVV}")

# 1. The Library (rvvpcl)
list(APPEND SOURCES 
    "src/point_cloud.cpp"
    "src/voxel_grid_downsamp.cpp"
    "src/rvv_common.cpp"
    "src/statistical_outlier_removal.cpp"
    "src/normal_estimation.cpp"
    "src/ransac_plane.cpp"
    "src/octree.cpp"
    "src/spatial_hashing.cpp"
)
# Note: placeholder.c removed, real source used.
add_library(rvvpcl ${SOURCES})
target_include_directories(rvvpcl PUBLIC src/include)
target_compile_options(rvvpcl PUBLIC "-march=${RISCV_ARCH}" "-mabi=${RISCV_ABI}")
if(RVV_PCL_ENABLE_RVV)
    target_compile_definitions(rvvpcl PUBLIC RVV_PCL_USE_RVV=1)
endif()


# 2. Test Executables
# Scalar Test
add_executable(test_scalar tests/test_scalar.c)
target_link_libraries(test_scalar PRIVATE rvvpcl)

# Vector-specific tests
if(RVV_PCL_ENABLE_RVV)
    add_executable(test_vector tests/test_vector.c)
    target_link_libraries(test_vector PRIVATE rvvpcl)

    if(EXISTS "${CMAKE_SOURCE_DIR}/tests/rvv_test.c")
        add_executable(rvv_test tests/rvv_test.c)
        target_link_libraries(rvv_test PRIVATE rvvpcl)
    endif()
endif()

# Voxel Grid Test
add_executable(test_voxel_grid tests/test_voxel_grid.cpp)
target_link_libraries(test_voxel_grid PRIVATE rvvpcl)

# SOR Test
add_executable(test_sor tests/test_sor.cpp)
target_link_libraries(test_sor PRIVATE rvvpcl)

# Normal Estimation Test
add_executable(test_normal tests/test_normal.cpp)
target_link_libraries(test_normal PRIVATE rvvpcl)

# Radius Search Test
add_executable(test_radius tests/test_radius.cpp)
target_link_libraries(test_radius PRIVATE rvvpcl)

# Octree Test (NEW)
add_executable(test_octree tests/test_octree.cpp)
target_link_libraries(test_octree PRIVATE rvvpcl)

# PCL Walkthrough Test (NEW)
add_executable(test_pipeline_walkthrough tests/test_pipeline_walkthrough.cpp)
target_link_libraries(test_pipeline_walkthrough PRIVATE rvvpcl)

# Spatial Hash Comparison Test
add_executable(test_spatial_hash_comparison tests/test_spatial_hash_comparison.cpp)
target_link_libraries(test_spatial_hash_comparison PRIVATE rvvpcl)

# RANSAC Test
add_executable(test_ransac tests/test_ransac.cpp)
target_link_libraries(test_ransac PRIVATE rvvpcl)

# Benchmark Suite
add_executable(benchmark tests/benchmark.cpp)
target_link_libraries(benchmark PRIVATE rvvpcl)

# Pipeline Export Utility
add_executable(pipeline_export tools/pipeline_export.cpp)
target_link_libraries(pipeline_export PRIVATE rvvpcl)
