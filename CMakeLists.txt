cmake_minimum_required(VERSION 3.16)
project(RVVPCL LANGUAGES C CXX)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# RISC-V Specific Flags
# These are critical for RVV support
set(RISCV_FLAGS "-march=rv64gcv -mabi=lp64d")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${RISCV_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${RISCV_FLAGS}")

# Ensure we use the correct toolchain paths if not set externally
# But usually best to let the environment (CC/CXX vars) handle it.
# We will just print them to be sure.
message(STATUS "Compiler Flags: ${CMAKE_CXX_FLAGS}")

# 1. The Library (rvvpcl)
list(APPEND SOURCES 
    "src/voxel_grid_downsamp.cpp"
    "src/rvv_common.cpp"
    "src/statistical_outlier_removal.cpp"
    "src/normal_estimation.cpp"
    "src/radius_search.cpp"
    "src/ransac_plane.cpp"
    "src/octree.cpp" 
)
# Note: placeholder.c removed, real source used.
add_library(rvvpcl ${SOURCES})
target_include_directories(rvvpcl PUBLIC src/include)


# 2. Test Executables
# Scalar Test
add_executable(test_scalar tests/test_scalar.c)
target_link_libraries(test_scalar PRIVATE rvvpcl)

# Vector Test
add_executable(test_vector tests/test_vector.c)
target_link_libraries(test_vector PRIVATE rvvpcl)

# Rvv Test (Main test file)
if(EXISTS "${CMAKE_SOURCE_DIR}/tests/rvv_test.c")
    add_executable(rvv_test tests/rvv_test.c)
    target_link_libraries(rvv_test PRIVATE rvvpcl)
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

# RANSAC Test
add_executable(test_ransac tests/test_ransac.cpp)
target_link_libraries(test_ransac PRIVATE rvvpcl)

# Benchmark Suite
add_executable(benchmark tests/benchmark.cpp)
target_link_libraries(benchmark PRIVATE rvvpcl)
