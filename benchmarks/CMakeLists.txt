cmake_minimum_required(VERSION 3.15)

# Benchmarking executable - Add your benchmark files here
# Example:
#
# add_executable(benchmark_suite
#     benchmark_main.cpp
# )
#
# target_link_libraries(benchmark_suite
#     PRIVATE pcl_riscv
# )
#
# target_compile_options(benchmark_suite PRIVATE -O3)

# ============================================================================
# Voxel Downsampling Benchmarks
# ============================================================================

add_executable(benchmark_voxel_downsampling
    benchmark_voxel_downsampling.cpp
    ${CMAKE_SOURCE_DIR}/src/voxel_downsampling_scalar.cpp
    ${CMAKE_SOURCE_DIR}/src/voxel_downsampling_rvv.cpp
)

target_link_libraries(benchmark_voxel_downsampling
    PRIVATE
        benchmark::benchmark
)

target_compile_options(benchmark_voxel_downsampling PRIVATE -O3)
