cmake_minimum_required(VERSION 3.16)
project(RVPoint LANGUAGES C CXX)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Static linking for QEMU compatibility
set(CMAKE_EXE_LINKER_FLAGS "-static")
option(RVV_PCL_USE_RVV "Enable RVV helper backend" ON)
option(ENABLE_PROFILING "Enable diagnostic profiling instrumentation" OFF)
option(GEM5_BUILD "Enable gem5 simulation compatibility fallbacks" OFF)
set(RISCV_ARCH "rv64gcv" CACHE STRING "RISC-V ISA string, e.g. rv64gc or rv64gcv")
set(RISCV_ABI "lp64d" CACHE STRING "RISC-V ABI")

# RISC-V Specific Flags
set(RISCV_FLAGS "-march=${RISCV_ARCH} -mabi=${RISCV_ABI}")
set(RVV_PCL_ENABLE_RVV OFF)
if(RVV_PCL_USE_RVV AND RISCV_ARCH MATCHES ".*v.*")
    set(RVV_PCL_ENABLE_RVV ON)
    set(BACKEND_SUBDIR "rvv")
else()
    set(BACKEND_SUBDIR "scalar")
endif()

if(ENABLE_PROFILING)
    add_compile_definitions(RVPOINT_ENABLE_PROFILING=1)
endif()

# Set output directory for executables (e.g. build/bin/rvv or build/bin/scalar)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/${BACKEND_SUBDIR}")

message(STATUS "Compiler Flags: ${CMAKE_CXX_FLAGS}")
message(STATUS "RISC-V ISA: ${RISCV_ARCH}")
message(STATUS "RISC-V ABI: ${RISCV_ABI}")
message(STATUS "RVV backend enabled: ${RVV_PCL_ENABLE_RVV}")
message(STATUS "GEM5_BUILD enabled: ${GEM5_BUILD}")
message(STATUS "Binary output dir: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

# 1. The Library (rvpoint)
list(APPEND SOURCES
    "src/core/rvv_common.cpp"
    "src/filters/voxel_grid_downsamp.cpp"
    "src/filters/statistical_outlier_removal.cpp"
    "src/features/normal_estimation.cpp"
    "src/segmentation/ransac_plane.cpp"
    "src/segmentation/euclidean_clustering.cpp"
    "src/search/octree.cpp"
    "src/search/spatial_hashing.cpp"
    "src/search/radius_search.cpp"
    "src/search/caravan_radius_search.cpp"
    "src/search/pointer_octree.cpp"
    "src/search/caravan_pointer_octree.cpp"
    "src/tools/benchmark_kernels.cpp"
)
add_library(rvpoint ${SOURCES})
add_library(rvvpcl ALIAS rvpoint)
target_include_directories(rvpoint PUBLIC "${CMAKE_SOURCE_DIR}/src")
target_compile_options(rvpoint PUBLIC "-O3" "-march=${RISCV_ARCH}" "-mabi=${RISCV_ABI}")
if(RVV_PCL_ENABLE_RVV)
    target_compile_definitions(rvpoint PUBLIC RVV_PCL_USE_RVV=1)
endif()
if(GEM5_BUILD)
    target_compile_definitions(rvpoint PUBLIC GEM5_BUILD=1)
endif()

# Helper: add a test target from a source file
function(add_rvv_test src)
    get_filename_component(name "${src}" NAME_WE)
    add_executable(${name} "${src}")
    target_link_libraries(${name} PRIVATE rvpoint)
    target_compile_options(${name} PRIVATE "-fopenmp")
    target_link_options(${name} PRIVATE "-fopenmp")
    target_include_directories(${name} PRIVATE "${CMAKE_SOURCE_DIR}/src")
endfunction()

# 2. Test Executables

# --- Always-built tests (no RVV dependency) ---
file(GLOB ALWAYS_TEST_SOURCES "tests/always/*.c" "tests/always/*.cpp")
foreach(src ${ALWAYS_TEST_SOURCES})
    add_rvv_test("${src}")
endforeach()

# --- Unit tests (algorithm layer) ---
file(GLOB UNIT_TEST_SOURCES "tests/unit/*.cpp")
foreach(src ${UNIT_TEST_SOURCES})
    add_rvv_test("${src}")
endforeach()

# --- Integration tests ---
file(GLOB INTEGRATION_TEST_SOURCES "tests/integration/*.cpp")
foreach(src ${INTEGRATION_TEST_SOURCES})
    add_rvv_test("${src}")
endforeach()

# --- RVV hardware verification (conditional on vector extension) ---
if(RVV_PCL_ENABLE_RVV)
    file(GLOB RVV_TEST_SOURCES "tests/rvv/*.c")
    foreach(src ${RVV_TEST_SOURCES})
        add_rvv_test("${src}")
    endforeach()
endif()

# --- Support tests (I/O, data loading) ---
file(GLOB SUPPORT_TEST_SOURCES "tests/support/*.cpp")
foreach(src ${SUPPORT_TEST_SOURCES})
    add_rvv_test("${src}")
endforeach()

# --- Benchmark ---
file(GLOB BENCHMARK_SOURCES "tests/benchmark_*.cpp")
foreach(src ${BENCHMARK_SOURCES})
    get_filename_component(name "${src}" NAME_WE)
    add_executable(${name} "${src}")
    target_include_directories(${name} PRIVATE "${CMAKE_SOURCE_DIR}/src")
    target_compile_options(${name} PRIVATE "-O3" "-march=${RISCV_ARCH}" "-mabi=${RISCV_ABI}")
    target_link_libraries(${name} PRIVATE rvpoint)
endforeach()

# --- Benchmarks in tests/bench ---
file(GLOB BENCH_DIR_SOURCES "tests/bench/*.cpp")
foreach(src ${BENCH_DIR_SOURCES})
    get_filename_component(name "${src}" NAME_WE)
    add_executable(${name} "${src}")
    target_include_directories(${name} PRIVATE "${CMAKE_SOURCE_DIR}/src" "${CMAKE_SOURCE_DIR}/tests/bench")
    target_compile_options(${name} PRIVATE "-O3" "-march=${RISCV_ARCH}" "-mabi=${RISCV_ABI}")
    target_link_libraries(${name} PRIVATE rvpoint)
endforeach()

# Pipeline Export Utility
add_executable(pipeline_export src/tools/pipeline_export.cpp)
target_include_directories(pipeline_export PRIVATE "${CMAKE_SOURCE_DIR}/src")
target_link_libraries(pipeline_export PRIVATE rvpoint)

# Ablation Benchmark Utility
add_executable(ablation_bench src/tools/ablation_bench.cpp)
target_include_directories(ablation_bench PRIVATE "${CMAKE_SOURCE_DIR}/src")
target_link_libraries(ablation_bench PRIVATE rvpoint)

# Neighbor Search & 4 Caravan Strategies Benchmark Tool
add_executable(neighbor_search_sor_bench src/tools/neighbor_search_sor_bench.cpp)
target_include_directories(neighbor_search_sor_bench PRIVATE "${CMAKE_SOURCE_DIR}/src")
target_link_libraries(neighbor_search_sor_bench PRIVATE rvpoint)

# Stage-by-Stage Scalar vs RVV Verification Benchmark
add_executable(pipeline_compare_bench src/tools/pipeline_compare_bench.cpp)
target_include_directories(pipeline_compare_bench PRIVATE "${CMAKE_SOURCE_DIR}/src")
target_link_libraries(pipeline_compare_bench PRIVATE rvpoint)

# Standalone Pure-Scalar PCL Pipeline for RISC-V QEMU Simulation
add_executable(pcl_standalone_pipeline src/tools/pcl_standalone_pipeline.cpp)
target_include_directories(pcl_standalone_pipeline PRIVATE "${CMAKE_SOURCE_DIR}/src")
target_link_libraries(pcl_standalone_pipeline PRIVATE rvpoint)

# Ultra-Fast Flat 2.5D Real-Time Perception Pipeline Export Utility
add_executable(pipeline_fast_export src/tools/pipeline_fast_export.cpp)
target_include_directories(pipeline_fast_export PRIVATE "${CMAKE_SOURCE_DIR}/src")
target_link_libraries(pipeline_fast_export PRIVATE rvpoint)

# Hardware-Accelerated RVV 1.0 Ultra Fast Pipeline
add_executable(pipeline_rvv_ultra_fast src/tools/pipeline_rvv_ultra_fast.cpp)
target_include_directories(pipeline_rvv_ultra_fast PRIVATE "${CMAKE_SOURCE_DIR}/src")
target_link_libraries(pipeline_rvv_ultra_fast PRIVATE rvpoint)

# Pure SoA Turbo Hardware RVV 1.0 Pipeline
add_executable(pipeline_rvv_turbo src/tools/pipeline_rvv_turbo.cpp)
target_include_directories(pipeline_rvv_turbo PRIVATE "${CMAKE_SOURCE_DIR}/src")
target_link_libraries(pipeline_rvv_turbo PRIVATE rvpoint)

# Official Reference Open-Source Scalar 2.5D Baseline (No RVV)
add_executable(scalar_25d_baseline src/tools/scalar_25d_baseline.cpp)
target_include_directories(scalar_25d_baseline PRIVATE "${CMAKE_SOURCE_DIR}/src")
target_link_libraries(scalar_25d_baseline PRIVATE rvpoint)

# Accelerated True 3D RVPoint Turbo 10-Stage Pipeline
add_executable(pipeline_3d_turbo src/tools/pipeline_3d_turbo.cpp)
target_include_directories(pipeline_3d_turbo PRIVATE "${CMAKE_SOURCE_DIR}/src")
target_link_libraries(pipeline_3d_turbo PRIVATE rvpoint)