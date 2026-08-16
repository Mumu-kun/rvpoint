cmake_minimum_required(VERSION 3.16)
project(RVVPCL LANGUAGES C CXX)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Static linking for QEMU compatibility
set(CMAKE_EXE_LINKER_FLAGS "-static")
option(RVV_PCL_USE_RVV "Enable RVV helper backend" ON)
option(ENABLE_PROFILING "Enable diagnostic profiling instrumentation" OFF)
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
message(STATUS "Binary output dir: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

# 1. The Library (rvvpcl)
list(APPEND SOURCES
    "src/voxel_grid_downsamp.cpp"
    "src/rvv_common.cpp"
    "src/statistical_outlier_removal.cpp"
    "src/normal_estimation.cpp"
    "src/ransac_plane.cpp"
    "src/euclidean_clustering.cpp"
    "src/neighbor_search/octree.cpp"
    "src/neighbor_search/spatial_hashing.cpp"
    "src/neighbor_search/radius_search.cpp"
    "src/neighbor_search/caravan_radius_search.cpp"
    "src/neighbor_search/pointer_octree.cpp"
    "src/neighbor_search/caravan_pointer_octree.cpp"
    "src/registration/icp_registration.cpp"
    "src/registration/tracking_pipeline.cpp"
)
add_library(rvvpcl ${SOURCES})
target_include_directories(rvvpcl PUBLIC "${CMAKE_SOURCE_DIR}/src/include" "${CMAKE_SOURCE_DIR}/src" "${CMAKE_SOURCE_DIR}/src/registration")
target_compile_options(rvvpcl PUBLIC "-O3" "-march=${RISCV_ARCH}" "-mabi=${RISCV_ABI}")
if(RVV_PCL_ENABLE_RVV)
    target_compile_definitions(rvvpcl PUBLIC RVV_PCL_USE_RVV=1)
endif()

# Helper: add a test target from a source file
function(add_rvv_test src)
    get_filename_component(name "${src}" NAME_WE)
    add_executable(${name} "${src}")
    target_link_libraries(${name} PRIVATE rvvpcl)
    target_include_directories(${name} PRIVATE "${CMAKE_SOURCE_DIR}/src/include" "${CMAKE_SOURCE_DIR}/src")
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
    target_include_directories(${name} PRIVATE "${CMAKE_SOURCE_DIR}/src/include" "${CMAKE_SOURCE_DIR}/src")
    target_compile_options(${name} PRIVATE "-O3" "-march=${RISCV_ARCH}" "-mabi=${RISCV_ABI}")
    target_link_libraries(${name} PRIVATE rvvpcl)
endforeach()

# Pipeline Export Utility
add_executable(pipeline_export src/tools/pipeline_export.cpp)
target_include_directories(pipeline_export PRIVATE "${CMAKE_SOURCE_DIR}/src/include")
target_link_libraries(pipeline_export PRIVATE rvvpcl)

# Ablation Benchmark Utility
add_executable(ablation_bench src/tools/ablation_bench.cpp)
target_include_directories(ablation_bench PRIVATE "${CMAKE_SOURCE_DIR}/src/include" "${CMAKE_SOURCE_DIR}/src")
target_link_libraries(ablation_bench PRIVATE rvvpcl)

# Neighbor Search & 4 Caravan Strategies Benchmark Tool
add_executable(neighbor_search_sor_bench src/tools/neighbor_search_sor_bench.cpp)
target_include_directories(neighbor_search_sor_bench PRIVATE "${CMAKE_SOURCE_DIR}/src/include" "${CMAKE_SOURCE_DIR}/src")
target_link_libraries(neighbor_search_sor_bench PRIVATE rvvpcl)

# Tracking Mode Benchmark Tool
add_executable(tracking_mode_bench tests/benchmark/tracking_mode_bench.cpp)
target_include_directories(tracking_mode_bench PRIVATE "${CMAKE_SOURCE_DIR}/src/include" "${CMAKE_SOURCE_DIR}/src" "${CMAKE_SOURCE_DIR}/src/registration")
target_link_libraries(tracking_mode_bench PRIVATE rvvpcl)