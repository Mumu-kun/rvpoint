cmake_minimum_required(VERSION 3.16)
project(RVPoint LANGUAGES C CXX)

# ==============================================================================
# 1. Project Configuration & Compiler Flags
# ==============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Static linking for QEMU compatibility
set(CMAKE_EXE_LINKER_FLAGS "-static")

option(RVPOINT_USE_RVV "Enable RVV helper backend" ON)
option(RVV_PCL_USE_RVV "Backward compatibility alias for RVPOINT_USE_RVV" ON)
option(ENABLE_PROFILING "Enable diagnostic profiling instrumentation" OFF)
option(GEM5_BUILD "Enable gem5 simulation compatibility fallbacks" OFF)

set(RISCV_ARCH "rv64gcv" CACHE STRING "RISC-V ISA string, e.g. rv64gc or rv64gcv")
set(RISCV_ABI "lp64d" CACHE STRING "RISC-V ABI")

set(RVPOINT_ENABLE_RVV OFF)
if((RVPOINT_USE_RVV OR RVV_PCL_USE_RVV) AND RISCV_ARCH MATCHES ".*v.*")
    set(RVPOINT_ENABLE_RVV ON)
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
message(STATUS "RVV backend enabled: ${RVPOINT_ENABLE_RVV}")
message(STATUS "GEM5_BUILD enabled: ${GEM5_BUILD}")
message(STATUS "Binary output dir: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

# ==============================================================================
# 2. RVPoint Core Library Target (src/)
# ==============================================================================
file(GLOB_RECURSE LIB_SOURCES 
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/*.c"
)
list(APPEND LIB_SOURCES "${CMAKE_SOURCE_DIR}/eval/benchmarks/benchmark_kernels.cpp")

add_library(rvpoint ${LIB_SOURCES})
add_library(rvvpcl ALIAS rvpoint)
add_library(rvv_pcl ALIAS rvpoint)

# Automatically discover all subdirectories containing headers/sources under src/
file(GLOB_RECURSE ALL_SRC_HEADERS 
    "${CMAKE_SOURCE_DIR}/src/*.h" 
    "${CMAKE_SOURCE_DIR}/src/*.hpp"
)
set(RVPOINT_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/src")
foreach(hdr_file ${ALL_SRC_HEADERS})
    get_filename_component(dir_path "${hdr_file}" DIRECTORY)
    list(APPEND RVPOINT_INCLUDE_DIRS "${dir_path}")
endforeach()
list(APPEND RVPOINT_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/eval/benchmarks")
list(REMOVE_DUPLICATES RVPOINT_INCLUDE_DIRS)

target_include_directories(rvpoint PUBLIC ${RVPOINT_INCLUDE_DIRS})
target_compile_options(rvpoint PUBLIC "-O3" "-march=${RISCV_ARCH}" "-mabi=${RISCV_ABI}")

if(RVPOINT_ENABLE_RVV)
    target_compile_definitions(rvpoint PUBLIC RVPOINT_USE_RVV=1 RVV_PCL_USE_RVV=1)
endif()
if(GEM5_BUILD)
    target_compile_definitions(rvpoint PUBLIC GEM5_BUILD=1)
endif()

# ==============================================================================
# 3. Perception Pipelines (eval/pipelines/)
# ==============================================================================
file(GLOB PIPELINE_SOURCES "${CMAKE_SOURCE_DIR}/eval/pipelines/*.cpp")
foreach(pipe_src ${PIPELINE_SOURCES})
    get_filename_component(pipe_name "${pipe_src}" NAME_WE)
    add_executable(${pipe_name} "${pipe_src}")
    target_link_libraries(${pipe_name} PRIVATE rvpoint)
endforeach()

# ==============================================================================
# 4. Standalone Benchmarks (eval/benchmarks/)
# ==============================================================================
file(GLOB BENCHMARK_SOURCES "${CMAKE_SOURCE_DIR}/eval/benchmarks/*.cpp")
list(FILTER BENCHMARK_SOURCES EXCLUDE REGEX "benchmark_kernels.cpp|kdtree_vs_pointer_octree_bench.cpp")

foreach(bench_src ${BENCHMARK_SOURCES})
    get_filename_component(bench_name "${bench_src}" NAME_WE)
    add_executable(${bench_name} "${bench_src}")
    target_link_libraries(${bench_name} PRIVATE rvpoint)
endforeach()

# ==============================================================================
# 5. Fast Essential Tests (eval/tests/fast/)
# ==============================================================================
function(add_rvpoint_test src)
    get_filename_component(name "${src}" NAME_WE)
    add_executable(${name} "${src}")
    target_link_libraries(${name} PRIVATE rvpoint)
    target_compile_options(${name} PRIVATE "-fopenmp")
    target_link_options(${name} PRIVATE "-fopenmp")
endfunction()

file(GLOB FAST_TEST_SOURCES 
    "${CMAKE_SOURCE_DIR}/eval/tests/fast/*.cpp"
    "${CMAKE_SOURCE_DIR}/eval/tests/fast/*.c"
)
foreach(src ${FAST_TEST_SOURCES})
    add_rvpoint_test("${src}")
endforeach()

# ==============================================================================
# 6. Experimental & Research Tests (eval/tests/experimental/)
# ==============================================================================
file(GLOB EXPERIMENTAL_TEST_SOURCES 
    "${CMAKE_SOURCE_DIR}/eval/tests/experimental/*.cpp"
)
foreach(src ${EXPERIMENTAL_TEST_SOURCES})
    add_rvpoint_test("${src}")
endforeach()