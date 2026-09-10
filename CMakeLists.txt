cmake_minimum_required(VERSION 3.16)
project(RVPoint LANGUAGES C CXX)

# ==============================================================================
# 1. Project Configuration & Compiler Flags
# ==============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Target-specific linking options (static for internal targets, dynamic allowed for official PCL)
# ==============================================================================

option(RVPOINT_USE_RVV "Enable RVV helper backend" ON)
option(RVV_PCL_USE_RVV "Backward compatibility alias for RVPOINT_USE_RVV" ON)
option(ENABLE_PROFILING "Enable diagnostic profiling instrumentation" OFF)
option(GEM5_BUILD "Enable gem5 simulation compatibility fallbacks" OFF)

set(RISCV_ARCH "rv64gcv" CACHE STRING "RISC-V ISA string, e.g. rv64gc or rv64gcv")
set(RISCV_ABI "lp64d" CACHE STRING "RISC-V ABI")

set(RVPOINT_ENABLE_RVV OFF)
if(RVPOINT_USE_RVV AND RVV_PCL_USE_RVV AND (RISCV_ARCH MATCHES "gcv" OR RISCV_ARCH MATCHES "_v" OR RISCV_ARCH MATCHES "v[0-9]"))
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
file(GLOB_RECURSE LIB_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/*.c"
)
list(APPEND LIB_SOURCES "${CMAKE_SOURCE_DIR}/eval/benchmarks/benchmark_kernels.cpp")

add_library(rvpoint ${LIB_SOURCES})
add_library(rvvpcl ALIAS rvpoint)
add_library(rvv_pcl ALIAS rvpoint)

# Automatically discover all subdirectories containing headers/sources under src/
file(GLOB_RECURSE ALL_SRC_HEADERS CONFIGURE_DEPENDS
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
if(NOT GEM5_BUILD)
    target_compile_options(rvpoint PUBLIC "-O3" "-march=${RISCV_ARCH}" "-mabi=${RISCV_ABI}" "-fopenmp" "-pthread")
    target_link_options(rvpoint PUBLIC "-fopenmp" "-pthread")
else()
    target_compile_options(rvpoint PUBLIC "-O3" "-march=${RISCV_ARCH}" "-mabi=${RISCV_ABI}")
endif()

if(RVPOINT_ENABLE_RVV)
    target_compile_definitions(rvpoint PUBLIC RVPOINT_USE_RVV=1 RVV_PCL_USE_RVV=1)
endif()
if(GEM5_BUILD)
    target_compile_definitions(rvpoint PUBLIC GEM5_BUILD=1)
endif()

# ==============================================================================
# 3. PCL Dependency Discovery (Dynamic System / Static env/deps/pcl)
# ==============================================================================
set(PCL_DEPS_DIR "${CMAKE_SOURCE_DIR}/env/deps")
set(PCL_SYSTEM_LIB_DIR "/usr/lib/riscv64-linux-gnu")
set(PCL_SYSTEM_INC_DIR "/usr/include/pcl-1.14")

if(EXISTS "${PCL_SYSTEM_LIB_DIR}/libpcl_common.so" AND EXISTS "${PCL_SYSTEM_INC_DIR}")
    set(PCL_SYSTEM_AVAILABLE ON)
    message(STATUS "Found system dynamic PCL 1.14 at ${PCL_SYSTEM_LIB_DIR}")
else()
    set(PCL_SYSTEM_AVAILABLE OFF)
endif()

if(EXISTS "${PCL_DEPS_DIR}/eigen/include/eigen3")
    include_directories(SYSTEM "${PCL_DEPS_DIR}/eigen/include/eigen3")
elseif(EXISTS "${PCL_DEPS_DIR}/eigen")
    include_directories(SYSTEM "${PCL_DEPS_DIR}/eigen")
endif()
if(EXISTS "${PCL_DEPS_DIR}/flann/include")
    include_directories(SYSTEM "${PCL_DEPS_DIR}/flann/include")
    link_directories("${PCL_DEPS_DIR}/flann/lib")
endif()
if(EXISTS "${PCL_DEPS_DIR}/pcl/usr/include/pcl-1.14" OR EXISTS "${PCL_DEPS_DIR}/pcl/include/pcl-1.14")
    include_directories(SYSTEM 
        "${PCL_DEPS_DIR}/pcl/usr/include/pcl-1.14"
        "${PCL_DEPS_DIR}/pcl/include/pcl-1.14"
        "${PCL_DEPS_DIR}/pcl/usr/include"
        "${PCL_DEPS_DIR}/pcl/include"
    )
    link_directories(
        "${PCL_DEPS_DIR}/pcl/usr/lib/riscv64-linux-gnu" 
        "${PCL_DEPS_DIR}/pcl/lib"
        "${PCL_DEPS_DIR}/flann/lib"
    )
    set(PCL_STATIC_AVAILABLE ON)
    message(STATUS "Found upstream PCL 1.14 RISC-V static dependencies at: ${PCL_DEPS_DIR}/pcl")
else()
    set(PCL_STATIC_AVAILABLE OFF)
endif()

# ==============================================================================
# 4. Perception Pipelines (eval/pipelines/)
# ==============================================================================
file(GLOB PIPELINE_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/eval/pipelines/*.cpp")
foreach(pipe_src ${PIPELINE_SOURCES})
    get_filename_component(pipe_name "${pipe_src}" NAME_WE)
    if(pipe_name MATCHES "^official_pcl_")
        if(PCL_SYSTEM_AVAILABLE)
            add_executable(${pipe_name} "${pipe_src}")
            target_link_libraries(${pipe_name} PRIVATE rvpoint)
            target_compile_options(${pipe_name} PRIVATE 
                "-I${PCL_SYSTEM_INC_DIR}" 
                "-I/usr/include/eigen3" 
                "-I/usr/include"
            )
            if(NOT GEM5_BUILD)
                target_compile_options(${pipe_name} PRIVATE "-fopenmp" "-pthread")
                target_link_options(${pipe_name} PRIVATE "-Wl,-rpath-link,${PCL_SYSTEM_LIB_DIR}" "-fopenmp" "-pthread")
            else()
                target_link_options(${pipe_name} PRIVATE "-Wl,-rpath-link,${PCL_SYSTEM_LIB_DIR}")
            endif()
            target_link_libraries(${pipe_name} PRIVATE 
                ${PCL_SYSTEM_LIB_DIR}/libpcl_common.so
                ${PCL_SYSTEM_LIB_DIR}/libpcl_io.so
                ${PCL_SYSTEM_LIB_DIR}/libpcl_filters.so
                ${PCL_SYSTEM_LIB_DIR}/libpcl_features.so
                ${PCL_SYSTEM_LIB_DIR}/libpcl_segmentation.so
                ${PCL_SYSTEM_LIB_DIR}/libpcl_search.so
                ${PCL_SYSTEM_LIB_DIR}/libpcl_octree.so
                ${PCL_SYSTEM_LIB_DIR}/libpcl_kdtree.so
                ${PCL_SYSTEM_LIB_DIR}/libpcl_sample_consensus.so
            )
        elseif(PCL_STATIC_AVAILABLE)
            add_executable(${pipe_name} "${pipe_src}")
            target_link_libraries(${pipe_name} PRIVATE rvpoint)
            set(PCL_LIB_DIR "${PCL_DEPS_DIR}/pcl/usr/lib/riscv64-linux-gnu")
            target_link_libraries(${pipe_name} PRIVATE 
                "${PCL_LIB_DIR}/libflann_cpp_s.a"
                "${PCL_LIB_DIR}/libboost_system.a"
                pthread
            )
            target_link_options(${pipe_name} PRIVATE "-static")
        else()
            message(STATUS "Skipping '${pipe_name}' (PCL shared or static library not found)")
            continue()
        endif()
    else()
        add_executable(${pipe_name} "${pipe_src}")
        target_link_libraries(${pipe_name} PRIVATE rvpoint)
        if(NOT GEM5_BUILD)
            target_compile_options(${pipe_name} PRIVATE "-fopenmp" "-pthread")
            target_link_options(${pipe_name} PRIVATE "-static" "-fopenmp" "-pthread")
        else()
            target_link_options(${pipe_name} PRIVATE "-static")
        endif()
    endif()
endforeach()

# ==============================================================================
# 5. Standalone Benchmarks (eval/benchmarks/)
# ==============================================================================
file(GLOB BENCHMARK_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/eval/benchmarks/*.cpp")
list(FILTER BENCHMARK_SOURCES EXCLUDE REGEX "benchmark_kernels.cpp|kdtree_vs_pointer_octree_bench.cpp")

foreach(bench_src ${BENCHMARK_SOURCES})
    get_filename_component(bench_name "${bench_src}" NAME_WE)
    add_executable(${bench_name} "${bench_src}")
    target_link_libraries(${bench_name} PRIVATE rvpoint)
    if(NOT GEM5_BUILD)
        target_compile_options(${bench_name} PRIVATE "-fopenmp" "-pthread")
        target_link_options(${bench_name} PRIVATE "-static" "-fopenmp" "-pthread")
    else()
        target_link_options(${bench_name} PRIVATE "-static")
    endif()
endforeach()

# ==============================================================================
# 6. Fast Essential Tests (eval/tests/fast/)
# ==============================================================================
function(add_rvpoint_test src)
    get_filename_component(name "${src}" NAME_WE)
    add_executable(${name} "${src}")
    target_link_libraries(${name} PRIVATE rvpoint)
    if(NOT GEM5_BUILD)
        target_compile_options(${name} PRIVATE "-fopenmp" "-pthread")
        target_link_options(${name} PRIVATE "-static" "-fopenmp" "-pthread")
    else()
        target_link_options(${name} PRIVATE "-static")
    endif()
endfunction()

file(GLOB FAST_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/eval/tests/fast/*.cpp"
    "${CMAKE_SOURCE_DIR}/eval/tests/fast/*.c"
)
foreach(src ${FAST_TEST_SOURCES})
    add_rvpoint_test("${src}")
endforeach()

# ==============================================================================
# 6. Experimental & Research Tests (eval/tests/experimental/)
# ==============================================================================
file(GLOB EXPERIMENTAL_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/eval/tests/experimental/*.cpp"
)
foreach(src ${EXPERIMENTAL_TEST_SOURCES})
    add_rvpoint_test("${src}")
endforeach()
