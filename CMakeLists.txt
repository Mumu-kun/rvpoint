cmake_minimum_required(VERSION 3.16)
project(RVVPCL LANGUAGES C CXX)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
    set(BACKEND_SUBDIR "rvv")
else()
    set(BACKEND_SUBDIR "scalar")
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
    "src/point_cloud.cpp"
    "src/voxel_grid_downsamp.cpp"
    "src/rvv_common.cpp"
    "src/statistical_outlier_removal.cpp"
    "src/normal_estimation.cpp"
    "src/ransac_plane.cpp"
    "src/octree.cpp"
    "src/spatial_hashing.cpp"
    "src/caravan_radius_search.cpp"
)
add_library(rvvpcl ${SOURCES})
target_include_directories(rvvpcl PUBLIC src/include)
target_compile_options(rvvpcl PUBLIC "-O3" "-march=${RISCV_ARCH}" "-mabi=${RISCV_ABI}")
if(RVV_PCL_ENABLE_RVV)
    target_compile_definitions(rvvpcl PUBLIC RVV_PCL_USE_RVV=1)
endif()

# Helper: add a test target from a source file
function(add_rvv_test src)
    get_filename_component(name "${src}" NAME_WE)
    add_executable(${name} "${src}")
    target_link_libraries(${name} PRIVATE rvvpcl)
    target_include_directories(${name} PRIVATE "${CMAKE_SOURCE_DIR}/src/include")
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
file(GLOB BENCHMARK_SOURCES "tests/benchmark/*.cpp")
foreach(src ${BENCHMARK_SOURCES})
    get_filename_component(name "${src}" NAME_WE)
    add_executable(${name} "${src}")
    target_include_directories(${name} PRIVATE "${CMAKE_SOURCE_DIR}/src/include")
    target_compile_options(${name} PRIVATE "-O3" "-march=${RISCV_ARCH}" "-mabi=${RISCV_ABI}")
    target_link_libraries(${name} PRIVATE rvvpcl)
    if(EXISTS "${CMAKE_SOURCE_DIR}/src/benchmark_kernels.cpp")
        target_sources(${name} PRIVATE "${CMAKE_SOURCE_DIR}/src/benchmark_kernels.cpp")
    endif()
endforeach()

# Pipeline Export Utility
add_executable(pipeline_export src/tools/pipeline_export.cpp)
target_include_directories(pipeline_export PRIVATE "${CMAKE_SOURCE_DIR}/src/include")
target_link_libraries(pipeline_export PRIVATE rvvpcl)