# RVPoint Tests

# Catch2 CMake integration
include(CTest)
include(Catch)

# Test executable
add_executable(rvpoint_tests
    test_main.cpp
    # Add your test files here:
    # test_point_types.cpp
    # test_point_cloud.cpp
    # test_backend_scalar.cpp
)

target_link_libraries(rvpoint_tests
    PRIVATE
        rvpoint
        Catch2::Catch2WithMain
)

target_compile_options(rvpoint_tests PRIVATE -Wall -Wextra)

# Discover tests
catch_discover_tests(rvpoint_tests)

# ============================================================================
# Voxel Downsampling Tests
# ============================================================================

# Test executable for voxel downsampling
add_executable(voxel_downsampling_tests
    test_voxel_downsampling.cpp
    ${CMAKE_SOURCE_DIR}/src/voxel_downsampling_scalar.cpp
    ${CMAKE_SOURCE_DIR}/src/voxel_downsampling_rvv.cpp
)

target_link_libraries(voxel_downsampling_tests
    PRIVATE
        Catch2::Catch2WithMain
)

target_include_directories(voxel_downsampling_tests PRIVATE ${CMAKE_SOURCE_DIR}/include)

target_compile_options(voxel_downsampling_tests PRIVATE -Wall -Wextra)

# Discover voxel downsampling tests
catch_discover_tests(voxel_downsampling_tests)
