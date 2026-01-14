cmake_minimum_required(VERSION 3.15)

# Download Catch2
include(FetchContent)
FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.5.2
)
FetchContent_MakeAvailable(Catch2)

# Enable testing
enable_testing()

# Add Catch2 CMake integration
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
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
        # pcl_riscv  # Uncomment when library is implemented
        Catch2::Catch2WithMain
)

target_compile_options(rvpoint_tests PRIVATE -Wall -Wextra)

# Discover tests
catch_discover_tests(rvpoint_tests)
