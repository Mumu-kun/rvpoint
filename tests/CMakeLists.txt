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
