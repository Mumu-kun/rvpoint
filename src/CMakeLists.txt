# RVPoint Library Source Files

# Add your source files here as you implement them
set(RVPOINT_SOURCES
    # Example structure (uncomment and add your files):
    # point.cpp
    # pointcloud.cpp
    # filters.cpp
)

# Create library target (currently header-only until sources are added)
add_library(rvpoint INTERFACE)

# When you add source files, change to:
# add_library(rvpoint ${RVPOINT_SOURCES})

target_include_directories(rvpoint INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(rvpoint INTERFACE
    spdlog::spdlog
    tl::expected
)

# When you have source files, link dependencies as PRIVATE instead:
# target_link_libraries(rvpoint
#     PUBLIC
#         spdlog::spdlog
#         tl::expected
#     PRIVATE
#         # Add private dependencies here
# )

# Compiler features
target_compile_features(rvpoint INTERFACE cxx_std_17)

# ============================================================================
# Voxel Downsampling Implementations
# ============================================================================

# Object library for voxel downsampling implementations (for testing)
add_library(voxel_downsampling_impl OBJECT
    voxel_downsampling_scalar.cpp
    voxel_downsampling_rvv.cpp
)

target_compile_features(voxel_downsampling_impl PUBLIC cxx_std_17)

# Standalone executable - Scalar implementation
add_executable(voxel_downsample_scalar
    voxel_downsampling_scalar.cpp
)

target_compile_definitions(voxel_downsample_scalar PRIVATE BUILD_STANDALONE)
target_compile_features(voxel_downsample_scalar PRIVATE cxx_std_17)
target_include_directories(voxel_downsample_scalar PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Standalone executable - RVV implementation
add_executable(voxel_downsample_rvv
    voxel_downsampling_rvv.cpp
)

target_compile_definitions(voxel_downsample_rvv PRIVATE BUILD_STANDALONE)
target_compile_features(voxel_downsample_rvv PRIVATE cxx_std_17)
target_include_directories(voxel_downsample_rvv PRIVATE ${CMAKE_SOURCE_DIR}/include)
