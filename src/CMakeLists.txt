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
