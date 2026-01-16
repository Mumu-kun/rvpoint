cmake_minimum_required(VERSION 3.15)

# Error handling and logging example
add_executable(error_handling_example
    error_handling_example.cpp
)

target_include_directories(error_handling_example
    PRIVATE ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(error_handling_example
    PRIVATE 
        spdlog::spdlog
        tl::expected
)

# Simple example demonstrating library usage
# Uncomment and add your example files as needed
#
# add_executable(example_basic
#     basic_usage.cpp
# )
#
# target_link_libraries(example_basic
#     PRIVATE pcl_riscv
# )
