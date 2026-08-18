# RVPoint Development Stack

This document describes the libraries and tools used in the RVPoint project.

## Core Dependencies

### vcpkg (Dependency Management)
- **Purpose**: Cross-platform C++ package manager
- **Why**: Simplifies dependency management across platforms
- **Integration**: Automatic via `CMAKE_TOOLCHAIN_FILE`
- **Dependencies defined in**: `vcpkg.json`

### tl::expected (Error Handling)
- **Purpose**: C++17 backport of C++23's `std::expected`
- **Why**: Type-safe error handling without exceptions
- **Usage**: Return `Result<T>` from fallible operations
- **Examples**: See `include/rvpoint/error.hpp` and `examples/error_handling_example.cpp`

### spdlog (Logging)
- **Purpose**: Fast, header-only C++ logging library
- **Why**: Production-grade logging with minimal overhead
- **Features**: Multiple log levels, sinks, thread-safe
- **Usage**: See `include/rvpoint/logger.hpp`

### Catch2 (Testing)
- **Purpose**: Modern C++ test framework
- **Why**: Easy to write, readable tests
- **Integration**: Via CMake and vcpkg

### Google Benchmark (Performance Testing)
- **Purpose**: Microbenchmarking library
- **Why**: Measure and compare scalar vs RVV performance
- **Integration**: Via CMake and vcpkg

## Development Tools

### pre-commit (Code Quality)
- **Purpose**: Git hooks for automated formatting checks
- **Why**: Enforces consistent code style before commits
- **Config**: `.pre-commit-config.yaml`
- **Usage**: Runs `clang-format` automatically

### clang-format (Code Formatting)
- **Purpose**: Automatic C++ code formatting
- **Why**: Consistent code style across team
- **Config**: `.clang-format`
- **Style**: Based on LLVM with modifications

## Future Considerations

### Potential Additions
- **Eigen3**: For advanced linear algebra (when needed)
- **nanoflann**: For KD-tree spatial queries
- **tinyply/happly**: For PLY file I/O
- **Doxygen**: For API documentation generation

## Why These Choices?

1. **C++17 Compatible**: All dependencies work with C++17
2. **Header-Only Options**: Minimal compilation overhead
3. **Cross-Platform**: Works on x86, ARM, and RISC-V
4. **Production-Ready**: Battle-tested in real projects
5. **Lightweight**: No heavy dependencies like Boost or PCL
6. **Modern**: Follow current C++ best practices

## Installation

### Via Devcontainer (Recommended)
All dependencies are pre-installed. Just rebuild the container.

### Manual Setup
```bash
# Install vcpkg
git clone https://github.com/microsoft/vcpkg.git
./vcpkg/bootstrap-vcpkg.sh

# Set environment variables
export VCPKG_ROOT=$(pwd)/vcpkg
export CMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake

# Install pre-commit
pip install pre-commit
pre-commit install
```

Dependencies (spdlog, tl-expected, catch2, benchmark) are installed automatically by vcpkg during CMake configuration.

## Usage Examples

### Error Handling
```cpp
#include <rvpoint/error.hpp>

Result<int> divide(int a, int b) {
    if (b == 0) return make_error<int>(ErrorCode::InvalidInput);
    return a / b;
}

auto result = divide(10, 2);
if (result) {
    std::cout << *result << std::endl;
} else {
    std::cout << error_to_string(result.error()) << std::endl;
}
```

### Logging
```cpp
#include <rvpoint/logger.hpp>

RVPOINT_INFO("Processing point cloud with {} points", count);
RVPOINT_ERROR("Failed to load file: {}", filename);
```

## References
- [vcpkg Documentation](https://vcpkg.io/)
- [tl::expected GitHub](https://github.com/TartanLlama/expected)
- [spdlog GitHub](https://github.com/gabime/spdlog)
- [Catch2 Documentation](https://github.com/catchorg/Catch2)
- [pre-commit Framework](https://pre-commit.com/)
