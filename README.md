# RV Point

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A high-performance point cloud processing library for RISC-V architecture with support for the RISC-V Vector (RVV) extension and future custom instructions.

🚀 **Try without hardware:** Runs on QEMU (no RISC-V board needed)  
📊 **Benchmarks:** Designed for significant speedup with RVV 1.0  
🎓 **Educational:** Documented for learning RISC-V vectorization  
🔧 **Extensible:** Architecture ready for custom instructions

## Features

- **Multiple Point Types**: Point3D, PointXYZI, PointXYZRGB, PointNormal
- **Backend Abstraction**: Clean separation between scalar and vector implementations
- **Modern Error Handling**: `tl::expected` for clean error propagation (C++17)
- **Structured Logging**: `spdlog` for fast, configurable logging
- **Dependency Management**: `vcpkg` for easy library integration
- **Code Quality**: Pre-commit hooks with `clang-format`
- **Header-Only Core**: Easy integration, minimal dependencies
- **Comprehensive Testing**: Catch2 test suite with QEMU/Spike emulation
- **CI/CD Ready**: GitHub Actions with cross-compilation
- **Docker Support**: Reproducible development environment

## Quick Start

### For Team Members (Recommended)

**Using VS Code Dev Container (ensures consistent environment):**

1. Install [Docker Desktop](https://docs.docker.com/get-docker/) and [VS Code](https://code.visualstudio.com/)
2. Install [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
3. Clone repository and checkout dev branch:
   ```bash
   git clone <repository-url>
   cd CSE450-Capstone-Project-RISC-V-Emulator
   git checkout dev
   ```
4. Open in VS Code: `code .`
5. Click "Reopen in Container" when prompted
6. Wait for container to build (~5-10 minutes first time)
7. Start developing!

See [Dev Container Setup](.devcontainer/README.md) for detailed guide.

### Manual Setup (Alternative)

**Dependencies:**
```bash
# System packages
sudo apt install cmake ninja-build g++ git zip unzip tar pkg-config

# For RISC-V cross-compilation
sudo apt install gcc-riscv64-linux-gnu g++-riscv64-linux-gnu qemu-user

# Python tools
pip install pre-commit
```

**Install vcpkg (dependency manager):**
```bash
git clone https://github.com/microsoft/vcpkg.git
./vcpkg/bootstrap-vcpkg.sh
export VCPKG_ROOT=$(pwd)/vcpkg
export CMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake
```

**Setup pre-commit hooks:**
```bash
pre-commit install
```

### Building

**Native build:**
```bash
./scripts/build.sh
```

**RISC-V cross-compilation:**
```bash
./scripts/build.sh --riscv
```

**With RVV support:**
```bash
./scripts/build.sh --riscv --rvv
```

**Run example:**
```bash
./build/examples/error_handling_example
```



### Project Structure

```
pcl-riscv/
├── cmake/                   # CMake configuration files
│   ├── riscv64-linux-gnu.cmake    # RISC-V toolchain
│   └── CompilerWarnings.cmake     # Warning flags
├── .github/workflows/       # CI/CD configuration
│   └── ci.yml              # GitHub Actions workflow
├── .devcontainer/           # Dev container setup
│   ├── Dockerfile          # Container with full toolchain
│   └── devcontainer.json   # VS Code configuration
├── include/rvpoint/         # Public library headers
│   ├── error.hpp           # Error handling with tl::expected
│   └── logger.hpp          # Logging with spdlog
├── scripts/                 # Build and utility scripts
│   └── build.sh            # Convenient build script
├── docs/                    # Documentation
│   └── BUILD.md            # Build instructions
├── tests/                   # Unit tests
│   └── CMakeLists.txt      # Test configuration template
├── examples/                # Usage examples
│   ├── error_handling_example.cpp  # Error & logging demo
│   └── CMakeLists.txt      # Example configuration
├── benchmarks/              # Performance benchmarks
│   └── CMakeLists.txt      # Benchmark configuration template
├── vcpkg.json              # Dependency manifest
├── vcpkg-configuration.json # vcpkg settings
├── .clang-format           # Code formatting rules
├── .pre-commit-config.yaml # Pre-commit hooks config
├── CMakeLists.txt          # Main build configuration
├── README.md               # This file
├── LICENSE                 # MIT License
└── .gitignore             # Git ignore patterns
```

## What's Included

This project provides the **complete setup infrastructure** for developing a RISC-V point cloud library:

✅ **CMake Build System**
- Cross-compilation support for RISC-V (riscv64-linux-gnu)
- Native compilation for x86/ARM development
- RVV (RISC-V Vector) extension support
- Custom instruction support (future)
- Multiple build types (Debug/Release)

✅ **Testing Framework**
- Catch2 integration
- Automatic test execution via QEMU or Spike
- CTest integration for CI/CD
- VS Code test explorer support

✅ **CI/CD Pipeline**
- GitHub Actions workflow
- Matrix builds (Debug/Release × Scalar/RVV)
- Automated testing on push/PR
- Weekly Spike validation (optional)

✅ **Docker Environment**
- Pre-configured container with RISC-V toolchain
- QEMU and Spike emulators
- Reproducible builds

✅ **Documentation**
- Comprehensive README
- Detailed build instructions
- Examples and usage patterns

## What You Need to Implement

The project structure is ready. Now implement your library:

1. **Create header files** in `include/` directory
2. **Add test files** in `tests/` directory  
3. **Implement algorithms** (filters, search, segmentation, etc.)
4. **Write examples** in `examples/` directory
5. **Add benchmarks** in `benchmarks/` directory

## Performance

Designed for benchmarking scalar vs RVV implementations:

| Operation | Scalar | RVV | Speedup |
|-----------|--------|-----|---------|
| Array Addition (10k) | 45 μs | TBD | TBD |
| Distance Computation | 120 μs | TBD | TBD |
| Find Min/Max | 80 μs | TBD | TBD |

*Note: Run benchmarks with real hardware for accurate measurements.*

## Testing

```bash
# Build and run all tests
cmake -B build && cmake --build build && cd build && ctest

# Run specific test
./build/tests/pcl_tests --gtest_filter=PointCloudTest.*

# Run with Spike instead of QEMU
cmake -B build -DUSE_SPIKE_EMULATOR=ON
cmake --build build && cd build && ctest
```

## Roadmap

- [x] Core data structures (Point3D, PointCloud)
- [x] Scalar backend implementation
- [x] RVV backend skeleton
- [x] Testing framework (Google Test)
- [x] CI/CD pipeline (GitHub Actions)
- [ ] Complete RVV vectorization
- [ ] Passthrough filter
- [ ] KD-tree construction and search
- [ ] Voxel grid downsampling
- [ ] Statistical outlier removal
- [ ] Custom RISC-V instructions (PCL_* extensions)
- [ ] Python bindings
- [ ] ROS2 integration

## Development

### Project Setup

```bash
# Clone repository
git clone https://github.com/yourusername/pcl-riscv.git
cd pcl-riscv

# Build in debug mode
./scripts/build.sh --debug

# Run tests
cd build && ctest --verbose
```

### Adding New Algorithms

1. Add scalar implementation in `include/pcl_riscv/backend/scalar.hpp`
2. Add tests in `tests/test_backend_scalar.cpp`
3. (Later) Add RVV implementation in `include/pcl_riscv/backend/rvv.hpp`
4. Add golden reference test comparing scalar vs RVV outputs

### Custom Instructions (Future)

For implementing custom RISC-V instructions:
1. Modify Spike: Add instruction in `spike/riscv/insns/pcl_*.h`
2. Update encoding table in `spike/riscv/encoding.h`
3. Rebuild Spike and test with `--isa=rv64gcv_xpcl`
4. Document in `docs/CUSTOM_ISA.md`

## Documentation

Detailed documentation is available in the [docs/](docs/) directory:

- **[Build Guide](docs/BUILD.md)** - Building for different targets (native, RISC-V, with RVV)
- **[Testing Guide](docs/TESTING.md)** - Writing and running tests with Catch2
- **[Contributing Guide](docs/CONTRIBUTING.md)** - Team workflow and collaboration practices
- **[Dev Container Setup](.devcontainer/README.md)** - Development environment setup

## Contributing

See [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md) for the detailed collaboration workflow.

**Quick checklist:**
1. Create feature branch from `dev`
2. Add tests for new functionality
3. Ensure all tests pass (`ctest --output-on-failure`)
4. Submit pull request to `dev` branch

## License

This project is licensed under the MIT License - see [LICENSE](LICENSE) file for details.

## Acknowledgments

- RISC-V International for the open ISA specification
- PCL (Point Cloud Library) for API inspiration
- Open3D for modern point cloud processing patterns

## Contact

- **Project Lead**: Your Name
- **Institution**: Your University
- **Email**: your.email@university.edu
- **Course**: CSE450 Capstone Project

## Citation

If you use this library in your research, please cite:

```bibtex
@software{pcl_riscv2026,
  title = {PCL-RISC-V: Point Cloud Library for RISC-V with Vector Extensions},
  author = {Your Name},
  year = {2026},
  url = {https://github.com/yourusername/pcl-riscv}
}
```

## References

- [RISC-V Vector Extension Specification](https://github.com/riscv/riscv-v-spec)
- [Point Cloud Library (PCL)](https://pointclouds.org/)
- [Open3D](http://www.open3d.org/)
- [RISC-V ISA Simulator (Spike)](https://github.com/riscv-software-src/riscv-isa-sim)
