# Documentation Index

This directory contains detailed documentation for the RV Point project.

## Getting Started

- **[Dev Container Setup](../.devcontainer/README.md)** - Set up your development environment
- **[Build Guide](BUILD.md)** - How to build the project for different targets
- **[Contributing Guide](CONTRIBUTING.md)** - Team workflow and collaboration practices

## Development

- **[Testing Guide](TESTING.md)** - Writing and running tests with Catch2

## Project Structure

```
rvpoint/
├── README.md                    # Project overview and quick start
├── LICENSE                      # MIT License
├── CMakeLists.txt              # Root CMake configuration
│
├── .devcontainer/              # Development environment
│   ├── Dockerfile              # Container image definition
│   ├── devcontainer.json       # VS Code dev container config
│   └── README.md               # Dev container setup guide
│
├── docs/                       # Detailed documentation
│   ├── README.md               # This file
│   ├── BUILD.md                # Build instructions
│   ├── CONTRIBUTING.md         # Collaboration workflow
│   └── TESTING.md              # Testing guide
│
├── include/                    # Public header files (to be created)
├── src/                        # Implementation files (if needed)
├── tests/                      # Unit tests
├── examples/                   # Example applications
├── benchmarks/                 # Performance benchmarks
├── scripts/                    # Build and utility scripts
└── cmake/                      # CMake modules
```

## Additional Resources

- **Main README**: [../README.md](../README.md)
- **GitHub Actions CI**: [../.github/workflows/ci.yml](../.github/workflows/ci.yml)

## Quick Links

### For New Team Members
1. [Dev Container Setup](../.devcontainer/README.md)
2. [Contributing Guide](CONTRIBUTING.md)
3. [Build Guide](BUILD.md)

### For Development
1. [Testing Guide](TESTING.md)
2. [Build Instructions](BUILD.md)
3. [Contributing Workflow](CONTRIBUTING.md)

## Future Documentation

As the project grows, consider adding:
- API documentation (generated with Doxygen)
- Architecture design documentation
- Performance analysis and benchmarking results
- RISC-V Vector extension usage guide
- Custom instruction set specification
