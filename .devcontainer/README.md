# Dev Container Setup

This project uses VS Code Dev Containers to provide a consistent development environment for all team members.

## Prerequisites

1. **Docker Desktop** (Windows/Mac) or **Docker Engine** (Linux)
   - Windows: https://docs.docker.com/desktop/install/windows-install/
   - Mac: https://docs.docker.com/desktop/install/mac-install/
   - Linux: https://docs.docker.com/engine/install/

2. **Visual Studio Code**
   - Download: https://code.visualstudio.com/

3. **Dev Containers Extension**
   - Install: https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers
   - Or search "Dev Containers" in VS Code Extensions

## Quick Start

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd CSE450-Capstone-Project-RISC-V-Emulator
   ```

2. **Checkout dev branch:**
   ```bash
   git checkout dev
   ```

3. **Open in VS Code:**
   ```bash
   code .
   ```

4. **Reopen in Container:**
   - VS Code will detect `.devcontainer/devcontainer.json`
   - Click "Reopen in Container" when prompted
   - Or use Command Palette (F1) → "Dev Containers: Reopen in Container"

5. **Wait for container to build** (first time only, ~5-10 minutes)

6. **Start developing!**
   ```bash
   # Inside the container
   ./scripts/build.sh --riscv
   ```

## What's Included

The dev container provides:

- ✅ **RISC-V Toolchains**
  - `riscv64-linux-gnu-gcc/g++` (Linux userspace target)
  - `riscv64-unknown-elf-gcc/g++` (Bare-metal target)

- ✅ **Emulators**
  - QEMU (user-mode and system emulation)
  - Spike (RISC-V ISA simulator)
  - pk (Proxy kernel for Spike)

- ✅ **Build Tools**
  - CMake 3.22+
  - Ninja build system
  - ccache (for faster rebuilds)

- ✅ **Development Tools**
  - GDB (with multiarch support)
  - Valgrind
  - clang-format, cppcheck
  - Git with LFS

- ✅ **VS Code Extensions** (auto-installed)
  - C/C++ Extension Pack
  - CMake Tools
  - GitLens
  - Git Graph

## Helpful Aliases

The container includes convenient aliases:

```bash
build           # Native build (./scripts/build.sh)
build-riscv     # RISC-V scalar build (./scripts/build.sh --riscv)
build-rvv       # RISC-V with RVV (./scripts/build.sh --riscv --rvv)
test-qemu       # Run tests (cd build && ctest --output-on-failure)
```

## Common Tasks

### Building the Project

**Native build (for quick testing):**
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

**Debug build:**
```bash
./scripts/build.sh --riscv --rvv --debug
```

### Running Tests

```bash
cd build
ctest --output-on-failure --verbose
```

Or use the alias:
```bash
test-qemu
```

### Verifying Toolchain

```bash
# Check RISC-V compiler
riscv64-linux-gnu-gcc --version

# Check QEMU
qemu-riscv64 --version

# Check Spike
spike --version

# Test QEMU with a simple program
echo 'int main() { return 0; }' > test.c
riscv64-linux-gnu-gcc test.c -o test
qemu-riscv64 test
echo $?  # Should print 0
```

## Troubleshooting

### Container won't start

1. **Check Docker is running:**
   ```bash
   docker ps
   ```

2. **Rebuild container:**
   - Command Palette (F1) → "Dev Containers: Rebuild Container"

3. **Check Docker resources:**
   - Ensure Docker has at least 4GB RAM allocated
   - Check Docker Desktop settings

### Build fails

1. **Clean build directory:**
   ```bash
   rm -rf build
   ./scripts/build.sh --clean --riscv
   ```

2. **Verify toolchain:**
   ```bash
   which riscv64-linux-gnu-gcc
   riscv64-linux-gnu-gcc --version
   ```

### QEMU tests timeout

1. **Check QEMU version:**
   ```bash
   qemu-riscv64 --version  # Should be 6.2+
   ```

2. **Run test manually:**
   ```bash
   qemu-riscv64 -L /usr/riscv64-linux-gnu ./build/tests/your_test
   ```

## Team Collaboration

### Git Workflow

1. **Always work on dev branch:**
   ```bash
   git checkout dev
   git pull origin dev
   ```

2. **Create feature branches:**
   ```bash
   git checkout -b feature/your-feature-name
   ```

3. **Commit and push:**
   ```bash
   git add .
   git commit -m "Description of changes"
   git push origin feature/your-feature-name
   ```

4. **Merge back to dev** (after review)

### Keeping Container Updated

When `.devcontainer/Dockerfile` changes:

1. **Pull latest changes:**
   ```bash
   git pull origin dev
   ```

2. **Rebuild container:**
   - Command Palette (F1) → "Dev Containers: Rebuild Container"

## Performance Tips

1. **Use ccache for faster rebuilds:**
   - Already configured in the container
   - Builds will be faster after the first compilation

2. **Parallel builds:**
   ```bash
   cmake --build build --parallel $(nproc)
   ```

3. **Use Ninja instead of Make:**
   - Already configured as default generator

## Container Details

- **Base Image:** Ubuntu 22.04
- **Architecture:** amd64 (x86_64)
- **Size:** ~3GB (after build)
- **Build Time:** ~5-10 minutes (first time)

## Getting Help

- Check [docs/BUILD.md](../docs/BUILD.md) for detailed build instructions
- Ask team members in your project chat
- Check container logs if something fails:
  ```bash
  docker logs <container-id>
  ```

## Updating This Guide

If you discover issues or improvements, please update this README and commit to dev branch.
