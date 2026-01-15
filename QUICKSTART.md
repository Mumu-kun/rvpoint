# Quick Start Guide for Team Members

## Option 1: Dev Container (Recommended) ⭐

**One-time setup (5-10 minutes):**

1. Install [Docker Desktop](https://www.docker.com/products/docker-desktop)
2. Install [VS Code](https://code.visualstudio.com/)
3. Install the [Dev Containers extension](vscode:extension/ms-vscode-remote.remote-containers)
4. Clone the repo:
   ```bash
   git clone <repo-url>
   cd CSE450-Capstone-Project-RISC-V-Emulator
   ```
5. Open in VS Code: `code .`
6. Click "Reopen in Container" when prompted
7. Wait for the container to build (only first time)

**Daily workflow:**
```bash
# Native x86 build (for testing)
./scripts/build.sh

# RISC-V build (scalar)
./scripts/build.sh --riscv

# RISC-V build with vectors (RVV)
./scripts/build.sh --riscv --rvv

# Use handy aliases
build           # Native build
build-riscv     # RISC-V scalar
build-rvv       # RISC-V with RVV
test-qemu       # Run tests
```

That's it! Everything is pre-configured. ✅

## Option 2: Manual Setup (Ubuntu/Debian)

**One-time setup:**

```bash
# Install tools
sudo apt-get update
sudo apt-get install -y \
    build-essential cmake ninja-build git \
    curl zip unzip tar pkg-config \
    gcc-riscv64-linux-gnu g++-riscv64-linux-gnu \
    qemu-user python3-pip

# Install vcpkg
git clone https://github.com/microsoft/vcpkg.git ~/vcpkg
~/vcpkg/bootstrap-vcpkg.sh
echo 'export VCPKG_ROOT=~/vcpkg' >> ~/.bashrc
source ~/.bashrc

# Setup project
cd <project-directory>
pip3 install pre-commit
pre-commit install
ln -sf $PWD/cmake/riscv64-linux-gnu.cmake $VCPKG_ROOT/cmake/
```

**Daily workflow:**
```bash
./scripts/build.sh --riscv --rvv
```

## Build Options

| Command | Description |
|---------|-------------|
| `./scripts/build.sh` | Native x86_64 build |
| `./scripts/build.sh --riscv` | RISC-V scalar (rv64gc) |
| `./scripts/build.sh --riscv --rvv` | RISC-V with vectors (rv64gcv) |
| `./scripts/build.sh --debug` | Debug build |
| `./scripts/build.sh --clean` | Clean before building |

## Troubleshooting

**"VCPKG_ROOT not set"**
```bash
export VCPKG_ROOT=/opt/vcpkg  # or ~/vcpkg for manual setup
```

**"Compiler not found"**
```bash
# In dev container, rebuild container
# For manual setup:
sudo apt-get install gcc-riscv64-linux-gnu g++-riscv64-linux-gnu
```

**First RISC-V build is slow**
- This is normal! vcpkg compiles all dependencies from source
- Subsequent builds use cache and are much faster
- First build: ~5-10 minutes
- Incremental builds: ~10-30 seconds

## CI/CD Pipeline

All builds are automatically tested on push/PR:
- ✅ Native x86_64 (Debug + Release)
- ✅ RISC-V scalar (Debug + Release)  
- ✅ RISC-V with RVV (Debug + Release)
- ✅ All tests run in QEMU

View results in GitHub Actions tab.

## Getting Help

- See [BUILDING.md](BUILDING.md) for detailed build instructions
- See [SETUP_SUMMARY.md](SETUP_SUMMARY.md) for system setup
- Ask in team Slack/Discord
- Check [GitHub Issues](../../issues)

## Common Tasks

**Run tests manually:**
```bash
cd build
ctest --output-on-failure
```

**Clean everything:**
```bash
rm -rf build
./scripts/build.sh --riscv --rvv
```

**Check code formatting:**
```bash
pre-commit run --all-files
```

**Run specific test:**
```bash
./build/tests/rvpoint_tests
```
