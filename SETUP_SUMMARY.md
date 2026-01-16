# RISC-V Cross-Compilation Setup Summary

## Changes Made

This document summarizes the changes made to enable RISC-V cross-compilation with vcpkg dependency management.

## Files Created/Modified

### 1. **cmake/riscv64-linux-gnu.cmake** (Modified)
- Added vcpkg integration via `VCPKG_TARGET_TRIPLET` and `VCPKG_CHAINLOAD_TOOLCHAIN_FILE`
- Enables proper toolchain chaining: vcpkg → RISC-V toolchain

**Key additions:**
```cmake
set(VCPKG_TARGET_TRIPLET riscv64-linux)
set(VCPKG_CHAINLOAD_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_FILE})
```

### 2. **triplets/riscv64-linux.cmake** (Created)
- Custom vcpkg triplet for RISC-V 64-bit Linux
- Specifies target architecture and toolchain file
- Uses static linking for libraries, dynamic for CRT

**Content:**
```cmake
set(VCPKG_TARGET_ARCHITECTURE riscv64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)
set(VCPKG_CMAKE_SYSTEM_NAME Linux)
set(VCPKG_CHAINLOAD_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/riscv64-linux-gnu.cmake)
```

### 3. **scripts/build.sh** (Modified)
- Updated to use vcpkg's buildsystem integration
- Added proper triplet specification for RISC-V builds
- Maintains backwards compatibility for native builds

**Key changes:**
```bash
# Native build
TOOLCHAIN_FILE="-DCMAKE_TOOLCHAIN_FILE=${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"

# RISC-V build
TOOLCHAIN_FILE="-DCMAKE_TOOLCHAIN_FILE=${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
VCPKG_TRIPLET="-DVCPKG_TARGET_TRIPLET=riscv64-linux -DVCPKG_CHAINLOAD_TOOLCHAIN_FILE=${PWD}/cmake/riscv64-linux-gnu.cmake"
```

### 4. **.devcontainer/devcontainer.json** (Modified)
- Updated environment variables for vcpkg
- Added `VCPKG_OVERLAY_TRIPLETS` to use custom triplets
- Removed hardcoded compiler environment variables (CC/CXX)

**Environment variables:**
```json
"remoteEnv": {
    "CMAKE_GENERATOR": "Ninja",
    "VCPKG_ROOT": "/opt/vcpkg",
    "VCPKG_DEFAULT_TRIPLET": "x64-linux",
    "VCPKG_OVERLAY_TRIPLETS": "${containerWorkspaceFolder}/triplets"
}
```

### 5. **.devcontainer/Dockerfile** (No changes needed)
- Already has all necessary RISC-V tools
- Contains: `gcc-riscv64-linux-gnu`, `g++-riscv64-linux-gnu`, `qemu-user`

### 6. **docs/CROSS_COMPILE.md** (Created)
- Comprehensive guide for cross-compilation
- Explains the toolchain chain: CMake → vcpkg → RISC-V toolchain
- Troubleshooting section
- Architecture flags reference

### 7. **BUILDING.md** (Created)
- Quick reference for building
- Common commands and options
- Troubleshooting tips

## How It Works

### Toolchain Chaining

```
CMake
  ↓
vcpkg.cmake (main toolchain)
  ↓
riscv64-linux.cmake (vcpkg triplet)
  ↓
riscv64-linux-gnu.cmake (RISC-V toolchain)
  ↓
riscv64-linux-gnu-gcc/g++ (actual compiler)
```

### Build Flow

1. **User runs:** `./scripts/build.sh --riscv`
2. **Script configures CMake with:**
   - Main toolchain: `$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake`
   - Target triplet: `riscv64-linux`
   - Chainload toolchain: `cmake/riscv64-linux-gnu.cmake`
3. **vcpkg:**
   - Reads `triplets/riscv64-linux.cmake`
   - Chains to `cmake/riscv64-linux-gnu.cmake`
   - Installs dependencies (spdlog, tl-expected, etc.) for RISC-V
4. **CMake:**
   - Configures project with RISC-V compiler
   - Links against RISC-V-compiled dependencies
5. **Ninja builds the project**
6. **CTest runs tests in QEMU**

## Dependencies Management

### Native Build (x86_64)
- Triplet: `x64-linux`
- Dependencies installed to: `$VCPKG_ROOT/installed/x64-linux/`

### RISC-V Build
- Triplet: `riscv64-linux`
- Dependencies installed to: `$VCPKG_ROOT/installed/riscv64-linux/`
- All dependencies compiled from source for RISC-V

## Testing

Tests automatically run in QEMU user-mode emulation:
- Scalar: `qemu-riscv64 -L /usr/riscv64-linux-gnu <test-binary>`
- Vector: `qemu-riscv64 -cpu rv64,v=true,vlen=256 -L /usr/riscv64-linux-gnu <test-binary>`

## Compiler Flags

### Scalar Build (rv64gc)
```
-march=rv64gc
```

### Vector Build (rv64gcv)
```
-march=rv64gcv
```

Set via `ENABLE_RVV` CMake option or `--rvv` script flag.

## Usage Examples

### Basic RISC-V Build
```bash
./scripts/build.sh --riscv
```

### RISC-V with Vectors
```bash
./scripts/build.sh --riscv --rvv
```

### Debug Build
```bash
./scripts/build.sh --riscv --debug
```

### Clean Build
```bash
./scripts/build.sh --riscv --clean
```

## Verification

After setup, verify everything works:

```bash
# 1. Check tools are available
riscv64-linux-gnu-gcc --version
qemu-riscv64 --version
vcpkg version

# 2. Try a build
./scripts/build.sh --riscv

# 3. Check output
ls -lh build/
file build/tests/rvpoint_tests  # Should show "RISC-V 64-bit"
```

## Next Steps

1. **Build native version:**
   ```bash
   ./scripts/build.sh
   ```

2. **Build RISC-V version:**
   ```bash
   ./scripts/build.sh --riscv
   ```

3. **Compare performance:**
   ```bash
   # Native
   ./build/benchmarks/benchmark_suite

   # RISC-V (in QEMU)
   qemu-riscv64 -L /usr/riscv64-linux-gnu ./build/benchmarks/benchmark_suite
   ```

## Troubleshooting

### Issue: "Compiler not found"
**Solution:** Make sure you're in the dev container with all tools installed.

### Issue: "vcpkg can't find dependencies"
**Solution:** First RISC-V build takes longer - vcpkg compiles everything from source.

### Issue: "Tests fail in QEMU"
**Solution:** Verify QEMU is installed and working:
```bash
qemu-riscv64 --version
qemu-riscv64 -L /usr/riscv64-linux-gnu /bin/true
```

## References

- [vcpkg Documentation](https://vcpkg.io/)
- [CMake Toolchains](https://cmake.org/cmake/help/latest/manual/cmake-toolchains.7.html)
- [RISC-V GNU Toolchain](https://github.com/riscv-collab/riscv-gnu-toolchain)
- [QEMU RISC-V](https://www.qemu.org/docs/master/system/target-riscv.html)

---

**Setup completed!** Your project is now ready for RISC-V cross-compilation with full dependency management through vcpkg.
