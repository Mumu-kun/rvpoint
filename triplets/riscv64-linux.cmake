# RISC-V 64-bit Linux triplet for vcpkg cross-compilation

set(VCPKG_TARGET_ARCHITECTURE riscv64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)

set(VCPKG_CMAKE_SYSTEM_NAME Linux)
set(VCPKG_CHAINLOAD_TOOLCHAIN_FILE /opt/vcpkg/cmake/riscv64-linux-gnu.cmake)
