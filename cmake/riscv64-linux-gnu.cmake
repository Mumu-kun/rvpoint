# Toolchain file for RISC-V cross-compilation (Linux userspace)
# Usage: cmake -DCMAKE_TOOLCHAIN_FILE=cmake/riscv64-linux-gnu.cmake ..

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR riscv64)

# RISC-V toolchain prefix
set(RISCV_TOOLCHAIN_PREFIX riscv64-linux-gnu-)

# Compilers
set(CMAKE_C_COMPILER ${RISCV_TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${RISCV_TOOLCHAIN_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${RISCV_TOOLCHAIN_PREFIX}gcc)

# Tools
set(CMAKE_AR ${RISCV_TOOLCHAIN_PREFIX}ar)
set(CMAKE_RANLIB ${RISCV_TOOLCHAIN_PREFIX}ranlib)
set(CMAKE_STRIP ${RISCV_TOOLCHAIN_PREFIX}strip)

# Find root path - allow vcpkg to find packages too
set(CMAKE_FIND_ROOT_PATH /usr/riscv64-linux-gnu)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)  # Changed from ONLY to BOTH for vcpkg
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)  # Changed from ONLY to BOTH for vcpkg
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)  # Changed from ONLY to BOTH for vcpkg

# Emulator configuration for running tests
# Only set emulator if not already set (avoid double-setting when toolchain loads multiple times)
if(NOT CMAKE_CROSSCOMPILING_EMULATOR)
    option(USE_SPIKE_EMULATOR "Use Spike instead of QEMU for testing" OFF)

    if(USE_SPIKE_EMULATOR)
    # Spike with proxy kernel (slower, more accurate)
    find_program(SPIKE_EXECUTABLE spike)
    find_program(PK_EXECUTABLE pk PATHS /usr/local/bin /usr/bin)

    if(SPIKE_EXECUTABLE AND PK_EXECUTABLE)
        if(ENABLE_RVV)
            set(CMAKE_CROSSCOMPILING_EMULATOR ${SPIKE_EXECUTABLE} --isa=rv64gcv ${PK_EXECUTABLE})
        else()
            set(CMAKE_CROSSCOMPILING_EMULATOR ${SPIKE_EXECUTABLE} --isa=rv64gc ${PK_EXECUTABLE})
        endif()
        message(STATUS "Using Spike for emulation: ${SPIKE_EXECUTABLE}")
    else()
        message(WARNING "Spike or pk not found, tests may not run")
    endif()
else()
    # QEMU user-mode (faster, good enough for most testing)
    find_program(QEMU_EXECUTABLE qemu-riscv64)

    if(QEMU_EXECUTABLE)
        # QEMU user-mode auto-detects ISA extensions from the binary
        # No need to specify -cpu options for RVV support in QEMU 6.2+
        set(CMAKE_CROSSCOMPILING_EMULATOR ${QEMU_EXECUTABLE} -L ${CMAKE_FIND_ROOT_PATH})
        if(ENABLE_RVV)
            message(STATUS "Using QEMU for emulation (RVV support): ${QEMU_EXECUTABLE}")
        else()
            message(STATUS "Using QEMU for emulation: ${QEMU_EXECUTABLE}")
        endif()
    else()
        message(WARNING "QEMU not found, tests may not run")
    endif()
    endif()
endif()

# Sysroot - Don't set CMAKE_SYSROOT for Ubuntu cross-compilation packages
# The riscv64-linux-gnu-gcc compiler is already configured with the correct
# library paths at /usr/riscv64-linux-gnu/lib
# Setting CMAKE_SYSROOT would cause double-prefixing issues
