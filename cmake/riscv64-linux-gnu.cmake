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

# Find root path
set(CMAKE_FIND_ROOT_PATH /usr/riscv64-linux-gnu)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Emulator configuration for running tests
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
        if(ENABLE_RVV)
            set(CMAKE_CROSSCOMPILING_EMULATOR ${QEMU_EXECUTABLE} -cpu rv64,v=true,vlen=256 -L ${CMAKE_FIND_ROOT_PATH})
        else()
            set(CMAKE_CROSSCOMPILING_EMULATOR ${QEMU_EXECUTABLE} -L ${CMAKE_FIND_ROOT_PATH})
        endif()
        message(STATUS "Using QEMU for emulation: ${QEMU_EXECUTABLE}")
    else()
        message(WARNING "QEMU not found, tests may not run")
    endif()
endif()

# Sysroot
if(EXISTS ${CMAKE_FIND_ROOT_PATH})
    set(CMAKE_SYSROOT ${CMAKE_FIND_ROOT_PATH})
endif()
