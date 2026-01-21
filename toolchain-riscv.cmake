set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR riscv64)

if(DEFINED ENV{RISCV_PATH})
    set(RISCV_PATH "$ENV{RISCV_PATH}")
else()
    set(RISCV_PATH "/opt/riscv")
endif()

set(CMAKE_C_COMPILER "${RISCV_PATH}/bin/riscv64-unknown-elf-gcc")
set(CMAKE_CXX_COMPILER "${RISCV_PATH}/bin/riscv64-unknown-elf-g++")

set(CMAKE_FIND_ROOT_PATH "${RISCV_PATH}/riscv64-unknown-elf")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(CMAKE_C_FLAGS "-march=rv64gcv -mabi=lp64d")
set(CMAKE_CXX_FLAGS "-march=rv64gcv -mabi=lp64d")
