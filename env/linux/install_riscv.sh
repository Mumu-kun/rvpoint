#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../common.sh"
source "${SCRIPT_DIR}/../versions.sh"

install_riscv_toolchain_linux() {
    RISCV_ROOT=/opt/riscv
    RISCV_BIN=${RISCV_ROOT}/bin
    echo "Installing RISC-V toolchain ${RISCV_ELF_VERSION}..."
    
    if [ -f "${RISCV_BIN}/riscv64-unknown-elf-gcc" ]; then
        echo "RISC-V toolchain already installed at ${RISCV_BIN}"
        return 0
    fi
    
    mkdir -p ${RISCV_ROOT}
    cd /tmp
    
    wget "${RISCV_ELF_URL}"
    tar -xJf ${RISCV_ELF_TAR} -C /opt 2>/dev/null || tar -xJf ${RISCV_ELF_TAR}
    rm -f ${RISCV_ELF_TAR}
    
    wget "${RISCV_GLIBC_URL}"
    tar -xJf ${RISCV_GLIBC_TAR} -C /opt
    rm -f ${RISCV_GLIBC_TAR}
    
    echo "RISC-V toolchain installed to ${RISCV_ROOT}"
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    install_riscv_toolchain_linux
fi
