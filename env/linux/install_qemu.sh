#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../common.sh"
source "${SCRIPT_DIR}/../versions.sh"

install_qemu_linux() {
    setup_env_paths
    QEMU_ROOT=/opt/qemu
    QEMU_BIN=${QEMU_ROOT}/bin
    QEMU_TARGET=riscv64-linux-user
    QEMU_URL=https://download.qemu.org/qemu-${QEMU_VERSION}.tar.xz
    echo "Installing QEMU ${QEMU_VERSION}..."
    
    if [ -f "${QEMU_BIN}/qemu-riscv64" ]; then
        echo "QEMU already installed at ${QEMU_BIN}"
        return 0
    fi
    
    if command -v qemu-riscv64-static &>/dev/null; then
        echo "Using system QEMU (qemu-user-static)"
        return 0
    fi
    if dpkg -l | grep -q qemu-user-static 2>/dev/null; then
        echo "Using system QEMU (qemu-user-static)"
        return 0
    fi
    
    echo "Building QEMU from source to ${QEMU_ROOT}..."
    mkdir -p /tmp/qemu-build
    cd /tmp/qemu-build
    
    wget -q "${QEMU_URL}"
    tar xf qemu-${QEMU_VERSION}.tar.xz
    cd qemu-${QEMU_VERSION}
    
    ./configure \
        --target-list=${QEMU_TARGET} \
        --prefix="${QEMU_ROOT}" \
        --disable-system \
        --enable-linux-user \
        --disable-docs \
        --disable-gtk \
        --disable-sdl \
        --disable-vnc \
        --disable-capstone \
        --disable-spice \
        --disable-libusb
    
    if make -j$(nproc); then
        make install
        cd /
        rm -rf /tmp/qemu-build
        echo "QEMU ${QEMU_VERSION} installed successfully"
    else
        echo "QEMU build from source failed, falling back to system package..."
        rm -rf /tmp/qemu-build
        apt-get install -y qemu-user-static qemu-system-misc
        echo "QEMU installed from system packages"
    fi
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    install_qemu_linux
fi
