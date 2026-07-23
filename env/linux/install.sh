#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

source "${SCRIPT_DIR}/../common.sh"
source "${SCRIPT_DIR}/../versions.sh"

LINUX_PREREQUISITES=(
    build-essential
    git
    curl
    wget
    unzip
    zip
    tar
    python3
    python3-pip
    python3-venv
    pkg-config
    meson
    ninja-build
    libglib2.0-dev
    libpixman-1-dev
)

download_cmake_linux() {
    setup_env_paths
    CMAKE_ROOT=/opt/cmake-${CMAKE_VERSION}
    CMAKE_BIN=${CMAKE_ROOT}/bin
    echo "Installing CMake ${CMAKE_VERSION}..."

    if [ -f "${CMAKE_BIN}/cmake" ]; then
        echo "CMake already installed at ${CMAKE_BIN}"
        return 0
    fi

    if command -v cmake &> /dev/null; then
        local system_version=$(cmake --version | head -1 | grep -oE '[0-9]+\.[0-9]+' | head -1)
        if [ -n "$system_version" ]; then
            local min_version="${CMAKE_VERSION}"
            if [ "$(printf '%s\n' "$system_version" "$min_version" | sort -V | head -1)" = "$min_version" ]; then
                echo "System CMake ${system_version} is adequate (>= ${CMAKE_VERSION})"
                return 0
            fi
            echo "System CMake ${system_version} is outdated, downloading ${CMAKE_VERSION}..."
        fi
    fi

    mkdir -p /tmp/cmake-build
    cd /tmp/cmake-build

    local url="https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-x86_64.sh"

    mkdir -p "${CMAKE_ROOT}"
    wget -q "$url" -O cmake-installer.sh
    bash cmake-installer.sh --skip-license --prefix="${CMAKE_ROOT}" --exclude-subdir
    rm cmake-installer.sh

    cd /
    rm -rf /tmp/cmake-build

    echo "CMake ${CMAKE_VERSION} installed successfully"
}

install_qemu_linux() {
    setup_env_paths
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
    if dpkg -l | grep -q qemu-user-static; then
        echo "Using system QEMU (qemu-user-static)"
        return 0
    fi
    
    echo "Building QEMU from source to env/.tools/qemu..."
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

verify_installation() {
    setup_env_paths
    echo "Verifying installation..."
    
    local temp_dir=$(mktemp -d)
    cd "$temp_dir"
    
    cat > test_rvv.c << 'EOF'
int main() {
    asm volatile("vsetvli zero,zero,e8,m1");
    return 0;
}
EOF

    ${RISCV_BIN}/riscv64-unknown-elf-gcc -march=${RISCV_ARCH} -mabi=${RISCV_ABI} -o test_rvv test_rvv.c
    
    if [ $? -ne 0 ]; then
        echo "Error: Failed to compile test program"
        rm -rf "$temp_dir"
        return 1
    fi

    local qemu_bin=$(find_qemu)
    if ! "$qemu_bin" -cpu rv64,v=true,vlen=128 -L ${RISCV_ROOT}/sysroot ./test_rvv; then
        echo "Error: QEMU execution failed"
        rm -rf "$temp_dir"
        return 1
    fi

    rm -rf "$temp_dir"
    echo "Verification passed"
    return 0
}

echo "Installing Linux prerequisites..."
apt-get update && apt-get install -y "${LINUX_PREREQUISITES[@]}"

echo "Installing CMake..."
download_cmake_linux

echo "Installing QEMU..."
install_qemu_linux

echo "Installing RISC-V toolchain..."
install_riscv_toolchain_linux

if ! id -u rvpoint &>/dev/null; then
    useradd -m -s /bin/bash rvpoint
    echo "Created user 'rvpoint'"
fi

verify_installation

echo "Linux environment setup complete!"