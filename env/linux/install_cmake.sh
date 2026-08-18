#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../common.sh"
source "${SCRIPT_DIR}/../versions.sh"

download_cmake_linux() {
    setup_env_paths
    CMAKE_ROOT=/opt/cmake
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

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    download_cmake_linux
fi
