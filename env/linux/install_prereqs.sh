#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../common.sh"
source "${SCRIPT_DIR}/../versions.sh"

LINUX_PREREQUISITES=(
    build-essential
    git
    curl
    wget
    ca-certificates
    dos2unix
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
    docker.io
)

install_prereqs() {
    echo "Installing Linux prerequisites..."
    apt-get update && apt-get install -y "${LINUX_PREREQUISITES[@]}"
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    install_prereqs
fi
