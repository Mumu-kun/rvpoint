#!/bin/bash
set -e

RISCV_PATH=${RISCV_PATH:-"/opt/riscv"}
TOOLCHAIN_URL="https://github.com/riscv-collab/riscv-gnu-toolchain/releases/download/2025.01.20/riscv64-glibc-ubuntu-22.04-gcc-nightly-2025.01.20-nightly.tar.xz"
ARCHIVE_NAME="riscv64-glibc-ubuntu-22.04-gcc-nightly-2025.01.20-nightly.tar.xz"

echo "Installing RISC-V Linux (glibc) Toolchain..."
echo "Target Directory: $RISCV_PATH"

# Create temp directory
WORKDIR=$(mktemp -d)
cd "$WORKDIR"

# Download
echo "Downloading toolchain from $TOOLCHAIN_URL..."
wget "$TOOLCHAIN_URL"

# Extract
echo "Extracting..."
if [ ! -d "$RISCV_PATH" ]; then
    mkdir -p "$RISCV_PATH"
fi
tar -xJf "$ARCHIVE_NAME" -C /opt

# Cleanup
cd /workspace
rm -rf "$WORKDIR"

echo "Installation complete."
echo "You can verify with: $RISCV_PATH/bin/riscv64-unknown-linux-gnu-gcc --version"
