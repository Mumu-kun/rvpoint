#!/bin/bash
# Optional: Spike/pk installer
# Run this if you need Spike or Proxy Kernel support

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/../common.sh"
source "${SCRIPT_DIR}/../versions.sh"
setup_env_paths

echo "Installing Spike ${SPIKE_VERSION}..."

cd /tmp
git clone "${SPIKE_REPO}"
cd riscv-isa-sim
git checkout "${SPIKE_VERSION}"
mkdir build && cd build
../configure --prefix=${RISCV_ROOT}
make -j$(nproc)
make install
cd /tmp && rm -rf riscv-isa-sim

echo "Installing Proxy Kernel..."

cd /tmp
git clone https://github.com/riscv-software-src/riscv-pk.git
cd riscv-pk
mkdir build && cd build
../configure --prefix=${RISCV_ROOT} --host=riscv64-unknown-elf
make -j$(nproc)
make install
cd /tmp && rm -rf riscv-pk

echo "Spike and Proxy Kernel installed to ${RISCV_ROOT}"