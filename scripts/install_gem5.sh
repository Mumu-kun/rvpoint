#!/usr/bin/env bash
set -euo pipefail

GEM5_REPO="${GEM5_REPO:-https://github.com/gem5/gem5.git}"
GEM5_DIR="${GEM5_DIR:-$HOME/gem5}"
GEM5_BUILD="${GEM5_BUILD:-$GEM5_DIR/build/RISCV/gem5.opt}"

if [ "$(id -u)" -ne 0 ]; then
  SUDO="sudo"
else
  SUDO=""
fi

$SUDO apt-get update
$SUDO apt-get install -y \
  scons \
  python3-dev \
  libprotobuf-dev \
  protobuf-compiler \
  libboost-all-dev \
  swig \
  m4 \
  libgoogle-perftools-dev

if [ ! -d "$GEM5_DIR/.git" ]; then
  git clone "$GEM5_REPO" "$GEM5_DIR"
fi

cd "$GEM5_DIR"
scons build/RISCV/gem5.opt -j"$(nproc)"

echo "gem5 built at: $GEM5_BUILD"
echo "Set:"
echo "  export GEM5_BIN=$GEM5_BUILD"
echo "  export GEM5_CONFIG=$GEM5_DIR/configs/example/se.py"