# === Version Manifest (single source of truth) ===
CMAKE_VERSION=3.28.0
QEMU_VERSION=9.2.0
QEMU_URL=https://download.qemu.org/qemu-${QEMU_VERSION}.tar.xz

RISCV_ELF_VERSION=2025.01.20
RISCV_ELF_TAR=riscv64-elf-ubuntu-22.04-gcc-nightly-${RISCV_ELF_VERSION}-nightly.tar.xz
RISCV_ELF_URL=https://github.com/riscv-collab/riscv-gnu-toolchain/releases/download/${RISCV_ELF_VERSION}/${RISCV_ELF_TAR}

RISCV_GLIBC_TAR=riscv64-glibc-ubuntu-22.04-gcc-nightly-${RISCV_ELF_VERSION}-nightly.tar.xz
RISCV_GLIBC_URL=https://github.com/riscv-collab/riscv-gnu-toolchain/releases/download/${RISCV_ELF_VERSION}/${RISCV_GLIBC_TAR}

SPIKE_VERSION=v1.1.0
SPIKE_REPO=https://github.com/riscv-software-src/riscv-isa-sim.git

# === Configuration ===
ENVIRONMENT_VERSION=1.0.0

RISCV_ARCH=rv64gcv
RISCV_ABI=lp64d
RISCV_SYSROOT=sysroot
QEMU_TARGET=riscv64-linux-user