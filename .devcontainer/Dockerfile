FROM ubuntu:22.04

# Environment version for layer caching
ARG ENVIRONMENT_VERSION=1.0.0
LABEL org.opencontainers.image.version="${ENVIRONMENT_VERSION}"

# === Layer 1: Base Image ===
ENV DEBIAN_FRONTEND=noninteractive

# === Layer 2: Core System Dependencies ===
RUN apt-get update && apt-get install -y --no-install-recommends \
    \
    # Core utilities
    bash \
    ca-certificates \
    curl \
    wget \
    git \
    zip \
    unzip \
    tar \
    xz-utils \
    pkg-config \
    \
    # Build system
    build-essential \
    cmake \
    ninja-build \
    \
    # Python
    python3 \
    python3-pip \
    python3-venv \
    \
    # QEMU build dependencies (only required if building QEMU from source)
    meson \
    libglib2.0-dev \
    libpixman-1-dev \
    \
    # Miscellaneous
    zlib1g-dev \
    libexpat-dev \
    \
    # Optional development utilities
    vim \
    htop

# Copy env directory for version manifest and package scripts
COPY env/ /opt/rvpoint-env/

# === Layer 3: QEMU (only changes when QEMU_VERSION changes) ===
WORKDIR /tmp
RUN cd /opt/rvpoint-env && \
    source versions.sh && \
    source linux/paths.sh && \
    source packages/qemu.sh && \
    install_qemu_linux

# === Layer 4: RISC-V Toolchain (only changes when RISCV_ELF_VERSION changes) ===
RUN cd /opt/rvpoint-env && \
    source versions.sh && \
    source linux/paths.sh && \
    source packages/riscv.sh && \
    install_riscv_toolchain_linux

# Set environment variables
ENV RISCV=/opt/riscv
ENV PATH="${RISCV}/bin:/opt/qemu/bin:${PATH}"

# Set working directory
WORKDIR /workspace

# Verify installations
RUN echo "Verifying RISC-V toolchain installation..." && \
    riscv64-unknown-elf-gcc --version && \
    qemu-riscv64 --version && \
    echo "RISC-V environment ready!"

# Default command
CMD ["/bin/bash"]

# === Layer 5: Welcome message (cosmetic - easily updated without rebuilding everything) ===
RUN echo '#!/bin/bash' > /usr/local/bin/welcome.sh && \
    echo 'echo "╔════════════════════════════════════════════════════════════╗"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║  RVPoint: RISC-V Vector Optimized Point Cloud Library      ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "╠════════════════════════════════════════════════════════════╣"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║  Environment Ready:                                        ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    - RISC-V GCC 14 Toolchain (rv64gcv)                     ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    - QEMU 9.2 (full RVV v1.0)                            ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    - CMake, Ninja, Python3                                 ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║                                                            ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║  Quick Commands:                                           ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    Verify All:   scripts/verify_container.sh               ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    Manual Build: mkdir build && cd build && cmake ..       ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    Run Tests:    qemu-riscv64 -L /opt/riscv/sysroot ...    ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "╚════════════════════════════════════════════════════════════╝"' >> /usr/local/bin/welcome.sh && \
    chmod +x /usr/local/bin/welcome.sh && \
    echo '/usr/local/bin/welcome.sh' >> /etc/bash.bashrc