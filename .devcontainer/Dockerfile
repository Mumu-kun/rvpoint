FROM ubuntu:22.04

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Layer 1: Basic system packages (includes QEMU 9.2 build dependencies)
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    git \
    wget \
    curl \
    zip \
    unzip \
    tar \
    pkg-config \
    python3 \
    python3-pip \
    python3-venv \
    autoconf \
    automake \
    autotools-dev \
    libmpc-dev \
    libmpfr-dev \
    libgmp-dev \
    gawk \
    bison \
    flex \
    texinfo \
    gperf \
    libtool \
    patchutils \
    bc \
    zlib1g-dev \
    libexpat-dev \
    device-tree-compiler \
    vim \
    nano \
    htop \
    gcc-riscv64-linux-gnu \
    g++-riscv64-linux-gnu \
    libglib2.0-dev \
    libpixman-1-dev \
    meson \
    && pip3 install --no-cache-dir pre-commit tomli \
    && rm -rf /var/lib/apt/lists/*

# Layer 1.5: Build QEMU 9.2.0 from source (required for full RVV v1.0 support)
# QEMU 7.2 has bugs with vfcvt_rtz, vfredosum, vmul/vadd integer vector ops.
# QEMU 9.2+ correctly implements the complete RVV v1.0 instruction set.
WORKDIR /tmp
RUN wget -q https://download.qemu.org/qemu-9.2.0.tar.xz \
    && tar xf qemu-9.2.0.tar.xz \
    && cd qemu-9.2.0 \
    && ./configure \
        --target-list=riscv64-linux-user \
        --prefix=/opt/qemu \
        --disable-system \
        --enable-linux-user \
        --disable-docs \
        --disable-gtk \
        --disable-sdl \
        --disable-vnc \
        --disable-capstone \
        --disable-spice \
        --disable-libusb \
    && make -j$(nproc) \
    && make install \
    && cd /tmp && rm -rf qemu-9.2.0 qemu-9.2.0.tar.xz

# Symlink QEMU 9.2 as the default qemu-riscv64
RUN ln -sf /opt/qemu/bin/qemu-riscv64 /usr/bin/qemu-riscv64

# Set environment variables
ENV RISCV=/opt/riscv
ENV PATH="${RISCV}/bin:/opt/qemu/bin:${PATH}"

# Layer 2: Install RISC-V GNU Toolchain (matching your version)
WORKDIR /tmp
RUN wget https://github.com/riscv-collab/riscv-gnu-toolchain/releases/download/2025.01.20/riscv64-elf-ubuntu-22.04-gcc-nightly-2025.01.20-nightly.tar.xz \
    && mkdir -p ${RISCV} \
    && tar -xJf riscv64-elf-ubuntu-22.04-gcc-nightly-2025.01.20-nightly.tar.xz -C /opt \
    && rm riscv64-elf-ubuntu-22.04-gcc-nightly-2025.01.20-nightly.tar.xz

# Layer 2.5: Install RISC-V Linux (glibc) Toolchain (GCC 14)
RUN wget https://github.com/riscv-collab/riscv-gnu-toolchain/releases/download/2025.01.20/riscv64-glibc-ubuntu-22.04-gcc-nightly-2025.01.20-nightly.tar.xz \
    && tar -xJf riscv64-glibc-ubuntu-22.04-gcc-nightly-2025.01.20-nightly.tar.xz -C /opt \
    && rm riscv64-glibc-ubuntu-22.04-gcc-nightly-2025.01.20-nightly.tar.xz

# Layer 3: Build and install Spike (RISC-V ISA Simulator)
RUN git clone https://github.com/riscv-software-src/riscv-isa-sim.git \
    && cd riscv-isa-sim \
    && git checkout v1.1.0 \
    && mkdir build && cd build \
    && ../configure --prefix=${RISCV} \
    && make -j$(nproc) \
    && make install \
    && cd /tmp && rm -rf riscv-isa-sim

# Layer 4: Build and install RISC-V Proxy Kernel (pk)
RUN git clone https://github.com/riscv-software-src/riscv-pk.git \
    && cd riscv-pk \
    && mkdir build && cd build \
    && ../configure --prefix=${RISCV} --host=riscv64-unknown-elf \
    && make -j$(nproc) \
    && make install \
    && cd /tmp && rm -rf riscv-pk

# Set working directory
WORKDIR /workspace

# Verify installations
RUN echo "Verifying RISC-V toolchain installation..." \
    && riscv64-unknown-elf-gcc --version \
    && qemu-riscv64 --version \
    && which spike \
    && echo "RISC-V environment ready!"

# Default command
CMD ["/bin/bash"]

# Layer 6: Welcome message (cosmetic - easily updated without rebuilding everything)
RUN echo '#!/bin/bash' > /usr/local/bin/welcome.sh && \
    echo 'echo "╔════════════════════════════════════════════════════════════╗"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║  RVPoint: RISC-V Vector Optimized Point Cloud Library      ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "╠════════════════════════════════════════════════════════════╣"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║  Environment Ready:                                        ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    - RISC-V GCC 14 Toolchain (rv64gcv)                     ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    - QEMU 9.2 (full RVV v1.0) & Spike Simulators          ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    - CMake, Ninja, Python3                                 ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║                                                            ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║  Quick Commands:                                           ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    Verify All:   scripts/verify_container.sh               ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    Manual Build: mkdir build && cd build && cmake ..       ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    Run Tests:    qemu-riscv64 -L /opt/riscv/sysroot ...   ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "╚════════════════════════════════════════════════════════════╝"' >> /usr/local/bin/welcome.sh && \
    chmod +x /usr/local/bin/welcome.sh && \
    echo '/usr/local/bin/welcome.sh' >> /etc/bash.bashrc
