FROM ubuntu:22.04

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Layer 1: Basic system packages
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
    && rm -rf /var/lib/apt/lists/*

# Set environment variables
ENV RISCV=/opt/riscv
ENV PATH="${RISCV}/bin:${PATH}"

# Layer 2: Install RISC-V GNU Toolchain (matching your version)
WORKDIR /tmp
RUN wget https://github.com/riscv-collab/riscv-gnu-toolchain/releases/download/2024.09.03/riscv64-elf-ubuntu-22.04-gcc-nightly-2024.09.03-nightly.tar.gz \
    && mkdir -p ${RISCV} \
    && tar -xzf riscv64-elf-ubuntu-22.04-gcc-nightly-2024.09.03-nightly.tar.gz -C /opt \
    && rm riscv64-elf-ubuntu-22.04-gcc-nightly-2024.09.03-nightly.tar.gz

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
    && which spike \
    && echo "RISC-V environment ready!"

# Default command
CMD ["/bin/bash"]
