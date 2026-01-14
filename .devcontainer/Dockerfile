# Dev Container for PCL-RISC-V Development
# Optimized for Docker layer caching

FROM ubuntu:22.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Layer 1: System packages (changes rarely - excellent caching)
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    git \
    wget \
    curl \
    python3 \
    python3-pip \
    gcc-riscv64-linux-gnu \
    g++-riscv64-linux-gnu \
    gcc-riscv64-unknown-elf \
    g++-riscv64-unknown-elf \
    qemu-user \
    qemu-system-misc \
    qemu-utils \
    device-tree-compiler \
    gdb \
    gdb-multiarch \
    valgrind \
    ccache \
    clang-format \
    cppcheck \
    doxygen \
    graphviz \
    git-lfs \
    vim \
    nano \
    htop \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Note: Spike is NOT included for faster setup. QEMU suffices for scalar/RVV.
# Install later if needed - see docs/SPIKE_SETUP.md

# Layer 2: Tool verification (fast, but separate for debugging)
RUN riscv64-linux-gnu-gcc --version && \
    qemu-riscv64 --version && \
    echo "Tools verified successfully"

# Layer 3: Global configuration (changes occasionally)
RUN ccache --max-size=2G && \
    git config --global core.editor "vim" && \
    git config --global init.defaultBranch main

WORKDIR /workspace

# Layer 4: Aliases (changes with project commands - separate for easy updates)
RUN echo 'alias ll="ls -lah"' >> /root/.bashrc && \
    echo 'alias build="./scripts/build.sh"' >> /root/.bashrc && \
    echo 'alias build-riscv="./scripts/build.sh --riscv"' >> /root/.bashrc && \
    echo 'alias build-rvv="./scripts/build.sh --riscv --rvv"' >> /root/.bashrc && \
    echo 'alias test-qemu="cd build && ctest --output-on-failure"' >> /root/.bashrc

# Layer 5: Welcome message (cosmetic - easily updated without rebuilding everything)
RUN echo '#!/bin/bash' > /usr/local/bin/welcome.sh && \
    echo 'echo "╔════════════════════════════════════════════════════════════╗"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║  PCL-RISC-V Development Container                         ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "╠════════════════════════════════════════════════════════════╣"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║  Available tools:                                          ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    - GCC/G++ (native)                                      ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    - riscv64-linux-gnu-gcc/g++ (cross-compiler)            ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    - QEMU (user-mode & system emulation)                   ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    - CMake, Ninja, ccache                                  ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║  Quick commands:                                           ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    build         - Native build                            ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    build-riscv   - RISC-V cross-compile (scalar)           ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    build-rvv     - RISC-V cross-compile (with RVV)         ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    test-qemu     - Run tests in QEMU                       ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "╚════════════════════════════════════════════════════════════╝"' >> /usr/local/bin/welcome.sh && \
    chmod +x /usr/local/bin/welcome.sh && \
    echo '/usr/local/bin/welcome.sh' >> /root/.bashrc

# Default command
CMD ["/bin/bash"]
