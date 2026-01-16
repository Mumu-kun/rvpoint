# Dev Container for PCL-RISC-V Development
# Optimized for Docker layer caching

FROM ubuntu:22.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Copy Bangladesh mirrors configuration with fallbacks
COPY sources.list /etc/apt/sources.list

# Layer 1: System packages (changes rarely - excellent caching)
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
    gcc-riscv64-linux-gnu \
    g++-riscv64-linux-gnu \
    binutils-riscv64-linux-gnu \
    qemu-user \
    gdb \
    gdb-multiarch \
    valgrind \
    ccache \
    clang-format \
    cppcheck \
    && rm -rf /var/lib/apt/lists/*

# Note: Spike is NOT included for faster setup. QEMU suffices for scalar/RVV.
# Install later if needed - see docs/SPIKE_SETUP.md

# Layer 2: Tool verification (fast, but separate for debugging)
RUN riscv64-linux-gnu-gcc --version && \
    qemu-riscv64 --version && \
    echo "Tools verified successfully"

# Layer 3: Python tools (changes occasionally)
RUN pip3 install --no-cache-dir pre-commit

# Layer 3.5: Install vcpkg
RUN git clone https://github.com/microsoft/vcpkg.git /opt/vcpkg && \
    /opt/vcpkg/bootstrap-vcpkg.sh && \
    ln -s /opt/vcpkg/vcpkg /usr/local/bin/vcpkg && \
    mkdir -p /opt/vcpkg/cmake

ENV VCPKG_ROOT=/opt/vcpkg

# Layer 4: Global configuration (changes occasionally)
RUN ccache --max-size=2G && \
    git config --global core.editor "vim" && \
    git config --global init.defaultBranch main

WORKDIR /workspace
RUN chmod 777 /workspace


# Layer 5: Aliases (changes with project commands - separate for easy updates)
RUN echo 'alias ll="ls -lah"' >> /root/.bashrc && \
    echo 'alias build="./scripts/build.sh"' >> /root/.bashrc && \
    echo 'alias build-riscv="./scripts/build.sh --riscv"' >> /root/.bashrc && \
    echo 'alias build-rvv="./scripts/build.sh --riscv --rvv"' >> /root/.bashrc && \
    echo 'alias test-qemu="cd build && ctest --output-on-failure"' >> /root/.bashrc && \
    echo 'alias run="./scripts/run.sh"' >> /root/.bashrc && \
    echo 'alias dev="./scripts/dev.sh"' >> /root/.bashrc

# Layer 6: Welcome message (cosmetic - easily updated without rebuilding everything)
RUN echo '#!/bin/bash' > /usr/local/bin/welcome.sh && \
    echo 'echo "╔════════════════════════════════════════════════════════════╗"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║  PCL-RISC-V Development Container                          ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "╠════════════════════════════════════════════════════════════╣"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║  Available tools:                                          ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    - GCC/G++ (native)                                      ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    - riscv64-linux-gnu-gcc/g++ (cross-compiler)            ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    - QEMU (user-mode & system emulation)                   ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    - CMake, Ninja, ccache, vcpkg                           ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    - spdlog, tl-expected (via vcpkg)                       ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║  Quick commands:                                           ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    build         - Native build                            ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    build-riscv   - RISC-V cross-compile (scalar)           ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    build-rvv     - RISC-V cross-compile (with RVV)         ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    test-qemu     - Run tests in QEMU                       ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    run           - Run executables (native/RISC-V)         ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    dev           - Smart build & run with ccache           ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║                  --tests, --examples, --benchmarks         ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║                  --native, --riscv, --qemu, --spike        ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "╚════════════════════════════════════════════════════════════╝"' >> /usr/local/bin/welcome.sh && \
    chmod +x /usr/local/bin/welcome.sh && \
    echo '/usr/local/bin/welcome.sh' >> /root/.bashrc

# Default command
CMD ["/bin/bash"]
