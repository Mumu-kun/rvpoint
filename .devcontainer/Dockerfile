# === Stage 1: Base System Prerequisites ===
FROM ubuntu:24.04 AS base

ARG ENVIRONMENT_VERSION=1.0.0
LABEL org.opencontainers.image.version="${ENVIRONMENT_VERSION}"

ENV DEBIAN_FRONTEND=noninteractive

# Copy common files and prereqs installer
COPY env/common.sh env/versions.sh /tmp/env/
COPY env/linux/install_prereqs.sh /tmp/env/linux/

# Install core apt prerequisites using single source of truth script
RUN bash /tmp/env/linux/install_prereqs.sh

# === Stage 2: CMake Builder ===
FROM base AS cmake-builder
COPY env/linux/install_cmake.sh /tmp/env/linux/
RUN bash /tmp/env/linux/install_cmake.sh

# === Stage 3: QEMU Builder ===
FROM base AS qemu-builder
COPY env/linux/install_qemu.sh /tmp/env/linux/
RUN bash /tmp/env/linux/install_qemu.sh

# === Stage 4: RISC-V Toolchain Builder ===
FROM base AS riscv-builder
COPY env/linux/install_riscv.sh /tmp/env/linux/
RUN bash /tmp/env/linux/install_riscv.sh

# === Stage 5: Final DevContainer Image ===
FROM base AS final

# Copy cached toolchain builds from dedicated stages
COPY --from=cmake-builder /opt/cmake /opt/cmake
COPY --from=qemu-builder /opt/qemu /opt/qemu
COPY --from=riscv-builder /opt/riscv /opt/riscv

# Clean up temp env scripts from base layer
RUN rm -rf /tmp/env

# Set environment variables
ENV CMAKE_ROOT=/opt/cmake
ENV QEMU_ROOT=/opt/qemu
ENV RISCV=/opt/riscv
ENV PATH="/opt/cmake/bin:/opt/riscv/bin:/opt/qemu/bin:${PATH}"

WORKDIR /workspace

# Verify installation inside image build
RUN cmake --version && \
    riscv64-unknown-elf-gcc --version && \
    qemu-riscv64 --version && \
    echo "RISC-V environment ready!"

# Automatically activate environment in interactive shells
RUN echo '[ -f /workspace/env/activate.sh ] && source /workspace/env/activate.sh' >> /etc/bash.bashrc

# Welcome banner
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
    echo 'echo "║    Verify All:   env/verify_container.sh                   ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    Manual Build: mkdir build && cd build && cmake ..       ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "║    Run Tests:    qemu-riscv64 -L /opt/riscv/sysroot ...    ║"' >> /usr/local/bin/welcome.sh && \
    echo 'echo "╚════════════════════════════════════════════════════════════╝"' >> /usr/local/bin/welcome.sh && \
    chmod +x /usr/local/bin/welcome.sh && \
    echo '/usr/local/bin/welcome.sh' >> /etc/bash.bashrc

CMD ["/bin/bash"]