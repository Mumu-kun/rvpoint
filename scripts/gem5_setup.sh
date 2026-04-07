#!/bin/bash
# =============================================================================
#  gem5 One-Time Setup Script
#  Builds gem5 with RISC-V support inside the rvpoint container.
#  Run once: ./scripts/rvpoint.sh  → option 6 (gem5 Setup)
#
#  Build time: ~30-45 minutes (downloads ~1 GB, compiles ~2 GB)
#  Install path: /opt/gem5
# =============================================================================

set -euo pipefail

GEM5_DIR="/opt/gem5"
GEM5_VERSION="v24.0.0.1"          # Tested tag; supports RVV basic instructions
GEM5_BIN="${GEM5_DIR}/build/RISCV/gem5.opt"

RED='\033[0;31m'; GREEN='\033[0;32m'; CYAN='\033[0;36m'; RESET='\033[0m'
info()    { echo -e "${CYAN}==>${RESET} $*"; }
success() { echo -e "${GREEN}[OK]${RESET} $*"; }
error()   { echo -e "${RED}[ERR]${RESET} $*" >&2; }

# ── Already built? ────────────────────────────────────────────────────────────
if [ -f "$GEM5_BIN" ]; then
    success "gem5 already built at ${GEM5_BIN}"
    "$GEM5_BIN" --version 2>/dev/null | head -1 || true
    exit 0
fi

# ── Dependencies ──────────────────────────────────────────────────────────────
info "Installing gem5 build dependencies..."
apt-get update -q
apt-get install -y -q \
    python3-dev python3-six python3-pydot \
    libprotobuf-dev protobuf-compiler \
    libgoogle-perftools-dev \
    scons m4 zlib1g-dev \
    python3-pip
pip3 install -q scons 2>/dev/null || true

# ── Clone ─────────────────────────────────────────────────────────────────────
if [ ! -d "$GEM5_DIR/.git" ]; then
    info "Cloning gem5 ${GEM5_VERSION} (shallow clone)..."
    git clone --depth=1 --branch "${GEM5_VERSION}" \
        https://github.com/gem5/gem5.git "${GEM5_DIR}"
else
    info "gem5 source already present at ${GEM5_DIR}, skipping clone."
fi


# ── Job count: scale with available RAM, cap at nproc ─────────────────────────
# gem5py_m5 and gem5.opt link steps each need ~3-4 GB.
# Rule: 1 job per ~1.5 GB available VM (generous for compile steps, safe for link).
# Upper bound: number of logical CPUs (no point exceeding hardware parallelism).
AVAIL_MB=$(awk '/MemAvailable/ {printf "%d", $2/1024}' /proc/meminfo)
SWAP_FREE_MB=$(awk '/SwapFree/ {printf "%d", $2/1024}' /proc/meminfo)
TOTAL_VM_MB=$(( AVAIL_MB + SWAP_FREE_MB ))
NPROC=$(nproc 2>/dev/null || echo 8)
# gem5's generated files (inst-constrs.cc, ~300K lines) need 4-6 GB each to compile.
# Use 1 job per 4 GB to keep peak memory under control even during heavy compile phases.
SAFE_JOBS=$(( TOTAL_VM_MB / 4000 ))
[ "$SAFE_JOBS" -lt 1 ]       && SAFE_JOBS=1
[ "$SAFE_JOBS" -gt "$NPROC" ] && SAFE_JOBS=$NPROC
info "Available virtual memory: ~${TOTAL_VM_MB} MB | CPUs: ${NPROC} → using ${SAFE_JOBS} parallel jobs"

# ── Build ─────────────────────────────────────────────────────────────────────
info "Building gem5 RISC-V (this takes 15-45 minutes with ${SAFE_JOBS} jobs)..."
info "Monitor in another terminal: docker exec -it rvpoint-dev tail -f /opt/gem5/build_log.txt"

cd "${GEM5_DIR}"
python3 "$(which scons)" \
    build/RISCV/gem5.opt \
    -j"${SAFE_JOBS}" \
    --ignore-style \
    2>&1 | tee /opt/gem5/build_log.txt

if [ ! -f "$GEM5_BIN" ]; then
    error "gem5 build failed. Check /opt/gem5/build_log.txt for details."
    exit 1
fi

success "gem5 built successfully at ${GEM5_BIN}"
"$GEM5_BIN" --version 2>/dev/null | head -1 || true

echo ""
echo "  ┌─────────────────────────────────────────────────────────┐"
echo "  │  gem5 is ready. You can now run:                        │"
echo "  │    rvpoint.sh → option 7  (gem5 Benchmark)              │"
echo "  └─────────────────────────────────────────────────────────┘"
