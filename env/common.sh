# === RVPoint Common Utilities ===
# Shared functions for platform detection, path conversion, and toolchain location

# --- Platform Detection ---
detect_platform() {
    if [ -f /.dockerenv ] || [ -n "$IN_RVPOINT_CONTAINER" ]; then
        echo "docker"
    elif grep -qi microsoft /proc/version 2>/dev/null; then
        echo "wsl2"
    else
        echo "linux"
    fi
}

is_windows_gitbash() {
    [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "win32" ]] || [[ "$OSTYPE" == "cygwin" ]]
}

is_wsl_available() {
    local wsl_distro="${1:-rvpoint}"
    wsl.exe -l -q 2>/dev/null | tr -d '\r\0' | grep -Fxq "$wsl_distro"
}

# --- Path Conversion ---
# Converts Windows paths to WSL format:
# E:/path/file -> /mnt/e/path/file
# /e/path/file -> /mnt/e/path/file (msys-style)
convert_to_wsl_path() {
    local path="$1"
    
    # Already in WSL format
    if [[ "$path" =~ ^/mnt/[a-z]/ ]]; then
        echo "$path"
        return 0
    fi
    
    # Handle msys-style path: /e/path -> /mnt/e/path
    if [[ "$path" =~ ^/[A-Za-z]/(.*)$ ]]; then
        local drive="${path:1:1}"
        local rest="${BASH_REMATCH[1]}"
        echo "/mnt/${drive,,}/${rest}"
        return 0
    fi
    
    # Handle Windows-style path: E:\path or E:/path
    if [[ "$path" =~ ^[A-Za-z]:[\\/] ]]; then
        local drive="${path:0:1}"
        local rest="${path:2}"
        # Replace backslashes with forward slashes
        rest="${rest//\\//}"
        echo "/mnt/${drive,,}/${rest}"
        return 0
    fi
    
    echo "$path"
}

# Quick path conversion using WSL's wslpath utility (if available)
wslpath_to_wsl() {
    local path="$1"
    convert_to_wsl_path "$path"
}

# --- QEMU Location ---
find_qemu() {
    if command -v qemu-riscv64-static &> /dev/null; then
        echo "qemu-riscv64-static"
    elif command -v qemu-riscv64 &> /dev/null; then
        echo "qemu-riscv64"
    elif [ -n "$QEMU_BIN" ] && [ -f "${QEMU_BIN}/qemu-riscv64" ]; then
        echo "${QEMU_BIN}/qemu-riscv64"
    elif [ -f "/opt/qemu/bin/qemu-riscv64" ]; then
        echo "/opt/qemu/bin/qemu-riscv64"
    else
        echo "Error: qemu-riscv64 not found" >&2
        echo "       Run: ./env/setup.sh" >&2
        exit 1
    fi
}

# --- Environment Activation ---
setup_env_paths() {
    COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    if [ -z "$PROJECT_ROOT" ]; then
        PROJECT_ROOT="$(cd "$COMMON_DIR/.." && pwd)"
    fi

    source "${COMMON_DIR}/versions.sh"

    CMAKE_ROOT=/opt/cmake
    QEMU_ROOT=/opt/qemu
    RISCV_ROOT=/opt/riscv

    export CMAKE_ROOT
    export QEMU_ROOT
    export RISCV="${RISCV_ROOT}"
    export RISCV_PATH="${RISCV_ROOT}"

    export PATH="${CMAKE_ROOT}/bin:${RISCV_ROOT}/bin:${QEMU_ROOT}/bin:${PATH}"

    export QEMU_CPU_FLAGS="-cpu rv64,v=true,vlen=128"
    export QEMU_SYSROOT_FLAGS="-L ${RISCV_ROOT}/sysroot"
    export RISCV_ARCH="${RISCV_ARCH:-rv64gcv}"
    export RISCV_ABI="${RISCV_ABI:-lp64d}"
    
    export GEM5_BIN="${GEM5_BIN:-${PROJECT_ROOT}/env/extras/gem5_docker.sh}"
    export GEM5_CONFIG="${GEM5_CONFIG:-/gem5/configs/deprecated/example/se.py}"
    
    QEMU_BIN="${QEMU_ROOT}/bin"
    RISCV_BIN="${RISCV_ROOT}/bin"
}

# --- Download Utilities ---
download_and_extract() {
    local url="$1"
    local dest="$2"
    local tarball=$(basename "$url")
    
    wget -q "$url"
    tar -xJf "$tarball" -C "$dest"
    rm -f "$tarball"
}

export -f detect_platform is_windows_gitbash is_wsl_available convert_to_wsl_path wslpath_to_wsl find_qemu setup_env_paths download_and_extract 2>/dev/null || true