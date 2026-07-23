#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

source "${SCRIPT_DIR}/../common.sh"
source "${SCRIPT_DIR}/../versions.sh"
source "${SCRIPT_DIR}/install_prereqs.sh"
source "${SCRIPT_DIR}/install_cmake.sh"
source "${SCRIPT_DIR}/install_qemu.sh"
source "${SCRIPT_DIR}/install_riscv.sh"

setup_user() {
    if ! id -u rvpoint &>/dev/null; then
        useradd -m -s /bin/bash rvpoint
        echo "Created user 'rvpoint'"
    fi
}

verify_installation() {
    setup_env_paths
    echo "Verifying installation..."
    
    local temp_dir=$(mktemp -d)
    cd "$temp_dir"
    
    cat > test_rvv.c << 'EOF'
int main() {
    asm volatile("vsetvli zero,zero,e8,m1");
    return 0;
}
EOF

    ${RISCV_BIN}/riscv64-unknown-elf-gcc -march=${RISCV_ARCH} -mabi=${RISCV_ABI} -o test_rvv test_rvv.c
    
    if [ $? -ne 0 ]; then
        echo "Error: Failed to compile test program"
        rm -rf "$temp_dir"
        return 1
    fi

    local qemu_bin=$(find_qemu)
    if ! "$qemu_bin" -cpu rv64,v=true,vlen=128 -L ${RISCV_ROOT}/sysroot ./test_rvv; then
        echo "Error: QEMU execution failed"
        rm -rf "$temp_dir"
        return 1
    fi

    rm -rf "$temp_dir"
    echo "Verification passed"
    return 0
}

run_all() {
    install_prereqs
    download_cmake_linux
    install_qemu_linux
    install_riscv_toolchain_linux
    setup_user
    verify_installation
    echo "Linux environment setup complete!"
}

if [ $# -eq 0 ]; then
    run_all
else
    while [ $# -gt 0 ]; do
        case "$1" in
            --prereqs|-p)
                install_prereqs
                shift
                ;;
            --cmake|-c)
                download_cmake_linux
                shift
                ;;
            --qemu|-q)
                install_qemu_linux
                shift
                ;;
            --riscv|-r)
                install_riscv_toolchain_linux
                shift
                ;;
            --user|-u)
                setup_user
                shift
                ;;
            --verify|-v)
                verify_installation
                shift
                ;;
            --all|-a)
                run_all
                shift
                ;;
            *)
                echo "Unknown option: $1" >&2
                echo "Usage: $0 [--prereqs] [--cmake] [--qemu] [--riscv] [--user] [--verify] [--all]" >&2
                exit 1
                ;;
        esac
    done
fi