# RVPoint Docker Environment Setup

Full setup of the RVPoint development environment using Docker only — no host-installed
RISC-V toolchain, CMake, or QEMU required. Everything (RISC-V GCC 14, QEMU 9.2 w/ RVV,
CMake, gem5) runs inside containers.

Run all commands from the project root: `/home/fatin-ishrak-arian/Capstone/rvpoint`

## 0. Prerequisites

- Docker Engine installed and running on the host.
- `/opt/gem5` present on the host — a gem5 source checkout with a prebuilt
  `build/RISCV/gem5.opt`. This is bind-mounted into the container; it is **not**
  built by any script here. If it doesn't exist, gem5 config files (`configs/...`)
  won't be available at that path — see the gem5 section below for the alternative
  Docker-image-based path that doesn't need it.

## 1. Build the image

```bash
docker build -f .devcontainer/Dockerfile -t rvpoint:latest .
```

This is a multi-stage build that compiles/stages, in parallel:
- CMake 3.28.0
- QEMU 9.2.0 (target `riscv64-linux-user`, full RVV v1.0 support)
- RISC-V GNU toolchain (GCC 14, `rv64gcv`/`lp64d`)

onto an `ubuntu:24.04` base, and verifies each tool at build time.

## 2. Create the dev container

```bash
docker run -d --name rvpoint-dev --privileged \
  -v /home/fatin-ishrak-arian/Capstone/rvpoint:/workspace \
  -v /opt/gem5:/opt/gem5 \
  -v /var/run/docker.sock:/var/run/docker.sock \
  rvpoint:latest tail -f /dev/null
```

- `--privileged` — required for kernel-level operations used by parts of the toolchain.
- `-v .../rvpoint:/workspace` — bind-mounts the project so edits on the host are
  immediately visible in the container (and vice versa).
- `-v /opt/gem5:/opt/gem5` — exposes the host gem5 checkout inside the container.
- `-v /var/run/docker.sock:/var/run/docker.sock` — gives the container access to the
  **host's** Docker daemon, so Docker-based tooling (the gem5 wrapper, see below) can
  be invoked from inside the workspace. Containers it spawns are siblings on the host
  daemon, not nested/child containers.

Do **not** rely on the `.devcontainer/devcontainer.json` `postCreateCommand`
(`scripts/download_datasets.sh`) — that script does not exist in this repo currently
and will fail if invoked through a devcontainer-aware tool (e.g. VS Code). The manual
`docker run` above avoids it entirely.

## 3. Install the Docker CLI inside the container

The base image has no `docker` client. Install it so the mounted socket is usable:

```bash
docker exec -it rvpoint-dev bash -c "apt-get update && apt-get install -y docker.io"
```

Verify it can talk to the host daemon:

```bash
docker exec -it rvpoint-dev bash -c "docker ps"
```

## 4. Enter the container / activate the environment

```bash
docker exec -it rvpoint-dev bash
```

The environment (`CMAKE_ROOT`, `RISCV`, `QEMU_CPU_FLAGS`, `PATH`, etc.) auto-activates
in interactive shells via `/etc/bash.bashrc` (which sources `/workspace/env/activate.sh`).
To activate manually in a non-interactive `exec`:

```bash
source /workspace/env/activate.sh
```

## 5. Verify the RISC-V + QEMU toolchain

```bash
docker exec -it rvpoint-dev bash -c "cd /workspace && ./scripts/verify_container.sh"
```

This builds the project (`scripts/build.sh --clean --toolchain linux`) and confirms
the RVV toolchain + QEMU work end to end. Pass `--test` / `-t` to also run the test suite:

```bash
docker exec -it rvpoint-dev bash -c "cd /workspace && ./scripts/verify_container.sh --test"
```

Manual one-off check (compiles and runs a trivial RVV instruction under QEMU):

```bash
docker exec -it rvpoint-dev bash -c '
  source /workspace/env/activate.sh
  cd /tmp
  cat > t.c << "EOF"
int main() { asm volatile("vsetvli zero,zero,e8,m1"); return 0; }
EOF
  riscv64-unknown-elf-gcc -march=$RISCV_ARCH -mabi=$RISCV_ABI -o t t.c
  qemu-riscv64 -cpu rv64,v=true,vlen=128 -L $RISCV/sysroot ./t && echo "RVV OK"
'
```

## 6. gem5 setup and verification

gem5 is **not** compiled locally by this project's scripts. It is used two ways:

### 6a. Via the `manuel313/gem5_v25` Docker image (the wrapper script's approach)

Pull the image (from inside the container, using the mounted host socket):

```bash
docker exec -it rvpoint-dev bash -c "docker pull manuel313/gem5_v25"
```

Run the project's wrapper, which mounts the project root + build dir into that image
and invokes `/gem5/build/RISCV/gem5.opt` inside it:

```bash
docker exec -it rvpoint-dev bash -c "cd /workspace && ./env/extras/gem5_docker.sh --version"
```

A version banner with no shared-library errors confirms gem5 is working.

### 6b. The bind-mounted `/opt/gem5` binary — known limitation

`/opt/gem5/build/RISCV/gem5.opt` (from the host bind mount) was compiled against
Python 3.10. Ubuntu 24.04 (the `rvpoint-dev` base image) ships Python 3.12 only —
`python3.10`/`libpython3.10` are **not available** in noble's repos, so this binary
cannot be made to run standalone inside `rvpoint-dev`:

```
error while loading shared libraries: libpython3.10.so.1.0: cannot open shared object file
```

Installing `python3.10`/`libpython3.10` via `apt-get` in this container is a dead
end — it silently resolves to nothing (`0 newly installed`) because the packages
don't exist for noble. Use option 6a instead; it's what all of the project's own
scripts (`GEM5_BIN`, `env/extras/install_gem5.sh`) are built around.

## 7. Building and running the project

Once inside the container (or via `docker exec`), standard project workflow:

```bash
# Build
./scripts/build.sh --clean --toolchain linux --backend rvv

# Run a specific target/test under QEMU
./scripts/run.sh test_voxel_grid
./scripts/run.sh benchmark

# Full build + test verification
./scripts/verify_container.sh --test
```

## 8. Tearing down / rebuilding clean

To fully reset (containers + images + volumes + build cache):

```bash
docker rm -f rvpoint-dev
docker image rm rvpoint:latest
docker volume prune -f
docker builder prune -a -f
```

Then repeat from step 1. Note: pruning images (`docker image prune -a -f`) also
removes `manuel313/gem5_v25` if it was pulled — re-pull it (step 6a) after any
full cleanup.
