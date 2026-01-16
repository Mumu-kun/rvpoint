# RISC-V Development Environment

This Docker development container provides a complete RISC-V development environment matching our Mac setup.

## What's Included

- **RISC-V GNU Toolchain** (riscv64-unknown-elf-gcc 14.2.0)
- **Spike RISC-V ISA Simulator** (v1.1.0)
- **RISC-V Proxy Kernel (pk)**
- Essential build tools (cmake, ninja, etc.)

## Prerequisites

1. **Docker Desktop** - [Download](https://www.docker.com/products/docker-desktop)
2. **Visual Studio Code** - [Download](https://code.visualstudio.com/)
3. **Dev Containers Extension** - Install from VS Code extensions

## Quick Start

1. Clone the repository:

```bash
   git clone <your-repo-url>
   cd rvpoint
```

2. Open in VS Code:

```bash
   code .
```

3. When prompted, click **"Reopen in Container"**

   - Or press `Cmd+Shift+P` and select "Dev Containers: Reopen in Container"

4. Wait for the container to build (first time takes ~15-20 minutes)

5. Verify installation in the VS Code terminal:

```bash
   riscv64-unknown-elf-gcc --version
   spike --version
```

## Usage Examples

### Compile a RISC-V Program

```bash
# Compile
riscv64-unknown-elf-gcc -o hello hello.c

# Run with Spike
spike pk hello
```

### Compile with Vector Extensions

```bash
riscv64-unknown-elf-gcc -march=rv64gcv -o vector_program vector_program.c
spike --isa=rv64gcv pk vector_program
```

## Troubleshooting

**Container build fails?**

- Ensure Docker Desktop is running
- Try: `docker system prune -a` to clean up space
- Rebuild: `Cmd+Shift+P` → "Dev Containers: Rebuild Container"

**Need to update packages?**

- Edit `.devcontainer/Dockerfile`
- Rebuild the container

## Environment Variables

- `RISCV=/opt/riscv` - Toolchain installation path
- `PATH` includes `/opt/riscv/bin`

## Contributing

When making changes to the dev environment:

1. Update the Dockerfile
2. Test locally by rebuilding the container
3. Document changes in this README
4. Commit and push
