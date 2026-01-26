# RVPoint Instruction Manual & Reference Guide

> **Purpose:** Comprehensive guide for understanding, building, and working with the RVPoint library.  
> **Audience:** Developers learning RISC-V Vector Extension (RVV) and point cloud processing.

---

## Table of Contents

1. [Codebase Architecture](#codebase-architecture)
2. [Target Hardware (Banana Pi BPI-F3)](#target-hardware-banana-pi-bpi-f3)
3. [Compilation Deep Dive](#compilation-deep-dive)
4. [Execution Flow](#execution-flow)
4. [RVV Programming Concepts](#rvv-programming-concepts)
5. [Testing & Verification](#testing--verification)
6. [Quick Reference](#quick-reference)

---

## Codebase Architecture

### Overview

RVPoint is a Point Cloud Library optimized for RISC-V processors using the Vector Extension (RVV). It demonstrates SIMD acceleration for 3D geometry algorithms.

### Data Structures

#### Array of Structures (AoS) - Scalar Code
```cpp
struct PointXYZ {
    float x, y, z;  // Coordinates grouped per point
}
// Memory: [x0,y0,z0][x1,y1,z1][x2,y2,z2]...
```

**Use case:** Traditional scalar processing, one point at a time.

#### Structure of Arrays (SoA) - Vector Code
```cpp
struct PointCloudSoA {
    float* x;  // All X coordinates contiguous
    float* y;  // All Y coordinates contiguous
    float* z;  // All Z coordinates contiguous
    size_t n;  // Number of points
}
// Memory X: [x0,x1,x2,x3,...]
// Memory Y: [y0,y1,y2,y3,...]
// Memory Z: [z0,z1,z2,z3,...]
```

**Use case:** Vector processing - load 8-32 coordinates at once!

**Why SoA for vectors?**
- Vector load `vle32.v` loads contiguous memory
- With SoA: Load 32 X coords in one instruction
- With AoS: Would need complex gather operations (slow)

### Component Structure

```
src/
├── include/
│   └── rvv_pcl.h           # Public API - all function declarations
├── rvv_common.cpp           # Reusable RVV kernels (distance calc)
├── voxel_grid_downsamp.cpp  # Voxel grid: _sc() and _rvv()
├── statistical_outlier_removal.cpp  # SOR: _sc() and _rvv()
├── normal_estimation.cpp    # Normals: _sc() and _rvv()
├── radius_search.cpp        # Radius: _sc() and _rvv()
└── ransac_plane.cpp         # RANSAC: _sc() and _rvv()

tests/
├── test_voxel_grid.cpp     # Test voxel downsampling
├── test_sor.cpp            # Test outlier removal
├── test_normal.cpp         # Test normal estimation
├── test_radius.cpp         # Test radius search
├── test_ransac.cpp         # Test RANSAC plane fitting
└── benchmark.cpp           # Performance benchmarking
```

**Pattern:** Every algorithm has two implementations:
- `*_sc()` - Scalar (reference implementation)
- `*_rvv()` - RVV vector (optimized implementation)

### The 5 Core Algorithms

| Algorithm | Input | Output | Optimization |
|-----------|-------|--------|--------------|
| **Voxel Grid** | Point cloud | Downsampled cloud | Vectorized coordinate scaling |
| **SOR** | Point cloud | Filtered cloud | Vectorized K-NN distance |
| **Normal Estimation** | Point cloud | Per-point normals | Vectorized covariance |
| **Radius Search** | Cloud + query | Neighbor indices | Vectorized distance + masks |
| **RANSAC** | Point cloud | Plane model | Vectorized inlier counting |

---

## Target Hardware: Banana Pi BPI-F3

**SoC:** SpacemiT K1 (8-core RISC-V)  
**Architecture:** RISC-V 64GCVB (RVA22 + RVV 1.0)  
**Vector Length (VLEN):** 256 bits

### Why this board?
- **RVV 1.0 Support:** Fully compatible with our codebase.
- **VLEN=256:** Processes 8 floats per instruction (vs 4 in our QEMU 128-bit simulation).
- **Auto-Scaling:** Our code uses `vsetvl`, so binaries compiled for generic RVV 1.0 will **automatically run 2x faster** on this board without recompilation!

---

## Compilation Deep Dive

### Compiler Selection: Linux vs Bare Metal

**For Banana Pi BPI-F3 (Linux):**
Use the **Linux GNU** toolchain. This allows standard file I/O, networking, and OS features.
```bash
riscv64-linux-gnu-g++ -mnative ... # (On board)
riscv64-linux-gnu-g++ ...          # (Cross-compile)
```

**For Bare Metal / Embedded:**
Use the **Unknown ELF** toolchain. No OS, no file I/O (unless semihosting).
```bash
riscv64-unknown-elf-g++ ...
```

### Basic Compilation Command (Linux Target)

```bash
riscv64-linux-gnu-g++ -march=rv64gcv -mabi=lp64d \
  -static \
  -I src/include \
  src/rvv_common.cpp \
  src/voxel_grid_downsamp.cpp \
  tests/test_voxel_grid.cpp \
  -o test_voxel_linux
```

**Key Flag:** `-static` is often needed when cross-compiling to run on QEMU user-mode without full library paths setup.

### Command Breakdown

#### 1. `riscv64-linux-gnu-g++` - The Linux Cross-Compiler

**What it is:**
- GNU C/C++ Compiler for RISC-V targets
- Runs on x86/ARM host, produces RISC-V binaries
- Version: GCC 14.2.0 (with RVV 1.0 support)

**Name breakdown:**
- `riscv64` = Target 64-bit RISC-V
- `unknown` = Vendor unspecified
- `elf` = Output ELF format (bare-metal)
- `gcc` = GNU Compiler Collection

**Why cross-compiler?**
You're on x86 Linux → Need to generate RISC-V code → Cross-compilation required!

#### 2. `-march=rv64gcv` - Architecture Specification

**Meaning:** "Machine Architecture = RISC-V 64-bit with extensions G, C, V"

**Extension breakdown:**
```
rv64gcv = rv64 + g + c + v
         │      │   │   └─ Vector Extension (RVV) ⭐
         │      │   └───── Compressed instructions (16-bit)
         │      └────────── General = imafd
         │                  i = Integer base
         │                  m = Multiply/Divide
         │                  a = Atomic operations
         │                  f = Single-precision float
         │                  d = Double-precision float
         └─────────────────── 64-bit base architecture
```

**Critical importance:**
- Enables vector instructions: `vle32.v`, `vfmul.vf`, `vse32.v`
- Without `v`: Compiler rejects `__riscv_vle32_v_f32m8()` intrinsics
- With `v`: Compiler generates SIMD code

**Try this experiment:**
```bash
# Without 'v' - will fail!
riscv64-unknown-elf-gcc -march=rv64gc src/rvv_common.cpp
# Error: unknown type name 'vfloat32m8_t'

# With 'v' - works!
riscv64-unknown-elf-gcc -march=rv64gcv src/rvv_common.cpp
# Success!
```

#### 3. `-mabi=lp64d` - Application Binary Interface

**Meaning:** "ABI where Long/Pointer = 64-bit, Double uses hardware float registers"

**ABI options:**

| ABI | Long/Ptr | Float Registers | Performance |
|-----|----------|----------------|-------------|
| `lp64` | 64-bit | None (soft float) | ❌ Slow |
| `lp64f` | 64-bit | Single-precision | ⚠️ OK |
| `lp64d` | 64-bit | Double-precision | ✅ **Fast** |

**What it controls:**
- Function calling conventions
- Register usage for arguments
- Stack frame layout

**Example difference:**

```cpp
float add(float a, float b) { return a + b; }
```

**With `-mabi=lp64d`:**
```assembly
# Args in float registers fa0, fa1
fadd.s fa0, fa0, fa1  # Fast!
ret
```

**With `-mabi=lp64`:**
```assembly
# Args in integer registers, must convert
mv     a5, a0
fmv.w.x fa5, a5      # Extra overhead
mv     a4, a1
fmv.w.x fa4, a4
fadd.s fa0, fa5, fa4
fmv.x.w a0, fa0
ret
```

**Rule of thumb:** Always use `lp64d` for float-heavy code!

#### 4. `-I src/include` - Include Directory

**Purpose:** Tell preprocessor where to find header files

**How it works:**
```cpp
// In your code:
#include "rvv_pcl.h"

// Compiler searches:
1. Current directory
2. src/include/        ← Added by -I flag
3. System paths (/opt/riscv/include)
```

**Without `-I src/include`:**
```bash
# Error: rvv_pcl.h: No such file or directory
```

**With `-I src/include`:**
```bash
# Success: Found at src/include/rvv_pcl.h
```

#### 5. Source Files - Input

**`src/rvv_common.cpp`**
- Contains: `get_dist_sq_rvv()` kernel
- Used by: Multiple algorithms
- Compiles to: Object file with vector instructions

**`src/voxel_grid_downsamp.cpp`**
- Contains: Scalar + RVV implementations
- Generates: Two code paths in same binary

**`tests/test_voxel_grid.cpp`**
- Contains: `main()`, test logic, verification
- Entry point: Program execution starts here

#### 6. `-o test_voxel_rvv` - Output Filename

**Purpose:** Name the output executable

- Without `-o`: Creates `a.out` (default name)
- With `-o test_voxel_rvv`: Creates `test_voxel_rvv`

### The 4 Compilation Stages

When you run the command, GCC performs 4 stages internally:

#### Stage 1: Preprocessing
```bash
# Conceptual command (GCC does this internally):
cpp -I src/include src/rvv_common.cpp > rvv_common.i
```

**Actions:**
- Expand `#include` directives
- Process macros (`#define`)
- Remove comments
- Output: `.i` file (all includes merged)

**Example:**
```cpp
// Before (in source):
#include "rvv_pcl.h"

// After (in .i file):
namespace rvv_pcl {
  struct PointXYZ { float x, y, z; };
  // ... 100+ lines from header
}
```

#### Stage 2: Compilation
```bash
# Conceptual:
cc1plus -march=rv64gcv -mabi=lp64d rvv_common.i > rvv_common.s
```

**Actions:**
- Parse C++ syntax → Abstract Syntax Tree (AST)
- Optimize intermediate representation
- Generate RISC-V assembly
- Output: `.s` file (human-readable assembly)

**Example output (rvv_common.s):**
```assembly
get_dist_sq_rvv:
    vsetvli a5, a3, e32, m8, ta, ma
    vle32.v v8, 0(a0)
    vle32.v v16, 0(a1)
    vle32.v v24, 0(a2)
    vfsub.vf v8, v8, fa0
    vfmul.vv v8, v8, v8
    ...
```

#### Stage 3: Assembly
```bash
# Conceptual:
as rvv_common.s -o rvv_common.o
```

**Actions:**
- Convert assembly text → binary machine code
- Encode instructions (e.g., `vle32.v` → `0x...`)
- Output: `.o` file (binary object code)

**Example:**
```
Assembly: vle32.v v8, 0(a0)
Binary:   02057407
```

#### Stage 4: Linking
```bash
# Conceptual:
ld rvv_common.o voxel_grid_downsamp.o test_voxel_grid.o \
   -lstdc++ -lm -o test_voxel_rvv
```

**Actions:**
- Combine all `.o` files
- Resolve function calls between files
- Link C++ standard library
- Link math library (`sqrt`, `floor`, etc.)
- Output: **Final executable**

**Result:**
```bash
$ file test_voxel_rvv
test_voxel_rvv: ELF 64-bit LSB executable, UCB RISC-V, version 1 (SYSV)
```

### Inspecting Compilation Output

**View preprocessor output:**
```bash
riscv64-unknown-elf-gcc -E -I src/include src/rvv_common.cpp > preprocessed.i
```

**View assembly:**
```bash
riscv64-unknown-elf-gcc -S -march=rv64gcv -mabi=lp64d \
  -I src/include src/rvv_common.cpp -o rvv_common.s
cat rvv_common.s
```

**View object file symbols:**
```bash
riscv64-unknown-elf-nm rvv_common.o
```

**Disassemble binary:**
```bash
riscv64-unknown-elf-objdump -d test_voxel_rvv | less
```

---

## Execution Flow

### QEMU Execution Command Breakdown

```bash
qemu-riscv64 -cpu rv64,v=true,vlen=128 test_voxel_rvv
```

Let's break down **every part** of this command:

---

#### 1️⃣ `qemu-riscv64` - The Emulator

**What it is:**
- **User-mode emulator** for RISC-V 64-bit binaries
- Part of QEMU (Quick EMUlator) project
- Runs RISC-V programs on x86/ARM hosts

**QEMU Variants:**

| Command | Mode | Purpose |
|---------|------|---------|
| `qemu-riscv64` | **User mode** | Run single RISC-V program (what we use) |
| `qemu-system-riscv64` | System mode | Emulate entire RISC-V machine + OS |

**Why user mode?**
- Faster (no full OS emulation)
- Direct syscall translation to host
- Perfect for running compiled binaries!

**How it works:**
1. Loads RISC-V ELF binary
2. Translates RISC-V instructions to host instructions (x86/ARM)
3. Executes on your actual CPU
4. Translates syscalls between RISC-V ↔ Linux

**Alternative names:**
- On some systems: `/opt/riscv/bin/qemu-riscv64`
- In containers: Usually in system `PATH`

---

#### 2️⃣ `-cpu rv64,v=true,vlen=128` - CPU Configuration

This is the **most critical part** - configures the emulated CPU.

**Breaking it down:**

##### **`rv64`** - Base CPU Type

**Meaning:** RISC-V 64-bit base architecture

**What it includes:**
- RV64I (Base integer instructions)
- Standard extensions loaded by default

**Alternative CPU models:**
```bash
-cpu rv64          # Basic RV64
-cpu max           # Maximum features (auto-enables everything)
-cpu rv64,v=true   # RV64 + Vector extension ✓
```

**Why not use `-cpu max`?**
- `max` enables ALL extensions
- May include experimental features
- Explicit configuration is clearer for testing!

---

##### **`v=true`** - Enable Vector Extension

**Meaning:** Turn on RVV (RISC-V Vector Extension)

**Critical importance:**
- **Without `v=true`**: Vector instructions cause "Illegal Instruction" error
- **With `v=true`**: QEMU emulates vector registers and instructions

**What it enables:**

| Component | Without v=true | With v=true |
|-----------|----------------|-------------|
| Vector registers | ❌ Not available | ✅ 32 vector registers (v0-v31) |
| Vector instructions | ❌ Illegal instruction | ✅ Emulated (vle, vadd, etc.) |
| Your RVV code | ❌ Crashes | ✅ Runs! |

**Example error without `v=true`:**
```bash
$ qemu-riscv64 -cpu rv64 test_voxel_rvv
qemu-riscv64: Invalid instruction 0x02007007 (vle32.v v8, (a0))
Illegal instruction (core dumped)
```

**With `v=true`:**
```bash
$ qemu-riscv64 -cpu rv64,v=true,vlen=128 test_voxel_rvv
Scalar Count: 951
RVV Count:    951
[PASS] ✓
```

**Note:** Some QEMU versions auto-enable `v` with certain `-cpu` models, but **explicit is always better!**

---

##### **`vlen=128`** - Vector Register Length

**Meaning:** Each vector register is **128 bits** wide

**What VLEN controls:**

```
VLEN = Width of each vector register in bits

Example with VLEN=128, element size=32 bits (float):
  One register holds: 128 / 32 = 4 floats

With LMUL=8 (using 8 registers):
  Total capacity = 4 × 8 = 32 floats per vector operation
```

**Common VLEN values:**

| VLEN | Elements per Register (f32) | Elements with m8 | Real Hardware |
|------|---------------------------|------------------|---------------|
| 128 | 4 | 32 | Embedded CPUs |
| 256 | 8 | 64 | Mid-range |
| 512 | 16 | 128 | High-performance |
| 1024 | 32 | 256 | Supercomputers |

**QEMU default:**
- If you omit `vlen=...`, QEMU uses **VLEN=128** (minimum spec)
- But **always specify explicitly** for reproducibility!

**Why VLEN matters:**

```cpp
size_t vl = __riscv_vsetvl_e32m8(1000);

With VLEN=128: vl = min(128/32 × 8, 1000) = min(32, 1000) = 32
With VLEN=256: vl = min(256/32 × 8, 1000) = min(64, 1000) = 64
With VLEN=512: vl = min(512/32 × 8, 1000) = min(128, 1000) = 128
```

**Hardware portability:**
- Code compiled with `-march=rv64gcv` runs on **any VLEN**
- RVV is designed to be VLEN-agnostic!
- Same binary works whether VLEN=128 or VLEN=1024

**Testing different VLENs:**
```bash
# Simulate embedded CPU (128-bit)
qemu-riscv64 -cpu rv64,v=true,vlen=128 test_voxel_rvv

# Simulate high-performance CPU (512-bit)
qemu-riscv64 -cpu rv64,v=true,vlen=512 test_voxel_rvv

# Same binary, different vector lengths!
```

---

#### 3️⃣ `test_voxel_rvv` - The Binary to Execute

**What it is:**
- ELF executable (compiled by riscv64-unknown-elf-gcc)
- Contains RISC-V machine code
- Includes RVV vector instructions

**File type check:**
```bash
$ file test_voxel_rvv
test_voxel_rvv: ELF 64-bit LSB executable, UCB RISC-V, version 1 (SYSV), 
                statically linked, not stripped
```

**What QEMU does with it:**
1. **Parse ELF headers** - Understand file structure
2. **Load sections** - Code, data, BSS into emulated memory
3. **Set entry point** - Usually `_start`, which calls `main()`
4. **Initialize registers** - Program counter, stack pointer, etc.
5. **Start execution** - Begin instruction-by-instruction emulation

---

### What QEMU Does Internally (Complete Flow)

#### **Phase 1: Initialization**

```
1. Parse command-line arguments
   ├─ CPU model: rv64
   ├─ Extensions: v=true
   └─ Config: vlen=128

2. Load ELF binary: test_voxel_rvv
   ├─ Read ELF header
   ├─ Load .text section (code) → 0x10000
   ├─ Load .data section (data) → 0x20000
   ├─ Setup .bss (uninitialized) → 0x30000
   └─ Map stack → 0x7ffffff000

3. Initialize CPU state
   ├─ Program Counter (PC) = entry point (e.g., 0x10400)
   ├─ Stack Pointer (SP) = 0x7ffffff000
   ├─ Vector Length (vl) = 0 (not set yet)
   └─ 32 general registers (x0-x31) = 0
   └─ 32 vector registers (v0-v31) = 0
```

#### **Phase 2: Instruction Execution Loop**

```
while (true) {
    // 1. Fetch instruction
    uint32_t instruction = read_memory(PC);
    
    // 2. Decode instruction
    if (is_vector_instruction(instruction)) {
        // e.g., vle32.v v8, (a0)
        decode_vector(instruction);
    } else if (is_scalar_instruction(instruction)) {
        // e.g., add x5, x6, x7
        decode_scalar(instruction);
    }
    
    // 3. Execute
    switch (instruction_type) {
        case VLE32:  // Vector load
            for (i = 0; i < vl; i++) {
                v_reg[dest][i] = memory[base_addr + i*4];
            }
            break;
            
        case VFMUL_VF:  // Vector × Scalar multiply
            for (i = 0; i < vl; i++) {
                v_reg[dest][i] = v_reg[src][i] * f_reg[scalar];
            }
            break;
    }
    
    // 4. Update PC
    PC += 4;  // Move to next instruction
    
    // 5. Check for syscalls
    if (instruction == ECALL) {
        handle_syscall();  // Translate to host OS
    }
}
```

#### **Phase 3: System Call Translation**

When your program does `std::cout << "Hello"`:

```
1. C++ calls write() syscall
   ├─ Syscall number in a7 register
   ├─ File descriptor in a0 (1 = stdout)
   ├─ Buffer pointer in a1
   └─ Length in a2

2. QEMU intercepts ECALL instruction
   ├─ Reads syscall number from a7
   └─ Translates RISC-V syscall → Host Linux syscall

3. QEMU calls host write()
   ├─ write(1, "Hello", 5)
   └─ Output appears on your terminal!

4. QEMU stores return value
   ├─ Result → a0 register
   └─ Resume execution
```

**Supported syscalls:**
- I/O: `read`, `write`, `open`, `close`
- Memory: `brk`, `mmap`, `munmap`
- Process: `exit`, `getpid`
- Time: `clock_gettime`, `gettimeofday`

#### **Phase 4: Vector Instruction Emulation**

**Example: `vle32.v v8, (a0)` - Load 32-bit vector**

```
1. QEMU sees instruction: 0x02007007
   
2. Decode:
   ├─ Opcode: VECTOR LOAD
   ├─ Element width: 32 bits
   ├─ Destination: v8
   ├─ Base address: register a0
   └─ Vector length: vl (from vl CSR)

3. Execute (loop in software):
   for (int i = 0; i < vl; i++) {
       uint32_t addr = a0 + i * 4;
       v8[i] = read_memory_32bit(addr);
   }
   
4. Update PC
   PC += 4;  // Move to next instruction
```

**Performance note:**
- QEMU emulates vector ops with **scalar loops**
- Not as fast as real hardware!
- But accurate for testing correctness

---

### Configuration Options Summary

**Minimal command:**
```bash
qemu-riscv64 test_voxel_rvv
# Uses defaults, but vectors won't work!
```

**Correct command:**
```bash
qemu-riscv64 -cpu rv64,v=true,vlen=128 test_voxel_rvv
# Explicit configuration, vectors work ✓
```

**Maximum features:**
```bash
qemu-riscv64 -cpu max test_voxel_rvv
# Enables all extensions (v=true included)
```

**Debug mode:**
```bash
qemu-riscv64 -cpu rv64,v=true,vlen=128 -d in_asm test_voxel_rvv
# Print every instruction executed (VERY verbose!)
```

**Trace mode (for benchmarking):**
```bash
qemu-riscv64 -cpu rv64,v=true,vlen=128 -d exec,nochain \
  test_voxel_rvv 2> trace.log
# Log translation blocks to trace.log
```

---

### QEMU vs Real Hardware

| Aspect | QEMU | Real RISC-V Hardware |
|--------|------|---------------------|
| **Speed** | Slower (emulated) | Fast (native) |
| **Vector ops** | Software loops | Parallel SIMD |
| **VLEN** | Configurable | Fixed per CPU |
| **Accuracy** | Instruction-perfect | Identical |
| **Use case** | Development, testing | Production |

**Why use QEMU?**
1. ✅ No need for expensive RISC-V hardware
2. ✅ Test different VLEN configurations
3. ✅ Easy debugging and tracing
4. ✅ Reproducible results
5. ✅ Available on any x86/ARM machine

**When to use real hardware?**
- Production deployment
- Performance benchmarking (real timing)
- Power consumption testing
- Hardware-specific features

---

### Common QEMU Errors & Solutions

#### Error 1: "Illegal instruction"
```bash
qemu-riscv64: Invalid instruction 0x02007007
```

**Cause:** Vector instructions without `v=true`

**Solution:**
```bash
qemu-riscv64 -cpu rv64,v=true,vlen=128 test_voxel_rvv
```

#### Error 2: "cannot execute binary file"
```bash
./test_voxel_rvv
bash: ./test_voxel_rvv: cannot execute binary file: Exec format error
```

**Cause:** Trying to run RISC-V binary directly on x86

**Solution:**
```bash
# Don't run directly:
./test_voxel_rvv  ❌

# Use QEMU:
qemu-riscv64 -cpu rv64,v=true,vlen=128 test_voxel_rvv  ✓
```

#### Error 3: "qemu-riscv64: command not found"
```bash
bash: qemu-riscv64: command not found
```

**Cause:** QEMU not installed or not in PATH

**Solution:**
```bash
# Check if installed:
which qemu-riscv64

# If in custom location:
/opt/riscv/bin/qemu-riscv64 -cpu rv64,v=true,vlen=128 test_voxel_rvv

# Or add to PATH:
export PATH="/opt/riscv/bin:$PATH"
```

---

### Complete Example: What Happens When You Run the Command

```bash
$ qemu-riscv64 -cpu rv64,v=true,vlen=128 test_voxel_rvv
```

**Step-by-step:**

```
1. Shell finds qemu-riscv64 in /usr/bin/

2. QEMU starts:
   ├─ Parse args: CPU=rv64, v=true, vlen=128
   └─ Target binary: test_voxel_rvv

3. Load binary:
   ├─ Read ELF: type=executable, arch=RISC-V 64
   ├─ Map .text @ 0x10000 (code: 8KB)
   ├─ Map .data @ 0x20000 (data: 2KB)
   └─ Setup stack @ 0x7ffffff000 (8MB)

4. Initialize CPU:
   ├─ PC = 0x10400 (_start function)
   ├─ SP = 0x7ffffff000
   ├─ Enable vector extension (v=true)
   └─ Set VLEN = 128 bits

5. Execute _start → main():
   ├─ Allocate test data (malloc → brk syscall)
   ├─ Generate 1000 random points
   ├─ Run voxel_grid_downsamp_sc() (scalar)
   │   └─ ~60,000 scalar instructions
   ├─ Run voxel_grid_downsamp_rvv() (vector)
   │   ├─ vsetvli a5, a3, e32, m8, ta, ma
   │   ├─ vle32.v v8, (a0)  ← QEMU loads 32 floats
   │   ├─ vfmul.vf v16, v8, fa0  ← QEMU multiplies 32 floats
   │   └─ ~12,000 vector instructions
   ├─ Compare results
   └─ Print "[PASS]" (write syscall)

6. Exit:
   ├─ main() returns 0
   ├─ exit(0) syscall
   └─ QEMU terminates

7. Shell displays output:
   Scalar Count: 951
   RVV Count:    951
   [PASS] Voxel Grid Verification Successful!
```

**Total time:** ~100ms (emulation overhead)  
**On real RISC-V HW:** ~10ms (10x faster!)

---

### Summary: QEMU Command Breakdown

| Part | Meaning | Why It Matters |
|------|---------|----------------|
| `qemu-riscv64` | User-mode RISC-V 64-bit emulator | Runs RISC-V programs on x86/ARM |
| `-cpu rv64` | Base CPU architecture | Defines instruction set baseline |
| `v=true` | Enable vector extension | **CRITICAL**: Without this, vector code crashes! |
| `vlen=128` | Vector register width = 128 bits | Controls how many elements processed per instruction |
| `test_voxel_rvv` | Binary to execute | Your compiled RISC-V program |

**In one sentence:** This command uses QEMU to emulate a RISC-V 64-bit CPU with 128-bit vector registers, allowing your x86 machine to run RISC-V binaries with vector instructions! 🚀

### Step-by-Step Execution: Voxel Grid Test

Let's trace execution of `test_voxel_rvv` line by line.

#### Program Start
```cpp
int main() {
    const size_t N = 1000;
    const float LEAF = 0.5f;
```

**QEMU:**
- Sets PC (program counter) to `main()` address
- Allocates stack frame
- Stores constants in registers

#### Data Generation
```cpp
std::vector<float> x(N), y(N), z(N);
std::vector<PointXYZ> input_aos(N);

std::mt19937 gen(42);
std::uniform_real_distribution<float> dist(0.0f, 10.0f);

for(size_t i=0; i<N; ++i) {
    x[i] = dist(gen);
    y[i] = dist(gen);
    z[i] = dist(gen);
    input_aos[i] = {x[i], y[i], z[i]};
}
```

**QEMU:**
1. Calls `malloc()` to allocate heap memory (12KB for 3×1000 floats)
2. Runs Mersenne Twister PRNG (generates 3000 random floats)
3. Memory layout after loop:

```
Heap Memory:

x[]:        [7.23, 2.45, 9.01, ...] (1000 floats, 4KB)
y[]:        [3.12, 8.76, 1.34, ...] (1000 floats, 4KB)
z[]:        [5.67, 0.89, 4.23, ...] (1000 floats, 4KB)

input_aos[]: [{7.23,3.12,5.67}, {2.45,8.76,0.89}, ...]
             (1000 structs, 12KB, interleaved XYZ)
```

#### Create SoA Structure
```cpp
PointCloudSoA input_soa = {x.data(), y.data(), z.data(), N};
```

**Memory (just pointers!):**
```
input_soa:
  .x → 0x12340000  (pointer to x[0])
  .y → 0x12341000  (pointer to y[0])
  .z → 0x12342000  (pointer to z[0])
  .n = 1000
```

No data copying! Efficient. ✅

#### Scalar Execution
```cpp
size_t count_sc = voxel_grid_downsamp_sc(
    input_aos.data(), N, out_sc.data(), LEAF
);
```

**Inside function (voxel_grid_downsamp.cpp:12):**

```cpp
float inv_leaf = 1.0f / leaf_size;  // = 2.0

for (size_t i = 0; i < n; ++i) {
    // Process ONE point at a time
    int vx = std::floor(in[i].x * inv_leaf);
    int vy = std::floor(in[i].y * inv_leaf);
    int vz = std::floor(in[i].z * inv_leaf);
    
    auto key = std::make_tuple(vx, vy, vz);
    grid[key].first.x += in[i].x;  // Accumulate
    grid[key].first.y += in[i].y;
    grid[key].first.z += in[i].z;
    grid[key].second++;  // Count
}
```

**QEMU executes (per iteration):**
```assembly
# Iteration i=0: Point {7.23, 3.12, 5.67}

flw   fa5, 0(a0)      # Load x[0] = 7.23
flw   fa4, 4(a0)      # Load y[0] = 3.12
flw   fa3, 8(a0)      # Load z[0] = 5.67

fmul.s fa5, fa5, fa2  # 7.23 * 2.0 = 14.46
fmul.s fa4, fa4, fa2  # 3.12 * 2.0 = 6.24
fmul.s fa3, fa3, fa2  # 5.67 * 2.0 = 11.34

fcvt.w.s a5, fa5      # floor → vx = 14
fcvt.w.s a4, fa4      # floor → vy = 6
fcvt.w.s a3, fa3      # floor → vz = 11

# ... map insertion (~50 instructions)
```

**Repeats 1000 times!** Total: ~60,000 instructions

#### RVV Vector Execution ⚡
```cpp
size_t count_rvv = voxel_grid_downsamp_rvv(
    input_soa, out_rvv.data(), LEAF
);
```

**Inside function (voxel_grid_downsamp.cpp:48):**

```cpp
while (i < n) {
    // STEP 1: Set vector length (dynamic!)
    size_t vl = __riscv_vsetvl_e32m8(n - i);
```

**QEMU executes:**
```assembly
vsetvli a5, a3, e32, m8, ta, ma
```

**Register state:**
```
a3 = n - i = 1000  (remaining elements)
After vsetvl:
  a5 = vl = 32     (will process 32 elements)
  
Why 32?
  VLEN = 128 bits
  Element size = 32 bits (float)
  LMUL = 8 (use 8 vector registers)
  vl = min(VLEN/32 × 8, remaining) = min(4×8, 1000) = 32
```

```cpp
    // STEP 2: Load 32 X values at once!
    vfloat32m8_t vx = __riscv_vle32_v_f32m8(&in.x[i], vl);
```

**QEMU executes:**
```assembly
vle32.v v8, (a0)  # Load 32 floats into v8-v15 (m8 = 8 regs)
```

**Vector register state:**
```
v8-v15 = [x[0], x[1], x[2], ..., x[31]]
       = [7.23, 2.45, 9.01, ..., 5.89]
       
All 32 values loaded in ONE instruction!
```

Similarly for Y and Z:
```cpp
    vfloat32m8_t vy = __riscv_vle32_v_f32m8(&in.y[i], vl);
    vfloat32m8_t vz = __riscv_vle32_v_f32m8(&in.z[i], vl);
```

```cpp
    // STEP 3: Multiply ALL 32 elements simultaneously!
    vfloat32m8_t vsx = __riscv_vfmul_vf_f32m8(vx, inv_leaf, vl);
```

**QEMU executes:**
```assembly
vfmul.vf v16, v8, fa0  # Multiply 32 floats by scalar 2.0
```

**Result:**
```
v16-v23 = [7.23*2.0, 2.45*2.0, 9.01*2.0, ..., 5.89*2.0]
        = [14.46, 4.90, 18.02, ..., 11.78]

32 multiplications in ONE instruction! 🚀
```

```cpp
    // STEP 4: Store back to memory
    float raw_sx[vl];
    __riscv_vse32_v_f32m8(raw_sx, vsx, vl);
```

**QEMU executes:**
```assembly
vse32.v v16, (a1)  # Store 32 floats
```

**Performance comparison for 1000 points:**

| Version | Loop Iterations | Instructions | Speedup |
|---------|----------------|--------------|---------|
| Scalar | 1000 | ~60,000 | 1x |
| RVV | 32 | ~12,000 | **5x faster** |

**Loop breakdown:**
- Iteration 1: Process points 0-31 (vl=32)
- Iteration 2: Process points 32-63 (vl=32)
- ...
- Iteration 31: Process points 960-991 (vl=32)
- Iteration 32: Process points 992-999 (vl=8, last chunk!)

---

## RVV Programming Concepts

### Key RVV Features

#### 1. Strip Mining Pattern
```cpp
size_t i = 0;
while (i < n) {
    size_t vl = __riscv_vsetvl_e32m8(n - i);
    
    // Process 'vl' elements
    vfloat32m8_t v = __riscv_vle32_v_f32m8(&data[i], vl);
    // ... operations
    
    i += vl;  // Move to next chunk
}
```

**Why strip mining?**
- Hardware vector length is variable
- Different CPUs have different VLEN (128, 256, 512, 1024 bits)
- Code automatically adapts to hardware!

#### 2. LMUL (Length Multiplier)
```cpp
vfloat32m1_t  // LMUL=1 (1 register group)
vfloat32m2_t  // LMUL=2 (2 register groups)  
vfloat32m4_t  // LMUL=4 (4 register groups)
vfloat32m8_t  // LMUL=8 (8 register groups) ✓ Max throughput
```

**Vector length formula:**
```
vl = min(requested, VLEN / element_size × LMUL)

Example (VLEN=128):
  m1: vl = min(n, 128/32 × 1) = min(n, 4)
  m8: vl = min(n, 128/32 × 8) = min(n, 32) ✓ Better!
```

**Rule of thumb:** Use m8 for maximum throughput (when registers available).

#### 3. Vector Intrinsics Naming Guide

Every RISC-V Vector (RVV) intrinsic follows a **strict naming pattern**. Understanding this pattern is crucial for reading and writing optimized code.

**General Pattern:**
`__riscv_<operation>_<operand_types>_<result_type><LMUL><policy>`

##### **Common Examples Breakdown**

| Intrinsic | Breakdown | Meaning |
|-----------|-----------|---------|
| `__riscv_vle32_v_f32m8` | `vle` (**V**ector **L**oad **E**lement)<br>`32` (32-bit)<br>`_v_` (Vector load)<br>`f32m8` (Float32, LMUL=8) | Load `vl` elements of 32-bit floats into a group of 8 vector registers. |
| `__riscv_vfmul_vv_f32m8`| `vfmul` (**V**ector **F**loat **MUL**)<br>`_vv_` (**V**ector x **V**ector)<br>`f32m8` (Float32, LMUL=8) | Multiply two vector groups element-wise and return the result. |
| `__riscv_vfmul_vf_f32m8`| `vfmul` (**V**ector **F**loat **MUL**)<br>`_vf_` (**V**ector x **F**loat scalar)<br>`f32m8` (Float32, LMUL=8) | Multiply every element in a vector by a single scalar float value. |
| `__riscv_vse32_v_f32m1` | `vse` (**V**ector **S**tore **E**lement)<br>`32` (32-bit)<br>`_v_` (Vector store)<br>`f32m1` (Float32, LMUL=1) | Store elements from a single vector register to memory. |

##### **Decoding the Components**

**1. Operand Type Suffixes (`_vv`, `_vf`, `_vx`)**
*   `_vv`: **V**ector-**V**ector (Operation between two vector registers).
*   `_vf`: **V**ector-**F**loat (Operation between a vector and a floating-point scalar).
*   `_vx`: **V**ector-**I**nteger (Operation between a vector and an integer scalar).
*   `_vs`: **V**ector-**S**calar (Reduction: Whole vector reduced into a single scalar element).

**2. Element Type Codes**
*   `f32`: 32-bit Floating Point (Standard `float`).
*   `i32`: 32-bit Signed Integer (`int32_t`).
*   `u32`: 32-bit Unsigned Integer (`uint32_t`).
*   `b4`: **B**oolean mask with 4-to-1 ratio (used for masks when LMUL=8).

**3. LMUL (Length Multiplier)**
Defines how many vector registers are grouped together to act as one large vector.
*   `m1`: 1 Register.
*   `m2`: 2 Registers grouped.
*   `m8`: 8 Registers grouped (Maximum throughput).

**4. Policy Suffixes (Optional)**
*   `_ta`: **T**ail **A**gnostic (Unused elements at the end are undefined).
*   `_ma`: **M**ask **A**gnostic (Masked-out elements are undefined).

---

#### 4. Vector Masks (Boolean Vectors)

**Example: Count inliers in RANSAC**
```cpp
vfloat32m8_t dist = /* ... compute distances ... */;

// Create masks: -threshold <= dist <= threshold
vbool4_t mask_le = __riscv_vmfle_vf_f32m8_b4(dist, thresh, vl);
vbool4_t mask_ge = __riscv_vmfge_vf_f32m8_b4(dist, -thresh, vl);
vbool4_t mask_in = __riscv_vmand_mm_b4(mask_le, mask_ge, vl);

// Count set bits (population count)
int count = __riscv_vcpop_m_b4(mask_in, vl);
```

**Assembly:**
```assembly
vmfle.vf v0, v8, fa0    # Compare: dist <= thresh → mask v0
vmfge.vf v1, v8, fa1    # Compare: dist >= -thresh → mask v1
vmand.mm v0, v0, v1     # AND masks → final inlier mask
vcpop.m  a0, v0         # Count set bits → inlier count!
```

**One instruction counts 32 comparisons!** 🎯

### Common RVV Patterns

#### Distance Calculation
```cpp
void get_dist_sq_rvv(const float* x, const float* y, const float* z,
                     float qx, float qy, float qz,
                     float* out_d2, size_t n) {
    size_t i = 0;
    while (i < n) {
        size_t vl = __riscv_vsetvl_e32m8(n - i);
        
        // Load coordinates
        vfloat32m8_t vx = __riscv_vle32_v_f32m8(&x[i], vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(&y[i], vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(&z[i], vl);
        
        // Compute differences
        vfloat32m8_t dx = __riscv_vfsub_vf_f32m8(vx, qx, vl);
        vfloat32m8_t dy = __riscv_vfsub_vf_f32m8(vy, qy, vl);
        vfloat32m8_t dz = __riscv_vfsub_vf_f32m8(vz, qz, vl);
        
        // Square
        vfloat32m8_t dx2 = __riscv_vfmul_vv_f32m8(dx, dx, vl);
        vfloat32m8_t dy2 = __riscv_vfmul_vv_f32m8(dy, dy, vl);
        vfloat32m8_t dz2 = __riscv_vfmul_vv_f32m8(dz, dz, vl);
        
        // Sum: d² = dx² + dy² + dz²
        vfloat32m8_t sum = __riscv_vfadd_vv_f32m8(dx2, dy2, vl);
        sum = __riscv_vfadd_vv_f32m8(sum, dz2, vl);
        
        // Store result
        __riscv_vse32_v_f32m8(&out_d2[i], sum, vl);
        i += vl;
    }
}
```

**Scalar equivalent:**
```cpp
for (size_t i = 0; i < n; ++i) {
    float dx = x[i] - qx;
    float dy = y[i] - qy;
    float dz = z[i] - qz;
    out_d2[i] = dx*dx + dy*dy + dz*dz;
}
```

**Performance:** RVV is ~8-10x faster!

---

## Testing & Verification

### Test Pattern

Every algorithm test follows this structure:

```cpp
int main() {
    // 1. Generate test data
    std::vector<float> x(N), y(N), z(N);
    std::vector<PointXYZ> input_aos(N);
    // ... random generation ...
    
    PointCloudSoA input_soa = {x.data(), y.data(), z.data(), N};
    
    // 2. Run scalar version
    size_t count_sc = algorithm_sc(input_aos.data(), N, out_sc.data(), params);
    
    // 3. Run RVV version
    size_t count_rvv = algorithm_rvv(input_soa, out_rvv.data(), params);
    
    // 4. Verify results match
    if (count_sc != count_rvv) {
        std::cerr << "[FAIL]" << std::endl;
        return 1;
    }
    
    for (size_t i = 0; i < count_sc; ++i) {
        if (!are_results_close(out_sc[i], out_rvv[i])) {
            std::cerr << "[FAIL]" << std::endl;
            return 1;
        }
    }
    
    std::cout << "[PASS]" << std::endl;
    return 0;
}
```

### Running Tests

**Individual test:**
```bash
# Build
riscv64-unknown-elf-gcc -march=rv64gcv -mabi=lp64d \
  -I src/include \
  src/rvv_common.cpp src/voxel_grid_downsamp.cpp \
  tests/test_voxel_grid.cpp \
  -o test_voxel

# Run
qemu-riscv64 -cpu rv64,v=true,vlen=128 test_voxel
```

**All tests:**
```bash
scripts/verify_container.sh
```

### Benchmarking

```bash
scripts/run_trace_benchmark.sh
```

**What it measures:**
- Instruction count (not runtime!)
- Uses QEMU tracing to count translation blocks
- Compares scalar vs RVV instruction efficiency

**Expected results:**
- Voxel Grid: ~3x reduction
- Normal Estimation: ~5x reduction
- RANSAC: ~4x reduction

---

## Quick Reference

### Compilation Commands

**Single test:**
```bash
riscv64-unknown-elf-gcc -march=rv64gcv -mabi=lp64d \
  -I src/include \
  src/rvv_common.cpp src/<algorithm>.cpp tests/test_<name>.cpp \
  -o test_<name>
```

**With CMake:**
```bash
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/riscv.cmake
make
```

### Execution Commands

**Run test:**
```bash
qemu-riscv64 -cpu rv64,v=true,vlen=128 <executable>
```

**View assembly:**
```bash
riscv64-unknown-elf-objdump -d <executable> | less
```

### Common Intrinsics

| Operation | Intrinsic | Assembly |
|-----------|-----------|----------|
| Set vector length | `__riscv_vsetvl_e32m8(n)` | `vsetvli` |
| Load vector | `__riscv_vle32_v_f32m8(ptr, vl)` | `vle32.v` |
| Store vector | `__riscv_vse32_v_f32m8(ptr, v, vl)` | `vse32.v` |
| Vec × Scalar | `__riscv_vfmul_vf_f32m8(v, s, vl)` | `vfmul.vf` |
| Vec + Vec | `__riscv_vfadd_vv_f32m8(v1, v2, vl)` | `vfadd.vv` |
| Vec - Scalar | `__riscv_vfsub_vf_f32m8(v, s, vl)` | `vfsub.vf` |
| Compare ≤ | `__riscv_vmfle_vf_f32m8_b4(v, s, vl)` | `vmfle.vf` |
| Count bits | `__riscv_vcpop_m_b4(mask, vl)` | `vcpop.m` |

### File Organization

```
Project Root
├── src/
│   ├── include/rvv_pcl.h       → API header
│   ├── rvv_common.cpp          → Shared kernels
│   └── <algorithm>.cpp         → Algorithm implementations
├── tests/
│   └── test_<algorithm>.cpp    → Test programs
├── scripts/
│   ├── verify_container.sh     → Full test suite
│   └── run_trace_benchmark.sh  → Performance analysis
├── CMakeLists.txt              → Build configuration
└── cmake/riscv.cmake           → Toolchain file
```

### Debugging Tips

**Check compiler version:**
```bash
riscv64-unknown-elf-gcc --version
# Should show GCC 14.x for RVV 1.0 support
```

**Verify QEMU supports vectors:**
```bash
qemu-riscv64 -cpu help | grep vector
# Should show: v=true - Enable Vector Extension
```

**Common errors:**

1. **"unknown type 'vfloat32m8_t'"**
   - Missing `-march=rv64gcv` flag
   - Add the `v` extension!

2. **Illegal instruction**
   - QEMU missing `v=true` flag
   - Add `-cpu rv64,v=true,vlen=128`

3. **Different results scalar vs RVV**
   - Floating point rounding differences (expected!)
   - Use epsilon comparison: `abs(a - b) < 1e-4`

---

## Team Integration & Usage Guide

This section is designed for team members who need to maintain, extend, or integrate the RVPoint library into their projects.

### 1. Standardized Development Workflow

To ensure performance consistency across the team, follow these steps:

#### **A. Clean Build (Targeting GCC 14)**
We use the Linux-GNU cross-compiler because it supports complex C++ features and dynamic linking.
```bash
# From the root of the project
rm -rf build_cmake && mkdir build_cmake && cd build_cmake
cmake -DCMAKE_C_COMPILER=/opt/riscv/bin/riscv64-unknown-linux-gnu-gcc \
      -DCMAKE_CXX_COMPILER=/opt/riscv/bin/riscv64-unknown-linux-gnu-g++ ..
make -j$(npos)
```

#### **B. Running with System Libraries**
Since we link against glibc, you **must** provide the sysroot path to QEMU so it can find `ld-linux-riscv64-lp64d.so.1`.
```bash
cd bin
/usr/bin/qemu-riscv64 -L /opt/riscv/sysroot -cpu rv64,v=true ./test_pipeline_walkthrough ../data/bunny.pcd
```

### 2. How to Function & Extend the Code

#### **Adding a New Algorithm**
When implementing a new point cloud filter (e.g., a "Pass-Through" filter), follow the established patterns:

1.  **Header (`src/include/rvv_pcl.h`)**:
    Add both Scalar and RVV declarations.
    ```cpp
    void pass_through_sc(const PointXYZ* in, size_t n, PointXYZ* out, float min_z, float max_z);
    void pass_through_rvv(const PointCloudSoA& in, PointXYZ* out, float min_z, float max_z);
    ```

2.  **Implementation**:
    - Use the **SoA (Structure of Arrays)** layout for the RVV path.
    - Use **LMUL=8** for maximum throughput.
    - Leverage existing kernels in `rvv_common.cpp` (like `get_dist_sq_rvv`) if applicable.

3.  **Spatial Search**:
    - For neighbor searches, prefer `SpatialHash` over `Octree` for datasets >10k points (it yields ~30% faster builds).
    ```cpp
    SpatialHash grid;
    grid.setInputCloud(soa, radius * 1.5f);
    grid.build();
    grid.radiusSearch(query, radius, indices, dists);
    ```

### 3. Serialization & Results
- **Results Directory**: All output images and PCDs are saved to the `results/` folder to keep the workspace clean.
- **Timestamps**: The pipeline automatically appends timestamps (e.g., `_20260126_103000`) to filenames. This allows for serialized benchmarking where you can track progress over multiple runs.

---

## Advanced Topics

### RVV 1.0 Features

**Fractional LMUL:**
```cpp
vfloat32mf2_t  // LMUL = 1/2 (half a register)
vfloat32mf4_t  // LMUL = 1/4
```

**Tuple Types:**
```cpp
vfloat32m1x2_t tuple = __riscv_vlseg2e32_v_f32m1x2(ptr, vl);
// Loads interleaved data in one instruction
```

**Auto-vectorization:**
```bash
# GCC 14 can auto-vectorize simple loops!
riscv64-unknown-elf-gcc -O3 -march=rv64gcv -S code.c
grep "vle\|vadd\|vse" code.s  # Should show vector instructions
```

### Performance Tuning

**Use maximum LMUL:**
```cpp
vfloat32m8_t  // Better throughput than m1/m2/m4
```

**Minimize scalar-vector transitions:**
```cpp
// Bad: Convert inside loop
for (i < n) {
    vl = vsetvl(n-i);
    v = vle32(...);
    scalar = extract_element(v, 0);  // ❌ Slow!
}

// Good: Process all elements vectorized
```

**Prefer fused operations:**
```cpp
// Instead of:
v = vfmul(va, vb, vl);
v = vfadd(v, vc, vl);

// Use fused multiply-add:
v = vfmacc(vc, va, vb, vl);  // ✓ Faster, one instruction
```

---

## Further Reading

- [RISC-V Vector Extension Spec](https://github.com/riscv/riscv-v-spec)
- [RVV Intrinsics Reference](https://github.com/riscv-non-isa/rvv-intrinsic-doc)
- [QEMU RISC-V](https://wiki.qemu.org/Documentation/Platforms/RISCV)
- Original PCL: [Point Cloud Library](https://pointclouds.org/)

---

**Last Updated:** 2026-01-25  
**Author:** RVPoint Development Team
