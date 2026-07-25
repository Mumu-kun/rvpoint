# Custom RISC-V Vector Instructions for Octree Search

This directory contains a hardware/software co-design proposal for speeding up 3D point cloud radius search on customized RISC-V Vector (RVV) processors.

---

## 1. The Core Architecture
Tree-based spatial partitioning structures (like octrees) spend execution cycles on two phases:
1. **Tree Traversal (Scalar)**: Walking down branch nodes. This contains complex branch instructions and pointer chasing, which do not vectorize well.
2. **Leaf Verification (Vector)**: Calculating Euclidean distances for all points inside a leaf. This is mathematically heavy, highly parallel, and ideal for vectorization.

To optimize the **Leaf Verification** loop, we propose two custom vector instructions to reduce instruction counts, minimize register pressure, and eliminate data-dependency pipeline stalls.

---

## 2. Custom Instructions Specification

### A. `vdist3d.vf` (Fused 3D Squared Distance)
Fuses the subtraction, squaring, and accumulation of X, Y, and Z coordinate vectors into a single floating-point pipeline execution.

* **Formula**:
  $$vd[i] = (vx[i] - qx)^2 + (vy[i] - qy)^2 + (vz[i] - qz)^2$$
* **Assembler Syntax**:
  ```assembly
  vdist3d.vf vd, vs1, vs2, rs1
  ```
* **Operands**:
  - `vd` (destination/source): Holds input $x$-coordinates on call; receives output squared distances.
  - `vs1` (source vector): Holds input $y$-coordinates.
  - `vs2` (source vector): Holds input $z$-coordinates.
  - `rs1` (source scalar): Points to the base address of a 3-element float array holding the query coordinates `[qx, qy, qz]`.
* **Hardware Benefit**: Replaces **6 vector arithmetic instructions** (`vfsub` x3, `vfmul` x1, `vfmacc` x2), eliminating data-dependency pipeline bubbles.

---

### B. `vstore_compressed.v` (Fused Compress-Store)
Fuses vector mask filtering/compression and writing to memory, bypassing the need for intermediate destination vector registers.

* **Formula**:
  $$Memory[rs1++] = vs[i] \quad \text{for all } i \text{ where } v0.mask[i] \text{ is true}$$
* **Assembler Syntax**:
  ```assembly
  vstore_compressed.v vs, v0, rs1
  ```
* **Operands**:
  - `vs` (source vector): Vector register containing the elements (indices or distances) to store.
  - `v0` (mask vector): Implicit mask register (standard RVV convention).
  - `rs1` (source scalar): Holds the memory destination pointer.
* **Hardware Benefit**: Fuses a vector compress and a vector store. This removes the `vcompress.vm` latency and reduces vector register file writes.

---

## 3. Instruction Encoding Formats

RISC-V reserves the `custom-0` and `custom-1` opcode spaces for custom extensions.

| Instruction | Opcode | funct3 | funct7 | rs2 (vs2) | rs1 (rs1) | rd (vd) | Description |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **`vdist3d.vf`** | `0x0b` (custom-0) | `6` (OPMVV) | `0x00` | Vector $z$ (`vs2`) | Query Array ptr | Dest / Vector $x$ | Fused 3D distance |
| **`vstore_compressed.v`** | `0x2b` (custom-1) | `0` (OPMVX) | `0x00` | Vector `vs` | Dest pointer | `x0` (unused) | Fused compress/store float |
| **`vstore_compressed.vi`** | `0x2b` (custom-1) | `1` (OPMVX) | `0x00` | Vector `vs` | Dest pointer | `x0` (unused) | Fused compress/store int |

---

## 4. Software Usage & Compiler Integration

To avoid needing to rebuild or patch GCC/Clang, software developers can write these custom instructions directly in C++ using **generic assembly templates** (`.insn` assembler directives):

```cpp
// Fused 3D Distance Intrinsic (LMUL=8)
asm volatile (
    ".insn r 0x0b, 6, 0, %0, %2, %3"
    : "+v" (vd)
    : "v" (vd), "v" (vy), "v" (vz), "r" (query_arr)
);

// Fused Compress-Store Intrinsic
asm volatile (
    ".insn r 0x2b, 0, 0, x0, %1, %2"
    :
    : "r" (dst_pointer), "v" (vs), "v" (mask)
    : "memory"
);
```

---

## 5. Emulator (QEMU) Modification Guide

To simulate these instructions under QEMU, the developer must modify the RISC-V translation and execution engines:

1. **Decoder Setup (`target/riscv/translate.c`)**:
   Add decoder hooks for `OPCODE_CUSTOM_0` (`0x0b`) and `OPCODE_CUSTOM_1` (`0x2b`).
2. **Translate Function (`target/riscv/insn_trans/trans_custom.c.inc`)**:
   Add decoding logic to unpack vector registers and call helper functions:
   ```c
   static bool trans_vdist3d(DisasContext *ctx, arg_r *a) {
       // Generate code to call helper_vdist3d
       tcg_gen_gvec_4_ptr(...);
       return true;
   }
   ```
3. **Execution Helper (`target/riscv/vector_helper.c`)**:
   Define the hardware behavior in C:
   ```c
   void helper_vdist3d(void *vd, void *vs1, void *vs2, target_ulong rs1, CPURISCVState *env) {
       float *query = (float *)rs1;
       float qx = query[0], qy = query[1], qz = query[2];
       // Compute (x - qx)^2 + (y - qy)^2 + (z - qz)^2 for all lanes
   }
   ```
