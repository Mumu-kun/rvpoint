# RISC-V Vector (RVV) Implementation Guide

This document explains the vector operations used in the project, specifically focusing on the `get_dist_sq_rvv` function found in `src/rvv_common.cpp`.

## Core Logic: `get_dist_sq_rvv`

The goal of this function is to calculate the squared Euclidean distance between a single query point $(qx, qy, qz)$ and a large array of points $(x[i], y[i], z[i])$.

$$ d^2 = (x_i - q_x)^2 + (y_i - q_y)^2 + (z_i - q_z)^2 $$

This is an "embarrassingly parallel" problem perfect for SIMD (Single Instruction, Multiple Data).

### The Code Breakdown

```cpp
void get_dist_sq_rvv(const float* x, const float* y, const float* z,
                     float qx, float qy, float qz,
                     float* out_d2, std::size_t n)
{
  std::size_t i = 0;
  while (i < n) {
    // 1. Dynamic Vector Length Configuration
    std::size_t vl = __riscv_vsetvl_e32m8(n - i);

    // 2. Vector Loads
    vfloat32m8_t vx = __riscv_vle32_v_f32m8(&x[i], vl);
    vfloat32m8_t vy = __riscv_vle32_v_f32m8(&y[i], vl);
    vfloat32m8_t vz = __riscv_vle32_v_f32m8(&z[i], vl);

    // 3. Vector-Scalar Subtraction
    vfloat32m8_t dx = __riscv_vfsub_vf_f32m8(vx, qx, vl);
    vfloat32m8_t dy = __riscv_vfsub_vf_f32m8(vy, qy, vl);
    vfloat32m8_t dz = __riscv_vfsub_vf_f32m8(vz, qz, vl);

    // 4. Vector Multiplication (Squaring)
    vfloat32m8_t dx2 = __riscv_vfmul_vv_f32m8(dx, dx, vl);
    vfloat32m8_t dy2 = __riscv_vfmul_vv_f32m8(dy, dy, vl);
    vfloat32m8_t dz2 = __riscv_vfmul_vv_f32m8(dz, dz, vl);

    // 5. Accumulation
    vfloat32m8_t sum = __riscv_vfadd_vv_f32m8(dx2, dy2, vl);
    sum = __riscv_vfadd_vv_f32m8(sum, dz2, vl);

    // 6. Vector Store
    __riscv_vse32_v_f32m8(&out_d2[i], sum, vl);

    i += vl;
  }
}
```

### Key RVV Concepts

#### 1. `vsetvl` (Vector Set Vector Length)
The instruction `__riscv_vsetvl_e32m8(n - i)` is the magic behind RVV's portability.
*   **Input**: The number of elements remaining to process (`n - i`).
*   **Output (`vl`)**: The number of elements the hardware *can* process in this iteration.
*   **Behavior**:
    *   If `n - i` is large, `vl` will be set to the hardware's maximum vector length (e.g., 4, 8, 16 elements).
    *   If `n - i` is small (the tail end of the loop), `vl` will be exactly `n - i`.
    *   **Result**: No need for "fringe" or "cleanup" loops common in SSE/AVX programming. One loop handles everything.

#### 2. LMUL (Register Grouping)
The `m8` suffix in `vfloat32m8_t` stands for **LMUL=8** (Length Multiplier).
*   **Reason**: It tells the CPU to group **8 consecutive vector registers** together to act as one giant register.
*   **Benefit**: If a single vector register holds 4 floats, `m8` allows us to process 32 floats in a single instruction. This significantly reduces instruction overhead and improves performance for long sequential operations like this one.

#### 3. Intrinsics Naming Convention
*   `vle32`: **V**ector **L**oad **E**lement (**32**-bit)
*   `vfsub_vf`: **V**ector **F**loating-point **Sub**tract (**V**ector - **F**loat/Scalar)
*   `vfmul_vv`: **V**ector **F**loating-point **Mul**tiply (**V**ector * **V**ector)
*   `vse32`: **V**ector **S**tore **E**lement (**32**-bit)

## Integration in SOR
In `src/statistical_outlier_removal.cpp`, the inner loop of the scalar algorithm:
```cpp
for(size_t j=0; j<n; ++j) {
    float dx = in[i].x - in[j].x;
    // ...
}
```
Is replaced by a single call to the kernel:
```cpp
get_dist_sq_rvv(in.x, in.y, in.z, in.x[i], in.y[i], in.z[i], dists.data(), in.n);
```
This accelerates the $O(N^2)$ distance calculation which is the bottleneck of the algorithm.
