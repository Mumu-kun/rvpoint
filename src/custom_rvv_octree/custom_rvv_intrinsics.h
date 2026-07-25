#pragma once
#include <riscv_vector.h>
#include <cstddef>

namespace rvv_pcl {

/**
 * @brief Fused 3D Distance Vector-Scalar Custom Intrinsic (LMUL=8)
 * 
 * Computes:
 *   vd[i] = (vx[i] - qx)^2 + (vy[i] - qy)^2 + (vz[i] - qz)^2
 * 
 * Hardware Instruction Specification:
 *   vdist3d.vf vd, vs1, vs2, rs1
 *   - vd (destination/source): Input vx, Output squared distances.
 *   - vs1: Input vy vector register.
 *   - vs2: Input vz vector register.
 *   - rs1: Scalar register holding pointer to the float query array [qx, qy, qz].
 * 
 * Opcode Mapping:
 *   Uses custom-0 space (opcode = 0x0b).
 */
static inline vfloat32m8_t __riscv_vdist3d_vf_f32m8(
    vfloat32m8_t vx, vfloat32m8_t vy, vfloat32m8_t vz,
    float qx, float qy, float qz, size_t vl) 
{
#ifdef USE_CUSTOM_RVV
    // Custom opcode execution on custom ASIP
    // Fuses 6 vector math instructions into 1.
    // Query point coordinates [qx, qy, qz] are loaded into a temporary array
    float query_arr[3] = { qx, qy, qz };
    
    register vfloat32m8_t vd_reg asm("v8") = vx;
    register vfloat32m8_t vy_reg asm("v16") = vy;
    register vfloat32m8_t vz_reg asm("v24") = vz;
    register const float* query_reg asm("a0") = query_arr;

    asm volatile (
        // Custom instruction: vdist3d.vf v8, v16, v24, a0 (R4-type format)
        // Uses integer register names x8, x16, x24 as aliases for v8, v16, v24
        ".insn r4 0x0b, 6, 0, x8, x16, x24, a0"
        : "+v" (vd_reg)
        : "v" (vy_reg), "v" (vz_reg), "r" (query_reg)
    );
    return vd_reg;
#else
    // Fallback standard RVV 1.0 implementation for QEMU emulation compatibility
    vfloat32m8_t dx = __riscv_vfsub_vf_f32m8(vx, qx, vl);
    vfloat32m8_t dy = __riscv_vfsub_vf_f32m8(vy, qy, vl);
    vfloat32m8_t dz = __riscv_vfsub_vf_f32m8(vz, qz, vl);

    vfloat32m8_t d2 = __riscv_vfmul_vv_f32m8(dx, dx, vl);
    d2 = __riscv_vfmacc_vv_f32m8(d2, dy, dy, vl);
    d2 = __riscv_vfmacc_vv_f32m8(d2, dz, dz, vl);
    return d2;
#endif
}

/**
 * @brief Fused Compress-Store Vector-Scalar Custom Intrinsic (float, LMUL=8)
 * 
 * Stores the compressed active elements of vector vs into contiguous memory.
 * 
 * Hardware Instruction Specification:
 *   vstore_compressed.v vs, v0, rs1
 *   - vs: Vector register containing elements to compress and write.
 *   - v0: The mask register (held in v0 by standard RVV convention).
 *   - rs1: Scalar register holding the base destination pointer.
 */
static inline void __riscv_vstore_compressed_v_f32m8(
    float* dst, vfloat32m8_t vs, vbool4_t mask, size_t vl, size_t count) 
{
#ifdef USE_CUSTOM_RVV
    // Custom compress-store instruction
    // Fuses vcompress and vse32 into a single hardware transaction.
    register float* dst_reg asm("a0") = dst;
    register vfloat32m8_t vs_reg asm("v8") = vs;
    register vbool4_t mask_reg asm("v0") = mask;

    asm volatile (
        // opcode=0x2b (custom-1), funct3=0, funct7=0
        // Reads vs, uses mask v0 implicitly, writes contiguously to rs1 (dst)
        // Uses x8 as alias for v8
        ".insn r 0x2b, 0, 0, x0, x8, a0"
        :
        : "r" (dst_reg), "v" (vs_reg), "v" (mask_reg)
        : "memory"
    );
#else
    // Fallback standard RVV 1.0 implementation
    vfloat32m8_t v_match = __riscv_vcompress_vm_f32m8(vs, mask, vl);
    __riscv_vse32_v_f32m8(dst, v_match, count);
#endif
}

/**
 * @brief Fused Compress-Store Vector-Scalar Custom Intrinsic (int, LMUL=8)
 */
static inline void __riscv_vstore_compressed_v_i32m8(
    int* dst, vint32m8_t vs, vbool4_t mask, size_t vl, size_t count) 
{
#ifdef USE_CUSTOM_RVV
    register int* dst_reg asm("a0") = dst;
    register vint32m8_t vs_reg asm("v8") = vs;
    register vbool4_t mask_reg asm("v0") = mask;

    asm volatile (
        // opcode=0x2b (custom-1), funct3=1, funct7=0
        // Uses x8 as alias for v8
        ".insn r 0x2b, 1, 0, x0, x8, a0"
        :
        : "r" (dst_reg), "v" (vs_reg), "v" (mask_reg)
        : "memory"
    );
#else
    vint32m8_t v_match = __riscv_vcompress_vm_i32m8(vs, mask, vl);
    __riscv_vse32_v_i32m8(dst, v_match, count);
#endif
}

} // namespace rvv_pcl
