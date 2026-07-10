#include <stdio.h>
#include <riscv_vector.h>

void test_tuples() {
    printf("Testing RVV Tuple Types...\n");
    
    // Arrays for segment load/store
    float src[] = {1.1, 2.2, 3.3, 4.4, 5.5, 6.6}; // 3 pairs of (x,y)
    float dst[6] = {0};
    
    size_t vl = __riscv_vsetvl_e32m1(3); // We want 3 elements, each is a pair
    
    // Test Tuple Load (vlseg2)
    // In newer GCC/RVV 1.0, this returns a tuple type like vfloat32m1x2_t
    vfloat32m1x2_t v_tuple = __riscv_vlseg2e32_v_f32m1x2(src, vl);
    
    // Access tuple fields (implementation dependent, but typically v_tuple.val[0] / .val[1] or via intrinsics)
    // GCC 14 usually supports .val[i] member access for these struct types
    vfloat32m1_t v0 = __riscv_vget_v_f32m1x2_f32m1(v_tuple, 0);
    vfloat32m1_t v1 = __riscv_vget_v_f32m1x2_f32m1(v_tuple, 1);
    
    // Add 1.0 to first component, 2.0 to second
    vfloat32m1_t v_ones = __riscv_vfmv_v_f_f32m1(1.0f, vl);
    vfloat32m1_t v_twos = __riscv_vfmv_v_f_f32m1(2.0f, vl);
    
    v0 = __riscv_vfadd_vv_f32m1(v0, v_ones, vl);
    v1 = __riscv_vfadd_vv_f32m1(v1, v_twos, vl);
    
    // Reconstruct tuple (not strictly necessary if we just store fields, but good for testing)
    v_tuple = __riscv_vset_v_f32m1_f32m1x2(v_tuple, 0, v0);
    v_tuple = __riscv_vset_v_f32m1_f32m1x2(v_tuple, 1, v1);
    
    // Test Tuple Store (vsseg2)
    __riscv_vsseg2e32_v_f32m1x2(dst, v_tuple, vl);
    
    printf("Dst: %.1f, %.1f (Expected 2.1, 4.2)\n", dst[0], dst[1]);
    
    if (dst[0] > 2.09 && dst[0] < 2.11 && dst[1] > 4.19 && dst[1] < 4.21) {
        printf("[PASS] Tuple Types & Segment Ops working.\n");
    } else {
        printf("[FAIL] Tuple functionality result incorrect.\n");
    }
}

int main() {
    test_tuples();
    return 0;
}
