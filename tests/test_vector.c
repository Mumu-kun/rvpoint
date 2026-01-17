#include <stdio.h>
#include <riscv_vector.h>

int main() {
    size_t vl = __riscv_vsetvl_e64m1(4);
    
    // Create two vectors with 4 elements
    int64_t a[] = {1, 2, 3, 4};
    int64_t b[] = {10, 20, 30, 40};
    int64_t c[4] = {0};

    // Load vectors
    vint64m1_t va = __riscv_vle64_v_i64m1(a, vl);
    vint64m1_t vb = __riscv_vle64_v_i64m1(b, vl);

    // Vector add
    vint64m1_t vc = __riscv_vadd_vv_i64m1(va, vb, vl);

    // Store result
    __riscv_vse64_v_i64m1(c, vc, vl);

    printf("Vector verification:\n");
    for(int i = 0; i < vl; i++) {
        printf("%ld + %ld = %ld\n", a[i], b[i], c[i]);
    }
    
    return 0;
}
