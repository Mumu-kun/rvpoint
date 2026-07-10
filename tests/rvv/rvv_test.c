#include <stdio.h>
#include <riscv_vector.h>

int main() {
    size_t vl = __riscv_vsetvl_e64m1(10);
    printf("RVV functionality verified: VL = %zu\n", vl);
    return 0;
}
