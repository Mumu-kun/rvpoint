#include <stdio.h>
#include <riscv_vector.h>

int main() {
    size_t vl = __riscv_vsetvl_e32m1(1);
    printf("Spike RVV 1.0 Check: vl=%zu\n", vl);
    return 0;
}
