#include <stdio.h>

void vector_add(float *a, float *b, float *c, int n) {
    // Simple loop that should be auto-vectorized by GCC 14 with -O3 -march=rv64gcv
    for (int i = 0; i < n; i++) {
        c[i] = a[i] + b[i];
    }
}

int main() {
    float a[1024], b[1024], c[1024];
    for(int i=0; i<1024; i++) { a[i]=i; b[i]=i; }
    vector_add(a, b, c, 1024);
    return 0;
}
