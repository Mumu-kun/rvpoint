#include <math.h>
#include <stdint.h>
#include <stdio.h>

static uint64_t spread_bits_by_3(uint32_t value)
{
    uint64_t x = (uint64_t)(value & 0x1fffffU);
    x = (x | (x << 32)) & 0x1f00000000ffffULL;
    x = (x | (x << 16)) & 0x1f0000ff0000ffULL;
    x = (x | (x << 8)) & 0x100f00f00f00f00fULL;
    x = (x | (x << 4)) & 0x10c30c30c30c30c3ULL;
    x = (x | (x << 2)) & 0x1249249249249249ULL;
    return x;
}

uint64_t morton_encode(uint32_t ix, uint32_t iy, uint32_t iz)
{
    return spread_bits_by_3(ix) | (spread_bits_by_3(iy) << 1) | (spread_bits_by_3(iz) << 2);
}

void morton_range(float qx, float qy, float qz,
                  float r, float voxel_size,
                  uint64_t* out_min, uint64_t* out_max)
{
    int ix0 = (int)floorf((qx - r) / voxel_size);
    int iy0 = (int)floorf((qy - r) / voxel_size);
    int iz0 = (int)floorf((qz - r) / voxel_size);
    int ix1 = (int)ceilf((qx + r) / voxel_size);
    int iy1 = (int)ceilf((qy + r) / voxel_size);
    int iz1 = (int)ceilf((qz + r) / voxel_size);

    if (ix0 < 0) ix0 = 0;
    if (iy0 < 0) iy0 = 0;
    if (iz0 < 0) iz0 = 0;
    if (ix1 < 0) ix1 = 0;
    if (iy1 < 0) iy1 = 0;
    if (iz1 < 0) iz1 = 0;

    uint64_t mn = UINT64_MAX;
    uint64_t mx = 0;

    for (int x = ix0; x <= ix1; x++)
    for (int y = iy0; y <= iy1; y++)
    for (int z = iz0; z <= iz1; z++) {
        uint64_t m = morton_encode((uint32_t)x,
                                   (uint32_t)y,
                                   (uint32_t)z);
        if (m < mn) mn = m;
        if (m > mx) mx = m;
    }
    *out_min = mn;
    *out_max = mx;

static int debug_count = 0;
if (debug_count < 5) {
    printf("bbox voxels: %d x %d x %d = %d\n",
           ix1-ix0+1, iy1-iy0+1, iz1-iz0+1,
           (ix1-ix0+1)*(iy1-iy0+1)*(iz1-iz0+1));
    ++debug_count;
}
}

#ifdef MORTON_SELF_TEST


static uint32_t compact_bits_by_3(uint64_t value)
{
    uint64_t x = value & 0x1249249249249249ULL;
    x = (x ^ (x >> 2)) & 0x10c30c30c30c30c3ULL;
    x = (x ^ (x >> 4)) & 0x100f00f00f00f00fULL;
    x = (x ^ (x >> 8)) & 0x1f0000ff0000ffULL;
    x = (x ^ (x >> 16)) & 0x1f00000000ffffULL;
    x = (x ^ (x >> 32)) & 0x1fffffULL;
    return (uint32_t)x;
}

int main(void)
{
    uint32_t ix = 12345U;
    uint32_t iy = 54321U;
    uint32_t iz = 77777U;
    uint64_t code = morton_encode(ix, iy, iz);
    if (compact_bits_by_3(code) != ix || compact_bits_by_3(code >> 1) != iy || compact_bits_by_3(code >> 2) != iz) {
        printf("FAIL morton self-test\n");
        return 1;
    }
    printf("PASS morton self-test\n");
    return 0;
}
#endif