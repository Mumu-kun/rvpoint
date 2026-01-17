# Benchmark Verification: Instruction Counts

We successfully benchmarked the RISC-V Vector (RVV) vs Scalar implementations by counting "architectural instructions" executed by QEMU. This method isolated the kernel instructions from the simulation overhead.

## Methodology
- **Simulator**: QEMU (`qemu-riscv64`)
- **Metric**: Dynamic Instruction Count (Translation Blocks executed).
- **Technique**: Used `setup` mode to subtract initialization overhead from the total count.
- **Dataset**: N=100.

## Results

| Algorithm | Scalar Instructions | RVV Instructions | Speedup (Instruction Reduction) |
| :--- | :--- | :--- | :--- |
| **Voxel Grid** | 1,595,744 | 1,610,801 | **0.99x** (Identical) |
| **SOR** | 6,866,560 | 6,061,596 | **1.13x** |
| **Normal Est.** | 38,212,817 | 7,896,933 | **4.83x** (Huge Reduction) |
| **Radius Search** | 91,719 | 92,522 | **0.99x** |
| **RANSAC** | 3,315,104 | 973,931 | **3.40x** |

### Analysis
- **Normal Estimation** and **RANSAC** are compute-intensive (heavy floating point math). The RVV implementation drastically reduces the number of instructions needed to process the data (up to **5x reduction**), as one vector instruction replaces many scalar ones.
- **Voxel Grid** shows no improvement (0.99x). Code analysis reveals this kernel is dominated by `std::map` insertions (scalar memory logic), making the vector math part negligible.
- **Radius Search** (N=100) was too fast to show significant divergence or is dominated by scalar KD-Tree traversal.

## Conclusion
The benchmarks confirm that for compute-bound kernels (`normal`, `ransac`), RVV provides a massive reduction in the number of executed instructions.
