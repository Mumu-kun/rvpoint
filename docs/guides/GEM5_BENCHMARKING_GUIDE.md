# gem5 Microarchitectural Benchmarking Guide

> **Universal, C++ File-Agnostic Guide for Cycle-Accurate Simulation & Microarchitectural Profiling in RVPoint.**

---

## 1. Overview & Simulation Philosophy

RVPoint provides two complementary execution layers:

| Layer | Engine | Primary Use Case | Speed | Fidelity |
| :--- | :--- | :--- | :--- | :--- |
| **Functional Emulation** | **QEMU User-Mode** (`scripts/run.sh`) | Rapid functional testing, regression suites, and debugging. | ~10–50× real-time | Functional correctness only (no cycle/cache model). |
| **Cycle-Accurate Simulation** | **gem5** (`scripts/gem5/run_sim.sh`) | Hardware profiling: clock cycles, instruction retirement, IPC, and L1D cache locality. | ~1,000× slowdown | Exact microarchitectural modeling. |

### Target Core Model: SpacemiT K1 Preset
- **CPU Model**: `MinorCPU` (in-order, dual-issue pipeline).
- **ISA**: RISC-V 64-bit (`rv64gcv`) with hardware RVV 1.0 support.
- **Vector Unit Configuration**: `VLEN = 256` bits, `ELEN = 64` bits.
- **Memory Subsystem**: L1 Data Cache (32 KB, 2-way), L1 Instruction Cache (32 KB), 512 MB physical address space.

---

## 2. Universal Invocation Syntax

`scripts/gem5/run_sim.sh` is **completely target-agnostic**. You can pass any target name, relative path, or `.cpp` file located in `eval/pipelines/`, `eval/benchmarks/`, `eval/tests/`, or custom directories:

```bash
./scripts/gem5/run_sim.sh [options] <target_name | path/to/file.cpp> [cpp_args...]
```

### Examples:
```bash
# 1. Perception Pipeline:
./scripts/gem5/run_sim.sh --dev pipeline_3d_ultimate data/01_table_scene_lms400.pcd --no-write

# 2. Standalone Benchmark Driver:
./scripts/gem5/run_sim.sh --dev ablation_bench --trials 3

# 3. Unit / Regression Test:
./scripts/gem5/run_sim.sh test_ransac

# 4. Direct C++ File Path:
./scripts/gem5/run_sim.sh --dev eval/pipelines/pipeline_3d_turbo.cpp data/01_table_scene_lms400.pcd --no-write
```

---

## 3. Simulation Budgeting & Decimation Tiers

Because cycle-accurate simulation models every clock cycle and cache access, dense point clouds ($>100\text{k}$ points) can require hours to simulate. 

When a `.pcd` input argument is detected alongside a tier flag, `run_sim.sh` automatically performs **geometric voxel decimation** (preserving 3D bounding boxes, surface normal geometry, and obstacle clusters) before launching simulation:

| Tier | CLI Flag | Target Points | Typical Host Simulation Time | Microarchitectural Focus |
| :--- | :--- | :--- | :--- | :--- |
| **Sanity** | `--sanity` | **150 pts** | ~5–15 seconds | Functional smoke testing & instruction decoding verification. |
| **Dev** | `--dev` | **1,000 pts** | ~1–2 minutes | Daily dev loop (fits entirely in L1D cache). |
| **Eval** | `--eval` | **5,000 pts** | ~10–15 minutes | Standard evaluation benchmark (stresses L2 & memory subsystem). |
| **Stress** | `--stress` | **10,000 pts** | ~20–30 minutes | Heavy spatial tree traversal and cluster allocation stress. |
| **Full** | `--full` | **100%** | Pass-through | Full uncompressed cloud without decimation. |
| **Custom** | `--pts <N>` | **N pts** | Custom | Exact arbitrary point count target (e.g. `--pts 2500`). |

---

## 4. Vector vs. Scalar Showdowns

To measure exact vector acceleration speedups, compile and simulate the target under both backends:

```bash
# 1. Run Vector-Accelerated Target (RVV 1.0):
./scripts/gem5/run_sim.sh --backend rvv --save-results --dev pipeline_3d_ultimate data/01_table_scene_lms400.pcd --no-write

# 2. Run Scalar Baseline Target:
./scripts/gem5/run_sim.sh --backend scalar --save-results --dev pcl_standalone_pipeline data/01_table_scene_lms400.pcd --no-write
```

### Comparing Two Runs with `compare_stats.py`
Use `scripts/gem5/compare_stats.py` to compare output stats directories or files:

```bash
python3 scripts/gem5/compare_stats.py results/gem5/scalar_* results/gem5/rvv_*
```

#### Sample Comparison Output:
```text
### gem5 Microarchitectural Comparison (SpacemiT K1 Preset)

| Metric | PCL Scalar Baseline | RVPoint RVV Pipeline | Speedup / Reduction |
| :--- | :---: | :---: | :---: |
| **Simulated CPU Time** | 0.241800 s | 0.058706 s | **4.12× faster** |
| **Committed Insts** | 412,890,120 | 96,831,051 | **4.26× fewer insts** |
| **Simulated Cycles** | 483,600,240 | 117,412,599 | - |
| **IPC (Insts/Cycle)** | 0.8538 | 0.8247 | -0.0291 |
| **L1D Miss Rate** | 1.84% | 0.14% | -1.70% |
```

---

## 5. Key Microarchitectural Metrics Explained

| Metric | Key in `stats.txt` | Meaning & Hardware Significance |
| :--- | :--- | :--- |
| **Simulated CPU Time** | `simSeconds` | True simulated execution time on a physical SpacemiT K1 CPU at nominal clock frequency ($1.6\text{ GHz}$). |
| **Committed Instructions** | `simInsts` / `system.cpu.numInsts` | Number of instructions successfully retired by the CPU core. On RVV, 1 vector instruction executes up to 32–128 float operations simultaneously, causing a massive reduction in committed instructions. |
| **Simulated Clock Cycles** | `system.cpu.numCycles` | Total hardware clock cycles elapsed. Directly determines execution latency. |
| **Instructions Per Cycle (IPC)** | `system.cpu.ipc` | Instruction execution throughput. The SpacemiT K1 MinorCPU has a theoretical peak dual-issue IPC of $2.0$. High-performance vector loops typically sustain an IPC of $\mathbf{0.80\text{--}0.95}$. |
| **Cycles Per Instruction (CPI)** | `system.cpu.cpi` | Reciprocal of IPC ($1 / \text{IPC}$). Lower is better. |
| **L1D Cache Miss Rate** | `system.cpu.dcache.overallMissRate` | Fraction of data memory accesses that missed L1 cache. Low miss rates ($<0.5\%$) confirm efficient contiguous SoA streaming. |

---

## 6. Best Practices & Troubleshooting

1. **Always Pass `--no-write` (or `--disable-disk`)**:
   - File I/O under gem5 syscall emulation triggers high host serialization overhead. Always disable writing intermediate PCDs during benchmarking.
2. **Build Isolation is Automatic**:
   - `run_sim.sh` builds into `/root/.cache/rvpoint/build/{rvv_gem5,scalar_gem5}/`. You can switch between native QEMU testing and gem5 simulation without CMake cache conflicts.
3. **Debugging Aborts & Missing Instructions**:
   - If gem5 panics on an unknown instruction (`Unknown instruction 0x...`), check `sim.log` in the output directory. gem5 v25 supports standard RVV arithmetic, logic, reduction, and load/stores.
