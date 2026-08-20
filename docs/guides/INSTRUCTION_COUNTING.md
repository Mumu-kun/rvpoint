# Instruction Counting Methodology

---

### 🗺️ Project Navigation

| Document | Purpose |
|----------|---------|
| [**Pipeline Demo**](../README_RVV_DEMO.md) | Quick-start guide for the optimized Octree/SpatialHash pipeline. |
| [**Instruction Manual**](../docs/INSTRUCTION_MANUAL.md) | **The Master Guide**. API docs, Team SOPs, and naming conventions. |
| [**Benchmark Results**](../results/benchmark_summary_report.md) | Actual instruction count data. |
| [**Main Overview**](../README.md) | General project scope and feature set. |

---

This document justifies the validity of the instruction counts reported by our benchmarks (`scripts/bench`). We use QEMU's internal tracing to generate **Architectural Instruction Counts**, which accurately reflect the computational work required by the CPU, independent of emulation speed.

## The Problem
QEMU is a fast emulator that uses Just-In-Time (JIT) compilation. It normally groups multiple guest instructions into a single "Translation Block" (TB) to amortize the cost of emulation. This makes standard profiling (`rdinstret`/`rdcycle`) inaccurate in a deterministic simulation environment, as they often behave like wall-clock timers or block counters rather than true instruction counters.

## The Solution: Trace-Based Counting

We utilize specific QEMU flags to force a 1-to-1 mapping between executed Translation Blocks and guest instructions.

### 1. `one-insn-per-tb`
We run QEMU with the `-one-insn-per-tb` flag.
*   **Default Behavior**: QEMU compiles a basic block of ~10-100 instructions into one TB.
*   **With Flag**: QEMU creates a new TB for **every single guest instruction**.
*   **Result**: Executing 1 Translation Block ≡ Executing 1 Guest Instruction.

### 2. Trace Logging (`-d exec,nochain`)
We enable debug logging with `-d exec,nochain`.
*   **`exec`**: Logs a trace entry to stderr every time a TB is executed.
*   **`nochain`**: Prevents QEMU from "chaining" TBs (jumping directly from one compiled block to another). This forces QEMU to return to the main loop after every block, ensuring *every* instruction execution generates a log entry.

### 3. Baseline Subtraction
To isolate the efficiency of our kernels (RVV vs Scalar) from the C++ runtime initialization overhead (allocating vectors, random number generation), we perform **Baseline Subtraction**:

$$
\text{Instruction Count} = \text{Run}_{\text{Kernel}} - \text{Run}_{\text{Setup}}
$$

*   **Setup Mode**: Runs data initialization and simply returns before the algorithm starts.
*   **Kernel Mode**: Runs data initialization and the full algorithm.
*   **Difference**: Represents exactly the instructions executed by the standard or vector algorithm.

## Why This Proves RVV Efficiency
Using this metric, we can validly compare Scalar vs Vector effiency:

*   **Scalar Loop**: To process 100 elements, the CPU fetches, decodes, and executes hundreds of "scalar" instructions (loads, adds, branches). QEMU counts each of these individually.
*   **RVV**: The CPU fetches **one** vector instruction (e.g., `vfmul.vv`). The Vector Unit then processes the 100 elements internally in parallel (or pipelined). QEMU, correctly imitating the architecture, counts this as **1 instruction**.

This massive reduction in "fetched instructions" (Instruction Count) directly correlates to reduced frontend pressure (fetch/decode bandwidth) in real hardware, validating the efficiency of the RVV implementation.
