# Codebase Design & RVV Optimization Standards

> **Authoritative engineering standards for AI agents and human developers implementing vector-accelerated algorithms and designing modules in the RVPoint codebase.**

---

## 1. Executive Charter & Core Principles

RVPoint targets 64-bit RISC-V vector processors (`rv64gcv` RVV 1.0), with the **SpacemiT K1 (8-core X60 SoC @ 1.6 GHz)** as the primary edge deployment target.

Every module added to RVPoint must achieve two simultaneous objectives:
1. **Architectural Depth**: Small, robust interfaces placed at clean seams that hide complex implementation details, maximize leverage for callers, and enable headless desktop/CI testing.
2. **Hardware Vector Saturation**: Near-100% memory bus and vector ALU saturation via RVV 1.0 intrinsics, strictly adhering to the 32 KB L1D cache constraints of the SpacemiT K1.

---

## 2. Codebase Design: Deep Modules & Clean Seams

### 2.1 The Architectural Glossary
Agents must use these terms strictly according to their precise systems definitions:

- **Module**: Anything with an interface and an implementation (function, class, file, or static library).
- **Interface**: Everything a caller must know to use the module correctly: type signatures, memory ownership, ordering constraints, error modes, and performance characteristics.
- **Implementation**: The internal body of code hidden behind the interface.
- **Depth**: The leverage provided by the interface: a large amount of complex behavior sitting behind a small, simple surface.
- **Seam**: The exact boundary where an interface lives, allowing behavior to be altered without modifying the call site.
- **Adapter**: A concrete implementation that satisfies an interface at a seam (e.g. production hardware driver vs. in-memory test mock).
- **Leverage**: The capability callers gain per unit of interface learned.
- **Locality**: Concentration of change, knowledge, and bugs in one place rather than scattered across callers.

---

### 2.2 Deep vs. Shallow Modules

```
┌─────────────────────────────────────────────────────────┐
│              DEEP MODULE (Target Architecture)          │
│  Interface: Small, simple parameters                    │  ← e.g. PerceptionEngine::process(raw_cloud, out)
├─────────────────────────────────────────────────────────┤
│  Implementation: Deep, hidden complexity                │  ← Hides PassThrough crop, Cardano normal solve,
│                                                         │    Euclidean clustering, and RVV distance transform
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│             SHALLOW MODULE (Anti-Pattern - AVOID)       │
│  Interface: Bloated surface area, exposes internals     │  ← Exposes vl strip-mining, register types,
├─────────────────────────────────────────────────────────┤    temporary buffers, and manual stride math
│  Implementation: Thin pass-through                      │  ← Just forwards calls with zero leverage
└─────────────────────────────────────────────────────────┘
```

#### The Deletion Test:
Imagine deleting the module. If complexity vanishes, it was a pass-through (shallow). If complexity reappears across $N$ callers, it was earning its keep (deep).

#### Seam Rule:
*"One adapter means a hypothetical seam. Two adapters means a real one."* Never introduce an abstract interface unless at least two real adapters exist (e.g. Production Hardware + Desktop/CI Test Mock).

---

### 2.3 The Two Architectural Seams of the Perception & Vehicle Stack

To decouple hardware I/O and operating system dependencies from the core mathematical pipeline, maintain exactly two external seams:

```
[StreamSource Seam]              [Core Deep Modules]                  [Actuator Seam]
┌───────────────────┐        ┌─────────────────────────┐          ┌───────────────────┐
│ UdpStreamSource   │        │    PerceptionEngine     │          │ LinuxPwmActuator  │
│ (Production)      ├──>     │ (librvpoint.a / RVV)    ├──>       │ (Production)      │
├───────────────────┤        ├─────────────────────────┤          ├───────────────────┤
│ MockPcdStream     │        │        Navigator        │          │ MockActuator      │
│ (Desk / CI / Test)│        │ (Planning, FSM, PID)    │          │ (Desk / CI / Test)│
└───────────────────┘        └─────────────────────────┘          └───────────────────┘
```

#### Seam 1: The Ingestion Stream Seam (`StreamSource`)
- **Role**: Supplies point cloud batches and 6-DoF VIO transformation matrices.
- **Production Adapter**: `UdpStreamSource` (non-blocking POSIX BSD socket on 5 GHz Wi-Fi).
- **Test / Replay Adapter**: `PcdReplayStreamSource` (reads local PCD files and generates synthetic trajectories for desktop testing without an active iPhone).

#### Seam 2: The Actuation Hardware Seam (`MotorActuator`)
- **Role**: Accepts normalized wheel effort commands $[-1.0, 1.0]$ and enforces emergency stops.
- **Production Adapter**: `LinuxSysfsPwmActuator` (drives `/sys/class/pwm` and `libgpiod` on Orange Pi RV2).
- **Test / CI Adapter**: `MockMotorActuator` (records commanded duty cycles and timestamps; verifies watchdog cutoff behavior in headless unit tests).

---

### 2.4 Compile-Time Zero-Cost Abstraction

Virtual functions and dynamic dispatch (`vtable` lookups) are strictly prohibited inside per-point or per-voxel processing loops.

Use these three zero-cost patterns instead:

#### Pattern A: Batch Boundary Placement (Clean Seams at Low Frequency)
Place virtual interfaces or polymorphic adapters exclusively at the outer frame boundary (30 Hz–50 Hz). A virtual function call taking 3 nanoseconds at 30 Hz consumes only $0.00001\%$ of the frame budget, providing 100% testability with zero measurable overhead.

#### Pattern B: Compile-Time Policy Templates (Zero Virtuals)
When an algorithm requires interchangeable strategies, use template policy injection:
```cpp
template <typename DistancePolicy>
class RadiusOutlierRemoval {
public:
    void apply(const PointCloudSoA& in, PointCloudSoA& out, float r) {
        // DistancePolicy is resolved at compile-time; inlines 100% with ZERO vtable overhead
        DistancePolicy::compute(in.x.data(), in.y.data(), in.z.data(), out, r);
    }
};
```

#### Pattern C: Link-Time Substitution
Maintain a clean, non-virtual C++ interface in a shared header. CMake compiles `actuator_linux_pwm.cpp` for target hardware builds, and links `actuator_mock.cpp` for desktop test binaries.

---

## 3. RVV 1.0 Vector Optimization Standards

### 3.1 The 5 Hardware Vector Invariants

Every vector kernel added to `src/` must strictly satisfy these five microarchitectural invariants:

1. **Vector Length Agnosticism (VLA) Strip-Mining**:
   Always query hardware vector length using dynamic strip-mining:
   ```cpp
   size_t vl = __riscv_vsetvl_e32m8(remaining);
   ```
   Never assume a fixed hardware VLEN. This guarantees identical binary execution on 128-bit, 256-bit, and 512-bit vector hardware.

2. **Register Grouping Allocation (The LMUL Trade-off)**:
   - Use **LMUL = 8** (`m8`) for arithmetic-heavy streaming passes with $\le 3$ live vector variables.
   - Drop to **LMUL = 4** (`m4`) or **LMUL = 2** (`m2`) when performing multi-variable geometric transformations (rotation, normal estimation, Cardano eigensolvers) or indexed gathers.
   - *Microarchitectural Reason*: LMUL=8 leaves only 4 logical register groups (`v0, v8, v16, v24`). Register `v0` is reserved for masks. Holding $>3$ intermediate variables forces GCC to emit vector stack spills (`vs8r.v`), destroying cache performance.

3. **100% Unit-Stride Memory Streaming**:
   Structure algorithms around sequential memory access (`vle32.v` / `vse32.v`) from `PointCloudSoA` contiguous buffers. Unit-stride accesses saturate the 64-byte burst capability of the SpacemiT K1 L1D cache bus.

4. **Zero Heap Allocations on Hot Paths**:
   No `malloc()`, `operator new`, or dynamic `std::vector::resize()` within per-frame loops. All scratchpad memory, hash tables, and costmap grids must be statically allocated or passed in by the caller.

5. **Fused Arithmetic (`vfmacc` / `vfnmsac`)**:
   Never issue separate `vfmul` and `vfadd`. Always fuse operations into multiply-accumulate to retire in a single FMA execution pipeline cycle.

---

### 3.2 Selective Deployment of Specialized RVV Instructions

While unit-stride streaming is the primary baseline, deploy specialized RVV instructions whenever the topological or mathematical structure demands non-contiguous access:

| Instruction | GCC 14.2 Intrinsic | Optimal Deployment | Microarchitectural Advantage |
|---|---|---|---|
| **Indexed Gather** | `__riscv_vluxei32_v_f32m2` | Spatial hash & octree neighbor search | VLSU issues parallel cache accesses; executes $3.2\times$ faster than scalar pointer-chasing. *Note: Byte offsets must be left-shifted by 2.* |
| **Segmented Load** | `__riscv_vlseg3e32_v_f32m2x3` | Ingesting legacy AoS (`PointXYZ`) or ROS buffers | Hardware de-interleaves $(X, Y, Z)$ triplets into 3 vector registers in 1 instruction with zero CPU unpacking overhead. |
| **Vector Compress** | `__riscv_vcompress_vm_f32m8` | PassThrough filtering, inlier compaction | Packs active mask elements contiguously; eliminates branch mispredictions on unstructured point clouds. |
| **Vector Slide** | `__riscv_vslidedown_vx_f32m8` | Rotating calipers, convex hull edges | In-register cross-lane shift (1 cycle latency); zero memory bus traffic. |
| **Widening FMA** | `__riscv_vfwmacc_vv_f64m8` | Cardano normal covariance accumulation | Promotes `float32` inputs to `float64` accumulator; guarantees numerical stability without slow double-precision throughout. |
| **Reduction** | `__riscv_vfredmax_vs_f32m8_f32m1` | Bounding sphere radius, extents min/max | Evaluates tree reduction across all vector lanes in $O(\log_2 \text{VLMAX})$ cycles. |

---

### 3.3 SpacemiT K1 / X60 Microarchitectural Budget

When designing algorithms for the SpacemiT K1 SoC, budget against these verified hardware limits:

- **Core Count**: 8 SpacemiT X60 cores across 2 clusters (Cores 0–3 and Cores 4–7).
- **L1 Data Cache**: **32 KB private per core** (64-byte cache lines). Holds at most 8,192 single-precision floats.
- **L2 Unified Cache**: **512 KB shared per 4-core cluster** (1 MB total SoC).
- **Execution Pipeline**: Dual-issue in-order superscalar. An L1D cache miss on a gather operation stalls the core; always interleave arithmetic (`vfmacc`) with memory loads (`vle32`) to achieve maximum instruction retirement.

---

## 4. Verification & Completion Criteria

Before any code is merged into `src/` or `eval/`, the implementation must satisfy these five checkable gates:

1. **Dual-Path Parity Gate**:
   Every new algorithm in `src/` must provide both an `#if defined(__riscv_vector)` kernel and an `#else` scalar baseline. Both paths must produce mathematically identical outputs within floating-point tolerance ($\epsilon < 10^{-5}$).
2. **Ablation Speedup Gate**:
   Run `eval/benchmarks/ablation_bench.cpp`. The RVV-accelerated implementation must achieve **$\ge 4.0\times$ speedup** over the scalar baseline on physical SpacemiT K1 hardware.
3. **Disassembly Audit Gate**:
   Run `riscv64-unknown-linux-gnu-objdump -d` on the compiled object file. Verify that:
   - `vsetvli`, `vle32.v`, `vse32.v`, and `vfmacc` dominate hot loops.
   - Zero vector register spill instructions (`vs8r.v` / `vl8r.v`) appear in the inner loop.
4. **Zero-Allocation Gate**:
   Verify with `valgrind --tool=massif` or static inspection that the per-frame perception loop performs zero heap allocations (`malloc`, `operator new`, `std::vector::push_back`).
5. **Headless Desktop Test Gate**:
   Run `./scripts/test.sh` on the desktop/WSL2 host using the mock adapters (`PcdReplayStreamSource`, `MockMotorActuator`). The complete closed-loop test suite must pass with zero hardware dependencies.

