# Zero-Heap Hot-Path Allocation Invariant

Strictly prohibit dynamic heap allocations (`malloc`, `operator new`, `std::vector::resize`, `std::shared_ptr`) inside the 30 Hz per-frame perception, planning, and control loops on the SpacemiT K1 SoC.

## Status
Accepted

## Context
The SpacemiT K1 SoC features an in-order execution pipeline with a 32 KB private L1 Data Cache per core. Dynamic memory allocations during the 30 Hz real-time perception loop cause glibc heap allocator mutex contention across the 8 cores, kernel page-table traps, and L1/L2 cache evictions. Furthermore, dynamic resizing introduces non-deterministic execution jitter that can violate real-time safety deadlines.

## Decision
Enforce a compile-time and runtime **Zero-Heap Invariant** on all hot processing paths:
1. **Caller-Owned Memory**: Deep modules (`PerceptionEngine`, `Navigator`) accept pre-allocated output buffers by reference (`PerceptionOutput& out`) rather than returning dynamically allocated objects.
2. **Fixed-Capacity Static Bounds**: Intermediate arrays (e.g. cluster lists, obstacle arrays, grid indices) declare static maximum capacities (e.g. `constexpr size_t kMaxObstacles = 32`).
3. **Pre-allocated Workspaces**: The 2.5D Rolling Costmap ($150 \times 150 \times 4\,\text{bytes} \approx 90\,\text{KB}$) and spatial hash tables are pre-allocated once at system initialization and reused permanently across frames.

## Consequences
- **Positive**: Eliminates memory fragmentation and allocator lock contention; guarantees deterministic sub-millisecond execution times; keeps working sets warm in L1/L2 cache.
- **Negative**: Imposes upper bounds on the maximum number of simultaneously trackable obstacles per frame.

