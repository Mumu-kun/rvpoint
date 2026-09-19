# ADR 0011: Non-Virtual Deep Class Pattern for Core Algorithms

- **Status**: accepted
- **Deciders**: Fahad, Antigravity
- **Date**: 2026-09-11

## Context & Decision

RVPoint core algorithms previously consisted of a fragmented mix of free functions taking raw C-style pointers, un-encapsulated scratch arrays, and ad-hoc parameters. Dynamic polymorphism (`virtual` base classes) was evaluated but rejected for inner-loop execution because vtable dereferencing and indirect branches prevent the compiler from inlining and vectorizing loops with RVV 1.0 intrinsics, while forcing heap-allocated pointer semantics.

We decided that all core library algorithms in `src/` will be structured as **stateful, non-virtual C++ classes with value semantics**. Each class encapsulates its configuration parameters, manages internal scratch workspaces to prevent hot-path heap allocations, and provides an `enum class Backend` (e.g. `Auto`, `RVV`, `Scalar`) that dispatches via inline switch to vectorized or scalar reference implementations.

## Consequences

- Algorithms can be allocated on the stack, embedded inside structs, or passed by value/reference with zero vtable pointer overhead.
- Benchmarks and CLI tools can switch between vectorized RVV, optimized scalar, and baseline backends at runtime via config enums without recompilation or virtual dispatch.
- Inner loops retain 100% compiler inlining and vector register allocation visibility.

