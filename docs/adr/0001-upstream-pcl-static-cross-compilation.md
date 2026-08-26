# 1. Upstream PCL Static Cross-Compilation for gem5 Microarchitectural Evaluation

## Context
Evaluating RVPoint against the open-source Point Cloud Library (PCL) requires an authentic, unmodified PCL baseline executing in gem5 System Emulation (SE) mode on the RISC-V 64-bit vector architecture (`rv64gcv`). Standard upstream PCL has extensive heavy dependencies (VTK, Qt, OpenNI, Boost multi-threading) which cannot run in gem5 SE mode.

## Decision
We cross-compile upstream PCL from source targeting RISC-V 64-bit statically (`-static`) with a focused modular profile: `pcl_common`, `pcl_kdtree`, `pcl_search`, `pcl_filters`, `pcl_features`, `pcl_segmentation`, and `pcl_io`, linking against header-only Eigen3, static FLANN, and minimal Boost. The resulting static binary `pcl_native_pipeline` executes the identical 10-stage perception pipeline against `pipeline_3d_ultra`.

## Consequences
- Guarantees 100% authentic upstream PCL algorithmic and template behavior in gem5 microarchitectural simulations.
- Prevents runtime dynamic linker and thread scheduling artifacts inside gem5 SE mode.
- Enables cycle-accurate speedup, IPC, and L1D cache miss attribution directly against standard industrial point cloud processing code.
