# RVPoint Contributor Guide

## Purpose

This document is a practical onboarding guide for a new contributor who wants to understand how RVPoint is structured, how to build it, and how to navigate the core source files without getting lost in the implementation details.

## What this project is

RVPoint is a C++ point-cloud processing library that focuses on RISC-V Vector (RVV) acceleration. The codebase implements classic point-cloud operations such as voxel grid downsampling, statistical outlier removal, normal estimation, radius search, RANSAC plane fitting, and clustering.

The important idea is that the project usually provides two versions of each algorithm:

- a scalar reference implementation for correctness and readability
- an RVV-optimized implementation for performance on RISC-V hardware

This makes the repository useful both as a learning project and as a performance-oriented research codebase.

## High-level architecture

The repository is organized around a few clear layers:

1. Public API layer
   - The main header is [src/include/rvv_pcl.h](src/include/rvv_pcl.h)
   - It declares the public data structures and algorithm entry points
   - It is the first place to look when you want to know what the library offers

2. Core algorithm implementations
   - Files such as [src/voxel_grid_downsamp.cpp](src/voxel_grid_downsamp.cpp), [src/statistical_outlier_removal.cpp](src/statistical_outlier_removal.cpp), [src/normal_estimation.cpp](src/normal_estimation.cpp), [src/ransac_plane.cpp](src/ransac_plane.cpp), and [src/euclidean_clustering.cpp](src/euclidean_clustering.cpp)
   - Each file usually contains a scalar implementation and an RVV version

3. Neighbor-search layer
   - The code under [src/neighbor_search](src/neighbor_search) implements spatial indexing strategies such as octrees, spatial hashing, and pointer-based octrees
   - These are used to speed up neighborhood queries and reduce the cost of repeated local operations

4. Build and test infrastructure
   - [CMakeLists.txt](CMakeLists.txt) defines the library targets and test executables
   - [scripts](scripts) contains build and verification helpers
   - [tests](tests) contains the automated checks and benchmark programs

## Main data structures

### PointXYZ

This is the simple scalar point type used in the public API and in many reference implementations.

It is a basic 3D point:

- x
- y
- z

### PointCloudSoA

This is the most important data structure for the vectorized code paths.

It stores point coordinates in Structure of Arrays (SoA) format:

- x pointer
- y pointer
- z pointer
- n, the number of points

Why this matters:

- RVV works best with contiguous memory access
- SoA makes it easy to load a vector of x values, then y values, then z values
- This layout is much better suited to vector instructions than a traditional AoS layout

If you are reading RVV code for the first time, this is the first thing to understand.

## How the algorithms are organized

A common pattern appears throughout the repository.

For each algorithm, you will usually see one of these patterns:

- a scalar version with the suffix \_sc
- an RVV version with the suffix \_rvv
- sometimes a version that uses an index structure such as Octree or SpatialHash

For example:

- voxel_grid_downsamp_sc and voxel_grid_downsamp_rvv_v2
- sor_sc and sor_rvv
- normal_estimation_sc and normal_estimation_rvv
- ransac_plane_sc and ransac_plane_rvv

That naming convention is very helpful when you need to compare the baseline against the optimized path.

## Where to start reading

If you are new to the codebase, this is the recommended order:

1. Start with [src/include/rvv_pcl.h](src/include/rvv_pcl.h)
   - This shows the public API and the major functions
   - It gives a good map of what the library can do

2. Read [src/voxel_grid_downsamp.cpp](src/voxel_grid_downsamp.cpp)
   - It is a good introduction to how RVV code is written in this project
   - It covers coordinate scaling, voxel key computation, and vectorized reductions

3. Read [src/statistical_outlier_removal.cpp](src/statistical_outlier_removal.cpp)
   - This shows how the library handles neighbor-distance calculations and filtering

4. Read [src/normal_estimation.cpp](src/normal_estimation.cpp)
   - This introduces covariance-based normal estimation and the use of spatial structures

5. Read [src/neighbor_search/octree.cpp](src/neighbor_search/octree.cpp) and [src/neighbor_search/spatial_hashing.cpp](src/neighbor_search/spatial_hashing.cpp)
   - These explain how the project speeds up neighbor queries

6. Read [src/euclidean_clustering.cpp](src/euclidean_clustering.cpp)
   - This shows how a higher-level algorithm is built on top of the lower-level search primitives

## How RVV code is typically written here

The RVV implementations follow a fairly consistent style:

- use vector loads such as vle32
- process data in chunks using vsetvl
- perform arithmetic with RVV instructions such as subtraction, multiply, and fused multiply-accumulate
- use masks for conditional filtering
- often fall back to scalar code in parts where vectorization is awkward or unnecessary

A beginner should not try to memorize every intrinsic. Instead, focus on the pattern:

- load a chunk of data
- transform it
- reduce or filter it
- store the results

That pattern repeats across many files.

## Neighbor-search strategies

The project has several spatial indexing approaches, and understanding them is important because many algorithms rely on them.

### Octree

The octree implementation in [src/neighbor_search/octree.cpp](src/neighbor_search/octree.cpp) recursively partitions space into 8 children. It is a classic search structure that is useful for radius queries and local neighborhood search.

### Spatial hash

The spatial hash implementation in [src/neighbor_search/spatial_hashing.cpp](src/neighbor_search/spatial_hashing.cpp) partitions space into cells. It is often faster for uniform point distributions because each query can check a small set of nearby cells.

### Pointer octree

The pointer octree implementation in [src/pointer_octree/pointer_octree.h](src/pointer_octree/pointer_octree.h) and [src/neighbor_search/pointer_octree.cpp](src/neighbor_search/pointer_octree.cpp) uses a pointer-based tree structure and is another variation for accelerating local searches.

These structures are important because they connect the low-level math operations to practical performance improvements.

## Build and run flow

### Build system

The main build configuration is in [CMakeLists.txt](CMakeLists.txt).

It defines:

- the RVVPCL library target
- the test executables
- the pipeline export and benchmarking utilities

### Typical build steps

From the repository root, the usual workflow is:

```bash
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../src/cmake/riscv.cmake
make
```

You can also use the repository scripts in [scripts](scripts) for a more guided workflow.

### Verification scripts

The script [scripts/verify_container.sh](scripts/verify_container.sh) is useful for validating the environment and running the default checks.

The script [scripts/run.sh](scripts/run.sh) and the benchmarking helpers in [scripts/bench](scripts/bench) are also worth exploring if you want to understand how the project is evaluated.

## Tests and examples

The test folders under [tests](tests) are a very good place to learn how the library is expected to behave.

You should look at:

- [tests/integration/test_pipeline_walkthrough.cpp](tests/integration/test_pipeline_walkthrough.cpp)
- [tests/benchmark/benchmark.cpp](tests/benchmark/benchmark.cpp)
- [tests/bench](tests/bench)

These files show how the algorithms are combined into a larger processing pipeline and how the project compares performance.

## Practical advice for newcomers

When you are first entering this repository, keep the following in mind:

- start from the public header and then move into one algorithm file
- compare the scalar and RVV implementations side by side
- use the tests as a guide for expected behavior
- treat the vector code as an optimization layer rather than as a separate project
- do not worry about fully understanding every intrinsic immediately

A good mental model is:

- the scalar code explains the algorithm
- the RVV code makes the same operation faster on suitable hardware
- the neighbor search code improves the cost of repeated search operations

## Suggested learning path

If you want a simple path to become productive quickly, follow this order:

1. Read the API header
2. Read one algorithm file end to end
3. Compare it with the scalar reference
4. Read one neighbor-search implementation
5. Run one of the test or benchmark binaries
6. Then start making small changes or adding a small experiment

## Summary

RVPoint is best understood as a layered codebase:

- public API and data structures
- algorithm implementations
- spatial index structures
- tools, tests, and benchmarks

Once you understand that layout, the code becomes much easier to navigate and extend.
