# 4 x 2 Pipeline Benchmark Commands Reference

This document lists the **8 execution commands** ($4 \text{ targets} \times 2 \text{ runners}$) for running the complete RVPoint vs. PCL baseline suite under **QEMU User Emulation** and **gem5 Cycle-Accurate Microarchitectural Simulation**.

---

## Target Matrix

| Target Name | Description | Backend / Search Structure |
|---|---|---|
| **`pipeline_3d_ultimate`** | RVPoint Hand-Crafted RVV 1.0 Vector Pipeline | SoA buffers, `vle32/vse32`, Flat SpatialGrid (Stages 2 & 5), flat Union-Find |
| **`pcl_native_pipeline`** | Standard PCL 1.14 KdTree Reference Baseline | Standard PCL KdTree (`pcl::search::KdTree`, Stages 2 & 5), AoS `pcl::PointXYZ` |
| **`pcl_octree_pipeline`** | Standard PCL 1.14 Octree Reference Baseline | Standard PCL Octree (`pcl::search::Octree`, Stages 2 & 5), AoS `pcl::PointXYZ` |
| **`pcl_optimized_pipeline`** | Optimized Standard PCL Pipeline | `is_dense` bounded ROR, unsorted FLANN KdTree (Stages 2 & 5), direct clustering |

### Unified 6-Stage Compute Architecture:
1. `[1/6] Voxel Grid Downsampling`
2. `[2/6] Build ROR Search Index` *(KdTree / Octree / SpatialGrid on downsampled cloud)*
3. `[3/6] Radius Outlier Removal (ROR)` *(Neighbor queries reusing Stage 2 index)*
4. `[4/6] RANSAC Ground Plane` *(Ground segmentation and non-ground point extraction)*
5. `[5/6] Build Cluster Search Index` *(KdTree / Octree / SpatialGrid on non-ground cloud)*
6. `[6/6] Euclidean Clustering` *(Clustering queries reusing Stage 5 index)*

---

## 1. QEMU Execution (Fast Functional Verification)

Targeting QEMU user emulation with RVV 1.0 support via `./scripts/run.sh`:

```bash
# 1. RVPoint Hand-Crafted RVV 1.0 Pipeline
./scripts/run.sh pipeline_3d_ultimate data/pcd_compressed/0000000090.pcd --progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write

# 2. Standard PCL 1.14 KdTree Baseline
./scripts/run.sh pcl_native_pipeline data/pcd_compressed/0000000090.pcd --progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write

# 3. Standard PCL 1.14 Octree Baseline
./scripts/run.sh pcl_octree_pipeline data/pcd_compressed/0000000090.pcd --progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write

# 4. Optimized Standard PCL Baseline
./scripts/run.sh pcl_optimized_pipeline data/pcd_compressed/0000000090.pcd --progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write
```

---

## 2. gem5 Microarchitectural Simulation (Cycle-Accurate)

Targeting SpacemiT K1 / MinorCPU microarchitecture via `./scripts/gem5/run_sim.sh`:

```bash
# 1. RVPoint Hand-Crafted RVV 1.0 Pipeline
./scripts/gem5/run_sim.sh pipeline_3d_ultimate data/pcd_compressed/0000000090.pcd --progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write

# 2. Standard PCL 1.14 KdTree Baseline
./scripts/gem5/run_sim.sh pcl_native_pipeline data/pcd_compressed/0000000090.pcd --progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write

# 3. Standard PCL 1.14 Octree Baseline
./scripts/gem5/run_sim.sh pcl_octree_pipeline data/pcd_compressed/0000000090.pcd --progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write

# 4. Optimized Standard PCL Baseline
./scripts/gem5/run_sim.sh pcl_optimized_pipeline data/pcd_compressed/0000000090.pcd --progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write
```

---

## Windows Host (WSL2 `rvpoint` Distribution Wrapper)

To run any of the above commands from a Windows terminal:

```powershell
wsl -d rvpoint bash -c "source env/activate.sh && ./scripts/bench/pipeline_run_commands.sh"
```
