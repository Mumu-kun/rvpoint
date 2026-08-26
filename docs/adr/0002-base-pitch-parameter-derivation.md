# 2. Single Source of Truth Base Pitch (Δ) Pipeline Parameter Derivation

## Context
3D point cloud perception pipelines involve multiple interconnected spatial algorithms: voxel downsampling, statistical outlier removal, normal estimation, RANSAC ground plane fitting, and Euclidean clustering. Manually and independently tuning distance thresholds for each algorithm causes configuration drift, empty neighborhood searches during normal estimation, and false cluster bridging across disparate point clouds.

## Decision
We enforce a **Single Source of Truth Base Pitch ($\Delta$)** architectural pattern across all perception pipelines (both RVPoint and PCL baseline):
1. **Voxel Grid Pitch**: $\Delta$ establishes the uniform spatial lattice.
2. **Neighbor Search Radius**: $R = 2.5 \cdot \Delta$ guarantees $\ge 15\text{--}30$ points per search sphere for stable Cardano/covariance normal estimation.
3. **RANSAC Inlier Tolerance**: $\varepsilon = 0.5 \cdot \Delta$ enforces strict sub-voxel planar consensus.
4. **Euclidean Clustering Distance**: $d_{\text{cluster}} = 1.25 \cdot \Delta$ ensures seamless intra-object point linkage without bridging disjoint obstacles.
5. **Statistical Outlier Removal**: Radius $R = 2.5 \cdot \Delta$ with $\alpha = 1.0\sigma$ threshold.

Standard presets are defined for standard domains:
- Dense / Object Preset: $\Delta = 0.01\text{ m}$
- Standard Tabletop Benchmark: $\Delta = 0.02\text{ m}$ (Default)
- Sparse Outdoor LiDAR: $\Delta = 0.05\text{ m}$

## Consequences
- Eliminates configuration mismatch between pipeline stages.
- Enables continuous scaling across varying point cloud densities by tuning only a single float parameter $\Delta$.
- Guarantees strict algorithmic parity during PCL vs RVPoint comparative benchmarking in gem5.
