We are building a lightweight 3D point cloud processing compute kernel library optimized specifically for RISC-V processors with the RISC-V Vector Extension (RVV). This is not a full perception stack or ML framework â€” it focuses on geometric primitives optimized for edge deployment and deterministic execution under power and latency constraints.

Pipeline modules:
Downsampling -> Radius Search (Octree / Spatial Hashing) -> Statistical Outlier Removal (SOR) -> Normal Estimation -> RANSAC (primitive fitting).

Design goals:

RVV-first vectorization (variable-length SIMD)

Data-parallel geometric compute optimization

Efficient neighbor search (dominant hotspot)

Low memory bandwidth pressure

Real-time processing of continuous LiDAR/3D streams

Suitable for embedded / edge / base-station deployment

Target scenarios:
Smart intersections, warehouse safety LiDAR, railway intrusion detection, construction monitoring, agricultural drone base stations, industrial collision monitoring, smart building occupancy mapping.

Positioning:
Edge alternative to GPU/cloud-heavy solutions; hardware-aware geometric acceleration on RISC-V for scalable, power-efficient distributed deployment.