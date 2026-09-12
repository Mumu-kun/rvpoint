---
marp: true
theme: default
paginate: true
size: 16:9
style: |
  @import url('https://fonts.googleapis.com/css2?family=Plus+Jakarta+Sans:wght@500;600;700;800;900&family=JetBrains+Mono:wght@500;700;800;900&display=swap');

  :root {
    --color-primary: #0B1120;
    --color-indigo: #4F46E5;
    --color-indigo-dark: #3730A3;
    --color-teal: #0D9488;
    --color-teal-dark: #0F766E;
    --color-amber: #D97706;
    --color-rose: #E11D48;
    --color-bg-canvas: #F1F4F9;
    --color-bg-card: #FFFFFF;
    --color-border: #E2E8F0;
    font-family: 'Plus Jakarta Sans', -apple-system, BlinkMacSystemFont, sans-serif;
  }

  section {
    background-color: #F1F4F9;
    color: #0F172A;
    padding: 38px 56px;
    font-size: 23px;
    font-weight: 600;
    line-height: 1.55;
  }

  /* Header structure */
  .kicker {
    font-family: 'JetBrains Mono', monospace;
    font-size: 16px;
    font-weight: 800;
    color: #4F46E5;
    letter-spacing: 1.5px;
    text-transform: uppercase;
    margin-bottom: 4px;
    display: flex;
    align-items: center;
    gap: 8px;
  }

  h1 {
    font-size: 38px;
    font-weight: 900;
    color: #0B1120;
    letter-spacing: -0.8px;
    margin-top: 0;
    margin-bottom: 16px;
    line-height: 1.15;
    border-bottom: 2px solid #E2E8F0;
    padding-bottom: 10px;
  }

  h2 {
    font-size: 27px;
    font-weight: 850;
    color: #0B1120;
    margin-bottom: 12px;
    letter-spacing: -0.3px;
  }

  /* Cards & Layouts */
  .grid-2 {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 26px;
    margin-top: 8px;
  }

  .grid-3 {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 22px;
    margin-top: 8px;
  }

  .card {
    background: #FFFFFF;
    border: 1px solid #E2E8F0;
    border-radius: 14px;
    padding: 28px 30px;
    box-shadow: 0 2px 6px -1px rgba(15, 23, 42, 0.03);
  }

  .card-indigo {
    background: #FFFFFF;
    border: 1px solid #E2E8F0;
    border-left: 6px solid #4F46E5;
    border-radius: 14px;
    padding: 28px 30px;
    box-shadow: 0 2px 6px -1px rgba(15, 23, 42, 0.03);
  }

  .card-teal {
    background: #FFFFFF;
    border: 1px solid #E2E8F0;
    border-left: 6px solid #0D9488;
    border-radius: 14px;
    padding: 28px 30px;
    box-shadow: 0 2px 6px -1px rgba(15, 23, 42, 0.03);
  }

  .card-amber {
    background: #FFFFFF;
    border: 1px solid #E2E8F0;
    border-left: 6px solid #D97706;
    border-radius: 14px;
    padding: 28px 30px;
    box-shadow: 0 2px 6px -1px rgba(15, 23, 42, 0.03);
  }

  /* Big Metric Callouts */
  .hero-metric {
    font-family: 'JetBrains Mono', monospace;
    font-size: 60px;
    font-weight: 900;
    line-height: 1;
    margin-bottom: 6px;
  }

  .text-indigo { color: #4F46E5; }
  .text-teal { color: #0D9488; }
  .text-amber { color: #D97706; }
  .text-rose { color: #E11D48; }

  .badge {
    display: inline-block;
    padding: 5px 14px;
    border-radius: 6px;
    font-family: 'JetBrains Mono', monospace;
    font-size: 15px;
    font-weight: 800;
    letter-spacing: 0.5px;
    width: fit-content;
  }

  .badge-indigo { background: #EEF2FF; color: #4338CA; border: 1px solid #C7D2FE; }
  .badge-teal { background: #F0FDFA; color: #0F766E; border: 1px solid #99F6E4; }
  .badge-amber { background: #FFFBEB; color: #92400E; border: 1px solid #FDE68A; }
  .badge-rose { background: #FFF1F2; color: #9F1239; border: 1px solid #FECDD3; }

  /* Tables */
  table {
    width: 100%;
    border-collapse: collapse;
    font-size: 17px;
    margin-top: 6px;
    background: #FFFFFF;
    border-radius: 10px;
    overflow: hidden;
    border: 1px solid #E2E8F0;
    box-shadow: 0 2px 6px -1px rgba(15, 23, 42, 0.03);
  }

  th {
    background: #0B1120;
    color: #FFFFFF;
    font-weight: 800;
    font-size: 17px;
    padding: 10px 14px;
    text-align: left;
  }

  td {
    padding: 9px 14px;
    border-bottom: 1px solid #E5E7EB;
    color: #1E293B;
    font-size: 16px;
    font-weight: 650;
  }

  tr:nth-child(even) { background: #F8FAFC; }
  tr.highlight { background: #F0FDFA; font-weight: 800; color: #0F766E; }
  tr.highlight td { color: #0F766E; font-weight: 800; }

  /* Title Slide */
  section.lead {
    background: linear-gradient(135deg, #090D16 0%, #131B2E 100%);
    color: #FFFFFF;
    display: flex;
    flex-direction: column;
    justify-content: center;
    padding: 60px 75px;
  }

  section.lead h1 {
    color: #FFFFFF;
    font-size: 48px;
    font-weight: 900;
    border-bottom: none;
    padding-bottom: 0;
    margin-bottom: 14px;
  }

  section.lead .subtitle {
    color: #94A3B8;
    font-size: 26px;
    font-weight: 600;
    margin-bottom: 32px;
  }

  /* Image Containers */
  .img-box {
    background: #070A12;
    border-radius: 10px;
    overflow: hidden;
    display: flex;
    align-items: center;
    justify-content: center;
    border: 1px solid #1E293B;
  }

  .img-box img {
    width: 100%;
    height: 100%;
    object-fit: cover;
  }
---

<!-- _class: lead -->
<!-- _paginate: false -->

<div class="badge badge-indigo" style="background: rgba(79, 70, 229, 0.25); color: #C7D2FE; border: 1.5px solid #818CF8; font-size: 16px; margin-bottom: 20px; padding: 6px 16px;">
  CAPSTONE PROJECT: EMBEDDED SPATIAL AI & ROBOTICS
</div>

# RVPoint: RISC-V Vector 3D Point Cloud Processing Library

<div class="subtitle">High-Performance Perception Library & Hardware Testbed Evaluation</div>

<div style="display: flex; gap: 48px; font-family: 'JetBrains Mono', monospace; font-size: 19px; font-weight: 700; color: #CBD5E1; border-top: 1px solid rgba(255, 255, 255, 0.2); padding-top: 22px; margin-top: 18px;">
  <div><strong>Type:</strong> Modular C++17 Library</div>
  <div><strong>Architecture:</strong> RVV 1.0 (64-bit)</div>
  <div><strong>Scope:</strong> Hardware Demonstration</div>
</div>

---

<div class="kicker">[01. MOTIVATION & PROBLEM]</div>

# The Point Cloud Processing Bottleneck in Edge Robotics

<div class="grid-2">
  <div class="card-indigo">
    <div class="badge badge-indigo" style="margin-bottom: 12px;">THE CHALLENGE</div>
    <h2>Dense 3D Perception Stacks</h2>
    <ul style="margin-left: 20px; color: #1E293B; font-size: 21px; font-weight: 600; line-height: 1.6;">
      <li>Autonomous mobile robots produce <strong>100,000+ points per frame</strong>.</li>
      <li>Classical libraries (PCL, Open3D) rely on heavy x86 CPUs or power-hungry GPUs.</li>
      <li>Existing RISC-V ecosystems lack an open-source, vector-accelerated point cloud library.</li>
    </ul>
  </div>

  <div class="card-teal">
    <div class="badge badge-teal" style="margin-bottom: 12px;">THE SOLUTION</div>
    <h2>RVPoint: Native RISC-V Vector Library</h2>
    <ul style="margin-left: 20px; color: #1E293B; font-size: 21px; font-weight: 600; line-height: 1.6;">
      <li>Open-standard, royalty-free data-parallel vector computing directly in low-power silicon.</li>
      <li>Wide vector registers process multiple 3D spatial points simultaneously.</li>
      <li><strong>RVPoint</strong> delivers a modular, zero-dependency C++17 library for embedded robotics.</li>
    </ul>
  </div>
</div>

---

<div class="kicker">[02. LIBRARY ARCHITECTURE]</div>

# RVPoint: Modular C++17 Vector Perception Library

<div class="grid-3">
  <div class="card">
    <div class="badge badge-indigo" style="margin-bottom: 14px;">ZERO RUNTIME BLOAT</div>
    <h2>Modular C++17 Core</h2>
    <p style="color: #334155; font-size: 20px; font-weight: 600; line-height: 1.5;">
      Zero third-party library dependencies (no Eigen, PCL, or Boost bloat). Direct RVV 1.0 vector intrinsic kernels with clean C++ APIs and scalar fallback paths.
    </p>
  </div>

  <div class="card">
    <div class="badge badge-indigo" style="margin-bottom: 14px;">SPATIAL ACCELERATION</div>
    <h2>Accelerated Data Structures</h2>
    <p style="color: #334155; font-size: 20px; font-weight: 600; line-height: 1.5;">
      Native Structure-of-Arrays (SoA) containers, cache-optimized PointerOctree, and SpatialHash grid engines built directly into the library.
    </p>
  </div>

  <div class="card-teal">
    <div class="badge badge-teal" style="margin-bottom: 14px;">PERCEPTION ALGORITHMS</div>
    <h2>Full Algorithmic Suite</h2>
    <p style="color: #0F766E; font-size: 20px; font-weight: 600; line-height: 1.5;">
      Modular library components for Voxel Downsampling, Statistical & Radius Filtering, Covariance & Normal Estimation, RANSAC Model Fitting, and Clustering.
    </p>
  </div>
</div>

---

<div class="kicker">[03. PIPELINE COMPOSITION]</div>

# 10-Stage Autonomous Perception Pipeline

<div style="display: grid; grid-template-columns: repeat(5, 1fr); gap: 16px; margin-bottom: 24px;">
  <div class="card" style="padding: 16px;">
    <div class="badge badge-indigo">STAGE 1-2</div>
    <div style="font-weight: 800; font-size: 20px; margin-top: 8px;">Voxel Grid</div>
    <div style="font-size: 16px; font-weight: 600; color: #64748B;">Spatial Downsampling</div>
  </div>
  <div class="card" style="padding: 16px;">
    <div class="badge badge-indigo">STAGE 3</div>
    <div style="font-weight: 800; font-size: 20px; margin-top: 8px;">PointerOctree</div>
    <div style="font-size: 16px; font-weight: 600; color: #64748B;">Spatial Indexing</div>
  </div>
  <div class="card" style="padding: 16px; border-left: 5px solid #0D9488;">
    <div class="badge badge-teal">STAGE 4-5</div>
    <div style="font-weight: 800; font-size: 20px; margin-top: 8px;">Pointer SOR</div>
    <div style="font-size: 16px; font-weight: 600; color: #0F766E;">Noise Purging</div>
  </div>
  <div class="card" style="padding: 16px;">
    <div class="badge badge-indigo">STAGE 6-7</div>
    <div style="font-weight: 800; font-size: 20px; margin-top: 0.08in;">Normals</div>
    <div style="font-size: 16px; font-weight: 600; color: #64748B;">Surface Covariance</div>
  </div>
  <div class="card" style="padding: 16px; border-left: 5px solid #0D9488;">
    <div class="badge badge-teal">STAGE 8-10</div>
    <div style="font-weight: 800; font-size: 20px; margin-top: 8px;">RANSAC & Cluster</div>
    <div style="font-size: 16px; font-weight: 600; color: #0F766E;">Obstacle Extraction</div>
  </div>
</div>

<div class="card-indigo">
  <h2>Deterministic Memory Hoisting & Pipeline Verification</h2>
  <p style="color: #1E293B; font-size: 21px; font-weight: 600; margin-top: 6px;">
    All intermediate stage memory buffers are pre-allocated upon startup, ensuring <strong>zero runtime heap allocations</strong> inside the library execution loop. Verified across 131 continuous LiDAR driving frames.
  </p>
</div>

---

<div class="kicker">[04. VISUALIZATION]</div>

# Staged Point Cloud Transformation (Open3D Renders)

<div style="display: grid; grid-template-columns: 1fr 1fr 1fr 1fr; gap: 16px; height: 210px; margin-bottom: 20px;">
  <div>
    <div class="img-box" style="height: 165px;"><img src="../renders/open3d_pipeline/01_voxel_grid.png"></div>
    <div style="font-size: 16px; font-weight: 700; color: #1E293B; margin-top: 4px;">1. Voxel Grid (18.5k pts, -84%)</div>
  </div>
  <div>
    <div class="img-box" style="height: 165px;"><img src="../renders/open3d_pipeline/02_pointer_sor.png"></div>
    <div style="font-size: 16px; font-weight: 700; color: #1E293B; margin-top: 4px;">2. Pointer SOR (16.6k inliers)</div>
  </div>
  <div>
    <div class="img-box" style="height: 165px;"><img src="../renders/open3d_pipeline/04_ground_plane.png"></div>
    <div style="font-size: 16px; font-weight: 700; color: #1E293B; margin-top: 4px;">3. RANSAC Ground (42.3k pts)</div>
  </div>
  <div>
    <div class="img-box" style="height: 165px;"><img src="../renders/open3d_pipeline/05_obstacles_only.png"></div>
    <div style="font-size: 16px; font-weight: 700; color: #1E293B; margin-top: 4px;">4. Obstacles Cloud (56.4k pts)</div>
  </div>
</div>

<div style="display: grid; grid-template-columns: 1.8fr 1.2fr; gap: 24px; height: 240px;">
  <div class="img-box"><img src="../renders/open3d_pipeline/06_euclidean_clusters.png"></div>
  <div class="card-teal" style="display: flex; flex-direction: column; justify-content: center;">
    <div class="badge badge-teal" style="margin-bottom: 8px;">STAGE 5: FINAL OUTPUT</div>
    <div class="hero-metric text-teal">74 Clusters</div>
    <p style="color: #0F766E; font-size: 19px; font-weight: 600; line-height: 1.4;">
      Physical objects (vehicles, pedestrians, barriers) segmented with unique cluster IDs and 3D height shading for path planning.
    </p>
  </div>
</div>

---

<div class="kicker">[05. SPATIAL SEARCH ABLATION]</div>

# Spatial Neighbor & Radius Search Engine Analysis

<div class="grid-2">
  <div>
    <table>
      <thead>
        <tr>
          <th>Spatial Search Engine</th>
          <th>Index Build</th>
          <th>Query Latency</th>
          <th>Search Mechanism</th>
        </tr>
      </thead>
      <tbody>
        <tr class="highlight">
          <td><strong>PointerOctree (RVV Accelerated)</strong></td>
          <td><strong>9.67 ms</strong></td>
          <td><strong>0.740 ms</strong></td>
          <td>Contiguous leaf buffers + vector distance checking</td>
        </tr>
        <tr>
          <td><strong>SpatialHash Grid</strong></td>
          <td>16.93 ms</td>
          <td><strong>0.884 ms</strong></td>
          <td>O(1) direct 3D grid cell spatial hashing</td>
        </tr>
        <tr>
          <td><strong>Global RVV Scan</strong></td>
          <td><strong>0.00 ms</strong></td>
          <td><strong>0.965 ms</strong></td>
          <td>Zero index build; linear vector memory stream</td>
        </tr>
        <tr style="background: #FFF1F2;">
          <td>Standard Octree (Baseline)</td>
          <td>11.11 ms</td>
          <td>0.794 ms</td>
          <td>Classical tree with dynamic pointer chasing</td>
        </tr>
      </tbody>
    </table>
    <div style="font-size: 15px; font-weight: 600; color: #64748B; margin-top: 6px;">
      *Empirically measured in QEMU (rv64gcv, 128-bit VLEN) on N=18,542 points, radius=0.03m.
    </div>
  </div>

  <div class="card-indigo" style="justify-content: center;">
    <div class="badge badge-indigo" style="margin-bottom: 10px;">VECTOR ACCELERATION</div>
    <div class="hero-metric text-indigo">RVV Native</div>
    <p style="color: #1E293B; font-size: 19px; font-weight: 600; line-height: 1.5;">
      <strong>PointerOctree</strong> combines logarithmic spatial tree partitioning with flattened leaf coordinate arrays, enabling vector distance calculations directly across contiguous memory without cache-thrashing pointer dereferences.
    </p>
  </div>
</div>

---

<div class="kicker">[06. NOISE REMOVAL ABLATION]</div>

# Statistical Outlier Removal (SOR) 5-Way Ablation

<div class="grid-2">
  <div>
    <table>
      <thead>
        <tr>
          <th>SOR Filter Variant</th>
          <th>Execution Time</th>
          <th>Speedup vs Scalar</th>
        </tr>
      </thead>
      <tbody>
        <tr>
          <td>Scalar Brute-Force O(N²)</td>
          <td>15,824.8 ms</td>
          <td>1.00x (Baseline)</td>
        </tr>
        <tr>
          <td>Vector Brute-Force O(N²)</td>
          <td>9,744.7 ms</td>
          <td><strong>1.62x</strong></td>
        </tr>
        <tr>
          <td>Spatial Hash Grid SOR</td>
          <td>1,511.9 ms</td>
          <td><strong>10.47x</strong></td>
        </tr>
        <tr>
          <td>Standard Octree SOR</td>
          <td>462.4 ms</td>
          <td><strong>34.22x</strong></td>
        </tr>
        <tr class="highlight">
          <td><strong>PointerOctree SOR [RVPoint]</strong></td>
          <td><strong>281.7 ms</strong></td>
          <td><span class="badge badge-teal" style="font-size: 18px;">56.17x FASTER</span></td>
        </tr>
      </tbody>
    </table>
    <div style="font-size: 15px; font-weight: 600; color: #64748B; margin-top: 6px;">
      *Empirically measured on N=18,542 downsampled points (MeanK=50, std_threshold=1.0).
    </div>
  </div>

  <div class="card-teal" style="justify-content: center;">
    <div class="badge badge-teal" style="margin-bottom: 10px;">EMPIRICAL SPEEDUP</div>
    <div class="hero-metric text-teal">56.17x</div>
    <p style="color: #0F766E; font-size: 19px; font-weight: 600; line-height: 1.5;">
      Noise removal previously consumed <strong>94% of perception latency</strong>. Combining O(N log N) PointerOctree spatial partitioning with vector memory loads accelerates filtering from <strong>15.82s down to 281.7 ms</strong>.
    </p>
  </div>
</div>

---

<div class="kicker">[07. FULL PIPELINE MATRIX]</div>

# Full-Pipeline Scalar vs. RVPoint Vector Speedup Matrix

<table>
  <thead>
    <tr>
      <th>Pipeline Processing Stage</th>
      <th>Scalar Baseline</th>
      <th>RVV Vectorized</th>
      <th>Speedup</th>
      <th>Optimization Mechanism</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><strong>Voxel Grid Downsampling</strong></td>
      <td>148.2 ms</td>
      <td><strong>40.9 ms</strong></td>
      <td><span class="badge badge-teal">3.62x</span></td>
      <td>Vector sorting & parallel reduction</td>
    </tr>
    <tr style="background: #FFF1F2;">
      <td><strong>Statistical Outlier (SOR)</strong></td>
      <td>32,912.4 ms (94.6%)</td>
      <td><strong>287.2 ms</strong></td>
      <td><span class="badge badge-rose">114.6x</span></td>
      <td>PointerOctree + vector coordinate loads</td>
    </tr>
    <tr>
      <td><strong>Surface Normal Estimation</strong></td>
      <td>425.4 ms</td>
      <td><strong>141.2 ms</strong></td>
      <td><span class="badge badge-teal">3.01x</span></td>
      <td>Vectorized covariance accumulation</td>
    </tr>
    <tr>
      <td><strong>RANSAC Ground Plane Fit</strong></td>
      <td>975.6 ms</td>
      <td><strong>397.4 ms</strong></td>
      <td><span class="badge badge-teal">2.45x</span></td>
      <td>Parallel plane distance & inlier masking</td>
    </tr>
    <tr>
      <td><strong>Euclidean Clustering</strong></td>
      <td>291.2 ms</td>
      <td><strong>148.5 ms</strong></td>
      <td><span class="badge badge-teal">1.96x</span></td>
      <td>PointerOctree BFS + zero-alloc buffers</td>
    </tr>
    <tr class="highlight" style="font-size: 21px;">
      <td><strong>TOTAL PIPELINE LATENCY</strong></td>
      <td><strong>34,790.8 ms (34.8s)</strong></td>
      <td><strong>888.6 ms (<0.9s)</strong></td>
      <td><span class="badge badge-teal" style="font-size: 18px;">39.15x END-TO-END</span></td>
      <td><strong>Full Stack Vector Optimization</strong></td>
    </tr>
  </tbody>
</table>
<div style="font-size: 15px; font-weight: 600; color: #64748B; margin-top: 8px;">
  *Full pipeline benchmark demonstrating RVPoint modular library performance on 114,278-point raw LiDAR input frame.
</div>

---

<div class="kicker">[08. HARDWARE JUSTIFICATION]</div>

# Complementary Verification: Simulation vs. Physical Silicon

<div class="grid-2">
  <div class="card">
    <div class="badge badge-indigo" style="margin-bottom: 12px;">SOFTWARE SIMULATION (COMPLETED)</div>
    <h2>What Simulation Verifies</h2>
    <ul style="margin-left: 20px; color: #334155; font-size: 20px; font-weight: 600; line-height: 1.6;">
      <li><strong>Algorithmic Correctness:</strong> Proves mathematical equivalence of RVV 1.0 intrinsics.</li>
      <li><strong>Asymptotic Complexity:</strong> Validates <strong style="color: #4F46E5; font-family: 'JetBrains Mono', monospace;">O(N log N)</strong> algorithmic scaling.</li>
      <li><strong>Functional Limits:</strong> Software translation cannot model hardware clock cycles or memory controller arbitration.</li>
    </ul>
  </div>

  <div class="card-indigo">
    <div class="badge badge-indigo" style="margin-bottom: 12px;">PHYSICAL SILICON (THE OBJECTIVE)</div>
    <h2>What Hardware Benchmarking Captures</h2>
    <ul style="margin-left: 20px; color: #1E293B; font-size: 20px; font-weight: 600; line-height: 1.6;">
      <li><strong>Physical Memory Hierarchy:</strong> Measures L1/L2 cache hit rates and DDR bus latency.</li>
      <li><strong>Hardware VPU Throughput:</strong> Benchmarks true parallel arithmetic execution units.</li>
      <li><strong>Hardware PMU Counters:</strong> Captures real cycle counts, branch predictions, and stalls.</li>
    </ul>
  </div>
</div>

---

<div class="kicker">[09. PERFORMANCE PROJECTIONS]</div>

# Projected Performance on Physical RISC-V Hardware

<div class="grid-3">
  <div class="card">
    <div class="badge badge-indigo" style="margin-bottom: 12px;">HARDWARE VPUS</div>
    <h2>Dedicated Vector Execution</h2>
    <p style="color: #334155; font-size: 20px; font-weight: 600; line-height: 1.5;">
      Physical vector arithmetic units execute wide SIMD operations in dedicated hardware ALUs per clock cycle, uninhibited by software translation overhead.
    </p>
  </div>

  <div class="card">
    <div class="badge badge-indigo" style="margin-bottom: 12px;">CACHE BURST FILLS</div>
    <h2>Memory Burst Synergy</h2>
    <p style="color: #334155; font-size: 20px; font-weight: 600; line-height: 1.5;">
      Contiguous Structure-of-Arrays coordinate buffers saturate 64-byte CPU cache lines with zero wasted bandwidth or strided memory stalls.
    </p>
  </div>

  <div class="card-teal">
    <div class="badge badge-teal" style="margin-bottom: 12px;">REAL-TIME OBJECTIVE</div>
    <h2>Sub-100ms Latency Target</h2>
    <p style="color: #0F766E; font-size: 20px; font-weight: 600; line-height: 1.5;">
      On native 1.6 GHz multi-core RISC-V development hardware, end-to-end perception is projected to achieve sub-100ms execution times for robotics navigation.
    </p>
  </div>
</div>

---

<div class="kicker">[10. HARDWARE TESTBED]</div>

# Candidate Dev Board Matrix (RVV 1.0 Focus)

<div style="display: grid; grid-template-columns: 1.3fr 1.7fr; gap: 28px;">
  <div class="card-indigo">
    <div class="badge badge-teal" style="margin-bottom: 10px;">PRIMARY TARGET PLATFORM</div>
    <h2 style="font-size: 30px; margin-bottom: 8px;">Orange Pi RV2</h2>
    <div style="font-size: 20px; color: #1E293B; font-weight: 600; line-height: 1.6;">
      <div><strong>Processor:</strong> SpacemiT K1 (8-Core 64-bit)</div>
      <div><strong>Vector Standard:</strong> Official RVV 1.0 Standard</div>
      <div><strong>Vector Width:</strong> 256-bit VLEN</div>
      <div><strong>Memory Config:</strong> 4GB / 8GB / 16GB LPDDR4X</div>
    </div>
    <p style="font-size: 18px; color: #4F46E5; margin-top: 14px; font-weight: 700;">
      8 CPU cores with 256-bit vector registers provide an 8-core RVV 1.0 testbed for multi-core vector benchmarking.
    </p>
  </div>

  <div>
    <table>
      <thead>
        <tr>
          <th>Dev Board</th>
          <th>Processor SoC</th>
          <th>Vector Standard</th>
          <th>Vector Width</th>
        </tr>
      </thead>
      <tbody>
        <tr class="highlight">
          <td><strong>Orange Pi RV2</strong></td>
          <td>SpacemiT K1 (8-Core)</td>
          <td>RVV 1.0 (Official)</td>
          <td>256-bit VLEN</td>
        </tr>
        <tr>
          <td><strong>Kendryte K230</strong></td>
          <td>Canaan K230 (Dual-Core)</td>
          <td>RVV 1.0 (Official)</td>
          <td>128-bit VLEN</td>
        </tr>
        <tr>
          <td><strong>Milk-V Meles</strong></td>
          <td>T-Head C910 (Quad-Core)</td>
          <td>RVV 0.7.1 (Draft)</td>
          <td>128-bit VLEN</td>
        </tr>
      </tbody>
    </table>
    <div class="card" style="margin-top: 18px; padding: 16px; font-weight: 600;">
      <strong>Target Platform:</strong> Orange Pi RV2 selected as the primary physical testbed for native RISC-V Vector benchmarking.
    </div>
  </div>
</div>

---

<div class="kicker">[11. ROADMAP & DELIVERABLES]</div>

# Capstone Project Roadmap & Demonstration Milestones

<div class="grid-3">
  <div class="card">
    <div class="badge badge-indigo" style="margin-bottom: 12px;">PHASE 1 [COMPLETED]</div>
    <h2>Modular Library Core</h2>
    <ul style="margin-left: 20px; font-size: 19px; color: #334155; font-weight: 600; line-height: 1.6;">
      <li>Complete modular C++17 library.</li>
      <li><strong>56.2x SOR speedup</strong> verified in QEMU.</li>
      <li>MCAP export validated across 131 frames.</li>
    </ul>
  </div>

  <div class="card-indigo">
    <div class="badge badge-indigo" style="margin-bottom: 12px;">PHASE 2 [HARDWARE SETUP]</div>
    <h2>Hardware Bring-Up</h2>
    <ul style="margin-left: 20px; font-size: 19px; color: #1E293B; font-weight: 600; line-height: 1.6;">
      <li>Deploy on RVV 1.0 development board.</li>
      <li>Native toolchain & Linux environment.</li>
      <li>Hardware PMU cycle & cache profiling.</li>
    </ul>
  </div>

  <div class="card-teal">
    <div class="badge badge-teal" style="margin-bottom: 12px;">PHASE 3 [FINAL DEFENSE]</div>
    <h2>Capstone Demonstration</h2>
    <ul style="margin-left: 20px; font-size: 19px; color: #0F766E; font-weight: 600; line-height: 1.6;">
      <li>Multi-frame continuous dataset playback.</li>
      <li>On-board hardware latency verification.</li>
      <li>Open-source library release & defense.</li>
    </ul>
  </div>
</div>

