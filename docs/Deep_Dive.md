# RVPoint Pipeline 3D Ultra: Intuitive Deep Dive & Visual Math Guide

> **Document Version**: 2.0.0  
> **Target Pipeline**: [`eval/pipelines/pipeline_3d_ultra.cpp`](../eval/pipelines/pipeline_3d_ultra.cpp)  
> **Target Architecture**: RISC-V 64-bit Vector Extension (RVV 1.0, `rv64gcv`)  
> **Audience**: Engineers, researchers, and developers who want a **crystal-clear, easy-to-understand, scenario-based explanation** of how the 3D perception algorithms work and how they are accelerated on hardware vector registers.

---

## 1. The Real-World Scenario: The Robot on the Street

To make every equation and line of code easy to grasp, let us imagine a concrete real-world mission:

```
                    [ 3D LiDAR Sensor (Shoots 100,000 laser beams/sec) ]
                                            │
                                            ▼
           =================== THE REAL-WORLD SCENE ===================

               Floating Dust Motes (Airborne Noise)
                     *     *            *
                                              [Pedestrian] (Obstacle A)
                                                  o
                                                 /|\
             [Parked Car] (Obstacle B)           / \
                 _______
               _/       \_
              [O=========O]
   ────────────────────────────────────────────────────────────────────
   ═════════════════════════ FLAT ROAD (Ground Plane) ═════════════════
```

### The Scenario:
Imagine an autonomous delivery robot or self-driving vehicle navigating an urban street:
1. **The Input**: Its LiDAR spins around and captures **100,000 raw 3D $(x, y, z)$ points** every 100 milliseconds.
2. **What is in the Point Cloud?**
   * **The Road**: A huge, flat surface beneath the robot's wheels (approx. 60,000 points).
   * **Obstacle A**: A pedestrian walking 4 meters ahead (approx. 2,000 points).
   * **Obstacle B**: A parked car along the sidewalk (approx. 15,000 points).
   * **Noise**: Floating dust, insects, or sensor reflections in the air (approx. 300 isolated points).
3. **The Robot's Questions**:
   * *"Where is the road?"* (So the robot can drive on it without falling).
   * *"Where are the obstacles?"* (So the robot can steer around the pedestrian and car).
   * *"Can we compute this in under 30 milliseconds on a low-power RISC-V chip?"*

To answer these questions, [`pipeline_3d_ultra.cpp`](../eval/pipelines/pipeline_3d_ultra.cpp) executes a **10-stage perception pipeline**. Here is how each stage solves a piece of the puzzle:

```
+─────────────────────────────────────────────────────────────────────────────────────+
|                        THE 10-STAGE ULTRA PERCEPTION FLOW                           |
|                                                                                     |
|   [Raw Cloud: 100k pts]                                                             |
|           │                                                                         |
|           ▼                                                                         |
|   1. Voxel Downsampling    ──► Replaces dense clumps with 1 average point per cube.  |
|           │                    (100k pts ──► 25k pts)                               |
|           ▼                                                                         |
|   2. Spatial Hash Grid     ──► Sorts points into instant O(1) lookup cubes.         |
|           │                                                                         |
|           ▼                                                                         |
|   3. Outlier Filter (ROR)  ──► Deletes floating lonely dust particles.              |
|           │                                                                         |
|           ▼                                                                         |
|   4. Surface Normals       ──► Determines which way surfaces tilt using Cardano.    |
|           │                                                                         |
|           ▼                                                                         |
|   5. RANSAC Ground Fit     ──► Detects the road plane & removes it completely!      |
|           │                    (Leaves ONLY above-ground obstacles)                 |
|           ▼                                                                         |
|   6. Euclidean Clustering  ──► Groups remaining points into discrete objects:       |
|           │                    Cluster #1 = Pedestrian, Cluster #2 = Car.           |
|           ▼                                                                         |
|   [Safe Navigation Decision!]                                                       |
+─────────────────────────────────────────────────────────────────────────────────────+
```

---

## 2. Core Intuition: What is RISC-V Vectorization?

Before diving into the algorithms, let's understand **why** standard C++ code is slow and **how vectorization makes it lightning-fast**.

### 2.1 The "Grocery Checkout" Analogy

Imagine a grocery store with 100,000 customers who all want to buy 1 item:

* **Scalar CPU (Standard Loop)**:  
  You have **1 single cashier**. The cashier serves customer 1, then customer 2, then customer 3... and repeats this loop 100,000 times. Even if the cashier is fast, it takes a long time.
* **Vector CPU (RVV 1.0)**:  
  You open a **giant mega-counter with 32 cashiers standing side-by-side**. In a single "tick" (1 clock cycle), all 32 cashiers serve 32 customers simultaneously!  
  Instead of 100,000 separate transactions, you only need $100,000 / 32 \approx 3,125$ transactions.

```
SCALAR PROCESSING (1 by 1):
Cycle 1:  Point 0  ──► Calculate Distance
Cycle 2:  Point 1  ──► Calculate Distance
Cycle 3:  Point 2  ──► Calculate Distance
...
Cycle 32: Point 31 ──► Calculate Distance

VECTOR PROCESSING (32 at once via RVV):
Cycle 1:  [Point 0, Point 1, Point 2, ... Point 31] ──► Calculate Distance for ALL 32 at once!
```

---

### 2.2 Why Memory Layout Matters: AoS vs. SoA

To feed 32 cashiers at the same time, the items must arrive neatly organized on a conveyor belt.

#### The Bad Way: Array of Structures (AoS)
```
Memory: [ X0, Y0, Z0 | X1, Y1, Z1 | X2, Y2, Z2 | X3, Y3, Z3 | ... ]
```
Standard C++ libraries (like PCL) store points as structs: `{float x, y, z;}`.  
The X, Y, and Z values are interleaved. If the vector unit wants to load 32 X-values, it has to skip over Y and Z values. This is called a **strided load**, and it causes the memory controller to stall.

#### The RVPoint Way: Structure of Arrays (SoA)
```
Array X: [ X0, X1, X2, X3, X4, X5, ... X31 ]  ──► Clean 128-byte block!
Array Y: [ Y0, Y1, Y2, Y3, Y4, Y5, ... Y31 ]  ──► Clean 128-byte block!
Array Z: [ Z0, Z1, Z2, Z3, Z4, Z5, ... Z31 ]  ──► Clean 128-byte block!
```
RVPoint stores each coordinate in its own flat, contiguous array.  
The hardware instruction `__riscv_vle32_v_f32m8` can scoop up **32 or 64 X-coordinates in a single memory transfer**.

---

## 3. Function & File Directory

Here is the quick-reference map of where every function lives in the codebase:

| Function / Component | Located In File | Repository File Path | What It Does |
| :--- | :--- | :--- | :--- |
| `loadPCD` / `savePCD` | `simple_pcd_loader.h` | [`src/io/simple_pcd_loader.h`](../src/io/simple_pcd_loader.h) | Reads & writes PCD point cloud files. |
| `voxel_grid_downsamp_rvv_v2` | `voxel_grid_downsamp.cpp` | [`src/filters/voxel_grid_downsamp.cpp`](../src/filters/voxel_grid_downsamp.cpp) | Sort-based vectorized 3D downsampling. |
| `compute_bbox_rvv` | `voxel_grid_downsamp.cpp` | [`src/filters/voxel_grid_downsamp.cpp`](../src/filters/voxel_grid_downsamp.cpp) | Vectorized min/max bounding box finder. |
| `vfloor_i32m4` | `voxel_grid_downsamp.cpp` | [`src/filters/voxel_grid_downsamp.cpp`](../src/filters/voxel_grid_downsamp.cpp) | Vectorized rounding down (floor) function. |
| `compute_voxel_keys_rvv` | `voxel_grid_downsamp.cpp` | [`src/filters/voxel_grid_downsamp.cpp`](../src/filters/voxel_grid_downsamp.cpp) | Converts 3D coordinates to 1D voxel numbers. |
| `centroid_reduce_rvv` | `voxel_grid_downsamp.cpp` | [`src/filters/voxel_grid_downsamp.cpp`](../src/filters/voxel_grid_downsamp.cpp) | Vectorized average calculation per voxel. |
| `Fast3DSpatialGrid` | `fast_3d_spatial_grid.h` | [`src/search/fast_3d_spatial_grid.h`](../src/search/fast_3d_spatial_grid.h) | $O(1)$ fast 3D spatial hash table. |
| `execute_voxel_ror_rvv` | `pipeline_3d_ultra.cpp` | [`eval/pipelines/pipeline_3d_ultra.cpp`](../eval/pipelines/pipeline_3d_ultra.cpp) | Radius Outlier Filter with self-cell fastpath. |
| `execute_fused_sor_normals_rvv`| `pipeline_3d_ultra.cpp` | [`eval/pipelines/pipeline_3d_ultra.cpp`](../eval/pipelines/pipeline_3d_ultra.cpp) | Fused Statistical Outlier + Normal Estimation. |
| `compute_plane_coeffs` | `pipeline_3d_ultra.cpp` | [`eval/pipelines/pipeline_3d_ultra.cpp`](../eval/pipelines/pipeline_3d_ultra.cpp) | Fits a flat plane $(a,b,c,d)$ from 3 points. |
| `ransac_plane_sprt_rvv` | `pipeline_3d_ultra.cpp` | [`eval/pipelines/pipeline_3d_ultra.cpp`](../eval/pipelines/pipeline_3d_ultra.cpp) | Vectorized RANSAC with early stop test. |
| `extract_inliers_outliers_direct_soa` | `pipeline_3d_ultra.cpp` | [`eval/pipelines/pipeline_3d_ultra.cpp`](../eval/pipelines/pipeline_3d_ultra.cpp) | Vectorized split into road and obstacles. |
| `FastClustGrid` | `pipeline_3d_ultra.cpp` | [`eval/pipelines/pipeline_3d_ultra.cpp`](../eval/pipelines/pipeline_3d_ultra.cpp) | Hash table for clustering distance search. |
| `UnionFind` | `pipeline_3d_ultra.cpp` | [`eval/pipelines/pipeline_3d_ultra.cpp`](../eval/pipelines/pipeline_3d_ultra.cpp) | Disjoint Set data structure to merge clusters. |
| `execute_union_find_clustering_rvv` | `pipeline_3d_ultra.cpp` | [`eval/pipelines/pipeline_3d_ultra.cpp`](../eval/pipelines/pipeline_3d_ultra.cpp) | 13-forward neighbor vectorized clustering. |

---

## 4. Stage 3: Vectorized Voxel Grid Downsampling

### 4.1 The Scenario
When the robot looks at a flat wall 1 meter away, the LiDAR sensor blasts 10,000 points all crammed into a small $1\text{ meter} \times 1\text{ meter}$ area.  
Having 10,000 points that say the exact same thing is a waste of memory and compute.  
**Goal**: Carve space into little $10\text{ cm}$ cubes (voxels). If 20 points fall into the same cube, merge them into **1 single average point**.

```
                BEFORE DOWNSAMPLING                       AFTER DOWNSAMPLING
            (Too dense! 9 points in 1 cube)           (1 clean centroid point)

              ┌───────────────────┐                     ┌───────────────────┐
              │  *    *     *     │                     │                   │
              │     *   *   *     │        ───►         │         ●         │
              │  *    *       *   │                     │    (Average Point)│
              └───────────────────┘                     └───────────────────┘
```

---

### 4.2 The Math: How It Works Step-by-Step

#### Step A: Find the World Bounds
Find the smallest and largest coordinates in the whole scan:
$$x_{\min} = \min(x_0, x_1, \dots, x_{N-1}), \quad x_{\max} = \max(x_0, x_1, \dots, x_{N-1})$$
*(Same for $y$ and $z$)*.

#### Step B: Convert 3D Point Coordinates into an Integer Box ID
If our cube size is $\text{leaf\_size} = 0.10\text{ m}$ ($10\text{ cm}$), the multiplier is $\text{inv\_leaf} = \frac{1}{0.10} = 10.0$.  
For any point $(x, y, z)$:
$$i_x = \lfloor x \cdot 10.0 \rfloor - \min(i_x)$$
$$i_y = \lfloor y \cdot 10.0 \rfloor - \min(i_y)$$
$$i_z = \lfloor z \cdot 10.0 \rfloor - \min(i_z)$$

We convert these 3 numbers $(i_x, i_y, i_z)$ into a single unique integer key:
$$\text{Voxel\_Key} = i_x + (i_y \times \text{grid}_x) + (i_z \times \text{grid}_x \times \text{grid}_y)$$

#### Concrete Numerical Example:
Suppose:
* Point A is at $(1.02, 2.04, 0.51)$.
  * $i_x = \lfloor 1.02 \times 10 \rfloor = \lfloor 10.2 \rfloor = 10$
  * $i_y = \lfloor 2.04 \times 10 \rfloor = \lfloor 20.4 \rfloor = 20$
  * $i_z = \lfloor 0.51 \times 10 \rfloor = \lfloor 5.1 \rfloor = 5$
* Point B is at $(1.08, 2.06, 0.55)$.
  * $i_x = \lfloor 1.08 \times 10 \rfloor = 10$
  * $i_y = \lfloor 2.06 \times 10 \rfloor = 20$
  * $i_z = \lfloor 0.55 \times 10 \rfloor = 5$
* Notice that Point A and Point B get the **exact same box numbers $(10, 20, 5)$**! They are in the same cube!

#### Step C: Average the Points in Each Box (Centroid)
$$\bar{x} = \frac{x_A + x_B}{2} = \frac{1.02 + 1.08}{2} = 1.05\text{ m}$$
$$\bar{y} = \frac{y_A + y_B}{2} = \frac{2.04 + 2.06}{2} = 2.05\text{ m}$$
$$\bar{z} = \frac{z_A + z_B}{2} = \frac{0.51 + 0.55}{2} = 0.53\text{ m}$$
Output point: $(1.05, 2.05, 0.53)$.

---

### 4.3 Which Portion is Vectorized?

| Operation | Scalar Way (Slow) | Vectorized Way (RVPoint) |
| :--- | :--- | :--- |
| **Finding Min & Max** | Loop 100,000 times with `if (x < min) min = x;` | Vector min/max instructions compare 32 points per cycle. |
| **Computing Voxel Keys** | Multiplies, rounds down, and hashes 1 point at a time. | Computes 32 voxel keys at the same instant in vector registers. |
| **Sorting Keys** | Standard PCL inserts into a slow C++ `std::map` tree ($O(N \log N)$). | Fast in-place Radix Sort rearranges memory so identical keys sit together. |
| **Centroid Reduction** | Adds points one-by-one. | Vector gather (`vluxei32`) + vector sum reduction (`vfredosum`). |

---

### 4.4 How Did We Vectorize It? (Plain Technical Explanation)

1. **Vector Bounding Box (`compute_bbox_rvv`)**:
   We load 32 points into vector register `vx`. We call:
   ```c
   vmin_x = __riscv_vfmin_vv_f32m4(vmin_x, vx, vl);
   vmax_x = __riscv_vfmax_vv_f32m4(vmax_x, vx, vl);
   ```
   This takes the minimum and maximum of 32 points in 1 cycle. At the end, a horizontal tree reduction (`vfredmin`) gives the global scalar min and max.
2. **Vector Floor Function (`vfloor_i32m4`)**:
   Hardware truncation rounds towards zero (so $-0.8$ becomes $0$ instead of $-1$). We fix this in parallel:
   * Convert float to int with truncation: `trunc_i = vfcvt.rtz(vx)`
   * Check if truncation rounded up: `needs_fix = vmflt(vx, trunc_f)`
   * Subtract 1 under mask: `floored = vsub(needs_fix, trunc_i, 1)`
3. **Vector Key Generation (`compute_voxel_keys_rvv`)**:
   In parallel across 32 lanes, we calculate:
   $$\text{key} = i_x + i_y \cdot \text{grid}_x + i_z \cdot \text{grid}_{xy}$$
   using vector multiply (`vmul.vx`) and vector add (`vadd.vv`).

---

## 5. Stage 5: Outlier Removal (Dust & Sensor Noise Filter)

### 5.1 The Scenario
LiDAR beams sometimes bounce off steam from a manhole, airborne dust, or bugs. These show up as **lonely, isolated points floating in thin air**.  
A real physical object (like a pedestrian's coat or a lamppost) will reflect dozens of nearby points. A dust particle is completely alone.

```
       SCENARIO: REAL OBJECT VS AIRBORNE NOISE

       [Real Obstacle: Pedestrian]              [Airborne Dust Mote]
             *   *   *   *
           *   *   *   *   *                             *  (Lonely!)
             *   *   *   *
        (Dense cluster: 20+ neighbors)          (0 neighbors within 25 cm)
                 KEEP!                                   DELETE!
```

---

### 5.2 The Math: Radius Outlier Removal (ROR)

For every query point $\mathbf{q} = (q_x, q_y, q_z)$:
1. Search within a sphere of radius $R = 0.25\text{ m}$ ($25\text{ cm}$).
2. For every nearby candidate point $\mathbf{p} = (x, y, z)$, compute squared Euclidean distance:
   $$d^2 = (x - q_x)^2 + (y - q_y)^2 + (z - q_z)^2$$
3. If $d^2 \le R^2$ (i.e. $d^2 \le 0.25^2 = 0.0625$), increment neighbor count.
4. **Decision Rule**:
   $$\text{Status} = \begin{cases} \text{KEEP} & \text{if } \text{Neighbor Count} \ge 2 \\ \text{DELETE (Outlier)} & \text{if } \text{Neighbor Count} < 2 \end{cases}$$

---

### 5.3 Which Portion is Vectorized & Optimized?

* **The Self-Cell Fastpath**:  
  Standard search algorithms query 27 spatial grid cells (the home cell + 26 surrounding neighbor cells).  
  *Intuition*: In downsampled clouds, if a point is part of a real object, its home cell **already contains enough neighbor points** to pass the test!  
  By checking the home cell first, the algorithm **skips the other 26 cells for $>80\%$ of all points**.
* **Vectorized Squared Distance**:  
  When checking neighbor candidates, we calculate $(x - q_x)^2 + (y - q_y)^2 + (z - q_z)^2$ across 32 candidate points simultaneously.

---

### 5.4 How Did We Vectorize It?

1. Load candidate $X, Y, Z$ arrays into vector registers `vx, vy, vz`.
2. Subtract query point coordinates:
   ```c
   vfloat32m8_t dx = __riscv_vfsub_vf_f32m8(vx, qx, vl);
   vfloat32m8_t dy = __riscv_vfsub_vf_f32m8(vy, qy, vl);
   vfloat32m8_t dz = __riscv_vfsub_vf_f32m8(vz, qz, vl);
   ```
3. Accumulate squared distances using **Fused Multiply-Accumulate (FMA)**:
   ```c
   vfloat32m8_t d2 = __riscv_vfmul_vv_f32m8(dx, dx, vl);           // dx^2
   d2 = __riscv_vfmacc_vv_f32m8(d2, dy, dy, vl);                   // + dy^2 in 1 cycle
   d2 = __riscv_vfmacc_vv_f32m8(d2, dz, dz, vl);                   // + dz^2 in 1 cycle
   ```
4. Compare against radius threshold $R^2$:
   ```c
   vbool4_t mask = __riscv_vmfle_vf_f32m8_b4(d2, r2, vl);
   ```
5. Count inliers instantly using hardware population count:
   ```c
   int in_radius = __riscv_vcpop_m_b4(mask, vl);
   ```

---

## 6. Stage 7: Surface Normal Estimation (Cardano Closed-Form)

### 6.1 The Scenario
The robot needs to know: **"Which direction is this surface facing?"**
* If an arrow perpendicular to the surface points straight UP into the sky, it's a **flat floor or road**.
* If the arrow points sideways, it's a **vertical wall, a tree, or the side of a car**.
* This perpendicular arrow is called the **Surface Normal** $\mathbf{n} = (n_x, n_y, n_z)$.

```
     VERTICAL WALL                      FLAT ROAD (GROUND)
       Normal points SIDEWAYS              Normal points UP
       n = (1.0, 0.0, 0.0)                 n = (0.0, 0.0, 1.0)

              │                                    ▲ n
              │ ──► n                              │
              │                                    │
              │                            ─────────────────
```

---

### 6.2 The Math: From Neighbors to Surface Normal

#### Step A: Compute the $3 \times 3$ Covariance Matrix
For $K$ neighbor points around our query point, we find their center (centroid) $\bar{\mathbf{p}} = (\bar{x}, \bar{y}, \bar{z})$.  
We measure how much the points spread along each direction:
$$\Delta x = x_i - \bar{x}, \quad \Delta y = y_i - \bar{y}, \quad \Delta z = z_i - \bar{z}$$
The $3 \times 3$ Covariance Matrix $\mathbf{C}$ is:
$$\mathbf{C} = \begin{bmatrix}
\sum \Delta x^2 & \sum \Delta x \Delta y & \sum \Delta x \Delta z \\
\sum \Delta x \Delta y & \sum \Delta y^2 & \sum \Delta y \Delta z \\
\sum \Delta x \Delta z & \sum \Delta y \Delta z & \sum \Delta z^2
\end{bmatrix}$$

#### Step B: The Problem with Standard PCL (Jacobi Rotations)
The surface normal is the eigenvector corresponding to the **smallest eigenvalue** $\lambda_{\min}$ of this matrix.  
Standard PCL runs an iterative algorithm called **Jacobi Rotations**: it guesses, rotates the matrix, checks error, rotates again, and repeats in a loop 10 to 20 times per point!  
For 30,000 points, that means running hundreds of thousands of slow loops.

#### Step C: The RVPoint Solution: Cardano's Exact Formula
A $3 \times 3$ matrix eigenvalue equation is just a 3rd-degree polynomial (a cubic equation):
$$\det(\mathbf{C} - \lambda \mathbf{I}) = -\lambda^3 + c_2 \lambda^2 - c_1 \lambda + c_0 = 0$$
Instead of guessing in a loop, we solve it **directly** using Girolamo Cardano and François Viète's closed-form trigonometric formula:
1. Compute matrix invariants:
   $$c_2 = \text{tr}(\mathbf{C}) = C_{00} + C_{11} + C_{22}$$
   $$p = c_1 - \frac{c_2^2}{3}, \quad q = -c_0 + \frac{c_1 c_2}{3} - \frac{2 c_2^3}{27}$$
2. Compute the exact angle:
   $$\phi = \frac{1}{3} \arccos\left( \frac{-q / 2}{\sqrt{-(p/3)^3}} \right)$$
3. The smallest eigenvalue $\lambda_{\min}$ drops out immediately in 1 step:
   $$\lambda_{\min} = 2 \sqrt{-\frac{p}{3}} \cos\left( \phi + \frac{4\pi}{3} \right) + \frac{c_2}{3}$$
4. The normal $\mathbf{n}$ is obtained by a single cross-product of matrix rows!

**Result**: Zero iteration loops. Constant $O(1)$ time. Over **$4\times$ faster** than PCL!

---

### 6.3 Which Portion is Vectorized?
* **Vectorized**: Accumulating the covariance sums ($\sum \Delta x^2, \sum \Delta x \Delta y, \dots$) across neighbor points using vector Multiply-Accumulate (`vfmacc`).
* **Closed-Form**: Solving the cubic roots directly on the scalar floating-point unit with zero loop overhead.

---

## 7. Stage 8: Hardware RVV 1.0 RANSAC Ground Plane Removal

### 7.1 The Scenario
The road represents $60\%$ to $70\%$ of all points in the scan.  
To detect obstacles, we must **find the road and remove it**.  
If we don't remove the road, the pedestrian and the road will look like one giant connected blob of points!

```
       SCENARIO: ROAD EXTRACTION & OBSTACLE ISOLATION

   Raw Cloud (Obstacles + Road):         Ground Removed (Obstacles Isolated!):
          o   (Pedestrian)                           o
         /|\                                        /|\
         / \                                        / \
     ─────────────── (Road: 60k pts)       (Road erased! Ready to cluster!)
```

---

### 7.2 The Math: How RANSAC Fits a Plane

A flat plane in 3D space is described by the equation:
$$a x + b y + c z + d = 0$$
where $(a, b, c)$ is the normal vector pointing perpendicular to the plane, and $d$ is the distance from the origin.

#### Step 1: Pick 3 Random Points
The algorithm randomly picks 3 points: $\mathbf{p}_1, \mathbf{p}_2, \mathbf{p}_3$.  
The plane passing through these 3 points has normal:
$$\mathbf{v} = (\mathbf{p}_2 - \mathbf{p}_1) \times (\mathbf{p}_3 - \mathbf{p}_1)$$
Normalizing $\mathbf{v}$ gives $(a, b, c)$, and $d = -(a x_1 + b y_1 + c z_1)$.

#### Step 2: The Ground Angle Prior Gate (Sanity Check)
In our robot scenario, the road is horizontal, meaning the ground normal should point mostly UP ($+Z$ axis).  
If the 3 randomly chosen points lie on the side of a building, the normal will point sideways ($+X$ or $+Y$).  
We test:
$$|c| \ge \cos(45^\circ) = 0.707$$
If $|c| < 0.707$, the plane is tilted more than $45^\circ$. It **cannot** be the ground! We discard it instantly in 1 nanosecond without checking any points.

#### Step 3: Count Inliers (Points on the Road)
For every point $\mathbf{p}_i = (x_i, y_i, z_i)$, its distance to the plane is:
$$\text{dist} = |a x_i + b y_i + c z_i + d|$$
If $\text{dist} \le 0.06\text{ m}$ ($6\text{ cm}$), the point is sitting on the road (an **Inlier**).  
If $\text{dist} > 0.06\text{ m}$, the point is an obstacle sticking up in the air (an **Outlier**).

#### Concrete Numerical Example:
Suppose our candidate road plane is $z - 0.10 = 0$ (meaning $a=0, b=0, c=1, d=-0.10$).
* **Point on the Road**: $(2.0, 3.0, 0.11)$  
  $$\text{dist} = |0(2.0) + 0(3.0) + 1(0.11) - 0.10| = |0.01| = 1\text{ cm}$$
  $1\text{ cm} \le 6\text{ cm} \rightarrow$ **ROAD INLIER!**
* **Point on Pedestrian's Head**: $(2.0, 3.0, 1.75)$  
  $$\text{dist} = |0(2.0) + 0(3.0) + 1(1.75) - 0.10| = |1.65| = 165\text{ cm}$$
  $165\text{ cm} > 6\text{ cm} \rightarrow$ **OBSTACLE OUTLIER!**

---

### 7.3 Which Portion is Vectorized?

Testing 30,000 points against the plane equation:
$$\text{dist}_i = a x_i + b y_i + c z_i + d$$
This formula is evaluated across **32 points at a time** using RISC-V vector fused multiply-accumulate instructions.

---

### 7.4 How Did We Vectorize It?

Here is the exact assembly instruction sequence running on the RISC-V vector core:
```c
// 1. Load 32 X, Y, Z coordinates simultaneously
vfloat32m8_t vx = __riscv_vle32_v_f32m8(&cloud.x[i], vl);
vfloat32m8_t vy = __riscv_vle32_v_f32m8(&cloud.y[i], vl);
vfloat32m8_t vz = __riscv_vle32_v_f32m8(&cloud.z[i], vl);

// 2. Compute dist = a*x + b*y + c*z + d in vector hardware
vfloat32m8_t dist = __riscv_vfmul_vf_f32m8(vx, a, vl);        // a * x
dist = __riscv_vfmacc_vf_f32m8(dist, b, vy, vl);              // + b * y (FMA)
dist = __riscv_vfmacc_vf_f32m8(dist, c, vz, vl);              // + c * z (FMA)
dist = __riscv_vfadd_vf_f32m8(dist, d, vl);                   // + d

// 3. Bilateral check: is -0.06 <= dist <= +0.06 ?
vbool4_t mask_le = __riscv_vmfle_vf_f32m8_b4(dist, 0.06f, vl);
vbool4_t mask_ge = __riscv_vmfge_vf_f32m8_b4(dist, -0.06f, vl);
vbool4_t inlier_mask = __riscv_vmand_mm_b4(mask_le, mask_ge, vl);

// 4. Count total inliers across all 32 lanes in 1 clock cycle!
sample_inliers += __riscv_vcpop_m_b4(inlier_mask, vl);
```

#### Step 4: Zero-Copy Vector Stream Partitioning (`vcompress.vm`)
Once the winning road plane is found, we must separate the points into two arrays: Road points and Obstacle points.  
Instead of slow `if/else` branches and `std::vector::push_back`, RVPoint uses the RISC-V **vector compress instruction**:
```c
vfloat32m8_t compressed_x = __riscv_vcompress_vm_f32m8(vx, outlier_mask, vl);
```
`vcompress` automatically packs only the obstacle points contiguously into the destination buffer in a single vector store!

---

## 8. Stage 9: Euclidean Clustering with Disjoint-Set Union (Union-Find)

### 8.1 The Scenario
Now the road is gone! All that remains in the point cloud are the **obstacles**:
* Point clump 1: The pedestrian.
* Point clump 2: The parked car.
* Point clump 3: A traffic cone.

**Goal**: Group nearby points together so the robot's planner knows:  
*"There are 3 distinct objects: Object #1 is at $(x=2, y=3)$ and Object #2 is at $(x=10, y=1)$."*

```
     REMAINING OBSTACLES:               AFTER EUCLIDEAN CLUSTERING:

        *  *                                  [CLUSTER 1: Pedestrian]
       *  *  *    (Points close together)         o  (Points grouped!)
        *  *                                     /|\
                                                 / \

                                              [CLUSTER 2: Parked Car]
            * * * * *                             _______
           * * * * * *                          _/       \_
            * * * * *                          [O=========O]
```

---

### 8.2 The Math: Euclidean Distance & Connectivity Rule

Two points $\mathbf{p}_1$ and $\mathbf{p}_2$ belong to the same object if their distance is less than or equal to the cluster tolerance (e.g., $d_{\text{tol}} = 0.15\text{ m} = 15\text{ cm}$):
$$\|\mathbf{p}_1 - \mathbf{p}_2\|_2 = \sqrt{(x_1 - x_2)^2 + (y_1 - y_2)^2 + (z_1 - z_2)^2} \le 0.15\text{ m}$$
To avoid slow square root operations, we compare squared distances:
$$d^2 \le (0.15)^2 = 0.0225\text{ m}^2$$

#### The Problem with Standard PCL:
Standard PCL uses **Breadth-First Search (BFS)** with a `std::queue`.  
It pushes and pops indices one point at a time from heap memory. On embedded CPUs, the constant cache misses kill performance.

---

### 8.3 The RVPoint Solution: Symmetric 13-Neighbor Grid + Union-Find

#### Step A: Symmetric 13-Forward Direction Culling
In a 3D grid, each cube has 26 neighboring cubes surrounding it.  
*Intuition*: Distance is symmetric! If Cube A is within 15 cm of Cube B, then Cube B is automatically within 15 cm of Cube A. Checking both directions is redundant!  
RVPoint only checks the **13 forward neighbors**:
$$\text{Directions to check} = 13 \quad (\text{Skips the opposite 13! Cuts work in half!})$$

```
                   13 FORWARD DIRECTIONS (Upper Hemisphere)
                           (-1, -1, 1), ( 0, -1, 1), ( 1, -1, 1)
                           (-1,  0, 1), ( 0,  0, 1), ( 1,  0, 1)
                           (-1,  1, 1), ( 0,  1, 1), ( 1,  1, 1)
                           (-1,  1, 0), ( 0,  1, 0), ( 1,  1, 0)
                                        ( 1,  0, 0)
```

#### Step B: Vectorized Distance Check on Candidate Batches
We gather all candidate points from the 13 forward cells into flat arrays (`cand_x`, `cand_y`, `cand_z`).  
For each point $\mathbf{q}$ in our cell, we test all candidate points in vector chunks of 32:
```c
vfloat32m8_t ddx = __riscv_vfsub_vf_f32m8(cand_vx, qx, vl);
vfloat32m8_t ddy = __riscv_vfsub_vf_f32m8(cand_vy, qy, vl);
vfloat32m8_t ddz = __riscv_vfsub_vf_f32m8(cand_vz, qz, vl);

vfloat32m8_t d2 = __riscv_vfmul_vv_f32m8(ddx, ddx, vl);
d2 = __riscv_vfmacc_vv_f32m8(d2, ddy, ddy, vl);
d2 = __riscv_vfmacc_vv_f32m8(d2, ddz, ddz, vl);

vbool4_t in_tol = __riscv_vmfle_vf_f32m8_b4(d2, tol_sq, vl);
```
If lane $k$ satisfies $d^2 \le d_{\text{tol}}^2$, an edge $(\mathbf{q}, \text{candidate}_k)$ is recorded.

#### Step C: Disjoint-Set Union-Find (DSU)
Instead of recursion and queues, we merge connected points using **Union-Find with Path Compression**:
* `find(i)`: Climbs the parent pointer tree to find the root cluster ID. Two-pass path compression points all intermediate nodes directly to the root, making subsequent lookups nearly $O(1)$!
* `unite(i, j)`: Merges two clusters into one by rank.

#### Step D: Two-Pass Zero-Allocation CSR Grouping
To assemble the final cluster lists without allocating lots of small vectors:
1. **Pass 1**: Count how many points belong to each root: `root_counts[uf.find(i)]++`.
2. **Filter**: If $50 \le \text{count} \le 100,000$, assign it a valid Cluster ID.
3. **Pass 2**: Populate the indices into pre-allocated memory.

---

## 9. Quick-Reference: Scalar vs. Vector Operations Glossary

This table summarizes the exact RISC-V vector intrinsics used across the pipeline and their plain-English meaning:

| RVV 1.0 Intrinsic Instruction | Plain English Meaning | What It Replaced from Standard C++ |
| :--- | :--- | :--- |
| `__riscv_vle32_v_f32m8` | **"Vector Load"**: Loads 32 consecutive floats from RAM into vector registers in 1 operation. | 32 separate scalar memory loads. |
| `__riscv_vfmul_vf_f32m8` | **"Vector Multiply by Scalar"**: Multiplies all 32 values in a vector by 1 scalar constant. | A 32-iteration `for` loop with `arr[i] * c`. |
| `__riscv_vfmacc_vv_f32m8` | **"Vector Fused Multiply-Add"**: Computes $A + (B \times C)$ for 32 elements in a single clock cycle. | Separate multiply and add operations. |
| `__riscv_vmfle_vf_f32m8_b4` | **"Vector Less-Than-or-Equal"**: Compares 32 floats against a threshold, returning a 32-bit boolean mask. | 32 scalar `if (val <= thresh)` branch checks. |
| `__riscv_vcpop_m_b4` | **"Population Count"**: Counts how many `true` bits are in a mask in 1 clock cycle. | A loop with `if (condition) count++;`. |
| `__riscv_vcompress_vm_f32m8` | **"Vector Stream Compress"**: Gathers only the `true` elements and packs them contiguously into memory. | A loop with `if (keep) out.push_back(val);`. |
| `__riscv_vfredmin_vs_f32m4_f32m1` | **"Horizontal Tree Reduction"**: Finds the smallest value across all vector lanes. | A loop searching for minimum element. |

---

## 10. Summary

By combining:
1. **Structure-of-Arrays (SoA)** memory layout (enabling unit-stride vector memory loads),
2. **RISC-V Vector 1.0 (RVV)** instructions (evaluating 32 to 64 points simultaneously),
3. **Cardano's Closed-Form Math** (solving cubic roots analytically with zero numerical iterations),
4. **Symmetric 13-neighbor spatial hash grids** and **Union-Find graph connectivity**,

[`pipeline_3d_ultra.cpp`](../eval/pipelines/pipeline_3d_ultra.cpp) achieves **full 3D LiDAR perception at high real-time frame rates** on embedded RISC-V silicon.
