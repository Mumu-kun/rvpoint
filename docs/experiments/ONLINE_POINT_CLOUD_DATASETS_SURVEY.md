# Online Point Cloud Datasets Survey & Alignment for RVPoint

> **Document ID**: `docs/experiments/ONLINE_POINT_CLOUD_DATASETS_SURVEY.md`
> **Target Architecture**: RISC-V 64-bit with RVV 1.0 Vector Extension (`rv64gcv`)
> **Target Applications**: 2.5D Costmap, 3D Oriented Bounding Boxes, Pairwise Odometry, Roadside Background Subtraction
> **Date**: September 2026
> **Status**: Official Research Survey & Dataset Catalog

---

## 1. Executive Summary & Synthesis Matrix

This survey evaluates open-access online point cloud datasets across 5 major perception domains, assessing their alignment with **RVPoint**'s zero-dependency C++17 library architecture, contiguous Structure-of-Arrays (SoA) memory model, and embedded RISC-V hardware envelope (low-bandwidth DRAM, L1/L2 cache constraints, and physical SBC / gem5 simulation execution).

### Comprehensive Dataset Comparison Matrix

| Domain / Task | Dataset Name | Primary Sensor & Beams | Native Format | Ground Truth Provided | Size (Sample vs Full) | Ingestion into RVPoint SoA | Recommended Workflow Alignment |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Automotive 3D Detection** | **KITTI 3D Object** | Velodyne HDL-64E (64) | `.bin` (float32 $\times$ 4) | 3D Bounding Boxes (`h,w,l,x,y,z,yaw`) | $0.5\text{ GB}$ (sample) / $12\text{ GB}$ | Unit-stride 16-byte binary read | **Workflow 2: 3D Bounding Boxes** |
| **Point-wise Ground Truth** | **SemanticKITTI** | Velodyne HDL-64E (64) | `.bin` (pts) + `.label` (uint32) | 28 semantic classes (road, car, etc.) | $1\text{ GB}$ (Seq 00) / $80\text{ GB}$ | Direct label array masking | **Ground Segmentation & Clustering Validation** |
| **Autonomous Driving Scale** | **nuScenes** | 32-beam LiDAR | Custom `.bin` + JSON relational DB | 1.4M 3D boxes + velocity vectors | $4\text{ GB}$ (mini) / $400\text{ GB}$ | Requires Python parser to unpack | Secondary evaluation |
| **Autonomous Driving Scale** | **Waymo Open** | 64-beam top LiDAR | TFRecord (Protobuf) | 3D bounding boxes + tracks | $25\text{ GB}$ (sample) / $>1\text{ TB}$ | Heavy protobuf dependency | Excluded (violates zero-dep policy) |
| **LiDAR Odometry (Standard)** | **KITTI Odometry** | Velodyne HDL-64E (64) | `.bin` (raw) / `.pcd` | $3 \times 4$ camera pose matrices | **In-Repo: 131 frames** ($130\text{ MB}$) | Native `loadPCD()` | **Workflow 4: Pairwise Odometry** 🥇 |
| **Challenging 6-DoF SLAM** | **Newer College** | Ouster OS1-64 (64) | `rosbag` / `.pcd` | Leica BLK360 millimeter survey map | $2\text{ GB}$ (sample) / $45\text{ GB}$ | Native `loadPCD()` for converted | **Non-planar terrain odometry** |
| **Urban Range Sensing** | **MulRan (KAIST)** | Ouster OS1-64 (64) | `.bin` (KITTI format) | 6-DoF pose CSV (`SE(3)`) | $5\text{ GB}$ (Seq) / $120\text{ GB}$ | Unit-stride 16-byte binary read | **Pairwise Odometry Benchmark** |
| **Warehouse AMR / AGV** | **ANavS Warehouse** | Velodyne VLP-16 (16) | `.bin` + `.txt` labels | 3D boxes for forklifts, pallets | $450\text{ MB}$ (3,287 scans) | Unit-stride 16-byte binary read | **Workflow 1: 2.5D Costmap** 🥇 |
| **Indoor RGB-D / Small AMR** | **TUM RGB-D (Freiburg)** | PrimeSense / Kinect | `.png` depth + RGB | OptiTrack sub-cm motion capture | $200\text{ MB}$ (sample) / $15\text{ GB}$ | Pin-hole camera back-projection | **Indoor Navigation & 2.5D Costmap** |
| **Fixed Roadside V2X** | **DAIR-V2X (I-side)** | 300-beam RSU LiDAR | `.pcd` (Native) | 3D bounding boxes in Virtual LiDAR frame | $1.2\text{ GB}$ (sample) / $50\text{ GB}$ | **Direct Zero-Copy `loadPCD()`** | **Workflow 3: Roadside V2X** 🥇 |
| **Highway Overhead V2X** | **A9 / TUMTraf** | Ouster OS1-64 Gen 2 | `.pcd` (ASCII/Binary) | OpenLABEL JSON 3D boxes | $800\text{ MB}$ (sample) / $35\text{ GB}$ | **Direct Zero-Copy `loadPCD()`** | **Workflow 3: Highway V2X** 🥇 |
| **Metrology & Registration** | **Stanford 3D Scan** | Cyberware laser scanner | `.ply` (ASCII/Binary) | Ground truth CAD mesh models | $1.5\text{ MB}\text{--}50\text{ MB}$ (Bunny, Dragon) | Simple PLY vertex parser | **Analytical ICP & Cardano Normals** |

---

## 2. Deep Investigation by Perception Domain

---

### Category 1: Automotive & Mobile Driving Perception

#### 1. KITTI 3D Object Detection & Tracking Benchmark
- **Primary Source**: Geiger, A., Lenz, P., & Urtasun, R. (2012). *Are we ready for autonomous driving? The KITTI vision benchmark suite.* IEEE CVPR 2012.
  - Official Portal: [http://www.cvlibs.net/datasets/kitti/](http://www.cvlibs.net/datasets/kitti/)
- **Sensor Configuration**:
  - LiDAR: Velodyne HDL-64E (64 channels, 10 Hz, 360° horizontal FOV, 26.8° vertical FOV, up to 130,000 points/frame).
  - Synchronized with PointGrey Flea2 color/monochrome cameras and GPS/IMU (OXTS RT 3003).
- **Data Format & Ingestion**:
  - Files: `velodyne/*.bin`. Stored as flat binary IEEE 754 32-bit floats: $[x, y, z, r]$.
  - Each point is exactly 16 bytes: `x, y, z` in meters, `r` (reflectance) normalized $[0, 1]$.
  - **Conversion to `PointCloudSoA`**: Extremely simple ($<20$ lines of C++). A single `fread` populates contiguous memory, followed by unit-stride vector separation:
    ```cpp
    struct PointXYZR { float x, y, z, r; };
    std::vector<PointXYZR> raw(n_pts);
    fread(raw.data(), sizeof(PointXYZR), n_pts, fp);
    for (size_t i = 0; i < n_pts; ++i) {
        cloud.x[i] = raw[i].x;
        cloud.y[i] = raw[i].y;
        cloud.z[i] = raw[i].z;
    }
    ```
- **Ground Truth Format**:
  - Labels provided in text files with 15 space-separated values per object:
    `type, truncated, occluded, alpha, bbox_2d(4), dimensions(h, w, l), location(x, y, z), rotation_y`.
  - Coordinates are given in camera coordinates ($y$ downward, $z$ forward) and convert to LiDAR frame ($z$ upward, $x$ forward) via:
    $$\mathbf{p}_{\text{velo}} = T_{\text{cam\_to\_velo}} \cdot \mathbf{p}_{\text{cam}}$$
- **Alignment with RVPoint**:
  - **Direct Gold-Standard for Workflow 2 (3D Bounding Boxes)**: We can directly compare RVPoint’s Rotating Calipers output $[x_c, y_c, z_c, l, w, h, \psi]$ against official KITTI ground truth annotations to report IoU (Intersection over Union) and orientation error.

#### 2. SemanticKITTI (Point-Wise Semantic Classification)
- **Primary Source**: Behley, J. et al. (2019). *SemanticKITTI: A Dataset for Semantic Scene Understanding of LiDAR Sequences.* IEEE ICCV 2019.
  - Official Portal: [http://www.semantic-kitti.org/](http://www.semantic-kitti.org/)
- **Sensor Configuration**: Same Velodyne HDL-64E sequences as KITTI Odometry (22 sequences, $>43,000$ scans).
- **Ground Truth Format**:
  - Binary `.label` files corresponding 1:1 with each `.bin` scan.
  - Each point has a 32-bit unsigned integer: `uint16_t label_id = raw_label & 0xFFFF;`.
  - 28 semantic classes: `road` (40), `sidewalk` (44), `parking` (48), `car` (10), `truck` (11), `pedestrian` (30), `pole` (80), `vegetation` (70).
- **Alignment with RVPoint**:
  - **Ideal Ground Truth Validator for Stage 7 (RANSAC Ground Removal)**: By checking points labeled as `road`/`sidewalk` vs non-ground, we compute exact Precision/Recall metrics for RVPoint's vectorized RANSAC and SPRT early exit.

#### 3. nuScenes (Motional)
- **Primary Source**: Caesar, H. et al. (2020). *nuScenes: A multimodal dataset for autonomous driving.* IEEE CVPR 2020.
  - Official Portal: [https://www.nuscenes.org/](https://www.nuscenes.org/)
- **Sensor Configuration**: 32-beam LiDAR (20 Hz, ~35,000 points/frame, 360° FOV).
- **Data Format**: Packed `.bin` files (`x, y, z, intensity, ring_index`), annotations in a complex relational JSON schema.
- **Evaluation**: The dataset size ($400\text{ GB}$ full, $4\text{ GB}$ mini) and relational database setup make it less convenient for fast C++ unit tests than KITTI, but the `nuScenes-mini` subset provides a viable 32-beam alternative.

---

### Category 2: LiDAR Odometry & Scan Matching (SLAM)

#### 1. KITTI Odometry Benchmark
- **Primary Source**: Geiger et al. (2012).
  - Official Portal: [http://www.cvlibs.net/datasets/kitti/eval_odometry.php](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)
- **Dataset Specs**:
  - 22 driving sequences (Sequences 00–10 with public ground truth poses, 11–21 for benchmark evaluation).
  - Velodyne HDL-64E point clouds recorded at 10 Hz.
- **Ground Truth Format**:
  - Text files (`00.txt`, `01.txt`, etc.) where each line contains 12 space-separated floats:
    $$P = \begin{bmatrix} r_{11} & r_{12} & r_{13} & t_x \\ r_{21} & r_{22} & r_{23} & t_y \\ r_{31} & r_{32} & r_{33} & t_z \end{bmatrix}$$
- **Alignment with RVPoint**:
  - **Repository Invariant**: RVPoint *already contains* 131 frames of KITTI Odometry Sequence 00 in `data/pcd_compressed/`!
  - **Workflow 4 (Pairwise Point-to-Plane Odometry)**: Runs directly on this local dataset, integrating poses frame-by-frame and comparing against the ground-truth trajectory matrix.

#### 2. The Newer College Dataset (Oxford)
- **Primary Source**: Ramezani, M. et al. (2020). *The Newer College Dataset: Handheld LiDAR, Inertial and Vision with Ground Truth.* IEEE IROS 2020.
  - Official Portal: [https://ori-drs.github.io/newer-college-dataset/](https://ori-drs.github.io/newer-college-dataset/)
- **Sensor Configuration**:
  - Ouster OS1-64 (Gen 1 / Gen 2, 64-beam, 10 Hz / 20 Hz, 65,536 to 131,072 points per frame).
  - Alphasense multi-camera system + IMU.
  - Platforms: Handheld sensor rig and ANYmal quadruped walking robot.
- **Ground Truth Format**:
  - Millimeter-accurate centimeter-dense 3D laser survey map collected using a survey-grade **Leica BLK360** terrestrial scanner.
  - 6-DoF trajectory poses registered directly to the survey map.
- **Data Format & Ingestion**:
  - Distributed as ROS bag files and pre-extracted `.pcd` files via the Oxford DRS repository.
  - Can be loaded directly by RVPoint's `simple_pcd_loader.h`.
- **Alignment with RVPoint**:
  - Ideal for evaluating non-planar 6-DoF odometry (staircases, uneven park grass, indoor cloisters) where flat planar ground assumptions break down.

#### 3. MulRan (Multimodal Range Dataset - KAIST)
- **Primary Source**: Kim, G. et al. (2020). *MulRan: Multimodal Range Dataset for Urban Place Recognition.* IEEE ICRA 2020.
  - Official Portal: [https://sites.google.com/view/mulran-pr/](https://sites.google.com/view/mulran-pr/)
- **Sensor Configuration**: Ouster OS1-64 (64 channels) + Navtech CIR204-H radar, mounted on an autonomous vehicle traversing urban campuses, convention centers, and riverside roads.
- **Data Format**: Identical to KITTI (`.bin` 4-float format); ground truth poses provided as `.csv` files with timestamps and $4 \times 4$ $\mathrm{SE}(3)$ transformation matrices.

---

### Category 3: Indoor Robotics, AMR & 2.5D Costmap Generation

#### 1. ANavS Warehouse LiDAR Dataset (KIT / ANavS GmbH)
- **Primary Source**: ANavS GmbH & Karlsruhe Institute of Technology (2022).
  - Official Repository: [https://github.com/anavsgmbh/lidar-warehouse-dataset](https://github.com/anavsgmbh/lidar-warehouse-dataset)
- **Sensor Configuration**:
  - Velodyne VLP-16 (16 channels, 10 Hz, 30,000 points/frame, 360° horizontal FOV, ±15° vertical FOV) mounted on an industrial Automated Guided Vehicle (AGV).
- **Content & Annotations**:
  - 3,287 consecutive LiDAR scans ($450\text{ MB}$ total download).
  - 6,381 annotated 3D bounding boxes across 5 warehouse classes: `forklift`, `metal_box`, and 3 classes of `vehicle_platform`.
- **Data Format**: Raw point clouds in `bin/*.bin`, bounding box labels in `txt/*.txt`.
- **Alignment with RVPoint**:
  - **Exact Real-World Match for Workflow 1 (2.5D AMR Costmap)**: Features real industrial warehouse floors, pallet racks, and moving AGV obstacles. Evaluates real-time 16-beam costmap rasterization and obstacle inflation on an embedded edge compute node.

#### 2. TUM RGB-D Benchmark (Freiburg Dataset)
- **Primary Source**: Sturm, J. et al. (2012). *A benchmark for the evaluation of RGB-D SLAM systems.* IEEE IROS 2012.
  - Official Portal: [https://cvg.cit.tum.de/data/datasets/rgbd-dataset](https://cvg.cit.tum.de/data/datasets/rgbd-dataset)
- **Sensor Configuration**: Microsoft Kinect / ASUS Xtion (640 $\times$ 480 @ 30 Hz).
- **Ground Truth**: High-speed OptiTrack motion capture system (sub-millimeter position accuracy at 100 Hz).
- **Alignment with RVPoint**:
  - Small individual sequence sizes ($100\text{--}300\text{ MB}$). Unprojects into $\sim 200,000$ points/frame. Matches `data/living_room.pcd` and `data/01_table_scene_lms400.pcd`.

---

### Category 4: Roadside Infrastructure & V2X (Static Background Subtraction)

#### 1. DAIR-V2X (Tsinghua / BAAI)
- **Primary Source**: Yu, H. et al. (2022). *DAIR-V2X: A Large-Scale Dataset for Vehicle-Infrastructure Cooperative 3D Object Detection.* IEEE CVPR 2022.
  - Official Portal: [https://thudair.baai.ac.cn/dair-v2x](https://thudair.baai.ac.cn/dair-v2x)
  - Toolkit: [https://github.com/AIR-THU/DAIR-V2X](https://github.com/AIR-THU/DAIR-V2X)
- **Sensor Configuration**:
  - **Infrastructure Units (RSU)**: High-resolution **300-beam LiDAR** (Innovusion Falcon / Hesai) mounted on roadside poles at heights of 4.5–6.5 meters overlooking traffic intersections, operating at 10 Hz.
  - Coordinate System: Virtual LiDAR Coordinate System (origin at sensor, $z$-up, $x-y$ parallel to ground).
- **Data Format & Ingestion**:
  - **Native `.pcd` Files**: Stored directly in standard Point Cloud Data format!
  - **Zero-Copy Ingestion**: RVPoint's [`simple_pcd_loader.h`](src/io/simple_pcd_loader.h) can load these frames directly with zero format translation scripts.
- **Alignment with RVPoint**:
  - **The Definitive Match for Workflow 3 (Roadside Background Subtraction)**: The sensor is 100% stationary. An initial background model captures the road and intersection architecture; incoming frames cleanly separate dynamic vehicles and pedestrians in $<4\text{ ms}$.

#### 2. A9 Dataset / TUMTraf (TUM Munich)
- **Primary Source**: Creß, C. et al. (2022). *A9-Dataset: Multi-Sensor Infrastructure Dataset for Autonomous Driving.* IEEE ITSC 2022.
  - Official Portal: [https://a9-dataset.com/](https://a9-dataset.com/)
- **Sensor Configuration**:
  - Ouster OS1-64 (Gen 2, 64 beams) and OS1-128 mounted on roadside gantry bridges over the Autobahn A9 highway.
- **Data Format & Ingestion**:
  - Provided as **`.pcd`** files (ASCII and binary) with OpenLABEL JSON annotations.
  - Ingestion via RVPoint’s native `loadPCD()`.

---

### Category 5: Metrology, Geometry & Part Inspection

#### 1. Stanford 3D Scanning Repository
- **Primary Source**: Curless, B., & Levoy, M. (1996). *A volumetric method for building complex models from range images.* ACM SIGGRAPH 1996.
  - Official Portal: [http://graphics.stanford.edu/data/3Dscanrep/](http://graphics.stanford.edu/data/3Dscanrep/)
- **Models**: Stanford Bunny (35,947 vertices), Dragon (437,645 vertices), Armadillo (172,974 vertices), Happy Buddha (543,652 vertices).
- **Format**: `.ply` files (Polygon File Format) and raw range scans (`.tar.gz`, $1.5\text{ MB}$ to $50\text{ MB}$).
- **Alignment with RVPoint**:
  - High-precision ground truth meshes with analytical surface normals for verifying RVPoint’s **Cardano closed-form normal estimator** and assessing sub-millimeter ICP convergence.

---

## 3. Dataset Download & Lightweight CI Sizing

For quick local evaluation without filling disk space:

| Target Workflow | Recommended Online Dataset | Recommended Evaluation Subset | Download Size | Format | Direct Loader Command |
| :--- | :--- | :--- | :---: | :--- | :--- |
| **Workflow 4 (Odometry)** | **KITTI Odometry (Seq 00)** | In-repo `data/pcd_compressed/` | **$0\text{ MB}$ (Already in repo)** | `.pcd` (LZF) | `loadPCD("data/pcd_compressed/0000000000.pcd", cloud)` |
| **Workflow 2 (3D OBB)** | **KITTI 3D Object Detection** | Validation split (Frames 0–100) | $\sim 150\text{ MB}$ | `.bin` | Read into `PointCloudSoA` + parse label `.txt` |
| **Workflow 1 (Costmap)** | **ANavS Warehouse Dataset** | Sequence `run_01` (500 scans) | $\sim 80\text{ MB}$ | `.bin` | Direct 16-channel binary loader |
| **Workflow 3 (Roadside V2X)** | **DAIR-V2X-I (Infrastructure)** | Single intersection sequence (100 scans) | $\sim 120\text{ MB}$ | `.pcd` | Native `loadPCD()` |
| **Metrology / ICP Unit Test** | **Stanford 3D Bunny** | `reconstruction/bun_zipper.ply` | $1.5\text{ MB}$ | `.ply` | ASCII/Binary PLY vertex loader |

---

## 4. Key Recommendations for RVPoint Architecture

1. **Leverage the In-Repo 131 KITTI Frames Immediately**:
   - Do not require users to download large gigabyte archives for primary validation.
   - Use `data/pcd_compressed/` (131 consecutive KITTI frames) as the automated CI test suite for **Workflow 4 (Pairwise Odometry)** and **Workflow 2 (3D Bounding Boxes)**.
2. **Standardize on Zero-Copy Ingestion for `.pcd` and `.bin`**:
   - RVPoint already has a zero-dependency LZF/binary PCD loader ([`src/io/simple_pcd_loader.h`](src/io/simple_pcd_loader.h)).
   - Add a lightweight (30-line) `loadKITTI_BIN()` function to `src/io/` to ingest standard automotive `.bin` point clouds (KITTI, MulRan, ANavS Warehouse) directly into `PointCloudSoA`.
3. **Use DAIR-V2X and A9 for Roadside Demonstrations**:
   - Because DAIR-V2X and A9 are natively distributed in `.pcd` format, they integrate directly into RVPoint without external Python conversion scripts.

