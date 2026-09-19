# Upstream PCL 1.14 Dependency & Dynamic Library Analysis

## 1. Executive Summary

This document analyzes the official Debian/Ubuntu 24.04 (Noble) RISC-V 64-bit packages for Point Cloud Library (PCL 1.14.0) and determines the exact dependency closure required to execute upstream PCL perception pipelines.

---

## 2. PCL Module Dependency Matrix

| PCL Module | Primary Shared Object | Transitive Dependencies | Required for Perception Benchmarks? |
| :--- | :--- | :--- | :--- |
| **pcl_common** | `libpcl_common.so.1.14` | `libc`, `libm`, `libgcc_s`, `libstdc++` | **YES** (Core structures) |
| **pcl_kdtree** | `libpcl_kdtree.so.1.14` | `libflann_cpp.so`, `liblz4.so`, `libzstd.so` | **YES** (Spatial Search) |
| **pcl_search** | `libpcl_search.so.1.14` | `libpcl_kdtree.so`, `libpcl_octree.so`, `libflann_cpp.so` | **YES** (Radius / KNN Search) |
| **pcl_octree** | `libpcl_octree.so.1.14` | `libpcl_common.so` | **YES** (Spatial Indexing) |
| **pcl_filters** | `libpcl_filters.so.1.14` | `libpcl_common.so`, `libpcl_kdtree.so`, `libpcl_octree.so` | **YES** (VoxelGrid, SOR) |
| **pcl_features** | `libpcl_features.so.1.14` | `libpcl_common.so`, `libpcl_search.so`, `libpcl_kdtree.so`, `libpcl_octree.so` | **YES** (Normal Estimation) |
| **pcl_sample_consensus** | `libpcl_sample_consensus.so.1.14` | `libpcl_common.so` | **YES** (RANSAC Plane Fitting) |
| **pcl_segmentation** | `libpcl_segmentation.so.1.14` | `libpcl_common.so`, `libpcl_features.so`, `libpcl_search.so`, `libpcl_sample_consensus.so`, `libpcl_ml.so` | **YES** (Euclidean Clustering) |
| **pcl_ml** | `libpcl_ml.so.1.14` | `libpcl_common.so` | **YES** (Needed by Segmentation) |
| **pcl_io** | `libpcl_io.so.1.14` | **VTK 9.1** (`libvtkIOGeometry`, `libvtkCommonCore`), **LibUSB**, **OpenNI**, **PCAP**, **PNG** | **NO** (Only for file IO) |

---

## 3. Findings on `libpcl_io.so` vs Algorithm Modules

1. **All 5 Core Perception Algorithms** (`pcl::VoxelGrid`, `pcl::StatisticalOutlierRemoval`, `pcl::NormalEstimation`, `pcl::SACSegmentation`, `pcl::EuclideanClusterExtraction`) reside exclusively within:
   - `libpcl_filters.so`
   - `libpcl_features.so`
   - `libpcl_segmentation.so`
   - `libpcl_sample_consensus.so`
   - `libpcl_search.so`
   - `libpcl_kdtree.so`
   - `libpcl_octree.so`
   - `libpcl_common.so`
   - `libpcl_ml.so`
   - `libflann_cpp.so`
   - `libboost_filesystem.so`, `libboost_system.so`, `libboost_iostreams.so`
   - `liblz4.so`, `libzstd.so`, `libbz2.so`, `libz.so`

2. **The VTK Dependency is Contained Entirely in `libpcl_io.so`**:
   `libpcl_io.so` is Ubuntu's catch-all IO module containing support for VTK-based file readers (`vtkPLYReader`, `vtkOBJReader`), live Kinect/OpenNI drivers, and raw network capture (`pcap`).
   
3. **Decoupling IO**:
   By feeding points into standard `pcl::PointCloud<pcl::PointXYZ>` using RVPoint's lightweight PCD parser and writing PCD outputs via native stream serializers:
   - We execute **100% authentic PCL algorithmic code** (`pcl::VoxelGrid`, `pcl::KdTreeFLANN`, `pcl::NormalEstimation`, `pcl::SACSegmentation`, `pcl::EuclideanClusterExtraction`).
   - We eliminate the entire 300MB+ VTK runtime dependency graph.
   - We ensure pure algorithmic microarchitectural comparisons without extraneous VTK runtime bloat.
