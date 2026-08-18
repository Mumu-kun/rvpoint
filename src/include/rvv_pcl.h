#pragma once
#include <cmath>
#include <cstddef>
#include <unordered_map>
#include <vector>

#ifndef __riscv_vector
#error "RISC-V Vector (RVV) support is mandatory. Compile with -march=rv64gcv"
#endif

#include <riscv_vector.h>

/**
 * @namespace rvv_pcl
 * @brief Main namespace for the RVPoint RISC-V Vector Optimized Point Cloud
 * Library.
 */
namespace rvv_pcl {

/**
 * @brief Standard 3D Point structure.
 *
 * Simple structure representing a point in 3D space with float coordinates.
 */
struct PointXYZ {
  float x, y, z;
};

/**
 * @brief Point Cloud stored in Structure of Arrays (SoA) layout.
 *
 * This layout is critical for RISC-V Vector (RVV) performance as it allows
 * for unit-stride loads/stores (vle32.v / vse32.v), which are significantly
 * faster than strided gather/scatter operations required for Array of
 * Structures (AoS).
 */
struct PointCloudSoA {
  float *x;      /**< Pointer to array of X coordinates */
  float *y;      /**< Pointer to array of Y coordinates */
  float *z;      /**< Pointer to array of Z coordinates */
  std::size_t n; /**< Number of points in the cloud */
};

class Octree;         // Forward declaration
class SpatialHash;    // Forward declaration
class PointerOctree;  // Forward declaration

// ============================================================================
// 1) Voxel Grid Downsampling
// ============================================================================

/**
 * @brief Voxel Grid Downsampling (Scalar Reference).
 *
 * Reduces the number of points by creating a 3D voxel grid over the input point
 * cloud. All points within each voxel are approximated by their centroid.
 *
 * @param in Pointer to input array of PointXYZ (AoS).
 * @param n Number of points in the input.
 * @param out Pointer to output array of PointXYZ (must be pre-allocated).
 * @param leaf_size Dimension of the voxel (leaf) along each axis.
 * @return std::size_t Number of points in the output cloud.
 */
std::size_t voxel_grid_downsamp_sc(const PointXYZ *in, std::size_t n,
                                   PointXYZ *out, float leaf_size);

/**
 * @brief Voxel Grid Downsampling (RVV Hybrid — DEPRECATED, use _rvv_v2).
 *
 * Vectorizes coordinate scaling but still uses std::map for grouping.
 * The map insertion bottleneck (O(n log n) scalar) dominates at large N.
 *
 * @deprecated Use voxel_grid_downsamp_rvv_v2 which eliminates std::map
 *             via a fully vectorized sort-based approach.
 */
[[deprecated("Use voxel_grid_downsamp_rvv_v2 — fully vectorized, no std::map")]]
std::size_t voxel_grid_downsamp_rvv(const PointCloudSoA &in, PointXYZ *out,
                                    float leaf_size);

/**
 * @brief Voxel Grid Downsampling (Fully Vectorized RVV, Sort-Based).
 *
 * Eliminates std::map by using a sort-based grouping approach:
 *  1. Vectorized bounding box computation (vfmin/vfmax + reductions)
 *  2. Vectorized voxel key computation (vfsub + vfmul + vfcvt_rtz + vmul/vadd)
 *  3. Sort indices by linear voxel key (groups same-voxel points contiguously)
 *  4. Vectorized centroid reduction per group (vluxei32 gather + vfredusum)
 *
 * @param in Input point cloud in SoA format.
 * @param out Pointer to output array of PointXYZ (must be pre-allocated).
 * @param leaf_size Dimension of the voxel (leaf) along each axis.
 * @return std::size_t Number of points in the output cloud.
 */
std::size_t voxel_grid_downsamp_rvv_v2(const PointCloudSoA &in, PointXYZ *out,
                                       float leaf_size);

// ============================================================================
// 2) Statistical Outlier Removal
// ============================================================================

/**
 * @brief Statistical Outlier Removal (Scalar Reference).
 *
 * Removes points that are further away from their neighbors compared to the
 * average.
 *
 * @param in Pointer to input array of PointXYZ (AoS).
 * @param n Number of points in the input.
 * @param out Pointer to output array of PointXYZ (AoS).
 * @param k Number of nearest neighbors to use for mean distance estimation.
 * @param alpha Standard deviation multiplier threshold.
 * @return std::size_t Number of inlier points remaining.
 */
std::size_t sor_sc(const PointXYZ *in, std::size_t n, PointXYZ *out, int k,
                   float alpha);

/**
 * @brief Statistical Outlier Removal (RVV Optimized).
 *
 * Vectorized implementation of SOR filter.
 * Accelerates K-NN distance calculation using `get_dist_sq_rvv` kernel and
 * uses vector reductions for mean and variance computation.
 *
 * @param in Input point cloud in SoA format.
 * @param out Pointer to output array of PointXYZ (AoS).
 * @param k Number of nearest neighbors to use.
 * @param alpha Standard deviation multiplier threshold.
 * @return std::size_t Number of inlier points remaining.
 */
std::size_t sor_rvv(const PointCloudSoA &in, PointXYZ *out, int k, float alpha);

/**
 * @brief Index-Accelerated SOR using Octree (O(N log N)).
 */
std::size_t sor_octree(const PointCloudSoA &in, const Octree &tree, PointXYZ *out,
                       int k, float alpha, float search_radius = 0.5f);

/**
 * @brief Index-Accelerated SOR using SpatialHash (O(N)).
 */
std::size_t sor_spatial_hash(const PointCloudSoA &in, const SpatialHash &hash, PointXYZ *out,
                             int k, float alpha, float search_radius = 0.5f);

/**
 * @brief Index-Accelerated SOR using PointerOctree (O(N log N)).
 */
std::size_t sor_pointer_octree(const PointCloudSoA &in, const PointerOctree &tree, PointXYZ *out,
                               int k, float alpha, float search_radius = 0.5f);

// ============================================================================
// 3) Normal Estimation
// ============================================================================

/**
 * @brief Normal Estimation (Scalar Reference).
 *
 * Estimates surface normals for each point by analyzing the covariance matrix
 * of its nearest neighbors. Supports consistent viewpoint usage.
 *
 * @param in Pointer to input array of PointXYZ.
 * @param n Number of points.
 * @param nx Output array for Normal X component.
 * @param ny Output array for Normal Y component.
 * @param nz Output array for Normal Z component.
 * @param k Number of neighbors to use for estimation.
 * @param radius Radius for neighbor search (if k is not used).
 * @param vp_x Viewpoint X coordinate (default 0).
 * @param vp_y Viewpoint Y coordinate (default 0).
 * @param vp_z Viewpoint Z coordinate (default 0).
 * @param eigen_iters Number of iterations for eigenvalue solver (default 4).
 */
void normal_estimation_sc(const PointXYZ *in, std::size_t n, float *nx,
                          float *ny, float *nz, int k, float radius,
                          float vp_x = 0, float vp_y = 0, float vp_z = 0,
                          int eigen_iters = 4);

/**
 * @brief Normal Estimation (RVV Optimized).
 *
 * Vectorized implementation of Normal Estimation.
 * Uses an Octree for efficient neighbor search and vectorized kernels for
 * Covariance Matrix accumulation.
 *
 * @param in Input point cloud in SoA format.
 * @param octree Pre-built Octree structure for the input cloud.
 * @param nx Output array for Normal X component.
 * @param ny Output array for Normal Y component.
 * @param nz Output array for Normal Z component.
 * @param k Number of neighbors to use.
 * @param radius Search radius.
 * @param vp_x Viewpoint X coordinate.
 * @param vp_y Viewpoint Y coordinate.
 * @param vp_z Viewpoint Z coordinate.
 * @param eigen_iters Number of iterations for eigenvalue solver.
 */
void normal_estimation_rvv(const PointCloudSoA &in, const Octree &octree,
                           float *nx, float *ny, float *nz, int k, float radius,
                           float vp_x = 0, float vp_y = 0, float vp_z = 0,
                           int eigen_iters = 4);

/**
 * @brief Normal Estimation (RVV + SpatialHash, pre-built hash passed in).
 *
 * Uses SpatialHash for neighbor search instead of Octree.
 * SpatialHash provides O(1) average-case query vs Octree's O(log n),
 * but requires knowing the search radius at build time (set cell_size =
 * radius).
 *
 * @param in   Input point cloud (SoA).
 * @param hash Pre-built SpatialHash (call setInputCloud(soa, radius) +
 * build()).
 */
void normal_estimation_rvv(const PointCloudSoA &in, const SpatialHash &hash,
                           float *nx, float *ny, float *nz, int k, float radius,
                           float vp_x = 0, float vp_y = 0, float vp_z = 0,
                           int eigen_iters = 4);

/**
 * @brief Normal Estimation (RVV + PointerOctree, pre-built tree passed in).
 */
void normal_estimation_rvv(const PointCloudSoA &in, const PointerOctree &octree,
                           float *nx, float *ny, float *nz, int k, float radius,
                           float vp_x = 0, float vp_y = 0, float vp_z = 0,
                           int eigen_iters = 4);

/**
 * @brief Normal Estimation (RVV, self-contained — builds Octree internally).
 *
 * Convenience overload that does not require a pre-built Octree.
 * The Octree build cost is included inside this call.
 * Use the Octree-overload above when you need to measure build cost separately.
 */
void normal_estimation_rvv(const PointCloudSoA &in, float *nx, float *ny,
                           float *nz, int k, float radius, float vp_x = 0,
                           float vp_y = 0, float vp_z = 0, int eigen_iters = 4);

// ============================================================================
// 4) Radius Search
// ============================================================================

/**
 * @brief Radius Search (Scalar Reference).
 *
 * Finds all points within a specified radius of a query point.
 *
 * @param cloud Pointer to input cloud (AoS).
 * @param n Number of points.
 * @param query The query point coordinates.
 * @param radius Search radius.
 * @param indices Output array to store indices of neighbors.
 * @param dists Output array to store squared distances to neighbors.
 * @param max_nn Maximum number of neighbors to return.
 * @return std::size_t Number of neighbors found.
 */
std::size_t radius_search_sc(const PointXYZ *cloud, std::size_t n,
                             PointXYZ query, float radius, int *indices,
                             float *dists, int max_nn);

/**
 * @brief Radius Search (RVV Optimized).
 *
 * Vectorized Global Radius Search.
 * Scans the entire cloud using vector instructions to compute distances
 * and filter points making use of `vmfle` (vector float less-equal) and `vcpop`
 * masks.
 *
 * @param cloud Input cloud in SoA format.
 * @param query The query point coordinates.
 * @param radius Search radius.
 * @param indices Output array for neighbor indices.
 * @param dists Output array for neighbor squared distances.
 * @param max_nn Maximum number of neighbors to return.
 * @return std::size_t Number of neighbors found.
 */
std::size_t radius_search_rvv(const PointCloudSoA &cloud, PointXYZ query,
                              float radius, int *indices, float *dists,
                              int max_nn);

// ============================================================================
// 5) RANSAC Plane Fitting
// ============================================================================

/**
 * @brief RANSAC Plane Fitting (Scalar Reference).
 *
 * Fits a plane model ax + by + cz + d = 0 to the point cloud.
 *
 * @param cloud Pointer to input cloud (AoS).
 * @param n Number of points.
 * @param dist_thresh Distance threshold to consider a point an inlier.
 * @param max_iters Maximum number of RANSAC iterations.
 * @param model Output array of size 4 to store plane coefficients [a, b, c, d].
 * @param collinear_thresh Threshold to check if points are collinear
 * (degenerate case).
 * @return int Number of inliers found for the best model.
 */
int ransac_plane_sc(const PointXYZ *cloud, std::size_t n, float dist_thresh,
                    int max_iters, float *model,
                    float collinear_thresh = 1e-6f,
                    float probability = 0.99f);

/**
 * @brief RANSAC Plane Fitting (RVV Optimized).
 *
 * Vectorized RANSAC implementation.
 * Calculates distances for a batch of points against the plane model in
 * parallel and counts inliers using high-throughput vector population count
 * instructions.
 *
 * @param cloud Input cloud in SoA format.
 * @param dist_thresh Distance threshold for inliers.
 * @param max_iters Maximum number of RANSAC iterations.
 * @param model Output array of size 4 for plane coefficients.
 * @param collinear_thresh Threshold for collinearity check.
 * @param probability Confidence probability for adaptive early stopping (default 0.99).
 * @return int Number of inliers found for the best model.
 */
int ransac_plane_rvv(const PointCloudSoA &cloud, float dist_thresh,
                     int max_iters, float *model,
                     float collinear_thresh = 1e-6f,
                     float probability = 0.99f);

/**
 * @brief Extract Plane Inliers (RVV Optimized).
 *
 * Extracts points that lie within a distance threshold of the plane model.
 *
 * @param cloud Input cloud in SoA format.
 * @param model Plane coefficients [a, b, c, d] where ax + by + cz + d = 0.
 * @param dist_thresh Distance threshold to consider a point an inlier.
 * @param inliers Output array for inlier points (must be pre-allocated to
 * cloud.n).
 * @return std::size_t Number of inlier points extracted.
 */
std::size_t extract_plane_inliers_rvv(const PointCloudSoA &cloud,
                                      const float *model, float dist_thresh,
                                      PointXYZ *inliers);

/**
 * @brief Extract Plane Outliers (RVV Optimized).
 *
 * Extracts points that lie beyond a distance threshold of the plane model.
 *
 * @param cloud Input cloud in SoA format.
 * @param model Plane coefficients [a, b, c, d].
 * @param dist_thresh Distance threshold (points > thresh are outliers).
 * @param outliers Output array for outlier points (must be pre-allocated to
 * cloud.n).
 * @return std::size_t Number of outlier points extracted.
 */
std::size_t extract_plane_outliers_rvv(const PointCloudSoA &cloud,
                                       const float *model, float dist_thresh,
                                       PointXYZ *outliers);

/**
 * @brief Extract Both Inliers and Outliers in Single Pass (RVV Optimized).
 *
 * More efficient than calling extract_plane_inliers and extract_plane_outliers
 * separately as it only computes distances once.
 *
 * @param cloud Input cloud in SoA format.
 * @param model Plane coefficients [a, b, c, d].
 * @param dist_thresh Distance threshold.
 * @param inliers Output array for inliers (must be pre-allocated to cloud.n).
 * @param outliers Output array for outliers (must be pre-allocated to cloud.n).
 * @param n_inliers Output: number of inliers.
 * @param n_outliers Output: number of outliers.
 */
void extract_plane_inliers_outliers_rvv(const PointCloudSoA &cloud,
                                        const float *model, float dist_thresh,
                                        PointXYZ *inliers, PointXYZ *outliers,
                                        std::size_t &n_inliers,
                                        std::size_t &n_outliers);

// ============================================================================
// Octree for Efficient Spatial Search
// ============================================================================

/**
 * @brief Node structure for the Octree.
 */
struct OctreeNode {
  float min_x, min_y, min_z;           /**< Minimum bounding box coordinates */
  float max_x, max_y, max_z;           /**< Maximum bounding box coordinates */
  OctreeNode *children[8] = {nullptr}; /**< Pointers to 8 child nodes */
  std::vector<int>
      indices;         /**< Point indices contained in this node (if leaf) */
  bool is_leaf = true; /**< True if this is a leaf node */

  ~OctreeNode();
};

/**
 * @brief Octree Data Structure for Spatial Indexing.
 *
 * Provides efficient spatial partitioning to accelerate neighbor search
 * operations.
 */
class Octree {
public:
  Octree();
  ~Octree();

  /**
   * @brief Set the Input Cloud for the Octree.
   * @param cloud The point cloud (SoA) to index.
   */
  void setInputCloud(const PointCloudSoA &cloud);

  /**
   * @brief Build the Octree structure.
   * Recursively subdivides the bounding box until termination criteria are met.
   */
  void build();

  /**
   * @brief Perform a Radius Search on the Octree.
   *
   * @param query Query point.
   * @param radius Search radius.
   * @param indices Output vector for found point indices.
   * @param dists Output vector for found squared distances.
   * @param max_nn Maximum number of neighbors (0 for unlimited).
   * @return std::size_t Number of neighbors found.
   */
  std::size_t radiusSearch(const PointXYZ &query, float radius,
                           std::vector<int> &indices, std::vector<float> &dists,
                           int max_nn = 0) const;

  /** @brief Set max points allowed per leaf node before subdivision. */
  void setMaxPointsPerLeaf(int n) { max_points_per_leaf_ = n; }
  /** @brief Set maximum depth of the tree. */
  void setMaxDepth(int d) { max_depth_ = d; }
  /** @brief Set epsilon value to prevent zero-size nodes. */
  void setBuildEpsilon(float eps) { build_epsilon_ = eps; }

private:
  PointCloudSoA cloud_;
  OctreeNode *root_ = nullptr;
  int max_points_per_leaf_ = 64;
  int max_depth_ = 8;
  float build_epsilon_ = 1e-4f;

  void buildParams(OctreeNode *node, const std::vector<int> &indices,
                   int depth);
  void recursiveSearch(OctreeNode *node, const PointXYZ &query, float radius_sq,
                       std::vector<int> &indices,
                       std::vector<float> &dists) const;
};

// ============================================================================
// Helper Kernels
// ============================================================================

/**
 * @brief Helper: Squared Euclidean Distance Kernel (RVV).
 *
 * Computes d^2 = (x-qx)^2 + (y-qy)^2 + (z-qz)^2 for 'n' points using vector
 * instructions.
 *
 * @param x Pointer to X coordinates array.
 * @param y Pointer to Y coordinates array.
 * @param z Pointer to Z coordinates array.
 * @param qx Query point X.
 * @param qy Query point Y.
 * @param qz Query point Z.
 * @param out_d2 Output array for squared distances.
 * @param n Number of elements to process.
 */
void get_dist_sq_rvv(const float *x, const float *y, const float *z, float qx,
                     float qy, float qz, float *out_d2, std::size_t n);

/**
 * @brief Fused Gather-Filter Kernel (RVV).
 *
 * Reads x,y,z at specified 'indices' (indirect addressing), computes distance
 * to query, and seamlessly stores matching indices/distances.
 *
 * @param x Pointer to global X coordinates.
 * @param y Pointer to global Y coordinates.
 * @param z Pointer to global Z coordinates.
 * @param subset_indices Array of indices to check (e.g., from an Octree/Grid
 * cell).
 * @param n Number of indices to check.
 * @param qx Query point X.
 * @param qy Query point Y.
 * @param qz Query point Z.
 * @param r2 Squared radius threshold.
 * @param out_indices Vector to append matching indices to.
 * @param out_dists Vector to append matching distances to.
 */
void get_inds_in_radius_rvv(const float *x, const float *y, const float *z,
                            const int *subset_indices, std::size_t n, float qx,
                            float qy, float qz, float r2,
                            std::vector<int> &out_indices,
                            std::vector<float> &out_dists);

// ============================================================================
// Spatial Hash Grid for Fast Neighbor Search (Alternative to Octree)
// ============================================================================

/**
 * @brief Spatial Hash Grid for Fast Neighbor Search.
 *
 * Alternative to Octree. Values are hashed into a grid for O(1) average lookup
 * time. Particularly effective for fixed-radius searches.
 */
class SpatialHash {
public:
  SpatialHash();
  ~SpatialHash();

  /**
   * @brief Set the Input Cloud and Cell Size.
   * @param cloud Input cloud (SoA).
   * @param cell_size Size of the hash grid cell. Expected to be roughly equal
   * to search radius.
   */
  void setInputCloud(const PointCloudSoA &cloud, float cell_size);

  /** @brief Build the Hash Grid. */
  void build();

  /**
   * @brief Perform Radius Search using Spatial Hashing.
   *
   * @param query Query point.
   * @param radius Search radius.
   * @param indices Output vector for found indices.
   * @param dists Output vector for found squared distances.
   * @param max_nn Maximum neighbors to return.
   * @return std::size_t Number of neighbors found.
   */
  std::size_t radiusSearch(const PointXYZ &query, float radius,
                           std::vector<int> &indices, std::vector<float> &dists,
                           int max_nn = 0) const;

  /** @brief Set custom hash primes (for tuning). */
  void setHashPrimes(int64_t p1, int64_t p2) {
    p1_ = p1;
    p2_ = p2;
  }

private:
  PointCloudSoA cloud_;
  float cell_size_;
  float eps_scale_ = 0.01f;
  float reserve_factor_ = 0.25f;
  int64_t p1_ = 73856093LL;
  int64_t p2_ = 19349663LL;

  // Hash table: key = cell hash, value = point indices in that cell
  std::unordered_map<int64_t, std::vector<int>> grid_;

  // Bounding box
  float min_x_, min_y_, min_z_;
  float max_x_, max_y_, max_z_;
  int grid_size_x_, grid_size_y_, grid_size_z_;

  // Hash function: (ix, iy, iz) -> unique key
  inline int64_t hashCell(int ix, int iy, int iz) const {
    return (int64_t)ix + (int64_t)iy * p1_ + (int64_t)iz * p2_;
  }

  // Get cell indices for a point
  inline void getCellIndices(float x, float y, float z, int &ix, int &iy,
                             int &iz) const {
    ix = (int)std::floor((x - min_x_) / cell_size_);
    iy = (int)std::floor((y - min_y_) / cell_size_);
    iz = (int)std::floor((z - min_z_) / cell_size_);
  }
};

} // namespace rvv_pcl
