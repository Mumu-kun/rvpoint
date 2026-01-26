#pragma once
#include <cstddef>
#include <vector>
#include <unordered_map>
#include <cmath>

#ifndef __riscv_vector
  #error "RISC-V Vector (RVV) support is mandatory. Compile with -march=rv64gcv"
#endif

#include <riscv_vector.h>

namespace rvv_pcl {

struct PointXYZ {
  float x, y, z;
};

// Structure of Arrays (SoA) layout - Critical for RVV performance
struct PointCloudSoA {
  float* x;
  float* y;
  float* z;
  std::size_t n;
};

class Octree; // Forward declaration

// Voxel Grid Downsampling
// Returns number of points in the output
std::size_t voxel_grid_downsamp_sc(const PointXYZ* in, std::size_t n,
                                   PointXYZ* out, float leaf_size);

std::size_t voxel_grid_downsamp_rvv(const PointCloudSoA& in,
                                    PointXYZ* out, float leaf_size);

// 2) Statistical Outlier Removal
std::size_t sor_sc(const PointXYZ* in, std::size_t n,
                   PointXYZ* out, int k, float alpha);

std::size_t sor_rvv(const PointCloudSoA& in,
                    PointXYZ* out, int k, float alpha);

// 3) Normal Estimation (output normals per point)
// Added ViewPoint support for consistent orientation
// Now requires a pre-built Octree for neighbor search
void normal_estimation_sc(const PointXYZ* in, std::size_t n,
                          float* nx, float* ny, float* nz, int k, float radius,
                          float vp_x = 0, float vp_y = 0, float vp_z = 0,
                          int eigen_iters = 4);

void normal_estimation_rvv(const PointCloudSoA& in,
                           const Octree& octree,
                           float* nx, float* ny, float* nz, int k, float radius,
                           float vp_x = 0, float vp_y = 0, float vp_z = 0,
                           int eigen_iters = 4);

// 4) Radius Search
std::size_t radius_search_sc(const PointXYZ* cloud, std::size_t n, 
                             PointXYZ query, float radius, 
                             int* indices, float* dists, int max_nn);

std::size_t radius_search_rvv(const PointCloudSoA& cloud, 
                              PointXYZ query, float radius, 
                              int* indices, float* dists, int max_nn);

// 5) RANSAC Plane Fitting
// Returns number of inliers. 'model' must be float[4] (a,b,c,d).
int ransac_plane_sc(const PointXYZ* cloud, std::size_t n, 
                    float dist_thresh, int max_iters, float* model,
                    float collinear_thresh = 1e-6f);

int ransac_plane_rvv(const PointCloudSoA& cloud, 
                     float dist_thresh, int max_iters, float* model,
                     float collinear_thresh = 1e-6f);

// ============================================================================
// Octree for Efficient Spatial Search
// ============================================================================
struct OctreeNode {
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    OctreeNode* children[8] = {nullptr};
    std::vector<int> indices; // Only for leaves
    bool is_leaf = true;

    ~OctreeNode(); 
};

class Octree {
public:
    Octree();
    ~Octree();

    void setInputCloud(const PointCloudSoA& cloud);
    void build();

    // Returns number of neighbors found
    std::size_t radiusSearch(const PointXYZ& query, float radius, 
                             std::vector<int>& indices, 
                             std::vector<float>& dists, int max_nn = 0) const;

    void setMaxPointsPerLeaf(int n) { max_points_per_leaf_ = n; }
    void setMaxDepth(int d) { max_depth_ = d; }
    void setBuildEpsilon(float eps) { build_epsilon_ = eps; }

private:
    PointCloudSoA cloud_;
    OctreeNode* root_ = nullptr;
    int max_points_per_leaf_ = 64; 
    int max_depth_ = 8;
    float build_epsilon_ = 1e-4f;

    void buildParams(OctreeNode* node, const std::vector<int>& indices, int depth);
    void recursiveSearch(OctreeNode* node, const PointXYZ& query, float radius_sq, 
                         std::vector<int>& indices, std::vector<float>& dists) const;
};

// Helper: Squared Euclidean Distance Kernel (RVV)
// Computes d2[i] = dist_sq( (x[i],y[i],z[i]), q )
void get_dist_sq_rvv(const float* x, const float* y, const float* z,
                     float qx, float qy, float qz,
                     float* out_d2, std::size_t n);

// Fused Gather-Filter Kernel
// Reads x,y,z at 'indices' (SoA), computes dist to q, and stores matching indices/dists
void get_inds_in_radius_rvv(const float* x, const float* y, const float* z,
                            const int* subset_indices, std::size_t n,
                            float qx, float qy, float qz, float r2,
                            std::vector<int>& out_indices, 
                            std::vector<float>& out_dists);

// ============================================================================
// Spatial Hash Grid for Fast Neighbor Search (Alternative to Octree)
// ============================================================================

class SpatialHash {
public:
    SpatialHash();
    ~SpatialHash();
    
    void setInputCloud(const PointCloudSoA& cloud, float cell_size);
    void build();
    
    // Returns number of neighbors found
    std::size_t radiusSearch(const PointXYZ& query, float radius,
                             std::vector<int>& indices,
                             std::vector<float>& dists, int max_nn = 0) const;

    void setHashPrimes(int64_t p1, int64_t p2) { p1_ = p1; p2_ = p2; }
    
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
    inline void getCellIndices(float x, float y, float z, int& ix, int& iy, int& iz) const {
        ix = (int)std::floor((x - min_x_) / cell_size_);
        iy = (int)std::floor((y - min_y_) / cell_size_);
        iz = (int)std::floor((z - min_z_) / cell_size_);
    }
};

} // namespace rvv_pcl
