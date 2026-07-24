#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvv_pcl {

struct PointXYZ {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

class PointCloudSoA;
using PointCloudSoAPtr = std::shared_ptr<PointCloudSoA>;

class PointCloudSoA {
public:
  void reserve(std::size_t capacity);
  void clear();
  void resize(std::size_t new_size);
  bool loadFromPCD(const std::string &filename);

  std::size_t size() const { return pointCount_; }
  bool empty() const { return pointCount_ == 0; }

  void push_back(const PointXYZ &point, float intensity = 0.0f);
  PointXYZ point(std::size_t index) const;
  void setPoint(std::size_t index, const PointXYZ &point);
  std::vector<PointXYZ> toAoS() const;
  void assign(const std::vector<PointXYZ> &points);

  const float *xData() const { return xCoords_.data(); }
  const float *yData() const { return yCoords_.data(); }
  const float *zData() const { return zCoords_.data(); }
  float *xData() { return xCoords_.data(); }
  float *yData() { return yCoords_.data(); }
  float *zData() { return zCoords_.data(); }

  const std::vector<float> &xCoords() const { return xCoords_; }
  const std::vector<float> &yCoords() const { return yCoords_; }
  const std::vector<float> &zCoords() const { return zCoords_; }
  const std::vector<float> &intensities() const { return intensities_; }
  std::vector<float> &xCoords() { return xCoords_; }
  std::vector<float> &yCoords() { return yCoords_; }
  std::vector<float> &zCoords() { return zCoords_; }
  std::vector<float> &intensities() { return intensities_; }

private:
  std::vector<float> xCoords_;
  std::vector<float> yCoords_;
  std::vector<float> zCoords_;
  std::vector<float> intensities_;
  std::size_t pointCount_ = 0;
};

class FeatureCloud {
public:
  virtual ~FeatureCloud() = default;
  virtual void resize(std::size_t n) = 0;
  std::size_t size() const { return pointCount_; }

protected:
  std::size_t pointCount_ = 0;
};

class NormalCloud : public FeatureCloud {
public:
  void resize(std::size_t n) override;
  PointXYZ normal(std::size_t index) const;
  void setNormal(std::size_t index, float nx, float ny, float nz);

  const std::vector<float> &nx() const { return nx_; }
  const std::vector<float> &ny() const { return ny_; }
  const std::vector<float> &nz() const { return nz_; }
  std::vector<float> &nx() { return nx_; }
  std::vector<float> &ny() { return ny_; }
  std::vector<float> &nz() { return nz_; }

private:
  std::vector<float> nx_;
  std::vector<float> ny_;
  std::vector<float> nz_;
};

class RVVHelper {
public:
  static void vadd(const float *a, const float *b, float *result, std::size_t n);
  static void vsub(const float *a, const float *b, float *result, std::size_t n);
  static void vmul(const float *a, const float *b, float *result, std::size_t n);
  static void vdiv(const float *a, const float *b, float *result, std::size_t n);
  static void vfmadd(const float *a, const float *b, const float *c, float *result,
                     std::size_t n);
  static void vsqrt(const float *a, float *result, std::size_t n);
  static void vrsqrt(const float *a, float *result, std::size_t n);
  static float vsum(const float *a, std::size_t n);
  static float vmax(const float *a, std::size_t n);
  static float vmin(const float *a, std::size_t n);
  static float vdot(const float *a, const float *b, std::size_t n);

  static void distanceSquared(const float *px, const float *py, const float *pz, std::size_t n,
                              float qx, float qy, float qz, float *out_d2);
  static void planeDistances(const float *px, const float *py, const float *pz, std::size_t n,
                             const std::array<float, 4> &coeffs, float *out_dist);
  static void gatherDistanceSquared(const float *px, const float *py, const float *pz,
                                    const int *subset_indices, std::size_t n,
                                    float qx, float qy, float qz, float *out_d2);
};

class Filter {
public:
  virtual ~Filter() = default;
  virtual void setInput(const PointCloudSoA &input);
  virtual void filter(PointCloudSoA &output) const = 0;

protected:
  const PointCloudSoA *input_ = nullptr;
};

class NeighborSearch {
public:
  virtual ~NeighborSearch() = default;
  virtual void setInputCloud(const PointCloudSoA &cloud);
  virtual void setSearchRadius(float radius);
  virtual void buildIndex() = 0;
  virtual std::size_t radiusSearch(int queryPointIndex, std::vector<int> &resultIndices,
                                   std::vector<float> *resultDistances = nullptr,
                                   int maxResults = 0) const = 0;
  virtual int nearestNeighborSearch(int queryPointIndex) const;

protected:
  PointXYZ queryPoint(int queryPointIndex) const;

  const PointCloudSoA *pointCloud_ = nullptr;
  float searchRadius_ = 0.0f;
};

class VoxelGridFilter : public Filter {
public:
  void setLeafSize(float size);
  float leafSize() const { return leafSize_; }
  void filter(PointCloudSoA &output) const override;

private:
  std::int64_t hashPointToVoxel(float x, float y, float z) const;
  PointXYZ computeVoxelAverage(const std::vector<int> &indices) const;

  float leafSize_ = 0.1f;
};

class OctreeNeighborSearch : public NeighborSearch {
public:
  OctreeNeighborSearch();
  ~OctreeNeighborSearch() override;

  void setMaxDepth(int depth);
  void setLeafCapacity(int capacity);
  void buildTree();
  void buildIndex() override;
  std::size_t radiusSearch(int queryPointIndex, std::vector<int> &resultIndices,
                           std::vector<float> *resultDistances = nullptr,
                           int maxResults = 0) const override;

private:
  struct Node {
    float min_x = 0.0f;
    float min_y = 0.0f;
    float min_z = 0.0f;
    float max_x = 0.0f;
    float max_y = 0.0f;
    float max_z = 0.0f;
    bool is_leaf = true;
    std::vector<int> indices;
    Node *children[8] = {nullptr, nullptr, nullptr, nullptr,
                         nullptr, nullptr, nullptr, nullptr};

    ~Node();
  };

  void insertPoint(Node *node, int pointIndex, int depth);
  void subdivide(Node *node, int depth);
  void recursiveSearch(Node *node, const PointXYZ &query, float radius_sq,
                       std::vector<int> &indices,
                       std::vector<float> *dists) const;
  static bool boxOverlapsSphere(const Node *node, const PointXYZ &center, float r2);

  Node *root_ = nullptr;
  int maxDepth_ = 8;
  int leafCapacity_ = 16;
  float buildEpsilon_ = 1e-4f;
};

class SpatialHashNeighborSearch : public NeighborSearch {
public:
  void setCellSize(float size);
  void buildHashTable();
  void buildIndex() override;
  std::size_t radiusSearch(int queryPointIndex, std::vector<int> &resultIndices,
                           std::vector<float> *resultDistances = nullptr,
                           int maxResults = 0) const override;

private:
  std::int64_t computeHashKey(int ix, int iy, int iz) const;
  void getCellIndices(float x, float y, float z, int &ix, int &iy, int &iz) const;
  void getCellNeighborPoints(std::int64_t cellKey, std::vector<int> &resultIndices) const;

  float cellSize_ = 0.1f;
  int tableSize_ = 0;
  float min_x_ = 0.0f;
  float min_y_ = 0.0f;
  float min_z_ = 0.0f;
  std::int64_t p1_ = 73856093LL;
  std::int64_t p2_ = 19349663LL;
  std::unordered_map<std::int64_t, std::vector<int>> grid_;
};

class SORFilter : public Filter {
public:
  void setMeanK(int k);
  void setStdThreshold(float threshold);
  void setNeighborSearch(NeighborSearch *search);
  void filter(PointCloudSoA &output) const override;

private:
  float computeMeanDistanceToNeighbors(int pointIndex,
                                       const std::vector<int> &neighborIndices) const;
  void computeGlobalStatistics(const std::vector<float> &meanDistances, float &meanDist,
                               float &stddev) const;

  int meanK_ = 20;
  float stdThreshold_ = 1.0f;
  NeighborSearch *searcher_ = nullptr;
};

class FeatureEstimator {
public:
  virtual ~FeatureEstimator() = default;
  virtual void setInputCloud(const PointCloudSoA &cloud);
  virtual void setK(int k);
  virtual void setNeighborSearch(NeighborSearch *search);
  virtual void estimate(FeatureCloud &featureCloud) const;

protected:
  virtual bool validateInputs() const;
  virtual void computeFeature(int index, const std::vector<int> &neighborIndices,
                              FeatureCloud &featureCloud) const = 0;

  const PointCloudSoA *input_ = nullptr;
  int k_ = 10;
  NeighborSearch *searcher_ = nullptr;
};

class NormalEstimation : public FeatureEstimator {
protected:
  void computeFeature(int index, const std::vector<int> &neighborIndices,
                      FeatureCloud &featureCloud) const override;
};

class ModelFitter {
public:
  virtual ~ModelFitter() = default;
  virtual void setInputCloud(const PointCloudSoA &cloud);
  virtual bool fit(std::array<float, 4> &coefficients, std::vector<int> &inlierIndices,
                   int maxInliers = 0) const = 0;

protected:
  const PointCloudSoA *input_ = nullptr;
};

class PlaneModel {
public:
  static int minSamples();
  static bool computeModel(const PointCloudSoA &points, const std::vector<int> &samples,
                           std::array<float, 4> &coeffs);
  static float evaluatePoint(const PointCloudSoA &points, int pointIdx,
                             const std::array<float, 4> &coeffs);
  static int evaluateAll(const PointCloudSoA &points, const std::array<float, 4> &coeffs,
                         std::vector<int> &inliers, int maxInliers, float threshold);
};

class RANSACFitter : public ModelFitter {
public:
  void setDistanceThreshold(float threshold);
  void setMaxIterations(int iter);
  void setProbability(float prob);
  bool fit(std::array<float, 4> &coefficients, std::vector<int> &inlierIndices,
           int maxInliers = 0) const override;

private:
  int evaluateModel(const std::array<float, 4> &coeffs, std::vector<int> &inliers,
                    int maxInliers) const;
  void sampleRandomIndices(std::vector<int> &sample) const;

  float distanceThreshold_ = 0.05f;
  int maxIterations_ = 1000;
  float probability_ = 0.99f;
};

} // namespace rvv_pcl
