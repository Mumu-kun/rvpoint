#include "include/rvv_pcl.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

namespace rvv_pcl {

// ===========================================
// OctreeNode
// ===========================================
OctreeNode::~OctreeNode() {
    for (int i = 0; i < 8; ++i) {
        if (children[i]) delete children[i];
    }
}

// ===========================================
// Octree
// ===========================================
Octree::Octree() : root_(nullptr) {}

Octree::~Octree() {
    if (root_) delete root_;
}

void Octree::setInputCloud(const PointCloudSoA& cloud) {
    cloud_ = cloud;
    // We don't own the data pointers in SoA, we just hold a copy of the struct
}

void Octree::build() {
    if (cloud_.n == 0) return;
    if (root_) { delete root_; root_ = nullptr; }

    root_ = new OctreeNode();
    
    // Bounds
    root_->min_x = root_->min_y = root_->min_z = std::numeric_limits<float>::max();
    root_->max_x = root_->max_y = root_->max_z = std::numeric_limits<float>::lowest();

    std::vector<int> all_indices(cloud_.n);
    for(size_t i=0; i<cloud_.n; ++i) {
        all_indices[i] = i;
        if(cloud_.x[i] < root_->min_x) root_->min_x = cloud_.x[i];
        if(cloud_.x[i] > root_->max_x) root_->max_x = cloud_.x[i];
        if(cloud_.y[i] < root_->min_y) root_->min_y = cloud_.y[i];
        if(cloud_.y[i] > root_->max_y) root_->max_y = cloud_.y[i];
        if(cloud_.z[i] < root_->min_z) root_->min_z = cloud_.z[i];
        if(cloud_.z[i] > root_->max_z) root_->max_z = cloud_.z[i];
    }

    // Add small epsilon to max to ensure points are strictly inside [min, max) logic
    root_->max_x += build_epsilon_; root_->max_y += build_epsilon_; root_->max_z += build_epsilon_;

    buildParams(root_, all_indices, 0);
}

void Octree::buildParams(OctreeNode* node, const std::vector<int>& indices, int depth) {
    if (indices.size() <= (size_t)max_points_per_leaf_ || depth >= max_depth_) {
        node->is_leaf = true;
        node->indices = indices;
        return;
    }

    node->is_leaf = false;
    float mid_x = (node->min_x + node->max_x) * 0.5f;
    float mid_y = (node->min_y + node->max_y) * 0.5f;
    float mid_z = (node->min_z + node->max_z) * 0.5f;

    std::vector<int> child_indices[8];

    for (int idx : indices) {
        int code = 0;
        if (cloud_.x[idx] >= mid_x) code |= 1;
        if (cloud_.y[idx] >= mid_y) code |= 2;
        if (cloud_.z[idx] >= mid_z) code |= 4;
        child_indices[code].push_back(idx);
    }

    for (int i = 0; i < 8; ++i) {
        if (!child_indices[i].empty()) {
            node->children[i] = new OctreeNode();
            // Set Bounds
            node->children[i]->min_x = (i & 1) ? mid_x : node->min_x;
            node->children[i]->max_x = (i & 1) ? node->max_x : mid_x;
            node->children[i]->min_y = (i & 2) ? mid_y : node->min_y;
            node->children[i]->max_y = (i & 2) ? node->max_y : mid_y;
            node->children[i]->min_z = (i & 4) ? mid_z : node->min_z;
            node->children[i]->max_z = (i & 4) ? node->max_z : mid_z;

            buildParams(node->children[i], child_indices[i], depth + 1);
        }
    }
}

// Bounding Box Overlap Check
bool boxOverlapsSphere(const OctreeNode* node, const PointXYZ& center, float r2) {
    float dist_sq = 0;
    
    if (center.x < node->min_x) dist_sq += (center.x - node->min_x) * (center.x - node->min_x);
    else if (center.x > node->max_x) dist_sq += (center.x - node->max_x) * (center.x - node->max_x);

    if (center.y < node->min_y) dist_sq += (center.y - node->min_y) * (center.y - node->min_y);
    else if (center.y > node->max_y) dist_sq += (center.y - node->max_y) * (center.y - node->max_y);

    if (center.z < node->min_z) dist_sq += (center.z - node->min_z) * (center.z - node->min_z);
    else if (center.z > node->max_z) dist_sq += (center.z - node->max_z) * (center.z - node->max_z);

    return dist_sq <= r2;
}

std::size_t Octree::radiusSearch(const PointXYZ& query, float radius, 
                                 std::vector<int>& indices, 
                                 std::vector<float>& dists, int max_nn) const {
    indices.clear();
    dists.clear();
    if (!root_) return 0;
    recursiveSearch(root_, query, radius * radius, indices, dists);
    return indices.size();
}

void Octree::recursiveSearch(OctreeNode* node, const PointXYZ& query, float radius_sq, 
                             std::vector<int>& indices, std::vector<float>& dists) const {
    if (!boxOverlapsSphere(node, query, radius_sq)) return;

    if (node->is_leaf) {
        // Optimize: Pack data for RVV
        size_t n = node->indices.size();
        if (n == 0) return;

        std::vector<float> tmp_x(n), tmp_y(n), tmp_z(n);
        std::vector<float> tmp_dists(n);
        
        // Use Fused Kernel instead of manual gather/filter
        get_inds_in_radius_rvv(cloud_.x, cloud_.y, cloud_.z, 
                               node->indices.data(), n, 
                               query.x, query.y, query.z, radius_sq, 
                               indices, dists);
        
        // Old Scalar Logic (Replaced by above)
        /*
        for(size_t i=0; i<n; ++i) {
            int idx = node->indices[i];
            tmp_x[i] = cloud_.x[idx];
            tmp_y[i] = cloud_.y[idx];
            tmp_z[i] = cloud_.z[idx];
        }

        // Call Kernel
        get_dist_sq_rvv(tmp_x.data(), tmp_y.data(), tmp_z.data(), 
                        query.x, query.y, query.z, tmp_dists.data(), n);

        for(size_t i=0; i<n; ++i) {
            if (tmp_dists[i] <= radius_sq) {
                indices.push_back(node->indices[i]);
                dists.push_back(tmp_dists[i]);
            }
        }
        */
    } else {
        for (int i = 0; i < 8; ++i) {
            if (node->children[i]) {
                recursiveSearch(node->children[i], query, radius_sq, indices, dists);
            }
        }
    }
}

} // namespace rvv_pcl
