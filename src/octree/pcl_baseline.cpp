#include "types.h"

#include <cstdlib>

#ifdef ENABLE_PCL
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

void radiusSearch_pcl(float* x, float* y, float* z, int N,
                      float* qx, float* qy, float* qz, int Q,
                      float r, float voxel_size,
                      struct Results* out)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->points.resize(static_cast<size_t>(N));
    for (int i = 0; i < N; ++i) {
        cloud->points[static_cast<size_t>(i)].x = x[static_cast<size_t>(i)];
        cloud->points[static_cast<size_t>(i)].y = y[static_cast<size_t>(i)];
        cloud->points[static_cast<size_t>(i)].z = z[static_cast<size_t>(i)];
    }
    cloud->width = static_cast<uint32_t>(N);
    cloud->height = 1U;

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(voxel_size);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    out->result_counts = static_cast<int*>(std::malloc(sizeof(int) * static_cast<size_t>(Q)));
    out->result_offsets = static_cast<int*>(std::malloc(sizeof(int) * static_cast<size_t>(Q + 1)));
    out->result_flat = static_cast<int*>(std::malloc(sizeof(int) * static_cast<size_t>(N) * static_cast<size_t>(Q)));

    int cursor = 0;
    std::vector<int> indices;
    std::vector<float> sqr_distances;
    for (int q = 0; q < Q; ++q) {
        pcl::PointXYZ query(qx[static_cast<size_t>(q)], qy[static_cast<size_t>(q)], qz[static_cast<size_t>(q)]);
        indices.clear();
        sqr_distances.clear();
        octree.radiusSearch(query, r, indices, sqr_distances);
        out->result_counts[static_cast<size_t>(q)] = static_cast<int>(indices.size());
        out->result_offsets[static_cast<size_t>(q)] = cursor;
        for (size_t i = 0; i < indices.size(); ++i) {
            out->result_flat[cursor++] = indices[i];
        }
    }

    out->result_offsets[Q] = cursor;
    out->total_results = cursor;

    if (cursor > 0) {
        out->result_flat = static_cast<int*>(std::realloc(out->result_flat,
                                                          sizeof(int) * static_cast<size_t>(cursor)));
    } else {
        std::free(out->result_flat);
        out->result_flat = nullptr;
    }
}

#else

void radiusSearch_pcl(float*, float*, float*, int,
                      float*, float*, float*, int,
                      float, float, struct Results* out)
{
    out->result_flat = nullptr;
    out->result_offsets = nullptr;
    out->result_counts = nullptr;
    out->total_results = 0;
}

#endif