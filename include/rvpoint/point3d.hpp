/**
 * @file point3d.hpp
 * @brief Simple 3D point structure for voxel downsampling
 */

#ifndef RVPOINT_POINT3D_HPP
#define RVPOINT_POINT3D_HPP

#include <cmath>

struct Point3D {
    float x, y, z;

    Point3D() : x(0.0f), y(0.0f), z(0.0f) {}
    Point3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    bool approx_equal(const Point3D& other, float epsilon = 1e-5f) const {
        return std::abs(x - other.x) < epsilon && std::abs(y - other.y) < epsilon &&
               std::abs(z - other.z) < epsilon;
    }
};

#endif // RVPOINT_POINT3D_HPP
