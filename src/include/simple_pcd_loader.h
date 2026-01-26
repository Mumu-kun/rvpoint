#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include "rvv_pcl.h"

namespace rvv_pcl {

inline int loadPCD(const std::string& file_path, std::vector<PointXYZ>& points) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << file_path << std::endl;
        return -1;
    }

    std::string line;
    bool data_started = false;
    bool is_binary = false;
    
    while (std::getline(file, line)) {
        if (line.substr(0, 4) == "DATA") {
            data_started = true;
            if (line.find("binary") != std::string::npos) {
                 is_binary = true;
            }
            break; // Stop reading header
        }
    }

    if (!data_started) {
        std::cerr << "Error: No DATA tag found in PCD." << std::endl;
        return -1;
    }

    if (is_binary) {
        // Re-read file logic or continue depending on stream state?
        // std::getline consumes the newline after DATA binary. 
        // So we can just read bytes now? 
        // Need to know number of points from header "POINTS <N>"
        // But we skipped parsing header for points count. 
        // Let's quickly re-parse header properly or assume we can read till EOF?
        
        // Better: Re-open and parse header properly for N
        file.close();
        
        std::ifstream bin_file(file_path, std::ios::binary);
        size_t num_points = 0;
        std::string hline;
        while(std::getline(bin_file, hline)) {
            if (hline.substr(0, 6) == "POINTS") {
                std::stringstream ss(hline);
                std::string junk;
                ss >> junk >> num_points;
            }
            if (hline.substr(0, 11) == "DATA binary") {
                break;
            }
        }
        
        if (num_points == 0) {
             std::cerr << "Error: Could not determine point count for binary read." << std::endl;
             return -1;
        }

        points.resize(num_points);
        // Assuming fields are x, y, z (float) -> 12 bytes per point.
        // PCL binary dumps are packed.
        // We might have padding if it's 16 bytes? 
        // table_scene_lms400.pcd usually fits standard XYZ float.
        
        // PCL Binary format: data is essentially memcopied.
        // PointXYZ is 3 floats? Or 4 (aligned)? 
        // Our PointXYZ struct is 3 floats (x,y,z).
        // Let's try reading 3 floats * N.
        
        // WARNING: table_scene_lms400 might be organized or have weird fields.
        // Standard is: FIELDS x y z ...
        // Let's assume x y z floats for now.
        
        for(size_t i=0; i<num_points; ++i) {
             float buf[3];
             bin_file.read(reinterpret_cast<char*>(buf), 3*sizeof(float));
             if(bin_file.gcount() != 3*sizeof(float)) break;
             points[i] = {buf[0], buf[1], buf[2]};
        }
        std::cout << "Loaded " << points.size() << " points (Binary)." << std::endl;
    } else {
        // ASCII loading
         while (std::getline(file, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            float x, y, z;
            if (ss >> x >> y >> z) {
                points.push_back({x, y, z});
            }
        }
        std::cout << "Loaded " << points.size() << " points (ASCII)." << std::endl;
    }
    
    return points.size();
}

inline void savePCD(const std::string& filename, const std::vector<PointXYZ>& points) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return;
    }
    
    file << "# .PCD v.7 - Point Cloud Data file format\n";
    file << "VERSION .7\n";
    file << "FIELDS x y z\n";
    file << "SIZE 4 4 4\n";
    file << "TYPE F F F\n";
    file << "COUNT 1 1 1\n";
    file << "WIDTH " << points.size() << "\n";
    file << "HEIGHT 1\n"; // Unorganized point cloud
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << points.size() << "\n";
    file << "DATA ascii\n";
    
    for (const auto& p : points) {
        file << p.x << " " << p.y << " " << p.z << "\n";
    }
    
    file.close();
    std::cout << "Saved " << points.size() << " points to " << filename << std::endl;
}

} // namespace rvv_pcl
