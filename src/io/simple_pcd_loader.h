#pragma once
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "core/point_types.h"

namespace rvpoint {

namespace detail {

struct PCDHeader {
    std::vector<std::string> fields;
    std::vector<int> sizes;
    std::vector<int> counts;
    std::vector<char> types;
    std::size_t points = 0;
    std::string data_type;
};

inline bool decompressLZF(const std::vector<unsigned char>& input,
                          std::vector<unsigned char>& output) {
    std::size_t ip = 0;
    std::size_t op = 0;

    while (ip < input.size()) {
        unsigned int ctrl = input[ip++];
        if (ctrl < (1u << 5)) {
            ctrl += 1;
            if (op + ctrl > output.size() || ip + ctrl > input.size()) {
                return false;
            }
            std::memcpy(output.data() + op, input.data() + ip, ctrl);
            op += ctrl;
            ip += ctrl;
        } else {
            unsigned int len = ctrl >> 5;
            if (ip >= input.size()) {
                return false;
            }
            std::size_t ref = op - ((ctrl & 0x1f) << 8) - 1;
            if (len == 7) {
                len += input[ip++];
                if (ip >= input.size()) {
                    return false;
                }
            }
            ref -= input[ip++];
            len += 2;

            if (op + len > output.size()) {
                return false;
            }
            if (ref >= op) {
                return false;
            }

            for (std::size_t k = 0; k < len; ++k) {
                output[op + k] = output[ref + k];
            }
            op += len;
        }
    }
    return op == output.size();
}

inline bool parsePCDHeader(std::ifstream& file, PCDHeader& header, std::size_t& data_start_pos) {
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);
        std::string tag;
        iss >> tag;

        if (tag == "FIELDS" || tag == "fields") {
            std::string field;
            while (iss >> field) header.fields.push_back(field);
        } else if (tag == "SIZE" || tag == "size") {
            int sz;
            while (iss >> sz) header.sizes.push_back(sz);
        } else if (tag == "TYPE" || tag == "type") {
            char t;
            while (iss >> t) header.types.push_back(t);
        } else if (tag == "COUNT" || tag == "count") {
            int c;
            while (iss >> c) header.counts.push_back(c);
        } else if (tag == "POINTS" || tag == "points") {
            iss >> header.points;
        } else if (tag == "DATA" || tag == "data") {
            iss >> header.data_type;
            data_start_pos = file.tellg();
            return true;
        }
    }
    return false;
}

} // namespace detail

inline bool loadPCD(const std::string& filename, PointCloudSoA& cloud) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return false;
    }

    detail::PCDHeader header;
    std::size_t data_pos = 0;
    if (!detail::parsePCDHeader(file, header, data_pos)) {
        std::cerr << "Error: Invalid PCD header in " << filename << std::endl;
        return false;
    }

    int x_idx = -1, y_idx = -1, z_idx = -1, i_idx = -1;
    int current_offset = 0;
    std::vector<int> offsets(header.fields.size(), 0);

    for (std::size_t i = 0; i < header.fields.size(); ++i) {
        offsets[i] = current_offset;
        int count = (i < header.counts.size()) ? header.counts[i] : 1;
        int size = (i < header.sizes.size()) ? header.sizes[i] : 4;
        current_offset += size * count;

        if (header.fields[i] == "x") x_idx = i;
        else if (header.fields[i] == "y") y_idx = i;
        else if (header.fields[i] == "z") z_idx = i;
        else if (header.fields[i] == "intensity" || header.fields[i] == "i") i_idx = i;
    }
    int point_step = current_offset;

    if (x_idx == -1 || y_idx == -1 || z_idx == -1) {
        std::cerr << "Error: PCD file must contain x, y, and z fields." << std::endl;
        return false;
    }

    cloud.n = header.points;
    file.seekg(data_pos);

    if (header.data_type == "binary_compressed") {
        std::uint32_t compressed_size = 0;
        std::uint32_t uncompressed_size = 0;
        file.read(reinterpret_cast<char*>(&compressed_size), 4);
        file.read(reinterpret_cast<char*>(&uncompressed_size), 4);

        std::vector<unsigned char> compressed_data(compressed_size);
        file.read(reinterpret_cast<char*>(compressed_data.data()), compressed_size);

        std::vector<unsigned char> uncompressed_data(uncompressed_size);
        if (!detail::decompressLZF(compressed_data, uncompressed_data)) {
            std::cerr << "Error: Failed to decompress LZF data." << std::endl;
            return false;
        }

        std::size_t field_offset_in_buf = 0;
        for (std::size_t f = 0; f < header.fields.size(); ++f) {
            int count = (f < header.counts.size()) ? header.counts[f] : 1;
            int size = (f < header.sizes.size()) ? header.sizes[f] : 4;
            std::size_t field_total_bytes = size * count * header.points;

            if (static_cast<int>(f) == x_idx && cloud.x) {
                std::memcpy(cloud.x, uncompressed_data.data() + field_offset_in_buf, header.points * sizeof(float));
            } else if (static_cast<int>(f) == y_idx && cloud.y) {
                std::memcpy(cloud.y, uncompressed_data.data() + field_offset_in_buf, header.points * sizeof(float));
            } else if (static_cast<int>(f) == z_idx && cloud.z) {
                std::memcpy(cloud.z, uncompressed_data.data() + field_offset_in_buf, header.points * sizeof(float));
            }
            field_offset_in_buf += field_total_bytes;
        }
        std::cout << "Loaded " << header.points << " points (Binary Compressed)." << std::endl;
        return true;
    } else if (header.data_type == "binary") {
        std::vector<char> point_buf(point_step);
        for (std::size_t i = 0; i < header.points; ++i) {
            file.read(point_buf.data(), point_step);
            if (cloud.x) cloud.x[i] = *reinterpret_cast<float*>(point_buf.data() + offsets[x_idx]);
            if (cloud.y) cloud.y[i] = *reinterpret_cast<float*>(point_buf.data() + offsets[y_idx]);
            if (cloud.z) cloud.z[i] = *reinterpret_cast<float*>(point_buf.data() + offsets[z_idx]);
        }
        std::cout << "Loaded " << header.points << " points (Binary)." << std::endl;
        return true;
    } else if (header.data_type == "ascii") {
        std::string line;
        std::size_t i = 0;
        while (i < header.points && std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::istringstream iss(line);
            std::vector<std::string> tokens;
            std::string token;
            while (iss >> token) tokens.push_back(token);

            if (tokens.size() >= header.fields.size()) {
                if (cloud.x) cloud.x[i] = std::stof(tokens[x_idx]);
                if (cloud.y) cloud.y[i] = std::stof(tokens[y_idx]);
                if (cloud.z) cloud.z[i] = std::stof(tokens[z_idx]);
                i++;
            }
        }
        cloud.n = i;
        std::cout << "Loaded " << cloud.n << " points (ASCII)." << std::endl;
        return true;
    }

    return false;
}

inline bool loadPCD(const std::string& filename, std::vector<PointXYZ>& points) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) return false;

    detail::PCDHeader header;
    std::size_t data_pos = 0;
    if (!detail::parsePCDHeader(file, header, data_pos)) return false;

    int x_idx = -1, y_idx = -1, z_idx = -1;
    int current_offset = 0;
    std::vector<int> offsets(header.fields.size(), 0);

    for (std::size_t i = 0; i < header.fields.size(); ++i) {
        offsets[i] = current_offset;
        int count = (i < header.counts.size()) ? header.counts[i] : 1;
        int size = (i < header.sizes.size()) ? header.sizes[i] : 4;
        current_offset += size * count;

        if (header.fields[i] == "x") x_idx = i;
        else if (header.fields[i] == "y") y_idx = i;
        else if (header.fields[i] == "z") z_idx = i;
    }
    int point_step = current_offset;
    if (x_idx == -1 || y_idx == -1 || z_idx == -1) return false;

    points.resize(header.points);
    file.seekg(data_pos);

    if (header.data_type == "binary_compressed") {
        std::uint32_t compressed_size = 0, uncompressed_size = 0;
        file.read(reinterpret_cast<char*>(&compressed_size), 4);
        file.read(reinterpret_cast<char*>(&uncompressed_size), 4);

        std::vector<unsigned char> compressed_data(compressed_size);
        file.read(reinterpret_cast<char*>(compressed_data.data()), compressed_size);

        std::vector<unsigned char> uncompressed_data(uncompressed_size);
        if (!detail::decompressLZF(compressed_data, uncompressed_data)) return false;

        std::vector<float> tmp_x(header.points), tmp_y(header.points), tmp_z(header.points);
        std::size_t field_offset = 0;
        for (std::size_t f = 0; f < header.fields.size(); ++f) {
            int count = (f < header.counts.size()) ? header.counts[f] : 1;
            int size = (f < header.sizes.size()) ? header.sizes[f] : 4;
            std::size_t total_bytes = size * count * header.points;
            if (static_cast<int>(f) == x_idx) {
                std::memcpy(tmp_x.data(), uncompressed_data.data() + field_offset, header.points * sizeof(float));
            } else if (static_cast<int>(f) == y_idx) {
                std::memcpy(tmp_y.data(), uncompressed_data.data() + field_offset, header.points * sizeof(float));
            } else if (static_cast<int>(f) == z_idx) {
                std::memcpy(tmp_z.data(), uncompressed_data.data() + field_offset, header.points * sizeof(float));
            }
            field_offset += total_bytes;
        }
        for (size_t i = 0; i < header.points; ++i) {
            points[i] = {tmp_x[i], tmp_y[i], tmp_z[i]};
        }
        return true;
    } else if (header.data_type == "binary") {
        std::vector<char> point_buf(point_step);
        for (std::size_t i = 0; i < header.points; ++i) {
            file.read(point_buf.data(), point_step);
            points[i].x = *reinterpret_cast<float*>(point_buf.data() + offsets[x_idx]);
            points[i].y = *reinterpret_cast<float*>(point_buf.data() + offsets[y_idx]);
            points[i].z = *reinterpret_cast<float*>(point_buf.data() + offsets[z_idx]);
        }
        return true;
    } else if (header.data_type == "ascii") {
        std::string line;
        std::size_t i = 0;
        while (i < header.points && std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::istringstream iss(line);
            std::vector<std::string> tokens;
            std::string token;
            while (iss >> token) tokens.push_back(token);
            if (tokens.size() >= header.fields.size()) {
                points[i].x = std::stof(tokens[x_idx]);
                points[i].y = std::stof(tokens[y_idx]);
                points[i].z = std::stof(tokens[z_idx]);
                i++;
            }
        }
        points.resize(i);
        return true;
    }
    return false;
}

inline bool savePCD(const std::string& filename, const PointCloudSoA& cloud, bool binary = true) {
    std::ofstream file(filename, binary ? (std::ios::binary | std::ios::out) : std::ios::out);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return false;
    }

    file << "# .PCD v.7 - Point Cloud Data file format\n";
    file << "VERSION .7\n";
    file << "FIELDS x y z\n";
    file << "SIZE 4 4 4\n";
    file << "TYPE F F F\n";
    file << "COUNT 1 1 1\n";
    file << "WIDTH " << cloud.n << "\n";
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << cloud.n << "\n";
    file << "DATA " << (binary ? "binary" : "ascii") << "\n";

    if (binary) {
        for (std::size_t i = 0; i < cloud.n; ++i) {
            file.write(reinterpret_cast<const char*>(&cloud.x[i]), sizeof(float));
            file.write(reinterpret_cast<const char*>(&cloud.y[i]), sizeof(float));
            file.write(reinterpret_cast<const char*>(&cloud.z[i]), sizeof(float));
        }
    } else {
        for (std::size_t i = 0; i < cloud.n; ++i) {
            file << cloud.x[i] << " " << cloud.y[i] << " " << cloud.z[i] << "\n";
        }
    }
    return true;
}

inline bool savePCD(const std::string& filename, const std::vector<PointXYZ>& points, bool binary = true) {
    std::ofstream file(filename, binary ? (std::ios::binary | std::ios::out) : std::ios::out);
    if (!file.is_open()) return false;

    file << "# .PCD v.7 - Point Cloud Data file format\n";
    file << "VERSION .7\n";
    file << "FIELDS x y z\n";
    file << "SIZE 4 4 4\n";
    file << "TYPE F F F\n";
    file << "COUNT 1 1 1\n";
    file << "WIDTH " << points.size() << "\n";
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << points.size() << "\n";
    file << "DATA " << (binary ? "binary" : "ascii") << "\n";

    if (binary) {
        for (const auto& p : points) {
            file.write(reinterpret_cast<const char*>(&p.x), sizeof(float));
            file.write(reinterpret_cast<const char*>(&p.y), sizeof(float));
            file.write(reinterpret_cast<const char*>(&p.z), sizeof(float));
        }
    } else {
        for (const auto& p : points) {
            file << p.x << " " << p.y << " " << p.z << "\n";
        }
    }
    return true;
}

inline bool savePCDRGB(const std::string& filename, const std::vector<PointXYZRGB>& points, bool binary = false) {
    std::ofstream file(filename, binary ? (std::ios::binary | std::ios::out) : std::ios::out);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return false;
    }
    
    file << "# .PCD v.7 - Point Cloud Data file format\n";
    file << "VERSION .7\n";
    file << "FIELDS x y z rgb\n";
    file << "SIZE 4 4 4 4\n";
    file << "TYPE F F F F\n";
    file << "COUNT 1 1 1 1\n";
    file << "WIDTH " << points.size() << "\n";
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << points.size() << "\n";
    file << "DATA " << (binary ? "binary" : "ascii") << "\n";

    if (binary) {
        for (const auto& p : points) {
            std::uint32_t rgb_int = (static_cast<std::uint32_t>(p.r) << 16) |
                                    (static_cast<std::uint32_t>(p.g) << 8) |
                                    static_cast<std::uint32_t>(p.b);
            float rgb_float;
            std::memcpy(&rgb_float, &rgb_int, sizeof(float));

            file.write(reinterpret_cast<const char*>(&p.x), sizeof(float));
            file.write(reinterpret_cast<const char*>(&p.y), sizeof(float));
            file.write(reinterpret_cast<const char*>(&p.z), sizeof(float));
            file.write(reinterpret_cast<const char*>(&rgb_float), sizeof(float));
        }
    } else {
        for (const auto& p : points) {
            std::uint32_t rgb_int = (static_cast<std::uint32_t>(p.r) << 16) |
                                    (static_cast<std::uint32_t>(p.g) << 8) |
                                    static_cast<std::uint32_t>(p.b);
            float rgb_float;
            std::memcpy(&rgb_float, &rgb_int, sizeof(float));
            file << p.x << " " << p.y << " " << p.z << " " << rgb_float << "\n";
        }
    }
    return true;
}

} // namespace rvpoint
