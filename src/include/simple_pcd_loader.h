#pragma once
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "rvv_pcl.h"

namespace rvv_pcl {

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

            if (ref >= op || op + len + 2 > output.size()) {
                return false;
            }

            output[op++] = output[ref++];
            output[op++] = output[ref++];
            while (len--) {
                output[op++] = output[ref++];
            }
        }
    }

    return op == output.size();
}

inline bool parsePCDHeader(std::istream& stream, PCDHeader& header) {
    std::string line;
    while (std::getline(stream, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::stringstream ss(line);
        std::string tag;
        ss >> tag;
        if (tag == "FIELDS") {
            header.fields.clear();
            std::string field;
            while (ss >> field) {
                header.fields.push_back(field);
            }
        } else if (tag == "SIZE") {
            header.sizes.clear();
            int value = 0;
            while (ss >> value) {
                header.sizes.push_back(value);
            }
        } else if (tag == "TYPE") {
            header.types.clear();
            char value = '\0';
            while (ss >> value) {
                header.types.push_back(value);
            }
        } else if (tag == "COUNT") {
            header.counts.clear();
            int value = 0;
            while (ss >> value) {
                header.counts.push_back(value);
            }
        } else if (tag == "POINTS") {
            ss >> header.points;
        } else if (tag == "WIDTH" && header.points == 0) {
            ss >> header.points;
        } else if (tag == "DATA") {
            ss >> header.data_type;
            break;
        }
    }

    if (header.fields.empty() || header.sizes.size() != header.fields.size() ||
        header.types.size() != header.fields.size()) {
        return false;
    }
    if (header.counts.empty()) {
        header.counts.assign(header.fields.size(), 1);
    }
    return header.points > 0 && !header.data_type.empty();
}

inline int findFieldIndex(const PCDHeader& header, const std::string& name) {
    for (std::size_t i = 0; i < header.fields.size(); ++i) {
        if (header.fields[i] == name) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

inline bool readFloat(const unsigned char* src, int size, float& value) {
    if (size != 4) {
        return false;
    }
    std::memcpy(&value, src, sizeof(float));
    return true;
}

} // namespace detail

inline int loadPCD(const std::string& file_path, std::vector<PointXYZ>& points) {
    std::ifstream file(file_path, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << file_path << std::endl;
        return -1;
    }

    detail::PCDHeader header;
    if (!detail::parsePCDHeader(file, header)) {
        std::cerr << "Error: Failed to parse PCD header." << std::endl;
        return -1;
    }

    const int x_index = detail::findFieldIndex(header, "x");
    const int y_index = detail::findFieldIndex(header, "y");
    const int z_index = detail::findFieldIndex(header, "z");
    if (x_index < 0 || y_index < 0 || z_index < 0) {
        std::cerr << "Error: PCD is missing x/y/z fields." << std::endl;
        return -1;
    }

    if (header.data_type == "ascii") {
        points.clear();
        points.reserve(header.points);
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty()) {
                continue;
            }
            std::stringstream ss(line);
            std::vector<float> values(header.fields.size(), 0.0f);
            for (std::size_t i = 0; i < header.fields.size(); ++i) {
                ss >> values[i];
            }
            if (!ss.fail()) {
                points.push_back({values[static_cast<std::size_t>(x_index)],
                                  values[static_cast<std::size_t>(y_index)],
                                  values[static_cast<std::size_t>(z_index)]});
            }
        }
        std::cout << "Loaded " << points.size() << " points (ASCII)." << std::endl;
        return static_cast<int>(points.size());
    }

    points.clear();
    points.resize(header.points);

    std::vector<std::size_t> point_offsets(header.fields.size(), 0);
    std::size_t point_stride = 0;
    for (std::size_t i = 0; i < header.fields.size(); ++i) {
        point_offsets[i] = point_stride;
        point_stride += static_cast<std::size_t>(header.sizes[i] * header.counts[i]);
    }

    if (header.data_type == "binary") {
        std::vector<unsigned char> raw(point_stride * header.points);
        file.read(reinterpret_cast<char*>(raw.data()), static_cast<std::streamsize>(raw.size()));
        if (static_cast<std::size_t>(file.gcount()) != raw.size()) {
            std::cerr << "Error: Could not read binary PCD payload." << std::endl;
            return -1;
        }

        for (std::size_t i = 0; i < header.points; ++i) {
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            const unsigned char* point_base = raw.data() + i * point_stride;
            if (!detail::readFloat(point_base + point_offsets[static_cast<std::size_t>(x_index)],
                                   header.sizes[static_cast<std::size_t>(x_index)], x) ||
                !detail::readFloat(point_base + point_offsets[static_cast<std::size_t>(y_index)],
                                   header.sizes[static_cast<std::size_t>(y_index)], y) ||
                !detail::readFloat(point_base + point_offsets[static_cast<std::size_t>(z_index)],
                                   header.sizes[static_cast<std::size_t>(z_index)], z)) {
                std::cerr << "Error: Unsupported x/y/z field layout in binary PCD." << std::endl;
                return -1;
            }
            points[i] = {x, y, z};
        }
        std::cout << "Loaded " << points.size() << " points (Binary)." << std::endl;
        return static_cast<int>(points.size());
    }

    if (header.data_type == "binary_compressed") {
        std::uint32_t compressed_size = 0;
        std::uint32_t uncompressed_size = 0;
        file.read(reinterpret_cast<char*>(&compressed_size), sizeof(compressed_size));
        file.read(reinterpret_cast<char*>(&uncompressed_size), sizeof(uncompressed_size));
        if (!file) {
            std::cerr << "Error: Could not read compressed PCD sizes." << std::endl;
            return -1;
        }

        std::vector<unsigned char> compressed(compressed_size);
        file.read(reinterpret_cast<char*>(compressed.data()),
                  static_cast<std::streamsize>(compressed.size()));
        if (static_cast<std::size_t>(file.gcount()) != compressed.size()) {
            std::cerr << "Error: Could not read compressed PCD payload." << std::endl;
            return -1;
        }

        std::vector<unsigned char> uncompressed(uncompressed_size);
        if (!detail::decompressLZF(compressed, uncompressed)) {
            std::cerr << "Error: Failed to decompress binary_compressed PCD payload."
                      << std::endl;
            return -1;
        }

        std::vector<std::size_t> field_block_offsets(header.fields.size(), 0);
        std::size_t cursor = 0;
        for (std::size_t i = 0; i < header.fields.size(); ++i) {
            field_block_offsets[i] = cursor;
            cursor += static_cast<std::size_t>(header.sizes[i] * header.counts[i]) * header.points;
        }
        if (cursor > uncompressed.size()) {
            std::cerr << "Error: Decompressed PCD payload is smaller than expected."
                      << std::endl;
            return -1;
        }

        for (std::size_t i = 0; i < header.points; ++i) {
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            const unsigned char* x_ptr =
                uncompressed.data() + field_block_offsets[static_cast<std::size_t>(x_index)] +
                i * static_cast<std::size_t>(header.sizes[static_cast<std::size_t>(x_index)] *
                                             header.counts[static_cast<std::size_t>(x_index)]);
            const unsigned char* y_ptr =
                uncompressed.data() + field_block_offsets[static_cast<std::size_t>(y_index)] +
                i * static_cast<std::size_t>(header.sizes[static_cast<std::size_t>(y_index)] *
                                             header.counts[static_cast<std::size_t>(y_index)]);
            const unsigned char* z_ptr =
                uncompressed.data() + field_block_offsets[static_cast<std::size_t>(z_index)] +
                i * static_cast<std::size_t>(header.sizes[static_cast<std::size_t>(z_index)] *
                                             header.counts[static_cast<std::size_t>(z_index)]);
            if (!detail::readFloat(x_ptr, header.sizes[static_cast<std::size_t>(x_index)], x) ||
                !detail::readFloat(y_ptr, header.sizes[static_cast<std::size_t>(y_index)], y) ||
                !detail::readFloat(z_ptr, header.sizes[static_cast<std::size_t>(z_index)], z)) {
                std::cerr << "Error: Unsupported x/y/z field layout in compressed PCD."
                          << std::endl;
                return -1;
            }
            points[i] = {x, y, z};
        }
        std::cout << "Loaded " << points.size() << " points (Binary Compressed)." << std::endl;
        return static_cast<int>(points.size());
    }

    std::cerr << "Error: Unsupported PCD DATA mode '" << header.data_type << "'." << std::endl;
    return -1;
}

inline void savePCD(const std::string& filename, const std::vector<PointXYZ>& points,
                    bool binary = false) {
    std::ofstream file(filename, binary ? (std::ios::binary | std::ios::out) : std::ios::out);
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
    
    file.close();
    std::cout << "Saved " << points.size() << " points to " << filename << std::endl;
}

} // namespace rvv_pcl
