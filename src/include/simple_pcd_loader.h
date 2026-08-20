#pragma once
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#ifdef __riscv_vector
#include "rvv_pcl.h"
#else
namespace rvv_pcl {
struct PointXYZ {
  float x, y, z;
};
}
#endif

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

inline int64_t loadPCD(const std::string& file_path, std::vector<PointXYZ>& points) {
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

    if (header.points > 100000000ULL) {
        std::cerr << "Error: Declared PCD points count exceeds safety limit (100M)." << std::endl;
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
        points.reserve(std::min(header.points, static_cast<std::size_t>(10000000ULL)));
        std::string line;
        
        // Calculate total float elements per line taking COUNT into account
        std::vector<std::size_t> field_elem_offsets(header.fields.size(), 0);
        std::size_t total_elements = 0;
        for (std::size_t i = 0; i < header.fields.size(); ++i) {
            field_elem_offsets[i] = total_elements;
            total_elements += static_cast<std::size_t>(std::max(1, header.counts[i]));
        }

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') {
                continue;
            }
            std::stringstream ss(line);
            std::vector<float> values(total_elements, 0.0f);
            bool parse_ok = true;
            for (std::size_t i = 0; i < total_elements; ++i) {
                if (!(ss >> values[i])) {
                    parse_ok = false;
                    break;
                }
            }
            if (parse_ok) {
                float x = values[field_elem_offsets[static_cast<std::size_t>(x_index)]];
                float y = values[field_elem_offsets[static_cast<std::size_t>(y_index)]];
                float z = values[field_elem_offsets[static_cast<std::size_t>(z_index)]];
                if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                    points.push_back({x, y, z});
                }
            }
        }
        std::cout << "Loaded " << points.size() << " points (ASCII)." << std::endl;
        return static_cast<int64_t>(points.size());
    }

    points.clear();
    points.resize(header.points);

    std::vector<std::size_t> point_offsets(header.fields.size(), 0);
    std::size_t point_stride = 0;
    for (std::size_t i = 0; i < header.fields.size(); ++i) {
        point_offsets[i] = point_stride;
        std::size_t field_sz = static_cast<std::size_t>(std::max(1, header.sizes[i]));
        std::size_t field_cnt = static_cast<std::size_t>(std::max(1, header.counts[i]));
        point_stride += field_sz * field_cnt;
    }

    if (point_stride == 0) {
        std::cerr << "Error: PCD point stride is zero." << std::endl;
        return -1;
    }

    if (header.data_type == "binary") {
        if (header.points > 0 && (SIZE_MAX / header.points < point_stride)) {
            std::cerr << "Error: PCD payload size arithmetic overflow." << std::endl;
            return -1;
        }
        std::size_t total_bytes = point_stride * header.points;
        std::vector<unsigned char> raw(total_bytes);
        file.read(reinterpret_cast<char*>(raw.data()), static_cast<std::streamsize>(raw.size()));
        if (static_cast<std::size_t>(file.gcount()) != raw.size()) {
            std::cerr << "Error: Could not read binary PCD payload." << std::endl;
            return -1;
        }

        std::size_t valid_cnt = 0;
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
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                points[valid_cnt++] = {x, y, z};
            }
        }
        points.resize(valid_cnt);
        std::cout << "Loaded " << points.size() << " points (Binary)." << std::endl;
        return static_cast<int64_t>(points.size());
    }

    if (header.data_type == "binary_compressed") {
        std::uint32_t compressed_size = 0;
        std::uint32_t uncompressed_size = 0;
        file.read(reinterpret_cast<char*>(&compressed_size), sizeof(compressed_size));
        file.read(reinterpret_cast<char*>(&uncompressed_size), sizeof(uncompressed_size));
        if (!file || compressed_size == 0 || uncompressed_size == 0 || uncompressed_size > 1000000000U) {
            std::cerr << "Error: Invalid compressed PCD payload sizes." << std::endl;
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
            std::size_t f_sz = static_cast<std::size_t>(std::max(1, header.sizes[i]));
            std::size_t f_cnt = static_cast<std::size_t>(std::max(1, header.counts[i]));
            if (header.points > 0 && SIZE_MAX / header.points < (f_sz * f_cnt)) {
                std::cerr << "Error: Compressed PCD field offset overflow." << std::endl;
                return -1;
            }
            cursor += (f_sz * f_cnt) * header.points;
        }
        if (cursor > uncompressed.size()) {
            std::cerr << "Error: Decompressed PCD payload is smaller than expected."
                      << std::endl;
            return -1;
        }

        std::size_t valid_cnt = 0;
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
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                points[valid_cnt++] = {x, y, z};
            }
        }
        points.resize(valid_cnt);
        std::cout << "Loaded " << points.size() << " points (Binary Compressed)." << std::endl;
        return static_cast<int64_t>(points.size());
    }

    std::cerr << "Error: Unsupported PCD DATA mode '" << header.data_type << "'." << std::endl;
    return -1;
}

inline bool savePCD(const std::string& filename, const std::vector<PointXYZ>& points,
                    bool binary = false) {
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
    
    file.flush();
    if (!file.good()) {
        std::cerr << "Error: File stream error while writing " << filename << std::endl;
        return false;
    }
    file.close();
    std::cout << "Saved " << points.size() << " points to " << filename << std::endl;
    return true;
}

struct PointXYZRGB {
    float x;
    float y;
    float z;
    std::uint8_t r;
    std::uint8_t g;
    std::uint8_t b;
};

inline bool savePCDRGB(const std::string& filename, const std::vector<PointXYZRGB>& points,
                       bool binary = false) {
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
    
    file.flush();
    if (!file.good()) {
        std::cerr << "Error: File stream error while writing " << filename << std::endl;
        return false;
    }
    file.close();
    std::cout << "Saved " << points.size() << " colored points to " << filename << std::endl;
    return true;
}

} // namespace rvv_pcl

