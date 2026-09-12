#include "io/streams/ldp_tcp_stream_source.h"

#include <chrono>
#include <cstring>

#include "io/streams/depth_unprojection.h"

namespace rvpoint {

LdpTcpStreamSource::LdpTcpStreamSource(int port, const std::string& bind_ip)
    : port_(port), bind_ip_(bind_ip) {
    init_network();
}

LdpTcpStreamSource::~LdpTcpStreamSource() {
    close_network();
}

bool LdpTcpStreamSource::poll_frame(StreamFrame& out) {
    accept_incoming_client();
    if (client_sock_ == INVALID_SOCKET) return false;

    // Drain incoming TCP bytes non-blocking into rx_buffer_
    uint8_t temp_buf[65536];
    while (true) {
        ssize_t bytes = recv(client_sock_, reinterpret_cast<char*>(temp_buf), sizeof(temp_buf), 0);
        if (bytes > 0) {
            size_t old_size = rx_buffer_.size();
            rx_buffer_.resize(old_size + bytes);
            std::memcpy(rx_buffer_.data() + old_size, temp_buf, bytes);
        } else if (bytes == 0) {
            // Client cleanly disconnected
            CLOSE_SOCKET(client_sock_);
            client_sock_ = INVALID_SOCKET;
            rx_buffer_.clear();
            return false;
        } else {
#ifdef _WIN32
            int err = WSAGetLastError();
            if (err == WSAEWOULDBLOCK) break;
#else
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
#endif
            // Other socket error
            CLOSE_SOCKET(client_sock_);
            client_sock_ = INVALID_SOCKET;
            rx_buffer_.clear();
            return false;
        }
    }

    // Process packets in rx_buffer_
    bool frame_assembled = false;
    while (rx_buffer_.size() >= 4) {
        // Find next magic
        size_t magic_pos = 0;
        bool found_magic = false;
        while (magic_pos + 4 <= rx_buffer_.size()) {
            uint32_t magic;
            std::memcpy(&magic, rx_buffer_.data() + magic_pos, 4);
            if (magic == kLdp1Magic || magic == kLdp2Magic) {
                found_magic = true;
                break;
            }
            magic_pos++;
        }

        if (!found_magic) {
            // Drop stale bytes
            if (rx_buffer_.size() > 3) {
                rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.end() - 3);
            }
            break;
        }

        if (magic_pos > 0) {
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + magic_pos);
        }

        uint32_t magic;
        std::memcpy(&magic, rx_buffer_.data(), 4);
        size_t header_size = (magic == kLdp2Magic) ? sizeof(Ldp2Header) : sizeof(Ldp1Header);
        if (rx_buffer_.size() < header_size) break;

        uint32_t w = 256, h = 192;
        float fx = 0, fy = 0, cx = 0, cy = 0;
        const float* pose_src = nullptr;

        if (magic == kLdp2Magic) {
            const auto* h2 = reinterpret_cast<const Ldp2Header*>(rx_buffer_.data());
            w = h2->width; h = h2->height;
            fx = h2->fx; fy = h2->fy; cx = h2->cx; cy = h2->cy;
            pose_src = h2->pose;
            out.seq = h2->frame_idx;
            out.vio.tracking_state = h2->tracking_state;
            std::memcpy(out.coremotion.gyro, h2->gyro, sizeof(h2->gyro));
            std::memcpy(out.coremotion.accel, h2->accel, sizeof(h2->accel));
            std::memcpy(out.coremotion.gravity, h2->gravity, sizeof(h2->gravity));
        } else {
            const auto* h1 = reinterpret_cast<const Ldp1Header*>(rx_buffer_.data());
            w = h1->width; h = h1->height;
            fx = h1->fx; fy = h1->fy; cx = h1->cx; cy = h1->cy;
            pose_src = h1->pose;
            out.seq = h1->frame_idx;
            out.vio.tracking_state = 2; // Normal
        }

        size_t depth_bytes = w * h * sizeof(float);
        size_t conf_bytes = w * h * sizeof(uint8_t);
        size_t total_frame_bytes = header_size + depth_bytes + conf_bytes;

        if (rx_buffer_.size() < total_frame_bytes) {
            break; // Incomplete payload, wait for more TCP packets
        }

        // Unpack 4x4 camera pose (convert row-major with Swift sign flips to column-major)
        // Swift pose layout: row-major [R00, R01, R02, Tx; R10...], with inverted indices [1,2,5,6,9,10,13,14]
        // We undo the sign flips and store column-major into out.vio.T_world_cam:
        float p[16];
        std::memcpy(p, pose_src, sizeof(p));
        // Undo Swift sign flips:
        for (int idx : {1, 2, 5, 6, 9, 10, 13, 14}) {
            p[idx] = -p[idx];
        }
        // Transpose from row-major to column-major:
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                out.vio.T_world_cam[c * 4 + r] = p[r * 4 + c];
            }
        }

        const float* depth_ptr = reinterpret_cast<const float*>(rx_buffer_.data() + header_size);
        const uint8_t* conf_ptr = rx_buffer_.data() + header_size + depth_bytes;

        // Fast onboard unprojection directly into PointCloud
        unproject_depth_map(depth_ptr, conf_ptr, w, h, fx, fy, cx, cy, 2, 0.25f, 4.5f, out.cloud);

        auto now = std::chrono::steady_clock::now().time_since_epoch();
        out.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();

        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + total_frame_bytes);
        frame_assembled = true;
    }

    return frame_assembled;
}

void LdpTcpStreamSource::init_network() {
#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
    server_sock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_sock_ == INVALID_SOCKET) return;

    int opt = 1;
#ifdef _WIN32
    setsockopt(server_sock_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&opt), sizeof(opt));
    u_long mode = 1;
    ioctlsocket(server_sock_, FIONBIO, &mode);
#else
    setsockopt(server_sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    int flags = fcntl(server_sock_, F_GETFL, 0);
    fcntl(server_sock_, F_SETFL, flags | O_NONBLOCK);
#endif

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port_));
    if (bind_ip_ == "0.0.0.0" || bind_ip_.empty()) {
        addr.sin_addr.s_addr = INADDR_ANY;
    } else {
        inet_pton(AF_INET, bind_ip_.c_str(), &addr.sin_addr);
    }

    if (bind(server_sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR) {
        CLOSE_SOCKET(server_sock_);
        server_sock_ = INVALID_SOCKET;
        return;
    }

    listen(server_sock_, 2);
}

void LdpTcpStreamSource::accept_incoming_client() {
    if (server_sock_ == INVALID_SOCKET) return;

    sockaddr_in client_addr{};
    socklen_t addr_len = sizeof(client_addr);
    socket_t new_client = accept(server_sock_, reinterpret_cast<sockaddr*>(&client_addr), &addr_len);
    if (new_client != INVALID_SOCKET) {
        // If already connected to another client, close old one
        if (client_sock_ != INVALID_SOCKET) {
            CLOSE_SOCKET(client_sock_);
        }
        client_sock_ = new_client;

        // Set client socket to non-blocking
#ifdef _WIN32
        u_long mode = 1;
        ioctlsocket(client_sock_, FIONBIO, &mode);
#else
        int flags = fcntl(client_sock_, F_GETFL, 0);
        fcntl(client_sock_, F_SETFL, flags | O_NONBLOCK);
#endif
        rx_buffer_.clear();
    }
}

void LdpTcpStreamSource::close_network() {
    if (client_sock_ != INVALID_SOCKET) {
        CLOSE_SOCKET(client_sock_);
        client_sock_ = INVALID_SOCKET;
    }
    if (server_sock_ != INVALID_SOCKET) {
        CLOSE_SOCKET(server_sock_);
        server_sock_ = INVALID_SOCKET;
    }
#ifdef _WIN32
    WSACleanup();
#endif
}

} // namespace rvpoint

