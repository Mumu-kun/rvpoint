#include "io/streams/udp_stream_source.h"

#include <cstring>

namespace rvpoint {

UdpStreamSource::UdpStreamSource(int port, const std::string& bind_ip)
    : port_(port), bind_ip_(bind_ip) {
    init_network();
}

UdpStreamSource::~UdpStreamSource() {
    close_network();
}

bool UdpStreamSource::poll_frame(StreamFrame& out) {
    if (sock_ == INVALID_SOCKET) return false;

    bool frame_received = false;
    while (true) {
        sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);
        ssize_t bytes_read = recvfrom(sock_,
                                      reinterpret_cast<char*>(rx_buffer_.data()),
                                      rx_buffer_.size(),
                                      0,
                                      reinterpret_cast<sockaddr*>(&client_addr),
                                      &addr_len);

        if (bytes_read < 0) {
#ifdef _WIN32
            int err = WSAGetLastError();
            if (err == WSAEWOULDBLOCK) break;
#else
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
#endif
            break;
        }

        if (static_cast<size_t>(bytes_read) < sizeof(UdpPacketHeader)) {
            continue;
        }

        const auto* hdr = reinterpret_cast<const UdpPacketHeader*>(rx_buffer_.data());
        if (hdr->magic != kStreamMagic) {
            continue;
        }

        size_t expected_bytes = sizeof(UdpPacketHeader) + hdr->num_points * 3 * sizeof(float);
        if (static_cast<size_t>(bytes_read) < expected_bytes) {
            continue;
        }

        if (hdr->total_chunks <= 1) {
            out.seq = hdr->seq;
            out.timestamp_ns = hdr->timestamp_ns;
            out.coremotion = hdr->coremotion;
            out.vio = hdr->vio;

            out.cloud.resize(hdr->num_points);
            const float* x_src = reinterpret_cast<const float*>(rx_buffer_.data() + sizeof(UdpPacketHeader));
            const float* y_src = x_src + hdr->num_points;
            const float* z_src = y_src + hdr->num_points;

            if (hdr->num_points > 0) {
                std::memcpy(out.cloud.x.data(), x_src, hdr->num_points * sizeof(float));
                std::memcpy(out.cloud.y.data(), y_src, hdr->num_points * sizeof(float));
                std::memcpy(out.cloud.z.data(), z_src, hdr->num_points * sizeof(float));
            }
            frame_received = true;
        } else {
            if (hdr->seq != assembly_seq_) {
                assembly_seq_ = hdr->seq;
                assembly_chunks_received_ = 0;
                assembly_frame_.seq = hdr->seq;
                assembly_frame_.timestamp_ns = hdr->timestamp_ns;
                assembly_frame_.coremotion = hdr->coremotion;
                assembly_frame_.vio = hdr->vio;
                assembly_frame_.cloud.clear();
            }

            const float* x_src = reinterpret_cast<const float*>(rx_buffer_.data() + sizeof(UdpPacketHeader));
            const float* y_src = x_src + hdr->num_points;
            const float* z_src = y_src + hdr->num_points;

            size_t base = assembly_frame_.cloud.size();
            assembly_frame_.cloud.resize(base + hdr->num_points);
            if (hdr->num_points > 0) {
                std::memcpy(assembly_frame_.cloud.x.data() + base, x_src, hdr->num_points * sizeof(float));
                std::memcpy(assembly_frame_.cloud.y.data() + base, y_src, hdr->num_points * sizeof(float));
                std::memcpy(assembly_frame_.cloud.z.data() + base, z_src, hdr->num_points * sizeof(float));
            }

            assembly_chunks_received_++;
            if (assembly_chunks_received_ >= hdr->total_chunks) {
                out.seq = assembly_frame_.seq;
                out.timestamp_ns = assembly_frame_.timestamp_ns;
                out.coremotion = assembly_frame_.coremotion;
                out.vio = assembly_frame_.vio;
                out.cloud.copy_from(assembly_frame_.cloud.view());
                frame_received = true;
                assembly_chunks_received_ = 0;
            }
        }
    }

    return frame_received;
}

void UdpStreamSource::init_network() {
    rx_buffer_.resize(kMaxUdpPayloadSize);
#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
    sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_ == INVALID_SOCKET) return;

    int opt = 1;
#ifdef _WIN32
    setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&opt), sizeof(opt));
    u_long mode = 1;
    ioctlsocket(sock_, FIONBIO, &mode);
#else
    setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    int flags = fcntl(sock_, F_GETFL, 0);
    fcntl(sock_, F_SETFL, flags | O_NONBLOCK);
#endif

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(static_cast<uint16_t>(port_));
    if (bind_ip_ == "0.0.0.0" || bind_ip_.empty()) {
        server_addr.sin_addr.s_addr = INADDR_ANY;
    } else {
        inet_pton(AF_INET, bind_ip_.c_str(), &server_addr.sin_addr);
    }

    if (bind(sock_, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) == SOCKET_ERROR) {
        CLOSE_SOCKET(sock_);
        sock_ = INVALID_SOCKET;
        return;
    }

    int rcvbuf_size = 1024 * 1024;
#ifdef _WIN32
    setsockopt(sock_, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char*>(&rcvbuf_size), sizeof(rcvbuf_size));
#else
    setsockopt(sock_, SOL_SOCKET, SO_RCVBUF, &rcvbuf_size, sizeof(rcvbuf_size));
#endif
}

void UdpStreamSource::close_network() {
    if (sock_ != INVALID_SOCKET) {
        CLOSE_SOCKET(sock_);
        sock_ = INVALID_SOCKET;
    }
#ifdef _WIN32
    WSACleanup();
#endif
}

} // namespace rvpoint

