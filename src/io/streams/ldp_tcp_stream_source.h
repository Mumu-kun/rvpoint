#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "io/streams/socket_compat.h"
#include "io/streams/stream_source.h"

namespace rvpoint {

/**
 * @brief Native TCP Ingestion Adapter for iPhone demonstration app (LDP1 & LDP2 protocols).
 *
 * Connects directly with `LiDARStreamer.swift` on port 9000 with zero Python intermediaries.
 * Performs fast vectorized depth map unprojection directly into `PointCloud`.
 */
class LdpTcpStreamSource : public StreamSource {
public:
    explicit LdpTcpStreamSource(int port = kDefaultTcpPort, const std::string& bind_ip = "0.0.0.0");
    ~LdpTcpStreamSource() override;

    bool is_valid() const noexcept { return server_sock_ != INVALID_SOCKET; }
    bool is_client_connected() const noexcept { return client_sock_ != INVALID_SOCKET; }

    bool poll_frame(StreamFrame& out) override;

private:
    int port_ = kDefaultTcpPort;
    std::string bind_ip_ = "0.0.0.0";
    socket_t server_sock_ = INVALID_SOCKET;
    socket_t client_sock_ = INVALID_SOCKET;
    std::vector<uint8_t> rx_buffer_;

    void init_network();
    void accept_incoming_client();
    void close_network();
};

} // namespace rvpoint

