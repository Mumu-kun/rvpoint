#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "io/streams/socket_compat.h"
#include "io/streams/stream_source.h"

namespace rvpoint {

/**
 * @brief Production adapter for non-blocking POSIX UDP binary stream reception.
 */
class UdpStreamSource : public StreamSource {
public:
    explicit UdpStreamSource(int port = kDefaultUdpPort, const std::string& bind_ip = "0.0.0.0");
    ~UdpStreamSource() override;

    bool is_valid() const noexcept { return sock_ != INVALID_SOCKET; }

    bool poll_frame(StreamFrame& out) override;

private:
    int port_ = kDefaultUdpPort;
    std::string bind_ip_ = "0.0.0.0";
    socket_t sock_ = INVALID_SOCKET;
    std::vector<uint8_t> rx_buffer_;

    uint32_t assembly_seq_ = 0;
    uint16_t assembly_chunks_received_ = 0;
    StreamFrame assembly_frame_;

    void init_network();
    void close_network();
};

} // namespace rvpoint

