#include <cassert>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

#include "include/rvpoint.h"
#include "io/streams/socket_compat.h"
#include "io/streams/stream_types.h"
#include "io/streams/stream_source.h"
#include "io/streams/depth_unprojection.h"
#include "io/streams/mock_pcd_stream_source.h"
#include "io/streams/udp_stream_source.h"
#include "io/streams/ldp_tcp_stream_source.h"

int main() {
    std::cout << "============================================================" << std::endl;
    std::cout << " Running test_stream_source (Unified Stream Ingestion)" << std::endl;
    std::cout << "============================================================" << std::endl;

    // 1. Verify Packet Header Size & Structure
    std::cout << "[1] Verifying protocol headers..." << std::endl;
    assert(sizeof(rvpoint::UdpPacketHeader) == 132);
    assert(sizeof(rvpoint::CoreMotionData) == 36);
    assert(sizeof(rvpoint::ArKitVioData) == 68);
    assert(sizeof(rvpoint::Ldp1Header) == 96);
    assert(sizeof(rvpoint::Ldp2Header) == 136);

    rvpoint::UdpPacketHeader hdr_init;
    assert(hdr_init.magic == rvpoint::kStreamMagic);
    assert(hdr_init.total_chunks == 1);
    assert(hdr_init.vio.tracking_state == 2);
    assert(std::abs(hdr_init.coremotion.gravity[2] - (-9.81f)) < 1e-4f);
    std::cout << "    [PASS] Wire headers match exact specifications (UDP: 132B, LDP1: 96B, LDP2: 136B)." << std::endl;

    // 2. Test Depth Map Unprojection Kernel
    std::cout << "[2] Verifying depth map pinhole unprojection..." << std::endl;
    const uint32_t tw = 4, th = 4;
    std::vector<float> test_depth(tw * th, 2.0f); // 2 meters uniform depth
    std::vector<uint8_t> test_conf(tw * th, 2);   // High confidence
    rvpoint::PointCloud unproj_cloud;

    float fx = 100.0f, fy = 100.0f, cx = 2.0f, cy = 2.0f;
    rvpoint::unproject_depth_map(test_depth.data(), test_conf.data(), tw, th, fx, fy, cx, cy, 2, 0.3f, 4.5f, unproj_cloud);
    assert(unproj_cloud.size() == 16);
    // Center pixel (u=2, v=2) -> dx = 0, dy = 0, z = 2.0
    size_t center_idx = 2 * tw + 2;
    assert(std::abs(unproj_cloud.x[center_idx] - 0.0f) < 1e-5f);
    assert(std::abs(unproj_cloud.y[center_idx] - 0.0f) < 1e-5f);
    assert(std::abs(unproj_cloud.z[center_idx] - 2.0f) < 1e-5f);
    std::cout << "    [PASS] Depth unprojection mathematically validated." << std::endl;

    // 3. Test MockPcdStreamSource
    std::cout << "[3] Testing MockPcdStreamSource with real PCD file..." << std::endl;
    std::vector<std::string> candidates = {
        "data/pcd_compressed/0000000000.pcd",
        "data/0000000000.pcd",
        "../data/pcd_compressed/0000000000.pcd",
        "../data/0000000000.pcd"
    };

    std::string valid_pcd;
    for (const auto& c : candidates) {
        std::ifstream f(c);
        if (f.good()) {
            valid_pcd = c;
            break;
        }
    }

    if (valid_pcd.empty()) {
        std::cerr << "[FAIL] Could not locate test PCD file in candidate paths." << std::endl;
        return 1;
    }

    rvpoint::MockPcdStreamSource mock_source(
        valid_pcd,
        30.0,
        true,
        rvpoint::MockTrajectoryMode::CircularArc,
        0.4f,
        0.2f
    );

    rvpoint::StreamFrame frame1;
    bool ok1 = mock_source.poll_frame(frame1);
    assert(ok1);
    assert(frame1.seq == 0);
    assert(frame1.cloud.size() > 0);
    assert(frame1.vio.tracking_state == 2);
    assert(std::abs(frame1.coremotion.gyro[2] - 0.2f) < 1e-4f);
    assert(std::abs(frame1.coremotion.gravity[2] - (-9.81f)) < 1e-4f);
    std::cout << "    [PASS] Frame 0 loaded " << frame1.cloud.size() << " points with gyro_z = "
              << frame1.coremotion.gyro[2] << " rad/s, grav_z = " << frame1.coremotion.gravity[2] << " m/s^2" << std::endl;

    // 4. Test UdpStreamSource Loopback Ingestion
    std::cout << "[4] Testing UdpStreamSource non-blocking loopback on port 18765..." << std::endl;
    const int test_udp_port = 18765;
    rvpoint::UdpStreamSource udp_source(test_udp_port, "127.0.0.1");
    assert(udp_source.is_valid());

    rvpoint::StreamFrame empty_frame;
    bool polled_empty = udp_source.poll_frame(empty_frame);
    assert(!polled_empty);

    socket_t client_udp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    assert(client_udp != INVALID_SOCKET);

    sockaddr_in dest_udp{};
    dest_udp.sin_family = AF_INET;
    dest_udp.sin_port = htons(test_udp_port);
    inet_pton(AF_INET, "127.0.0.1", &dest_udp.sin_addr);

    rvpoint::UdpPacketHeader tx_hdr;
    tx_hdr.magic = rvpoint::kStreamMagic;
    tx_hdr.seq = 77;
    tx_hdr.timestamp_ns = 999999999ULL;
    tx_hdr.coremotion.gyro[2] = 0.35f;
    tx_hdr.coremotion.accel[0] = 0.15f;
    tx_hdr.coremotion.gravity[0] = 0.1f;
    tx_hdr.coremotion.gravity[2] = -9.80f;
    tx_hdr.vio.T_world_cam[12] = 2.5f;
    tx_hdr.vio.T_world_cam[13] = -1.2f;
    tx_hdr.vio.tracking_state = 2;
    tx_hdr.num_points = 3;

    std::vector<float> tx_x = {10.0f, 20.0f, 30.0f};
    std::vector<float> tx_y = {1.0f, 2.0f, 3.0f};
    std::vector<float> tx_z = {0.5f, 0.6f, 0.7f};

    std::vector<uint8_t> tx_udp_pkt(sizeof(tx_hdr) + 3 * 3 * sizeof(float));
    std::memcpy(tx_udp_pkt.data(), &tx_hdr, sizeof(tx_hdr));
    float* p_ptr = reinterpret_cast<float*>(tx_udp_pkt.data() + sizeof(tx_hdr));
    std::memcpy(p_ptr, tx_x.data(), 3 * sizeof(float));
    std::memcpy(p_ptr + 3, tx_y.data(), 3 * sizeof(float));
    std::memcpy(p_ptr + 6, tx_z.data(), 3 * sizeof(float));

    sendto(client_udp, reinterpret_cast<const char*>(tx_udp_pkt.data()), tx_udp_pkt.size(), 0,
           reinterpret_cast<const sockaddr*>(&dest_udp), sizeof(dest_udp));
    CLOSE_SOCKET(client_udp);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    rvpoint::StreamFrame rx_udp;
    bool polled_udp = udp_source.poll_frame(rx_udp);
    assert(polled_udp);
    assert(rx_udp.seq == 77);
    assert(rx_udp.cloud.size() == 3);
    std::cout << "    [PASS] UDP loopback transmission and reception verified." << std::endl;

    // 5. Test LdpTcpStreamSource Loopback Ingestion (iPhone LDP2 TCP Protocol)
    std::cout << "[5] Testing LdpTcpStreamSource TCP loopback on port 19001..." << std::endl;
    const int test_tcp_port = 19001;
    rvpoint::LdpTcpStreamSource tcp_source(test_tcp_port, "127.0.0.1");
    assert(tcp_source.is_valid());

    socket_t client_tcp = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    assert(client_tcp != INVALID_SOCKET);

    sockaddr_in dest_tcp{};
    dest_tcp.sin_family = AF_INET;
    dest_tcp.sin_port = htons(test_tcp_port);
    inet_pton(AF_INET, "127.0.0.1", &dest_tcp.sin_addr);

    int conn_res = connect(client_tcp, reinterpret_cast<const sockaddr*>(&dest_tcp), sizeof(dest_tcp));
    assert(conn_res == 0);

    // Formulate LDP2 packet
    rvpoint::Ldp2Header ldp_hdr{};
    ldp_hdr.magic = rvpoint::kLdp2Magic; // "LDP2"
    ldp_hdr.frame_idx = 101;
    ldp_hdr.width = 4;
    ldp_hdr.height = 4;
    ldp_hdr.fx = 100.0f; ldp_hdr.fy = 100.0f; ldp_hdr.cx = 2.0f; ldp_hdr.cy = 2.0f;
    // Identity pose
    ldp_hdr.pose[0] = 1.0f; ldp_hdr.pose[5] = 1.0f; ldp_hdr.pose[10] = 1.0f; ldp_hdr.pose[15] = 1.0f;
    ldp_hdr.tracking_state = 2; // Normal
    ldp_hdr.gyro[2] = 0.5f;
    ldp_hdr.gravity[2] = -9.81f;

    std::vector<float> tcp_depth(16, 1.5f); // 1.5m depth
    std::vector<uint8_t> tcp_conf(16, 2);   // Valid confidence

    std::vector<uint8_t> ldp_packet(sizeof(ldp_hdr) + 16 * sizeof(float) + 16 * sizeof(uint8_t));
    std::memcpy(ldp_packet.data(), &ldp_hdr, sizeof(ldp_hdr));
    std::memcpy(ldp_packet.data() + sizeof(ldp_hdr), tcp_depth.data(), 16 * sizeof(float));
    std::memcpy(ldp_packet.data() + sizeof(ldp_hdr) + 16 * sizeof(float), tcp_conf.data(), 16 * sizeof(uint8_t));

    send(client_tcp, reinterpret_cast<const char*>(ldp_packet.data()), ldp_packet.size(), 0);
    CLOSE_SOCKET(client_tcp);

    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    rvpoint::StreamFrame rx_tcp;
    bool polled_tcp = tcp_source.poll_frame(rx_tcp);
    assert(polled_tcp);
    assert(rx_tcp.seq == 101);
    assert(rx_tcp.vio.tracking_state == 2);
    assert(std::abs(rx_tcp.coremotion.gyro[2] - 0.5f) < 1e-5f);
    assert(rx_tcp.cloud.size() == 16);
    assert(std::abs(rx_tcp.cloud.z[0] - 1.5f) < 1e-5f);
    std::cout << "    [PASS] LDP2 TCP packet received, parsed, and unprojected directly into PointCloud!" << std::endl;

    std::cout << "============================================================" << std::endl;
    std::cout << " test_stream_source PASSED ALL CHECKS" << std::endl;
    std::cout << "============================================================" << std::endl;

    return 0;
}
