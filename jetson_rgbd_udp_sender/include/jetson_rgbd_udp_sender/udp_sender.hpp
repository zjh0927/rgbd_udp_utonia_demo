#pragma once

#include "jetson_rgbd_udp_sender/frame_types.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

#include <sys/socket.h>

namespace jetson_rgbd_udp_sender
{

struct UdpSenderConfig
{
  std::string destination;
  std::uint16_t port{0};
  std::size_t datagram_bytes{1200};
  std::size_t socket_send_buffer_bytes{4U * 1024U * 1024U};
  std::size_t max_frame_bytes{16U * 1024U * 1024U};
};

struct UdpSenderStats
{
  std::uint64_t frames_sent{0};
  std::uint64_t frames_dropped{0};
  std::uint64_t datagrams_sent{0};
  std::uint64_t bytes_sent{0};
};

class UdpSender
{
public:
  explicit UdpSender(const UdpSenderConfig & config);
  ~UdpSender();

  UdpSender(const UdpSender &) = delete;
  UdpSender & operator=(const UdpSender &) = delete;

  UdpSender(UdpSender &&) = delete;
  UdpSender & operator=(UdpSender &&) = delete;

  SendResult send_frame(const EncodedFrame & frame);
  UdpSenderStats stats() const noexcept;

private:
  UdpSenderConfig config_;
  int socket_fd_{-1};
  sockaddr_storage destination_address_{};
  socklen_t destination_address_length_{0};

  std::atomic<std::uint64_t> frames_sent_{0};
  std::atomic<std::uint64_t> frames_dropped_{0};
  std::atomic<std::uint64_t> datagrams_sent_{0};
  std::atomic<std::uint64_t> bytes_sent_{0};
};

}  // namespace jetson_rgbd_udp_sender
