#include "jetson_rgbd_udp_sender/udp_sender.hpp"

#include "jetson_rgbd_udp_sender/udp_protocol.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

#include <fcntl.h>
#include <netdb.h>
#include <unistd.h>

namespace jetson_rgbd_udp_sender
{
namespace
{

std::uint64_t system_time_ns()
{
  return static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
}

std::string socket_error(const char * operation)
{
  return std::string(operation) + ": " + std::strerror(errno);
}

}  // namespace

UdpSender::UdpSender(const UdpSenderConfig & config)
: config_(config)
{
  if (config_.destination.empty()) {
    throw std::invalid_argument("UDP destination is empty");
  }
  if (config_.port == 0U) {
    throw std::invalid_argument("UDP port must be in 1..65535");
  }
  if (config_.datagram_bytes <= kPacketHeaderBytes ||
    config_.datagram_bytes > kMaximumUdpDatagramBytes)
  {
    throw std::invalid_argument("UDP datagram size is outside protocol limits");
  }
  if (config_.max_frame_bytes == 0U ||
    config_.max_frame_bytes > std::numeric_limits<std::uint32_t>::max())
  {
    throw std::invalid_argument("max_frame_bytes is outside protocol limits");
  }

  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_protocol = IPPROTO_UDP;

  addrinfo * raw_results = nullptr;
  const std::string service = std::to_string(config_.port);
  const int lookup_result = getaddrinfo(
    config_.destination.c_str(), service.c_str(), &hints, &raw_results);
  if (lookup_result != 0) {
    throw std::runtime_error(
            "cannot resolve UDP destination " + config_.destination + ": " +
            gai_strerror(lookup_result));
  }
  std::unique_ptr<addrinfo, decltype(&freeaddrinfo)> results(raw_results, freeaddrinfo);

  std::string last_error = "no usable address";
  for (addrinfo * item = results.get(); item != nullptr; item = item->ai_next) {
    const int fd = socket(item->ai_family, item->ai_socktype, item->ai_protocol);
    if (fd < 0) {
      last_error = socket_error("socket");
      continue;
    }

    const int requested_buffer = static_cast<int>(std::min<std::size_t>(
      config_.socket_send_buffer_bytes,
      static_cast<std::size_t>(std::numeric_limits<int>::max())));
    if (setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &requested_buffer, sizeof(requested_buffer)) != 0) {
      last_error = socket_error("setsockopt(SO_SNDBUF)");
      close(fd);
      continue;
    }

    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) != 0) {
      last_error = socket_error("fcntl(O_NONBLOCK)");
      close(fd);
      continue;
    }

    if (item->ai_addrlen > sizeof(destination_address_)) {
      last_error = "resolved destination address is too large";
      close(fd);
      continue;
    }

    socket_fd_ = fd;
    destination_address_length_ = static_cast<socklen_t>(item->ai_addrlen);
    std::memcpy(&destination_address_, item->ai_addr, item->ai_addrlen);
    break;
  }

  if (socket_fd_ < 0) {
    throw std::runtime_error(
            "cannot create UDP sender for " + config_.destination + ":" + service +
            ": " + last_error);
  }
}

UdpSender::~UdpSender()
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

SendResult UdpSender::send_frame(const EncodedFrame & frame)
{
  SendResult result;
  if (frame.payload.empty()) {
    result.error = "refusing to send an empty compressed frame";
    ++frames_dropped_;
    return result;
  }
  if (frame.payload.size() > config_.max_frame_bytes) {
    result.code = SendCode::kFrameTooLarge;
    result.error = "compressed frame exceeds max_frame_bytes";
    ++frames_dropped_;
    return result;
  }

  auto fragments = fragment_frame(frame, config_.datagram_bytes, system_time_ns());
  if (!fragments.ok) {
    result.code = SendCode::kFragmentationError;
    result.error = std::move(fragments.error);
    ++frames_dropped_;
    return result;
  }

  for (const auto & datagram : fragments.datagrams) {
    const ssize_t sent = sendto(
      socket_fd_,
      datagram.bytes.data(),
      datagram.bytes.size(),
      MSG_DONTWAIT,
      reinterpret_cast<const sockaddr *>(&destination_address_),
      destination_address_length_);

    if (sent < 0 || static_cast<std::size_t>(sent) != datagram.bytes.size()) {
      result.code = SendCode::kSocketError;
      result.error = sent < 0 ? socket_error("sendto") : "sendto returned a short datagram write";
      ++frames_dropped_;
      return result;
    }

    ++result.datagrams_sent;
    result.bytes_sent += static_cast<std::size_t>(sent);
    ++datagrams_sent_;
    bytes_sent_.fetch_add(static_cast<std::size_t>(sent));
  }

  result.code = SendCode::kSuccess;
  ++frames_sent_;
  return result;
}

UdpSenderStats UdpSender::stats() const noexcept
{
  return UdpSenderStats{
    frames_sent_.load(),
    frames_dropped_.load(),
    datagrams_sent_.load(),
    bytes_sent_.load()};
}

}  // namespace jetson_rgbd_udp_sender
