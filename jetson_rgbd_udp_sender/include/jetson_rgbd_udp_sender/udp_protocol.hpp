#pragma once

#include "jetson_rgbd_udp_sender/frame_types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace jetson_rgbd_udp_sender
{

constexpr std::uint32_t kPacketMagic = 0x52474244U;
constexpr std::uint8_t kProtocolVersion = 1U;
constexpr std::size_t kPacketHeaderBytes = 56U;
constexpr std::size_t kMaximumUdpDatagramBytes = 65507U;

struct PacketHeader
{
  std::uint32_t magic{kPacketMagic};
  std::uint8_t version{kProtocolVersion};
  StreamType stream_type{StreamType::kH265};
  std::uint16_t flags{kFlagNone};
  std::uint32_t source_id{1};
  std::uint32_t calibration_id{0};
  std::uint64_t bundle_sequence{0};
  std::uint64_t source_timestamp_ns{0};
  std::uint64_t send_timestamp_ns{0};
  std::uint32_t frame_size{0};
  std::uint32_t frame_crc32{0};
  std::uint16_t fragment_index{0};
  std::uint16_t fragment_count{0};
  std::uint16_t payload_size{0};
  std::uint16_t reserved{0};
};

struct Datagram
{
  PacketHeader header;
  std::vector<std::uint8_t> bytes;
};

struct FragmentResult
{
  bool ok{false};
  std::vector<Datagram> datagrams;
  std::string error;
};

std::array<std::uint8_t, kPacketHeaderBytes> serialize_header(const PacketHeader & header);

bool parse_header(
  const std::uint8_t * data,
  std::size_t size,
  PacketHeader & header,
  std::string & error);

std::uint32_t frame_crc32(const std::vector<std::uint8_t> & payload);

FragmentResult fragment_frame(
  const EncodedFrame & frame,
  std::size_t datagram_bytes,
  std::uint64_t send_timestamp_ns);

}  // namespace jetson_rgbd_udp_sender
