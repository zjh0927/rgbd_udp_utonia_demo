#include "jetson_rgbd_udp_sender/udp_protocol.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <utility>

#include <zlib.h>

namespace jetson_rgbd_udp_sender
{
namespace
{

void write_u16(std::uint8_t * output, std::uint16_t value)
{
  output[0] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
  output[1] = static_cast<std::uint8_t>(value & 0xffU);
}

void write_u32(std::uint8_t * output, std::uint32_t value)
{
  output[0] = static_cast<std::uint8_t>((value >> 24U) & 0xffU);
  output[1] = static_cast<std::uint8_t>((value >> 16U) & 0xffU);
  output[2] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
  output[3] = static_cast<std::uint8_t>(value & 0xffU);
}

void write_u64(std::uint8_t * output, std::uint64_t value)
{
  write_u32(output, static_cast<std::uint32_t>(value >> 32U));
  write_u32(output + 4, static_cast<std::uint32_t>(value & 0xffffffffULL));
}

std::uint16_t read_u16(const std::uint8_t * input)
{
  return static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(input[0]) << 8U) |
    static_cast<std::uint16_t>(input[1]));
}

std::uint32_t read_u32(const std::uint8_t * input)
{
  return
    (static_cast<std::uint32_t>(input[0]) << 24U) |
    (static_cast<std::uint32_t>(input[1]) << 16U) |
    (static_cast<std::uint32_t>(input[2]) << 8U) |
    static_cast<std::uint32_t>(input[3]);
}

std::uint64_t read_u64(const std::uint8_t * input)
{
  return
    (static_cast<std::uint64_t>(read_u32(input)) << 32U) |
    static_cast<std::uint64_t>(read_u32(input + 4));
}

bool valid_stream_type(StreamType type)
{
  return type == StreamType::kH265 ||
         type == StreamType::kZdepth ||
         type == StreamType::kCalibration;
}

}  // namespace

std::array<std::uint8_t, kPacketHeaderBytes> serialize_header(const PacketHeader & header)
{
  std::array<std::uint8_t, kPacketHeaderBytes> bytes{};
  write_u32(bytes.data() + 0, header.magic);
  bytes[4] = header.version;
  bytes[5] = static_cast<std::uint8_t>(header.stream_type);
  write_u16(bytes.data() + 6, header.flags);
  write_u32(bytes.data() + 8, header.source_id);
  write_u32(bytes.data() + 12, header.calibration_id);
  write_u64(bytes.data() + 16, header.bundle_sequence);
  write_u64(bytes.data() + 24, header.source_timestamp_ns);
  write_u64(bytes.data() + 32, header.send_timestamp_ns);
  write_u32(bytes.data() + 40, header.frame_size);
  write_u32(bytes.data() + 44, header.frame_crc32);
  write_u16(bytes.data() + 48, header.fragment_index);
  write_u16(bytes.data() + 50, header.fragment_count);
  write_u16(bytes.data() + 52, header.payload_size);
  write_u16(bytes.data() + 54, header.reserved);
  return bytes;
}

bool parse_header(
  const std::uint8_t * data,
  std::size_t size,
  PacketHeader & header,
  std::string & error)
{
  if (data == nullptr || size < kPacketHeaderBytes) {
    error = "datagram is shorter than the 56-byte header";
    return false;
  }

  header.magic = read_u32(data + 0);
  header.version = data[4];
  header.stream_type = static_cast<StreamType>(data[5]);
  header.flags = read_u16(data + 6);
  header.source_id = read_u32(data + 8);
  header.calibration_id = read_u32(data + 12);
  header.bundle_sequence = read_u64(data + 16);
  header.source_timestamp_ns = read_u64(data + 24);
  header.send_timestamp_ns = read_u64(data + 32);
  header.frame_size = read_u32(data + 40);
  header.frame_crc32 = read_u32(data + 44);
  header.fragment_index = read_u16(data + 48);
  header.fragment_count = read_u16(data + 50);
  header.payload_size = read_u16(data + 52);
  header.reserved = read_u16(data + 54);

  if (header.magic != kPacketMagic) {
    error = "wrong packet magic";
    return false;
  }
  if (header.version != kProtocolVersion) {
    error = "unsupported protocol version";
    return false;
  }
  if (!valid_stream_type(header.stream_type)) {
    error = "invalid stream type";
    return false;
  }
  if (header.reserved != 0U) {
    error = "reserved header field is not zero";
    return false;
  }
  if (header.fragment_count == 0U || header.fragment_index >= header.fragment_count) {
    error = "invalid fragment index/count";
    return false;
  }
  if (header.payload_size != size - kPacketHeaderBytes) {
    error = "payload size does not match datagram size";
    return false;
  }

  error.clear();
  return true;
}

std::uint32_t frame_crc32(const std::vector<std::uint8_t> & payload)
{
  uLong checksum = crc32(0L, Z_NULL, 0);
  std::size_t offset = 0;
  while (offset < payload.size()) {
    const std::size_t remaining = payload.size() - offset;
    const uInt chunk = static_cast<uInt>(std::min<std::size_t>(
      remaining, std::numeric_limits<uInt>::max()));
    checksum = crc32(checksum, payload.data() + offset, chunk);
    offset += chunk;
  }
  return static_cast<std::uint32_t>(checksum);
}

FragmentResult fragment_frame(
  const EncodedFrame & frame,
  std::size_t datagram_bytes,
  std::uint64_t send_timestamp_ns)
{
  FragmentResult result;
  if (!valid_stream_type(frame.stream_type)) {
    result.error = "invalid stream type";
    return result;
  }
  if (frame.payload.empty()) {
    result.error = "compressed frame payload is empty";
    return result;
  }
  if (frame.payload.size() > std::numeric_limits<std::uint32_t>::max()) {
    result.error = "compressed frame is larger than protocol uint32 frame_size";
    return result;
  }
  if (datagram_bytes <= kPacketHeaderBytes || datagram_bytes > kMaximumUdpDatagramBytes) {
    result.error = "UDP datagram size is outside protocol limits";
    return result;
  }

  const std::size_t payload_capacity = datagram_bytes - kPacketHeaderBytes;
  const std::size_t fragment_count =
    1U + ((frame.payload.size() - 1U) / payload_capacity);
  if (fragment_count > std::numeric_limits<std::uint16_t>::max()) {
    result.error = "compressed frame needs more than 65535 fragments";
    return result;
  }

  const std::uint32_t checksum = frame_crc32(frame.payload);
  result.datagrams.reserve(fragment_count);
  for (std::size_t index = 0; index < fragment_count; ++index) {
    const std::size_t offset = index * payload_capacity;
    const std::size_t payload_size = std::min(payload_capacity, frame.payload.size() - offset);

    PacketHeader header;
    header.stream_type = frame.stream_type;
    header.flags = frame.flags;
    header.source_id = frame.source_id;
    header.calibration_id = frame.calibration_id;
    header.bundle_sequence = frame.bundle_sequence;
    header.source_timestamp_ns = frame.source_timestamp_ns;
    header.send_timestamp_ns = send_timestamp_ns;
    header.frame_size = static_cast<std::uint32_t>(frame.payload.size());
    header.frame_crc32 = checksum;
    header.fragment_index = static_cast<std::uint16_t>(index);
    header.fragment_count = static_cast<std::uint16_t>(fragment_count);
    header.payload_size = static_cast<std::uint16_t>(payload_size);

    Datagram datagram;
    datagram.header = header;
    datagram.bytes.resize(kPacketHeaderBytes + payload_size);
    const auto header_bytes = serialize_header(header);
    std::memcpy(datagram.bytes.data(), header_bytes.data(), header_bytes.size());
    std::memcpy(
      datagram.bytes.data() + kPacketHeaderBytes,
      frame.payload.data() + offset,
      payload_size);
    result.datagrams.emplace_back(std::move(datagram));
  }

  result.ok = true;
  return result;
}

}  // namespace jetson_rgbd_udp_sender
