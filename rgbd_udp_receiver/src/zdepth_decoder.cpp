#include "rgbd_udp_receiver/zdepth_decoder.hpp"

#include <algorithm>
#include <limits>
#include <utility>

namespace rgbd_udp_receiver
{
namespace
{

constexpr std::size_t kHeaderBytes = 40U;
constexpr std::uint8_t kMagic = 0xcaU;
constexpr std::uint8_t kRequiredEncodeMode = 5U;

std::uint16_t read_u16_le(const std::uint8_t * data)
{
  return static_cast<std::uint16_t>(data[0]) |
         static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8U);
}

std::uint32_t read_u32_le(const std::uint8_t * data)
{
  return static_cast<std::uint32_t>(data[0]) |
         (static_cast<std::uint32_t>(data[1]) << 8U) |
         (static_cast<std::uint32_t>(data[2]) << 16U) |
         (static_cast<std::uint32_t>(data[3]) << 24U);
}

std::string bounded_error(const std::string & value)
{
  constexpr std::size_t kMaxErrorBytes = 240U;
  return value.substr(0, std::min(value.size(), kMaxErrorBytes));
}

}  // namespace

ZdepthDecoderNative::ZdepthDecoderNative(ZdepthDecodeLimits limits)
: limits_(std::move(limits))
{
}

ZdepthDecodeResult ZdepthDecoderNative::decode(const std::uint8_t * data, std::size_t size)
{
  ZdepthDecodeResult output;
  if (data == nullptr || size < kHeaderBytes) {
    output.error = "Zdepth frame is shorter than its 40-byte header";
    return output;
  }
  if (size > limits_.max_compressed_bytes) {
    output.error = "Zdepth frame exceeds compressed-byte limit";
    return output;
  }
  if (data[0] != kMagic) {
    output.error = "Zdepth magic is invalid";
    return output;
  }
  const bool keyframe = (data[1] & 1U) != 0U;
  const std::uint8_t encode_mode = static_cast<std::uint8_t>((data[1] >> 1U) & 0x0fU);
  if (!keyframe || encode_mode != kRequiredEncodeMode) {
    output.error = "Zdepth frame must be an all-keyframe not_quantized_8191mm frame";
    return output;
  }

  const std::uint32_t declared_width = read_u16_le(data + 4U);
  const std::uint32_t declared_height = read_u16_le(data + 6U);
  if (declared_width == 0U || declared_height == 0U ||
    declared_width % static_cast<std::uint32_t>(zdepth::kBlockSize) != 0U ||
    declared_height % static_cast<std::uint32_t>(zdepth::kBlockSize) != 0U)
  {
    output.error = "Zdepth dimensions must be nonzero multiples of 8";
    return output;
  }
  if (declared_width > limits_.max_width || declared_height > limits_.max_height) {
    output.error = "Zdepth dimensions exceed configured limits";
    return output;
  }
  if (declared_height > std::numeric_limits<std::size_t>::max() / declared_width) {
    output.error = "Zdepth pixel count overflows size_t";
    return output;
  }
  const std::size_t declared_pixels =
    static_cast<std::size_t>(declared_width) * declared_height;
  if (declared_pixels > limits_.max_pixels) {
    output.error = "Zdepth pixel count exceeds configured limit";
    return output;
  }

  const std::uint32_t zeroes_uncompressed = read_u32_le(data + 8U);
  const std::uint32_t zeroes_compressed = read_u32_le(data + 12U);
  const std::uint32_t blocks_uncompressed = read_u32_le(data + 16U);
  const std::uint32_t blocks_compressed = read_u32_le(data + 20U);
  const std::uint32_t edges_uncompressed = read_u32_le(data + 24U);
  const std::uint32_t edges_compressed = read_u32_le(data + 28U);
  const std::uint32_t surfaces_uncompressed = read_u32_le(data + 32U);
  const std::uint32_t surfaces_compressed = read_u32_le(data + 36U);
  const std::uint64_t declared_compressed =
    static_cast<std::uint64_t>(zeroes_compressed) + blocks_compressed +
    edges_compressed + surfaces_compressed;
  if (declared_compressed != size - kHeaderBytes) {
    output.error = "Zdepth component sizes do not match the compressed frame length";
    return output;
  }
  const std::size_t expected_zeroes = declared_pixels / 8U;
  const std::size_t blocks_x = declared_width / static_cast<std::uint32_t>(zdepth::kBlockSize);
  const std::size_t blocks_y = declared_height / static_cast<std::uint32_t>(zdepth::kBlockSize);
  const std::size_t expected_blocks = (blocks_x - 1U) * (blocks_y - 1U);
  const std::size_t max_packed_residual_bytes = (declared_pixels * 14U + 7U) / 8U;
  if (zeroes_uncompressed != expected_zeroes || blocks_uncompressed != expected_blocks ||
    edges_uncompressed > max_packed_residual_bytes ||
    surfaces_uncompressed > max_packed_residual_bytes)
  {
    output.error = "Zdepth declared uncompressed component sizes exceed image bounds";
    return output;
  }

  std::vector<std::uint8_t> compressed(data, data + size);
  int width = 0;
  int height = 0;
  std::vector<std::uint16_t> depth;
  const zdepth::DepthResult result = decoder_.Decompress(compressed, width, height, depth);
  if (result != zdepth::DepthResult::Success) {
    output.error = bounded_error(
      std::string("Zdepth decompression failed: ") + zdepth::DepthResultString(result));
    return output;
  }
  if (width != static_cast<int>(declared_width) || height != static_cast<int>(declared_height) ||
    depth.size() != declared_pixels)
  {
    output.error = "Zdepth decoder output dimensions do not match the validated header";
    return output;
  }

  output.ok = true;
  output.width = width;
  output.height = height;
  output.millimeters = std::move(depth);
  return output;
}

}  // namespace rgbd_udp_receiver
