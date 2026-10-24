#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <zdepth.hpp>

namespace rgbd_udp_receiver
{

struct ZdepthDecodeLimits
{
  std::uint32_t max_width{4096};
  std::uint32_t max_height{4096};
  std::size_t max_pixels{16U * 1024U * 1024U};
  std::size_t max_compressed_bytes{16U * 1024U * 1024U};
};

struct ZdepthDecodeResult
{
  bool ok{false};
  int width{0};
  int height{0};
  std::vector<std::uint16_t> millimeters;
  std::string error;
};

class ZdepthDecoderNative
{
public:
  explicit ZdepthDecoderNative(ZdepthDecodeLimits limits);

  ZdepthDecodeResult decode(const std::uint8_t * data, std::size_t size);

private:
  ZdepthDecodeLimits limits_;
  zdepth::DepthCompressor decoder_;
};

}  // namespace rgbd_udp_receiver
