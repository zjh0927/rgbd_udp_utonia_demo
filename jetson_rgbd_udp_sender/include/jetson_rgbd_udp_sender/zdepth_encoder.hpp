#pragma once

#include "jetson_rgbd_udp_sender/frame_types.hpp"

#include <string>
#include <vector>

#include <zdepth.hpp>

namespace jetson_rgbd_udp_sender
{

struct ZdepthEncodeResult
{
  bool ok{false};
  bool keyframe{true};
  std::vector<std::uint8_t> payload;
  std::string error;
};

class ZdepthEncoder
{
public:
  ZdepthEncoder();

  ZdepthEncodeResult encode(const DepthFrame & frame);

private:
  zdepth::DepthCompressor compressor_;
};

}  // namespace jetson_rgbd_udp_sender
