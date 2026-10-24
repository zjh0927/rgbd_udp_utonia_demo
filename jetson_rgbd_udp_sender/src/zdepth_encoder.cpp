#include "jetson_rgbd_udp_sender/zdepth_encoder.hpp"

#include <limits>

namespace jetson_rgbd_udp_sender
{

ZdepthEncoder::ZdepthEncoder()
{
  compressor_.set_encode_mode(zdepth::EncodeMode::kNotQuantized8191mm);
}

ZdepthEncodeResult ZdepthEncoder::encode(const DepthFrame & frame)
{
  ZdepthEncodeResult result;
  if (frame.width == 0U || frame.height == 0U ||
    frame.width % zdepth::kBlockSize != 0U ||
    frame.height % zdepth::kBlockSize != 0U)
  {
    result.error = "Zdepth dimensions must be nonzero multiples of 8";
    return result;
  }

  const std::size_t expected_values =
    static_cast<std::size_t>(frame.width) * frame.height;
  if (frame.millimeters.size() != expected_values) {
    result.error = "depth buffer length does not match width * height";
    return result;
  }
  if (frame.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
    frame.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()))
  {
    result.error = "depth dimensions exceed Zdepth int range";
    return result;
  }

  const zdepth::DepthResult depth_result = compressor_.Compress(
    static_cast<int>(frame.width),
    static_cast<int>(frame.height),
    frame.millimeters.data(),
    result.payload,
    true);
  if (depth_result != zdepth::DepthResult::Success) {
    result.payload.clear();
    result.error = std::string("Zdepth compression failed: ") +
      zdepth::DepthResultString(depth_result);
    return result;
  }
  if (result.payload.empty()) {
    result.error = "Zdepth returned an empty frame";
    return result;
  }

  result.ok = true;
  result.keyframe = true;
  return result;
}

}  // namespace jetson_rgbd_udp_sender
