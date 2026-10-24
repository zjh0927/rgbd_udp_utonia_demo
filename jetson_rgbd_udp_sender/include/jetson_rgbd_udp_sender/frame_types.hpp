#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace jetson_rgbd_udp_sender
{

enum class StreamType : std::uint8_t
{
  kH265 = 1,
  kZdepth = 2,
  kCalibration = 3,
};

enum FrameFlags : std::uint16_t
{
  kFlagNone = 0,
  kFlagKeyframe = 1U << 0U,
  kFlagConfig = 1U << 1U,
};

enum class PixelFormat
{
  kRgb8,
  kBgr8,
};

struct BundleMetadata
{
  std::uint32_t source_id{1};
  std::uint32_t calibration_id{0};
  std::uint64_t bundle_sequence{0};
  std::uint64_t rgb_timestamp_ns{0};
  std::uint64_t depth_timestamp_ns{0};
};

struct ColorFrame
{
  std::uint32_t width{0};
  std::uint32_t height{0};
  PixelFormat format{PixelFormat::kRgb8};
  std::vector<std::uint8_t> pixels;
};

struct DepthFrame
{
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::vector<std::uint16_t> millimeters;
};

struct FrameBundle
{
  BundleMetadata metadata;
  ColorFrame color;
  DepthFrame depth;
};

struct EncodedFrame
{
  StreamType stream_type{StreamType::kH265};
  std::uint16_t flags{kFlagNone};
  std::uint32_t source_id{1};
  std::uint32_t calibration_id{0};
  std::uint64_t bundle_sequence{0};
  std::uint64_t source_timestamp_ns{0};
  std::vector<std::uint8_t> payload;
};

enum class SendCode
{
  kSuccess,
  kInvalidFrame,
  kFrameTooLarge,
  kFragmentationError,
  kSocketError,
};

struct SendResult
{
  SendCode code{SendCode::kInvalidFrame};
  std::size_t datagrams_sent{0};
  std::size_t bytes_sent{0};
  std::string error;

  explicit operator bool() const noexcept
  {
    return code == SendCode::kSuccess;
  }
};

}  // namespace jetson_rgbd_udp_sender
