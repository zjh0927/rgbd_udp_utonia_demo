#pragma once

#include "jetson_rgbd_udp_sender/frame_types.hpp"

#include <cstdint>
#include <optional>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace jetson_rgbd_udp_sender
{

enum class InputError
{
  kNone,
  kInvalidTimestamp,
  kInvalidDimensions,
  kDimensionMismatch,
  kUnsupportedColorEncoding,
  kUnsupportedDepthEncoding,
  kBigEndianDepth,
  kInvalidStep,
  kTruncatedData,
};

struct InputResult
{
  InputError error{InputError::kNone};
  std::string message;
  std::optional<FrameBundle> bundle;

  explicit operator bool() const noexcept
  {
    return error == InputError::kNone && bundle.has_value();
  }
};

bool ros_stamp_to_nanoseconds(
  const builtin_interfaces::msg::Time & stamp,
  std::uint64_t & nanoseconds,
  std::string & error);

InputResult make_frame_bundle(
  const sensor_msgs::msg::Image & color,
  const sensor_msgs::msg::Image & depth,
  std::uint64_t bundle_sequence,
  std::uint32_t source_id,
  std::uint32_t calibration_id);

}  // namespace jetson_rgbd_udp_sender
