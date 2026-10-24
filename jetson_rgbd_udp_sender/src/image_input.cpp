#include "jetson_rgbd_udp_sender/image_input.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <utility>

#include <sensor_msgs/image_encodings.hpp>

namespace jetson_rgbd_udp_sender
{
namespace
{

bool checked_image_bytes(
  std::uint32_t step,
  std::uint32_t height,
  std::size_t & bytes)
{
  if (height != 0U && step > std::numeric_limits<std::size_t>::max() / height) {
    return false;
  }
  bytes = static_cast<std::size_t>(step) * height;
  return true;
}

InputResult input_failure(InputError error, std::string message)
{
  InputResult result;
  result.error = error;
  result.message = std::move(message);
  return result;
}

}  // namespace

bool ros_stamp_to_nanoseconds(
  const builtin_interfaces::msg::Time & stamp,
  std::uint64_t & nanoseconds,
  std::string & error)
{
  constexpr std::uint64_t kNanosecondsPerSecond = 1000000000ULL;
  if (stamp.sec < 0) {
    error = "negative ROS timestamps are not supported";
    return false;
  }
  if (stamp.nanosec >= kNanosecondsPerSecond) {
    error = "ROS timestamp nanosec field is outside 0..999999999";
    return false;
  }

  nanoseconds = static_cast<std::uint64_t>(stamp.sec) * kNanosecondsPerSecond +
    static_cast<std::uint64_t>(stamp.nanosec);
  error.clear();
  return true;
}

InputResult make_frame_bundle(
  const sensor_msgs::msg::Image & color,
  const sensor_msgs::msg::Image & depth,
  std::uint64_t bundle_sequence,
  std::uint32_t source_id,
  std::uint32_t calibration_id)
{
  if (color.width == 0U || color.height == 0U ||
    color.width % 8U != 0U || color.height % 8U != 0U)
  {
    return input_failure(
      InputError::kInvalidDimensions,
      "image width and height must be nonzero multiples of 8");
  }
  if (color.width != depth.width || color.height != depth.height) {
    return input_failure(
      InputError::kDimensionMismatch,
      "aligned depth dimensions do not match the color image");
  }

  PixelFormat pixel_format;
  if (color.encoding == sensor_msgs::image_encodings::RGB8) {
    pixel_format = PixelFormat::kRgb8;
  } else if (color.encoding == sensor_msgs::image_encodings::BGR8) {
    pixel_format = PixelFormat::kBgr8;
  } else {
    return input_failure(
      InputError::kUnsupportedColorEncoding,
      "color encoding must be rgb8 or bgr8, got: " + color.encoding);
  }

  if (depth.encoding != sensor_msgs::image_encodings::TYPE_16UC1) {
    return input_failure(
      InputError::kUnsupportedDepthEncoding,
      "depth encoding must be 16UC1, got: " + depth.encoding);
  }
  if (depth.is_bigendian != 0U) {
    return input_failure(
      InputError::kBigEndianDepth,
      "big-endian 16UC1 depth is not supported");
  }

  const std::size_t color_row_bytes = static_cast<std::size_t>(color.width) * 3U;
  const std::size_t depth_row_bytes = static_cast<std::size_t>(depth.width) * 2U;
  if (color.step < color_row_bytes || depth.step < depth_row_bytes) {
    return input_failure(
      InputError::kInvalidStep,
      "ROS image step is shorter than one packed row");
  }

  std::size_t required_color_bytes = 0;
  std::size_t required_depth_bytes = 0;
  if (!checked_image_bytes(color.step, color.height, required_color_bytes) ||
    !checked_image_bytes(depth.step, depth.height, required_depth_bytes) ||
    color.data.size() < required_color_bytes || depth.data.size() < required_depth_bytes)
  {
    return input_failure(
      InputError::kTruncatedData,
      "ROS image data is shorter than step * height");
  }

  std::uint64_t rgb_timestamp_ns = 0;
  std::uint64_t depth_timestamp_ns = 0;
  std::string timestamp_error;
  if (!ros_stamp_to_nanoseconds(color.header.stamp, rgb_timestamp_ns, timestamp_error) ||
    !ros_stamp_to_nanoseconds(depth.header.stamp, depth_timestamp_ns, timestamp_error))
  {
    return input_failure(InputError::kInvalidTimestamp, timestamp_error);
  }

  FrameBundle bundle;
  bundle.metadata.source_id = source_id;
  bundle.metadata.calibration_id = calibration_id;
  bundle.metadata.bundle_sequence = bundle_sequence;
  bundle.metadata.rgb_timestamp_ns = rgb_timestamp_ns;
  bundle.metadata.depth_timestamp_ns = depth_timestamp_ns;

  bundle.color.width = color.width;
  bundle.color.height = color.height;
  bundle.color.format = pixel_format;
  bundle.color.pixels.resize(color_row_bytes * color.height);
  for (std::uint32_t row = 0; row < color.height; ++row) {
    std::memcpy(
      bundle.color.pixels.data() + static_cast<std::size_t>(row) * color_row_bytes,
      color.data.data() + static_cast<std::size_t>(row) * color.step,
      color_row_bytes);
  }

  bundle.depth.width = depth.width;
  bundle.depth.height = depth.height;
  bundle.depth.millimeters.resize(
    static_cast<std::size_t>(depth.width) * depth.height);
  for (std::uint32_t row = 0; row < depth.height; ++row) {
    const std::uint8_t * source =
      depth.data.data() + static_cast<std::size_t>(row) * depth.step;
    std::uint16_t * destination =
      bundle.depth.millimeters.data() + static_cast<std::size_t>(row) * depth.width;
    for (std::uint32_t column = 0; column < depth.width; ++column) {
      destination[column] = static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(source[column * 2U]) |
        (static_cast<std::uint16_t>(source[column * 2U + 1U]) << 8U));
    }
  }

  InputResult result;
  result.bundle = std::move(bundle);
  return result;
}

}  // namespace jetson_rgbd_udp_sender
