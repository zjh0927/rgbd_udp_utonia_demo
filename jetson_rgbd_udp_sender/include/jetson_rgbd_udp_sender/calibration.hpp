#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include <sensor_msgs/msg/camera_info.hpp>

namespace jetson_rgbd_udp_sender
{

struct CalibrationRecord
{
  std::uint32_t calibration_id{0};
  std::uint64_t source_timestamp_ns{0};
  std::string json;
};

struct CalibrationUpdate
{
  bool changed{false};
  CalibrationRecord record;
};

class CalibrationState
{
public:
  CalibrationState(
    double depth_scale_m,
    std::uint32_t h265_bitrate,
    std::uint32_t h265_gop,
    std::uint32_t h265_fps);

  CalibrationUpdate update(
    const sensor_msgs::msg::CameraInfo & camera_info,
    const std::string & color_encoding,
    const std::string & depth_encoding,
    const std::string & depth_aligned_frame);

  std::optional<CalibrationRecord> current() const;

private:
  double depth_scale_m_{0.001};
  std::uint32_t h265_bitrate_{5000000};
  std::uint32_t h265_gop_{10};
  std::uint32_t h265_fps_{30};
  mutable std::mutex mutex_;
  std::string fingerprint_;
  std::optional<CalibrationRecord> current_;
  std::uint32_t next_calibration_id_{1};
};

}  // namespace jetson_rgbd_udp_sender
