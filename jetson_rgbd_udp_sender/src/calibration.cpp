#include "jetson_rgbd_udp_sender/calibration.hpp"

#include "jetson_rgbd_udp_sender/image_input.hpp"
#include "jetson_rgbd_udp_sender/udp_protocol.hpp"

#include <stdexcept>
#include <vector>

#include <nlohmann/json.hpp>

namespace jetson_rgbd_udp_sender
{

CalibrationState::CalibrationState(
  double depth_scale_m,
  std::uint32_t h265_bitrate,
  std::uint32_t h265_gop,
  std::uint32_t h265_fps)
: depth_scale_m_(depth_scale_m),
  h265_bitrate_(h265_bitrate),
  h265_gop_(h265_gop),
  h265_fps_(h265_fps)
{
  if (!(depth_scale_m_ > 0.0)) {
    throw std::invalid_argument("depth_scale_m must be positive");
  }
}

CalibrationUpdate CalibrationState::update(
  const sensor_msgs::msg::CameraInfo & camera_info,
  const std::string & color_encoding,
  const std::string & depth_encoding,
  const std::string & depth_aligned_frame)
{
  nlohmann::json contents;
  contents["width"] = camera_info.width;
  contents["height"] = camera_info.height;
  contents["color_encoding"] = color_encoding;
  contents["depth_encoding"] = depth_encoding;
  contents["distortion_model"] = camera_info.distortion_model;
  contents["d"] = camera_info.d;
  contents["k"] = std::vector<double>(camera_info.k.begin(), camera_info.k.end());
  contents["r"] = std::vector<double>(camera_info.r.begin(), camera_info.r.end());
  contents["p"] = std::vector<double>(camera_info.p.begin(), camera_info.p.end());
  contents["binning_x"] = camera_info.binning_x;
  contents["binning_y"] = camera_info.binning_y;
  contents["roi"] = {
    {"x_offset", camera_info.roi.x_offset},
    {"y_offset", camera_info.roi.y_offset},
    {"height", camera_info.roi.height},
    {"width", camera_info.roi.width},
    {"do_rectify", camera_info.roi.do_rectify}};
  contents["depth_scale_m"] = depth_scale_m_;
  contents["color_optical_frame"] = camera_info.header.frame_id;
  contents["depth_aligned_frame"] = depth_aligned_frame;
  contents["zdepth"] = {
    {"encode_mode", "not_quantized_8191mm"},
    {"all_keyframes", true}};
  contents["h265"] = {
    {"stream_format", "byte-stream"},
    {"alignment", "au"},
    {"bitrate", h265_bitrate_},
    {"gop", h265_gop_},
    {"idr_interval", h265_gop_},
    {"fps", h265_fps_},
    {"rate_control", "cbr_if_supported"},
    {"b_frames", 0}};

  const std::string fingerprint = contents.dump();
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_ && fingerprint == fingerprint_) {
    return CalibrationUpdate{false, *current_};
  }

  std::uint64_t timestamp_ns = 0;
  std::string timestamp_error;
  if (!ros_stamp_to_nanoseconds(camera_info.header.stamp, timestamp_ns, timestamp_error)) {
    timestamp_ns = 0;
  }

  if (next_calibration_id_ == 0U) {
    throw std::overflow_error("calibration_id exhausted");
  }

  CalibrationRecord record;
  record.calibration_id = next_calibration_id_++;
  record.source_timestamp_ns = timestamp_ns;

  contents["type"] = "rgbd_calibration";
  contents["protocol_version"] = kProtocolVersion;
  contents["calibration_id"] = record.calibration_id;
  record.json = contents.dump();

  fingerprint_ = fingerprint;
  current_ = record;
  return CalibrationUpdate{true, record};
}

std::optional<CalibrationRecord> CalibrationState::current() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_;
}

}  // namespace jetson_rgbd_udp_sender
