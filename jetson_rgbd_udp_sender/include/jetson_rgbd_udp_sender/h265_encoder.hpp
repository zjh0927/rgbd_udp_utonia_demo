#pragma once

#include "jetson_rgbd_udp_sender/frame_types.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>

namespace jetson_rgbd_udp_sender
{

struct H265Config
{
  std::uint32_t width{640};
  std::uint32_t height{480};
  std::uint32_t fps{30};
  std::uint32_t bitrate{5000000};
  std::uint32_t gop{10};
  PixelFormat input_format{PixelFormat::kRgb8};
};

enum class H265PullStatus
{
  kSample,
  kTimeout,
  kEos,
  kError,
  kNotStarted,
};

struct H265Output
{
  H265PullStatus status{H265PullStatus::kNotStarted};
  BundleMetadata metadata;
  bool keyframe{false};
  std::vector<std::uint8_t> payload;
  std::string error;
};

class H265Encoder
{
public:
  explicit H265Encoder(H265Config config);
  ~H265Encoder();

  H265Encoder(const H265Encoder &) = delete;
  H265Encoder & operator=(const H265Encoder &) = delete;

  static bool probe_required_elements(std::string & error);
  static std::string pipeline_summary(const H265Config & config);

  bool start(std::string & error);
  bool push(
    const ColorFrame & frame,
    const BundleMetadata & metadata,
    std::string & error);
  H265Output pull(std::chrono::milliseconds timeout);
  bool end_of_stream(std::string & error);
  void stop();
  bool started() const noexcept;

private:
  static constexpr std::size_t kMaxPendingMetadata = 32U;

  bool poll_bus(H265Output & output);
  void remove_pending_if_last(std::uint64_t bundle_sequence);

  H265Config config_;
  GstElement * pipeline_{nullptr};
  GstAppSrc * appsrc_{nullptr};
  GstAppSink * appsink_{nullptr};
  GstBus * bus_{nullptr};
  bool started_{false};

  mutable std::mutex metadata_mutex_;
  std::deque<BundleMetadata> pending_metadata_;
  std::mutex push_mutex_;
};

}  // namespace jetson_rgbd_udp_sender
