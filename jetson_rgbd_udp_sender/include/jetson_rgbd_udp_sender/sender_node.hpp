#pragma once

#include "jetson_rgbd_udp_sender/calibration.hpp"
#include "jetson_rgbd_udp_sender/frame_types.hpp"
#include "jetson_rgbd_udp_sender/h265_encoder.hpp"
#include "jetson_rgbd_udp_sender/latest_slot.hpp"
#include "jetson_rgbd_udp_sender/udp_sender.hpp"
#include "jetson_rgbd_udp_sender/zdepth_encoder.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace jetson_rgbd_udp_sender
{

struct SenderConfig
{
  std::string rgb_topic{"/camera/camera/color/image_raw"};
  std::string depth_topic{"/camera/camera/aligned_depth_to_color/image_raw"};
  std::string camera_info_topic{"/camera/camera/color/camera_info"};
  std::string receiver_ip{"192.168.1.107"};
  std::int64_t h265_port{5000};
  std::int64_t depth_port{5001};
  std::int64_t calibration_port{5002};
  std::int64_t source_id{1};
  std::int64_t sync_slop_ms{10};
  std::int64_t h265_bitrate{3000000};
  std::int64_t h265_gop{10};
  std::int64_t h265_fps{30};
  std::int64_t expected_width{640};
  std::int64_t expected_height{480};
  std::int64_t udp_datagram_bytes{1200};
  std::int64_t socket_send_buffer_bytes{4 * 1024 * 1024};
  std::int64_t max_frame_bytes{16 * 1024 * 1024};
  std::int64_t calibration_period_ms{1000};
  std::int64_t input_timeout_ms{2000};
  double depth_scale_m{0.001};

  std::optional<std::string> validation_error() const;
};

class SenderNode : public rclcpp::Node
{
public:
  SenderNode();
  ~SenderNode() override;

  SenderNode(const SenderNode &) = delete;
  SenderNode & operator=(const SenderNode &) = delete;

  void shutdown();

private:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  struct RuntimeCounters
  {
    std::atomic<std::uint64_t> synchronized{0};
    std::atomic<std::uint64_t> input_rejected{0};
    std::atomic<std::uint64_t> input_overwritten{0};
    std::atomic<std::uint64_t> zdepth_encoded{0};
    std::atomic<std::uint64_t> zdepth_encode_errors{0};
    std::atomic<std::uint64_t> h265_accepted{0};
    std::atomic<std::uint64_t> h265_encoded{0};
    std::atomic<std::uint64_t> h265_errors{0};
    std::atomic<std::uint64_t> h265_send_overwritten{0};
    std::atomic<std::uint64_t> depth_send_overwritten{0};
    std::atomic<std::uint64_t> udp_errors{0};
    std::atomic<std::uint64_t> zdepth_encode_microseconds{0};
  };

  struct MetricsSnapshot
  {
    std::uint64_t synchronized{0};
    std::uint64_t h265_encoded{0};
    std::uint64_t zdepth_encoded{0};
    std::uint64_t h265_bytes{0};
    std::uint64_t depth_bytes{0};
    std::uint64_t calibration_bytes{0};
  };

  SenderConfig read_config();
  void configure_ros_inputs();
  void synchronized_callback(
    const Image::ConstSharedPtr & color,
    const Image::ConstSharedPtr & depth);
  void camera_info_callback(const CameraInfo::ConstSharedPtr message);
  void enqueue_calibration(const CalibrationRecord & record);
  void calibration_timer_callback();
  void metrics_timer_callback();

  void compression_loop();
  void h265_output_loop();
  void udp_loop(
    const char * stream_name,
    LatestSlot<EncodedFrame> & slot,
    UdpSender & sender);

  SenderConfig config_;
  std::atomic<bool> running_{false};
  std::atomic<bool> shutdown_started_{false};
  std::atomic<bool> h265_failed_{false};
  std::atomic<bool> h265_eos_requested_{false};
  std::atomic<std::uint64_t> next_bundle_sequence_{1};
  std::atomic<std::int64_t> last_input_steady_ns_{0};

  message_filters::Subscriber<Image> rgb_subscriber_;
  message_filters::Subscriber<Image> depth_subscriber_;
  std::unique_ptr<Synchronizer> synchronizer_;
  rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_subscriber_;
  rclcpp::TimerBase::SharedPtr calibration_timer_;
  rclcpp::TimerBase::SharedPtr metrics_timer_;

  mutable std::mutex camera_info_mutex_;
  CameraInfo::ConstSharedPtr latest_camera_info_;

  std::unique_ptr<CalibrationState> calibration_state_;
  ZdepthEncoder zdepth_encoder_;
  std::unique_ptr<H265Encoder> h265_encoder_;
  std::unique_ptr<UdpSender> h265_udp_sender_;
  std::unique_ptr<UdpSender> depth_udp_sender_;
  std::unique_ptr<UdpSender> calibration_udp_sender_;

  LatestSlot<FrameBundle> bundle_slot_;
  LatestSlot<EncodedFrame> h265_send_slot_;
  LatestSlot<EncodedFrame> depth_send_slot_;
  LatestSlot<EncodedFrame> calibration_send_slot_;

  std::thread compression_thread_;
  std::thread h265_output_thread_;
  std::thread h265_udp_thread_;
  std::thread depth_udp_thread_;
  std::thread calibration_udp_thread_;

  RuntimeCounters counters_;
  MetricsSnapshot previous_metrics_;
};

}  // namespace jetson_rgbd_udp_sender
