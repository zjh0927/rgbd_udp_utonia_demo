#include "jetson_rgbd_udp_sender/sender_node.hpp"

#include "jetson_rgbd_udp_sender/image_input.hpp"
#include "jetson_rgbd_udp_sender/udp_protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <rmw/qos_profiles.h>
#include <sensor_msgs/image_encodings.hpp>

namespace jetson_rgbd_udp_sender
{
namespace
{

template<typename T>
bool outside(T value, T minimum, T maximum)
{
  return value < minimum || value > maximum;
}

std::int64_t steady_time_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string pixel_format_name(PixelFormat format)
{
  return format == PixelFormat::kRgb8 ? sensor_msgs::image_encodings::RGB8 :
         sensor_msgs::image_encodings::BGR8;
}

}  // namespace

std::optional<std::string> SenderConfig::validation_error() const
{
  if (rgb_topic.empty() || depth_topic.empty() || camera_info_topic.empty()) {
    return "RGB, depth and camera-info topics must be nonempty";
  }
  if (receiver_ip.empty()) {
    return "receiver_ip must be nonempty";
  }
  if (outside<std::int64_t>(h265_port, 1, 65535) ||
    outside<std::int64_t>(depth_port, 1, 65535) ||
    outside<std::int64_t>(calibration_port, 1, 65535))
  {
    return "all UDP ports must be in 1..65535";
  }
  if (h265_port == depth_port || h265_port == calibration_port ||
    depth_port == calibration_port)
  {
    return "H.265, depth and calibration UDP ports must be distinct";
  }
  if (outside<std::int64_t>(source_id, 1, std::numeric_limits<std::uint32_t>::max())) {
    return "source_id must be in 1..4294967295";
  }
  if (outside<std::int64_t>(sync_slop_ms, 1, 1000)) {
    return "sync_slop_ms must be in 1..1000";
  }
  if (outside<std::int64_t>(h265_bitrate, 1, std::numeric_limits<std::uint32_t>::max())) {
    return "h265_bitrate must fit a positive uint32";
  }
  if (outside<std::int64_t>(h265_gop, 1, std::numeric_limits<std::uint32_t>::max())) {
    return "h265_gop must fit a positive uint32";
  }
  if (outside<std::int64_t>(h265_fps, 1, 240)) {
    return "h265_fps must be in 1..240";
  }
  if (outside<std::int64_t>(expected_width, 8, 16384) || expected_width % 8 != 0 ||
    outside<std::int64_t>(expected_height, 8, 16384) || expected_height % 8 != 0)
  {
    return "expected width and height must be positive multiples of 8";
  }
  if (outside<std::int64_t>(
      udp_datagram_bytes,
      static_cast<std::int64_t>(kPacketHeaderBytes + 1U),
      static_cast<std::int64_t>(kMaximumUdpDatagramBytes)))
  {
    return "udp_datagram_bytes is outside protocol limits";
  }
  if (socket_send_buffer_bytes <= 0) {
    return "socket_send_buffer_bytes must be positive";
  }
  if (outside<std::int64_t>(
      max_frame_bytes, 1,
      static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max())))
  {
    return "max_frame_bytes must fit a positive uint32";
  }
  if (outside<std::int64_t>(calibration_period_ms, 100, 60000)) {
    return "calibration_period_ms must be in 100..60000";
  }
  if (outside<std::int64_t>(input_timeout_ms, 100, 60000)) {
    return "input_timeout_ms must be in 100..60000";
  }
  if (!std::isfinite(depth_scale_m) || depth_scale_m <= 0.0 || depth_scale_m > 1.0) {
    return "depth_scale_m must be finite and in (0, 1]";
  }
  return std::nullopt;
}

SenderNode::SenderNode()
: Node("rgbd_udp_sender")
{
  config_ = read_config();
  if (const auto error = config_.validation_error()) {
    throw std::invalid_argument("invalid sender configuration: " + *error);
  }

  std::string plugin_error;
  if (!H265Encoder::probe_required_elements(plugin_error)) {
    throw std::runtime_error(plugin_error + "; CPU H.265 fallback is disabled");
  }

  calibration_state_ = std::make_unique<CalibrationState>(
    config_.depth_scale_m,
    static_cast<std::uint32_t>(config_.h265_bitrate),
    static_cast<std::uint32_t>(config_.h265_gop),
    static_cast<std::uint32_t>(config_.h265_fps));

  const auto make_udp_config = [this](std::int64_t port) {
      UdpSenderConfig udp;
      udp.destination = config_.receiver_ip;
      udp.port = static_cast<std::uint16_t>(port);
      udp.datagram_bytes = static_cast<std::size_t>(config_.udp_datagram_bytes);
      udp.socket_send_buffer_bytes =
        static_cast<std::size_t>(config_.socket_send_buffer_bytes);
      udp.max_frame_bytes = static_cast<std::size_t>(config_.max_frame_bytes);
      return udp;
    };

  h265_udp_sender_ = std::make_unique<UdpSender>(make_udp_config(config_.h265_port));
  depth_udp_sender_ = std::make_unique<UdpSender>(make_udp_config(config_.depth_port));
  calibration_udp_sender_ =
    std::make_unique<UdpSender>(make_udp_config(config_.calibration_port));

  configure_ros_inputs();
  running_ = true;
  last_input_steady_ns_ = steady_time_ns();

  compression_thread_ = std::thread(&SenderNode::compression_loop, this);
  h265_udp_thread_ = std::thread(
    &SenderNode::udp_loop, this, "h265", std::ref(h265_send_slot_),
    std::ref(*h265_udp_sender_));
  depth_udp_thread_ = std::thread(
    &SenderNode::udp_loop, this, "zdepth", std::ref(depth_send_slot_),
    std::ref(*depth_udp_sender_));
  calibration_udp_thread_ = std::thread(
    &SenderNode::udp_loop, this, "calibration", std::ref(calibration_send_slot_),
    std::ref(*calibration_udp_sender_));

  calibration_timer_ = create_wall_timer(
    std::chrono::milliseconds(config_.calibration_period_ms),
    std::bind(&SenderNode::calibration_timer_callback, this));
  metrics_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&SenderNode::metrics_timer_callback, this));

  RCLCPP_INFO(
    get_logger(),
    "RGB-D sender ready: %s -> %s:%ld/%ld/%ld, datagram=%ld bytes",
    config_.rgb_topic.c_str(), config_.receiver_ip.c_str(),
    static_cast<long>(config_.h265_port), static_cast<long>(config_.depth_port),
    static_cast<long>(config_.calibration_port),
    static_cast<long>(config_.udp_datagram_bytes));
}

SenderNode::~SenderNode()
{
  shutdown();
}

void SenderNode::shutdown()
{
  if (shutdown_started_.exchange(true)) {
    return;
  }
  running_ = false;
  bundle_slot_.close();
  if (compression_thread_.joinable()) {
    compression_thread_.join();
  }

  if (h265_encoder_ && h265_encoder_->started()) {
    h265_eos_requested_ = true;
    std::string eos_error;
    if (!h265_encoder_->end_of_stream(eos_error)) {
      RCLCPP_WARN(get_logger(), "%s", eos_error.c_str());
    }
  }
  if (h265_output_thread_.joinable()) {
    h265_output_thread_.join();
  }

  h265_send_slot_.close();
  depth_send_slot_.close();
  calibration_send_slot_.close();
  if (h265_udp_thread_.joinable()) {
    h265_udp_thread_.join();
  }
  if (depth_udp_thread_.joinable()) {
    depth_udp_thread_.join();
  }
  if (calibration_udp_thread_.joinable()) {
    calibration_udp_thread_.join();
  }
  if (h265_encoder_) {
    h265_encoder_->stop();
  }
}

SenderConfig SenderNode::read_config()
{
  SenderConfig config;
  config.rgb_topic = declare_parameter<std::string>("rgb_topic", config.rgb_topic);
  config.depth_topic = declare_parameter<std::string>("depth_topic", config.depth_topic);
  config.camera_info_topic =
    declare_parameter<std::string>("camera_info_topic", config.camera_info_topic);
  config.receiver_ip = declare_parameter<std::string>("receiver_ip", config.receiver_ip);
  config.h265_port = declare_parameter<std::int64_t>("h265_port", config.h265_port);
  config.depth_port = declare_parameter<std::int64_t>("depth_port", config.depth_port);
  config.calibration_port =
    declare_parameter<std::int64_t>("calibration_port", config.calibration_port);
  config.source_id = declare_parameter<std::int64_t>("source_id", config.source_id);
  config.sync_slop_ms =
    declare_parameter<std::int64_t>("sync_slop_ms", config.sync_slop_ms);
  config.h265_bitrate =
    declare_parameter<std::int64_t>("h265_bitrate", config.h265_bitrate);
  config.h265_gop = declare_parameter<std::int64_t>("h265_gop", config.h265_gop);
  config.h265_fps = declare_parameter<std::int64_t>("h265_fps", config.h265_fps);
  config.expected_width =
    declare_parameter<std::int64_t>("expected_width", config.expected_width);
  config.expected_height =
    declare_parameter<std::int64_t>("expected_height", config.expected_height);
  config.udp_datagram_bytes =
    declare_parameter<std::int64_t>("udp_datagram_bytes", config.udp_datagram_bytes);
  config.socket_send_buffer_bytes = declare_parameter<std::int64_t>(
    "socket_send_buffer_bytes", config.socket_send_buffer_bytes);
  config.max_frame_bytes =
    declare_parameter<std::int64_t>("max_frame_bytes", config.max_frame_bytes);
  config.calibration_period_ms = declare_parameter<std::int64_t>(
    "calibration_period_ms", config.calibration_period_ms);
  config.input_timeout_ms =
    declare_parameter<std::int64_t>("input_timeout_ms", config.input_timeout_ms);
  config.depth_scale_m =
    declare_parameter<double>("depth_scale_m", config.depth_scale_m);
  return config;
}

void SenderNode::configure_ros_inputs()
{
  rgb_subscriber_.subscribe(this, config_.rgb_topic, rmw_qos_profile_sensor_data);
  depth_subscriber_.subscribe(this, config_.depth_topic, rmw_qos_profile_sensor_data);

  SyncPolicy policy(5);
  policy.setMaxIntervalDuration(
    rclcpp::Duration::from_nanoseconds(config_.sync_slop_ms * 1000000LL));
  synchronizer_ = std::make_unique<Synchronizer>(policy);
  synchronizer_->connectInput(rgb_subscriber_, depth_subscriber_);
  synchronizer_->registerCallback(
    std::bind(
      &SenderNode::synchronized_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  camera_info_subscriber_ = create_subscription<CameraInfo>(
    config_.camera_info_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&SenderNode::camera_info_callback, this, std::placeholders::_1));
}

void SenderNode::synchronized_callback(
  const Image::ConstSharedPtr & color,
  const Image::ConstSharedPtr & depth)
{
  if (!running_) {
    return;
  }
  ++counters_.synchronized;
  last_input_steady_ns_ = steady_time_ns();

  if (color->width != static_cast<std::uint32_t>(config_.expected_width) ||
    color->height != static_cast<std::uint32_t>(config_.expected_height))
  {
    ++counters_.input_rejected;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejecting %ux%u RGB-D input; configured size is %ldx%ld",
      color->width, color->height,
      static_cast<long>(config_.expected_width),
      static_cast<long>(config_.expected_height));
    return;
  }

  CameraInfo::ConstSharedPtr camera_info;
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    camera_info = latest_camera_info_;
  }
  if (!camera_info) {
    ++counters_.input_rejected;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Waiting for CameraInfo on %s", config_.camera_info_topic.c_str());
    return;
  }
  if (camera_info->width != color->width || camera_info->height != color->height) {
    ++counters_.input_rejected;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "CameraInfo dimensions do not match synchronized RGB-D frames");
    return;
  }

  CalibrationUpdate calibration;
  try {
    calibration = calibration_state_->update(
      *camera_info, color->encoding, depth->encoding, depth->header.frame_id);
  } catch (const std::exception & exception) {
    ++counters_.input_rejected;
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Calibration update failed: %s", exception.what());
    return;
  }
  if (calibration.changed) {
    enqueue_calibration(calibration.record);
  }

  const std::uint64_t sequence = next_bundle_sequence_.fetch_add(1);
  auto input = make_frame_bundle(
    *color, *depth, sequence,
    static_cast<std::uint32_t>(config_.source_id),
    calibration.record.calibration_id);
  if (!input) {
    ++counters_.input_rejected;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "RGB-D input rejected: %s", input.message.c_str());
    return;
  }

  const auto put_result = bundle_slot_.put(std::move(*input.bundle));
  if (put_result.overwrote) {
    ++counters_.input_overwritten;
  }
}

void SenderNode::camera_info_callback(const CameraInfo::ConstSharedPtr message)
{
  std::lock_guard<std::mutex> lock(camera_info_mutex_);
  latest_camera_info_ = std::move(message);
}

void SenderNode::enqueue_calibration(const CalibrationRecord & record)
{
  EncodedFrame frame;
  frame.stream_type = StreamType::kCalibration;
  frame.flags = kFlagConfig | kFlagKeyframe;
  frame.source_id = static_cast<std::uint32_t>(config_.source_id);
  frame.calibration_id = record.calibration_id;
  frame.bundle_sequence = 0;
  frame.source_timestamp_ns = record.source_timestamp_ns;
  frame.payload.assign(record.json.begin(), record.json.end());
  calibration_send_slot_.put(std::move(frame));
}

void SenderNode::calibration_timer_callback()
{
  if (!running_ || !calibration_state_) {
    return;
  }
  if (const auto record = calibration_state_->current()) {
    enqueue_calibration(*record);
  }
}

void SenderNode::metrics_timer_callback()
{
  if (!running_) {
    return;
  }

  const auto h265_stats = h265_udp_sender_->stats();
  const auto depth_stats = depth_udp_sender_->stats();
  const auto calibration_stats = calibration_udp_sender_->stats();
  MetricsSnapshot now;
  now.synchronized = counters_.synchronized.load();
  now.h265_encoded = counters_.h265_encoded.load();
  now.zdepth_encoded = counters_.zdepth_encoded.load();
  now.h265_bytes = h265_stats.bytes_sent;
  now.depth_bytes = depth_stats.bytes_sent;
  now.calibration_bytes = calibration_stats.bytes_sent;

  const double h265_mbps =
    static_cast<double>(now.h265_bytes - previous_metrics_.h265_bytes) * 8.0 / 1000000.0;
  const double depth_mbps =
    static_cast<double>(now.depth_bytes - previous_metrics_.depth_bytes) * 8.0 / 1000000.0;
  const std::uint64_t depth_frames = now.zdepth_encoded - previous_metrics_.zdepth_encoded;
  const std::uint64_t depth_us = counters_.zdepth_encode_microseconds.exchange(0);
  const double depth_ms = depth_frames == 0U ? 0.0 :
    static_cast<double>(depth_us) / static_cast<double>(depth_frames) / 1000.0;

  RCLCPP_INFO(
    get_logger(),
    "fps sync=%lu h265=%lu depth=%lu | Mbps h265=%.2f depth=%.2f | "
    "overwrite input=%lu h265_tx=%lu depth_tx=%lu | errors input=%lu h265=%lu depth=%lu udp=%lu | zdepth=%.2f ms",
    static_cast<unsigned long>(now.synchronized - previous_metrics_.synchronized),
    static_cast<unsigned long>(now.h265_encoded - previous_metrics_.h265_encoded),
    static_cast<unsigned long>(depth_frames),
    h265_mbps, depth_mbps,
    static_cast<unsigned long>(counters_.input_overwritten.load()),
    static_cast<unsigned long>(counters_.h265_send_overwritten.load()),
    static_cast<unsigned long>(counters_.depth_send_overwritten.load()),
    static_cast<unsigned long>(counters_.input_rejected.load()),
    static_cast<unsigned long>(counters_.h265_errors.load()),
    static_cast<unsigned long>(counters_.zdepth_encode_errors.load()),
    static_cast<unsigned long>(counters_.udp_errors.load()),
    depth_ms);

  const std::int64_t last_input = last_input_steady_ns_.load();
  if (last_input != 0 && steady_time_ns() - last_input > config_.input_timeout_ms * 1000000LL) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "No synchronized RGB-D input for more than %ld ms",
      static_cast<long>(config_.input_timeout_ms));
  }
  previous_metrics_ = now;
}

void SenderNode::compression_loop()
{
  while (true) {
    auto bundle = bundle_slot_.wait_take();
    if (!bundle) {
      break;
    }

    const auto depth_start = std::chrono::steady_clock::now();
    auto depth_result = zdepth_encoder_.encode(bundle->depth);
    const auto depth_end = std::chrono::steady_clock::now();
    counters_.zdepth_encode_microseconds.fetch_add(
      static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
          depth_end - depth_start).count()));
    if (depth_result.ok) {
      EncodedFrame depth_frame;
      depth_frame.stream_type = StreamType::kZdepth;
      depth_frame.flags = kFlagKeyframe;
      depth_frame.source_id = bundle->metadata.source_id;
      depth_frame.calibration_id = bundle->metadata.calibration_id;
      depth_frame.bundle_sequence = bundle->metadata.bundle_sequence;
      depth_frame.source_timestamp_ns = bundle->metadata.depth_timestamp_ns;
      depth_frame.payload = std::move(depth_result.payload);
      const auto put_result = depth_send_slot_.put(std::move(depth_frame));
      if (put_result.overwrote) {
        ++counters_.depth_send_overwritten;
      }
      ++counters_.zdepth_encoded;
    } else {
      ++counters_.zdepth_encode_errors;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s", depth_result.error.c_str());
    }

    if (h265_failed_) {
      continue;
    }
    if (!h265_encoder_) {
      H265Config encoder_config;
      encoder_config.width = bundle->color.width;
      encoder_config.height = bundle->color.height;
      encoder_config.fps = static_cast<std::uint32_t>(config_.h265_fps);
      encoder_config.bitrate = static_cast<std::uint32_t>(config_.h265_bitrate);
      encoder_config.gop = static_cast<std::uint32_t>(config_.h265_gop);
      encoder_config.input_format = bundle->color.format;
      h265_encoder_ = std::make_unique<H265Encoder>(encoder_config);

      std::string start_error;
      if (!h265_encoder_->start(start_error)) {
        h265_failed_ = true;
        ++counters_.h265_errors;
        RCLCPP_ERROR(
          get_logger(), "Hardware H.265 startup failed: %s", start_error.c_str());
        continue;
      }
      h265_output_thread_ = std::thread(&SenderNode::h265_output_loop, this);
      RCLCPP_INFO(
        get_logger(), "Hardware H.265 started: %s",
        H265Encoder::pipeline_summary(encoder_config).c_str());
    }

    std::string push_error;
    if (h265_encoder_->push(bundle->color, bundle->metadata, push_error)) {
      ++counters_.h265_accepted;
    } else {
      h265_failed_ = true;
      ++counters_.h265_errors;
      RCLCPP_ERROR(
        get_logger(), "Hardware H.265 input failed: %s", push_error.c_str());
    }
  }
}

void SenderNode::h265_output_loop()
{
  std::uint32_t eos_timeouts = 0;
  while (true) {
    auto output = h265_encoder_->pull(std::chrono::milliseconds(200));
    if (output.status == H265PullStatus::kTimeout) {
      if (h265_eos_requested_ && ++eos_timeouts >= 10U) {
        ++counters_.h265_errors;
        RCLCPP_ERROR(get_logger(), "Timed out draining H.265 after EOS for 2 seconds");
        break;
      }
      continue;
    }
    eos_timeouts = 0;
    if (output.status == H265PullStatus::kEos) {
      break;
    }
    if (output.status == H265PullStatus::kError ||
      output.status == H265PullStatus::kNotStarted)
    {
      h265_failed_ = true;
      ++counters_.h265_errors;
      RCLCPP_ERROR(
        get_logger(), "Hardware H.265 output failed: %s", output.error.c_str());
      break;
    }

    EncodedFrame frame;
    frame.stream_type = StreamType::kH265;
    frame.flags = output.keyframe ? kFlagKeyframe : kFlagNone;
    frame.source_id = output.metadata.source_id;
    frame.calibration_id = output.metadata.calibration_id;
    frame.bundle_sequence = output.metadata.bundle_sequence;
    frame.source_timestamp_ns = output.metadata.rgb_timestamp_ns;
    frame.payload = std::move(output.payload);
    const auto put_result = h265_send_slot_.put(std::move(frame));
    if (put_result.overwrote) {
      ++counters_.h265_send_overwritten;
    }
    ++counters_.h265_encoded;
  }
}

void SenderNode::udp_loop(
  const char * stream_name,
  LatestSlot<EncodedFrame> & slot,
  UdpSender & sender)
{
  while (true) {
    auto frame = slot.wait_take();
    if (!frame) {
      break;
    }
    const auto result = sender.send_frame(*frame);
    if (!result) {
      ++counters_.udp_errors;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "UDP %s frame %lu dropped after %zu datagrams: %s",
        stream_name,
        static_cast<unsigned long>(frame->bundle_sequence),
        result.datagrams_sent,
        result.error.c_str());
    }
  }
}

}  // namespace jetson_rgbd_udp_sender
