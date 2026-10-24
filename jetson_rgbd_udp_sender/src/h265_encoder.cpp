#include "jetson_rgbd_udp_sender/h265_encoder.hpp"

#include <cstring>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace jetson_rgbd_udp_sender
{
namespace
{

std::once_flag gst_init_once;

void initialize_gstreamer()
{
  std::call_once(gst_init_once, []() {gst_init(nullptr, nullptr);});
}

bool has_property(GstElement * element, const char * name)
{
  return g_object_class_find_property(G_OBJECT_GET_CLASS(element), name) != nullptr;
}

bool factory_exists(const char * name)
{
  GstElementFactory * factory = gst_element_factory_find(name);
  if (factory == nullptr) {
    return false;
  }
  gst_object_unref(factory);
  return true;
}

std::string format_name(PixelFormat format)
{
  return format == PixelFormat::kRgb8 ? "RGB" : "BGR";
}

GstElement * make_element(const char * factory, const char * name, std::string & error)
{
  GstElement * element = gst_element_factory_make(factory, name);
  if (element == nullptr) {
    error = std::string("cannot create GStreamer element: ") + factory;
  }
  return element;
}

}  // namespace

H265Encoder::H265Encoder(H265Config config)
: config_(config)
{
  if (config_.width == 0U || config_.height == 0U || config_.fps == 0U ||
    config_.bitrate == 0U || config_.gop == 0U)
  {
    throw std::invalid_argument("H.265 width, height, fps, bitrate and GOP must be positive");
  }
  if (config_.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
    config_.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
    config_.fps > static_cast<std::uint32_t>(std::numeric_limits<int>::max()))
  {
    throw std::invalid_argument("H.265 dimensions or FPS exceed GStreamer int range");
  }
}

H265Encoder::~H265Encoder()
{
  stop();
}

bool H265Encoder::probe_required_elements(std::string & error)
{
  initialize_gstreamer();
  constexpr const char * required[] = {
    "appsrc", "videoconvert", "capsfilter", "nvvidconv",
    "nvv4l2h265enc", "h265parse", "appsink"};
  for (const char * name : required) {
    if (!factory_exists(name)) {
      error = std::string("required GStreamer element is unavailable: ") + name;
      return false;
    }
  }

  GstElement * encoder = gst_element_factory_make("nvv4l2h265enc", nullptr);
  if (encoder == nullptr) {
    error = "nvv4l2h265enc factory exists but an encoder instance cannot be created";
    return false;
  }
  constexpr const char * required_properties[] = {
    "bitrate", "iframeinterval", "idrinterval", "insert-sps-pps"};
  for (const char * property : required_properties) {
    if (!has_property(encoder, property)) {
      error = std::string("nvv4l2h265enc is missing required property: ") + property;
      gst_object_unref(encoder);
      return false;
    }
  }
  gst_object_unref(encoder);
  error.clear();
  return true;
}

std::string H265Encoder::pipeline_summary(const H265Config & config)
{
  std::ostringstream output;
  output << "appsrc(format=" << format_name(config.input_format) << ","
         << config.width << "x" << config.height << "@" << config.fps << ")"
         << " ! videoconvert ! I420 ! nvvidconv ! NVMM/NV12"
         << " ! nvv4l2h265enc(bitrate=" << config.bitrate
         << ",iframe=" << config.gop << ",idr=" << config.gop
         << ",insert-sps-pps=true,no-b-frames)"
         << " ! h265parse ! video/x-h265,stream-format=byte-stream,alignment=au ! appsink";
  return output.str();
}

bool H265Encoder::start(std::string & error)
{
  if (started_) {
    error.clear();
    return true;
  }
  if (!probe_required_elements(error)) {
    return false;
  }

  pipeline_ = gst_pipeline_new("rgbd-h265-pipeline");
  GstElement * source = make_element("appsrc", "rgb-source", error);
  GstElement * convert = make_element("videoconvert", "color-convert", error);
  GstElement * system_caps_filter = make_element("capsfilter", "system-caps", error);
  GstElement * nv_convert = make_element("nvvidconv", "nv-convert", error);
  GstElement * nvmm_caps_filter = make_element("capsfilter", "nvmm-caps", error);
  GstElement * encoder = make_element("nvv4l2h265enc", "hardware-h265-encoder", error);
  GstElement * parser = make_element("h265parse", "h265-parser", error);
  GstElement * output_caps_filter = make_element("capsfilter", "h265-output-caps", error);
  GstElement * sink = make_element("appsink", "h265-sink", error);

  if (pipeline_ == nullptr || source == nullptr || convert == nullptr ||
    system_caps_filter == nullptr || nv_convert == nullptr ||
    nvmm_caps_filter == nullptr || encoder == nullptr || parser == nullptr ||
    output_caps_filter == nullptr || sink == nullptr)
  {
    stop();
    if (error.empty()) {
      error = "cannot construct the GStreamer H.265 pipeline";
    }
    return false;
  }

  gst_bin_add_many(
    GST_BIN(pipeline_), source, convert, system_caps_filter, nv_convert,
    nvmm_caps_filter, encoder, parser, output_caps_filter, sink, nullptr);

  appsrc_ = GST_APP_SRC(source);
  appsink_ = GST_APP_SINK(sink);

  GstCaps * source_caps = gst_caps_new_simple(
    "video/x-raw",
    "format", G_TYPE_STRING, format_name(config_.input_format).c_str(),
    "width", G_TYPE_INT, static_cast<int>(config_.width),
    "height", G_TYPE_INT, static_cast<int>(config_.height),
    "framerate", GST_TYPE_FRACTION, static_cast<int>(config_.fps), 1,
    nullptr);
  GstCaps * system_caps = gst_caps_new_simple(
    "video/x-raw", "format", G_TYPE_STRING, "I420", nullptr);
  GstCaps * nvmm_caps = gst_caps_from_string("video/x-raw(memory:NVMM),format=NV12");
  GstCaps * output_caps = gst_caps_from_string(
    "video/x-h265,stream-format=byte-stream,alignment=au");
  if (source_caps == nullptr || system_caps == nullptr || nvmm_caps == nullptr ||
    output_caps == nullptr)
  {
    if (source_caps != nullptr) {gst_caps_unref(source_caps);}
    if (system_caps != nullptr) {gst_caps_unref(system_caps);}
    if (nvmm_caps != nullptr) {gst_caps_unref(nvmm_caps);}
    if (output_caps != nullptr) {gst_caps_unref(output_caps);}
    error = "cannot create GStreamer caps";
    stop();
    return false;
  }

  gst_app_src_set_caps(appsrc_, source_caps);
  g_object_set(system_caps_filter, "caps", system_caps, nullptr);
  g_object_set(nvmm_caps_filter, "caps", nvmm_caps, nullptr);
  g_object_set(output_caps_filter, "caps", output_caps, nullptr);
  gst_caps_unref(source_caps);
  gst_caps_unref(system_caps);
  gst_caps_unref(nvmm_caps);
  gst_caps_unref(output_caps);

  g_object_set(
    source,
    "is-live", TRUE,
    "format", GST_FORMAT_TIME,
    "block", TRUE,
    "do-timestamp", FALSE,
    nullptr);
  if (has_property(source, "max-buffers")) {
    g_object_set(source, "max-buffers", static_cast<guint64>(2), nullptr);
  }

  g_object_set(
    encoder,
    "bitrate", static_cast<guint>(config_.bitrate),
    "iframeinterval", static_cast<guint>(config_.gop),
    "idrinterval", static_cast<guint>(config_.gop),
    "insert-sps-pps", TRUE,
    nullptr);
  if (has_property(encoder, "control-rate")) {
    g_object_set(encoder, "control-rate", static_cast<guint>(1), nullptr);
  }
  if (has_property(encoder, "num-B-Frames")) {
    g_object_set(encoder, "num-B-Frames", static_cast<guint>(0), nullptr);
  }
  if (has_property(encoder, "maxperf-enable")) {
    g_object_set(encoder, "maxperf-enable", TRUE, nullptr);
  }
  if (has_property(parser, "config-interval")) {
    g_object_set(parser, "config-interval", -1, nullptr);
  }

  g_object_set(
    sink,
    "emit-signals", FALSE,
    "sync", FALSE,
    "max-buffers", static_cast<guint>(4),
    "drop", FALSE,
    "enable-last-sample", FALSE,
    nullptr);

  if (!gst_element_link_many(
      source, convert, system_caps_filter, nv_convert, nvmm_caps_filter,
      encoder, parser, output_caps_filter, sink, nullptr))
  {
    error = "cannot link the GStreamer H.265 pipeline";
    stop();
    return false;
  }

  bus_ = gst_element_get_bus(pipeline_);
  const GstStateChangeReturn state_result =
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (state_result == GST_STATE_CHANGE_FAILURE) {
    error = "GStreamer H.265 pipeline refused PLAYING state";
    stop();
    return false;
  }

  started_ = true;
  error.clear();
  return true;
}

bool H265Encoder::push(
  const ColorFrame & frame,
  const BundleMetadata & metadata,
  std::string & error)
{
  std::lock_guard<std::mutex> push_lock(push_mutex_);
  if (!started_ || appsrc_ == nullptr) {
    error = "H.265 encoder is not started";
    return false;
  }
  if (frame.width != config_.width || frame.height != config_.height ||
    frame.format != config_.input_format)
  {
    error = "color frame dimensions or format changed after H.265 startup";
    return false;
  }
  const std::size_t expected_bytes =
    static_cast<std::size_t>(frame.width) * frame.height * 3U;
  if (frame.pixels.size() != expected_bytes) {
    error = "color buffer length does not match width * height * 3";
    return false;
  }

  {
    std::lock_guard<std::mutex> metadata_lock(metadata_mutex_);
    if (pending_metadata_.size() >= kMaxPendingMetadata) {
      error = "H.265 pending metadata queue reached its 32-frame limit";
      return false;
    }
  }

  GstBuffer * buffer = gst_buffer_new_allocate(nullptr, frame.pixels.size(), nullptr);
  if (buffer == nullptr) {
    error = "cannot allocate GStreamer input buffer";
    return false;
  }
  if (gst_buffer_fill(buffer, 0, frame.pixels.data(), frame.pixels.size()) !=
    frame.pixels.size())
  {
    gst_buffer_unref(buffer);
    error = "cannot fill GStreamer input buffer";
    return false;
  }

  GST_BUFFER_PTS(buffer) = static_cast<GstClockTime>(metadata.rgb_timestamp_ns);
  GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(GST_SECOND, 1, config_.fps);

  {
    std::lock_guard<std::mutex> metadata_lock(metadata_mutex_);
    pending_metadata_.push_back(metadata);
  }

  const GstFlowReturn flow_result = gst_app_src_push_buffer(appsrc_, buffer);
  if (flow_result != GST_FLOW_OK) {
    remove_pending_if_last(metadata.bundle_sequence);
    error = std::string("GStreamer rejected RGB buffer: ") +
      gst_flow_get_name(flow_result);
    return false;
  }

  error.clear();
  return true;
}

H265Output H265Encoder::pull(std::chrono::milliseconds timeout)
{
  H265Output output;
  if (!started_ || appsink_ == nullptr) {
    output.status = H265PullStatus::kNotStarted;
    return output;
  }

  GstSample * sample = gst_app_sink_try_pull_sample(
    appsink_, static_cast<GstClockTime>(timeout.count()) * GST_MSECOND);
  if (sample == nullptr) {
    if (poll_bus(output)) {
      return output;
    }
    if (gst_app_sink_is_eos(appsink_)) {
      output.status = H265PullStatus::kEos;
    } else {
      output.status = H265PullStatus::kTimeout;
    }
    return output;
  }

  GstBuffer * buffer = gst_sample_get_buffer(sample);
  if (buffer == nullptr) {
    gst_sample_unref(sample);
    output.status = H265PullStatus::kError;
    output.error = "GStreamer sample has no buffer";
    return output;
  }

  GstMapInfo map{};
  if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    gst_sample_unref(sample);
    output.status = H265PullStatus::kError;
    output.error = "cannot map encoded H.265 buffer";
    return output;
  }
  if (map.size > 0U) {
    output.payload.assign(map.data, map.data + map.size);
  }
  gst_buffer_unmap(buffer, &map);
  output.keyframe = !GST_BUFFER_FLAG_IS_SET(buffer, GST_BUFFER_FLAG_DELTA_UNIT);
  gst_sample_unref(sample);

  {
    std::lock_guard<std::mutex> metadata_lock(metadata_mutex_);
    if (pending_metadata_.empty()) {
      output.payload.clear();
      output.status = H265PullStatus::kError;
      output.error = "encoded H.265 AU has no pending bundle metadata";
      return output;
    }
    output.metadata = pending_metadata_.front();
    pending_metadata_.pop_front();
  }

  if (output.payload.empty()) {
    output.status = H265PullStatus::kError;
    output.error = "GStreamer produced an empty H.265 AU";
    return output;
  }
  output.status = H265PullStatus::kSample;
  return output;
}

bool H265Encoder::end_of_stream(std::string & error)
{
  std::lock_guard<std::mutex> push_lock(push_mutex_);
  if (!started_ || appsrc_ == nullptr) {
    error.clear();
    return true;
  }
  const GstFlowReturn flow_result = gst_app_src_end_of_stream(appsrc_);
  if (flow_result != GST_FLOW_OK) {
    error = std::string("cannot send GStreamer EOS: ") + gst_flow_get_name(flow_result);
    return false;
  }
  error.clear();
  return true;
}

void H265Encoder::stop()
{
  std::lock_guard<std::mutex> push_lock(push_mutex_);
  started_ = false;
  if (pipeline_ != nullptr) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
  }
  if (bus_ != nullptr) {
    gst_object_unref(bus_);
    bus_ = nullptr;
  }
  if (pipeline_ != nullptr) {
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }
  appsrc_ = nullptr;
  appsink_ = nullptr;
  std::lock_guard<std::mutex> metadata_lock(metadata_mutex_);
  pending_metadata_.clear();
}

bool H265Encoder::started() const noexcept
{
  return started_;
}

bool H265Encoder::poll_bus(H265Output & output)
{
  if (bus_ == nullptr) {
    return false;
  }
  GstMessage * message = gst_bus_pop_filtered(
    bus_, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
  if (message == nullptr) {
    return false;
  }

  if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
    GError * parsed_error = nullptr;
    gchar * debug = nullptr;
    gst_message_parse_error(message, &parsed_error, &debug);
    output.status = H265PullStatus::kError;
    output.error = parsed_error != nullptr ? parsed_error->message : "unknown GStreamer error";
    if (debug != nullptr && std::strlen(debug) > 0U) {
      output.error += std::string("; debug: ") + debug;
    }
    if (parsed_error != nullptr) {
      g_error_free(parsed_error);
    }
    g_free(debug);
  } else {
    output.status = H265PullStatus::kEos;
  }
  gst_message_unref(message);
  return true;
}

void H265Encoder::remove_pending_if_last(std::uint64_t bundle_sequence)
{
  std::lock_guard<std::mutex> metadata_lock(metadata_mutex_);
  if (!pending_metadata_.empty() &&
    pending_metadata_.back().bundle_sequence == bundle_sequence)
  {
    pending_metadata_.pop_back();
  }
}

}  // namespace jetson_rgbd_udp_sender
