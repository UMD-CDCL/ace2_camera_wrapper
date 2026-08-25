// ROS2 wrapper for the Basler ace 2 a2A2448-75ucPRO (5 MP color, USB3) using the
// pylon 8 C++ SDK. Publishes sensor_msgs/Image plus sensor_msgs/CameraInfo.
//
// Design notes worth knowing before editing:
//
// * Camera features are written through the generic GenApi node map, looked up by
//   name at runtime, instead of a model-specific CBaslerUniversal* parameter
//   class. A feature the camera does not implement is then skipped with a warning
//   rather than throwing. That is not hypothetical on this model: it has no
//   Binning or Decimation nodes at all, and BslSharpnessEnhancement /
//   BslNoiseReduction are read-only whenever PixelFormat is a Bayer format,
//   because those PGI steps only run when the camera itself demosaics.
//
// * The default is BayerRG8 off the camera, debayered on the host to bgr8. The
//   camera's DeviceLinkThroughputLimit defaults to 360 MB/s, and a frame is
//   2448 x 2048. At 1 byte/pixel that is 5.01 MB and ~71 fps fits in the link.
//   Letting the camera output BGR8 instead triples it to 15.0 MB and caps the
//   link at ~24 fps. Host debayering measured 1.2 ms per frame on this machine,
//   so paying it is much cheaper than giving up two thirds of the frame rate.
//
// * The device timestamp counter ticks at 1 GHz (1 tick = 1 ns), confirmed
//   against the host clock. "camera" timestamp mode uses it, re-anchored to ROS
//   time periodically so the two oscillators cannot drift apart without bound.

#include "ace2_camera_wrapper/ace2_camera_wrapper_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

namespace ace2_camera_wrapper
{


namespace
{

// A parameter set to this value means "leave whatever the camera already has".
// Used for the optional knobs so that a default value cannot spam a warning on
// every startup for a feature this model exposes as read-only.
constexpr double kLeaveAlone = -1.0;

bool leave_alone(double v)
{
  return std::isnan(v) || v < 0.0;
}

// Rounds v into [lo,hi] and onto the feature's increment grid. GenApi rejects an
// off-grid write outright, and Width/Height/OffsetX/OffsetY all use inc=2 here.
int64_t clamp_to_increment(int64_t v, int64_t lo, int64_t hi, int64_t inc)
{
  v = std::max(lo, std::min(hi, v));
  if (inc > 1) {
    v = lo + ((v - lo) / inc) * inc;
  }
  return v;
}

// Maps a pylon pixel type onto the ROS encoding carrying the same bytes, for the
// "passthrough" output mode. The packed formats (Mono10p, BayerRG12p, ...) have
// no ROS equivalent and are rejected by the caller instead of being republished
// as meaningless bytes.
bool ros_encoding_for(Pylon::EPixelType type, std::string & encoding)
{
  namespace enc = sensor_msgs::image_encodings;
  switch (type) {
    case Pylon::PixelType_Mono8:      encoding = enc::MONO8;        return true;
    case Pylon::PixelType_Mono10:
    case Pylon::PixelType_Mono12:
    case Pylon::PixelType_Mono16:     encoding = enc::MONO16;       return true;
    case Pylon::PixelType_BayerRG8:   encoding = enc::BAYER_RGGB8;  return true;
    case Pylon::PixelType_BayerRG10:
    case Pylon::PixelType_BayerRG12:  encoding = enc::BAYER_RGGB16; return true;
    case Pylon::PixelType_RGB8packed: encoding = enc::RGB8;         return true;
    case Pylon::PixelType_BGR8packed: encoding = enc::BGR8;         return true;
    default:                                                        return false;
  }
}

}  // namespace

Ace2CameraWrapperNode::Ace2CameraWrapperNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("ace2_camera_wrapper", options)
{
  declareParameters();

  // Validated before anything else touches hardware: a typo here is a permanent
  // configuration error, and the reconnect loop would otherwise retry it forever.
  validateOutputEncoding();

  // Say the orientation out loud when there is one. It changes the published
  // geometry, and an image that is merely turned is easy to blame on the wrong
  // part of the stack.
  if (!orientation_.identity()) {
    RCLCPP_INFO(
      get_logger(),
      "Turning every frame on the host: flip_horizontal=%s, flip_vertical=%s, then %ld "
      "degrees clockwise.%s",
      flip_horizontal_ ? "true" : "false", flip_vertical_ ? "true" : "false", image_rotation_,
      orientation_.transpose ? " Width and height are swapped." : "");
  }

  // CameraInfoManager also brings up the standard set_camera_info service, so
  // camera_calibration can write a calibration back to camera_info_url.
  info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, camera_name_, camera_info_url_);

  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  qos_profile.depth = static_cast<size_t>(qos_depth_);
  qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qos_profile.reliability = (qos_reliability_ == "reliable")
    ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
    : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

  if (use_image_transport_) {
    // image_transport derives the camera_info topic as a sibling of the image
    // topic, so the base name "ace2/image_raw" yields "ace2/camera_info" and the
    // two layouts agree.
    camera_pub_ = image_transport::create_camera_publisher(this, "ace2/image_raw", qos_profile);
    RCLCPP_INFO(
      get_logger(), "Publishing '%s' and '%s' through image_transport (%s, depth %ld)",
      camera_pub_.getTopic().c_str(), camera_pub_.getInfoTopic().c_str(),
      qos_reliability_.c_str(), qos_depth_);
    RCLCPP_WARN(
      get_logger(),
      "image_transport copies every frame on publish, so intra-process zero-copy is off. "
      "Set use_image_transport to false to hand frames to subscribers by move instead.");
  } else {
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(qos_profile));
    qos.reliability(qos_profile.reliability);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("ace2/image_raw", qos);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("ace2/camera_info", qos);
    RCLCPP_INFO(
      get_logger(), "Publishing '%s' and '%s' (%s, depth %ld, frames moved to subscribers)",
      image_pub_->get_topic_name(), info_pub_->get_topic_name(),
      qos_reliability_.c_str(), qos_depth_);
  }

  trigger_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/execute_software_trigger",
    std::bind(&Ace2CameraWrapperNode::onSoftwareTrigger, this,
              std::placeholders::_1, std::placeholders::_2));

  if (temperature_period_ > 0.0) {
    temperature_pub_ = create_publisher<sensor_msgs::msg::Temperature>("~/temperature", 10);
    temperature_timer_ = create_wall_timer(
      std::chrono::duration<double>(temperature_period_),
      std::bind(&Ace2CameraWrapperNode::publishTemperature, this));
  }

  param_cb_ = add_on_set_parameters_callback(
    std::bind(&Ace2CameraWrapperNode::onSetParameters, this, std::placeholders::_1));

  Pylon::PylonInitialize();
  pylon_initialized_ = true;

  // The grab thread owns the camera for its whole lifetime, including
  // reconnection, so a camera unplugged mid-run does not take the node down.
  grab_thread_ = std::thread(&Ace2CameraWrapperNode::grabThread, this);
}


Ace2CameraWrapperNode::~Ace2CameraWrapperNode()
{
  running_ = false;
  {
    std::lock_guard<std::mutex> lock(sleep_mutex_);
    sleep_cv_.notify_all();
  }
  if (grab_thread_.joinable()) {
    grab_thread_.join();
  }
  if (pylon_initialized_) {
    Pylon::PylonTerminate();
  }
}


  // GenICam spells these values Off, On, Once and Continuous, but YAML 1.1 reads
  // "Off" and "On" as booleans. So `-p exposure_auto:=Off` on the command line, and
  // the same value in a params.yaml, both arrive as a bool, and a plainly declared
  // string parameter rejects them before the node can say anything useful. These
  // are declared with dynamic typing instead and coerced back to the GenICam
  // spelling on read.
std::string Ace2CameraWrapperNode::declareEnum(
  const std::string & name, const std::string & default_value, const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.description = description + " (Off and On may be written unquoted.)";
  desc.dynamic_typing = true;
  this->declare_parameter(name, rclcpp::ParameterValue(default_value), desc);
  return enumParam(name);
}


std::string Ace2CameraWrapperNode::enumValueOf(const rclcpp::Parameter & p)
{
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
    return p.as_bool() ? "On" : "Off";
  }
  return p.as_string();
}


std::string Ace2CameraWrapperNode::enumParam(const std::string & name) const
{
  return enumValueOf(this->get_parameter(name));
}


void Ace2CameraWrapperNode::declareParameters()
{
  // -- device selection ----------------------------------------------------
  device_serial_ = declare<std::string>(
    "device_serial_number", "",
    "Serial number of the camera to open. Empty opens the first camera found.");
  device_user_id_ = declare<std::string>(
    "device_user_id", "",
    "DeviceUserID of the camera to open. Empty ignores this filter.");

  // -- ROS interface -------------------------------------------------------
  camera_name_ = declare<std::string>(
    "camera_name", "ace2_camera",
    "Calibration name, used to name and validate the camera_info_url file.");
  frame_id_ = declare<std::string>(
    "frame_id", "ace2_camera_optical_frame",
    "frame_id stamped on Image and CameraInfo.");
  camera_info_url_ = declare<std::string>(
    "camera_info_url", "",
    "Calibration URL, e.g. file:///path/calib.yaml or package://pkg/calib.yaml. "
    "Empty publishes an uncalibrated CameraInfo carrying only the image size.");
  qos_reliability_ = declare<std::string>(
    "qos_reliability", "best_effort",
    "Publisher reliability: best_effort or reliable. A subscriber that requests "
    "reliable will not connect to a best_effort publisher.");
  qos_depth_ = declare<int64_t>("qos_depth", 1, "Publisher queue depth.");
  use_image_transport_ = declare<bool>(
    "use_image_transport", false,
    "Publish through image_transport, which adds the compressed transports but copies "
    "every frame. False uses plain publishers, which can move a frame to a subscriber "
    "in the same process with no copy at all.");

  // -- acquisition ---------------------------------------------------------
  startup_user_set_ = declare<std::string>(
    "startup_user_set", "Default",
    "User set loaded before applying parameters, so startup is repeatable: "
    "Default, UserSet1, UserSet2, UserSet3, or empty to skip.");
  pixel_format_ = declare<std::string>(
    "pixel_format", "BayerRG8",
    "Camera-side PixelFormat: Mono8, Mono12, RGB8, BGR8, YCbCr422_8, BayerRG8, "
    "BayerRG12, ... Bayer keeps the USB link cheap and is debayered on the host.");
  output_encoding_ = declare<std::string>(
    "output_encoding", "bgr8",
    "ROS-side encoding: bgr8, rgb8, mono8, or passthrough to publish the "
    "camera's own format without conversion.");
  frame_rate_ = declare<double>(
    "frame_rate", 30.0,
    "Acquisition frame rate in Hz. 0 or less frees the rate, letting the camera "
    "run as fast as exposure and DeviceLinkThroughputLimit allow.");
  grab_strategy_ = declare<std::string>(
    "grab_strategy", "latest_image_only",
    "latest_image_only drops backlog to keep latency low; one_by_one keeps every "
    "frame in order and is what you want when recording.");
  grab_timeout_ms_ = declare<int64_t>(
    "grab_timeout_ms", 5000, "Timeout waiting for a frame before treating the stream as stalled.");
  max_num_buffer_ = declare<int64_t>(
    "max_num_buffer", 16, "Number of frame buffers pylon allocates for the stream.");
  reconnect_delay_ = declare<double>(
    "reconnect_delay", 2.0, "Delay before retrying after a camera error or disconnect.");

  // -- region of interest --------------------------------------------------
  image_width_ = declare<int64_t>(
    "image_width", 0,
    "ROI width in pixels. 0 keeps the width of the loaded user set (2448 here). "
    "The sensor is addressable to 2472, but those columns are outside the "
    "specified imaging area.");
  image_height_ = declare<int64_t>(
    "image_height", 0,
    "ROI height in pixels. 0 keeps the height of the loaded user set (2048 here, "
    "addressable to 2064).");
  offset_x_ = declare<int64_t>(
    "offset_x", -1, "ROI x offset. -1 centres the ROI. Ignored while the ROI is left at its default.");
  offset_y_ = declare<int64_t>(
    "offset_y", -1, "ROI y offset. -1 centres the ROI. Ignored while the ROI is left at its default.");

  // -- exposure ------------------------------------------------------------
  declareEnum("exposure_auto", "Continuous", "ExposureAuto: Off, Once or Continuous.");
  declare<double>("exposure_time", 5000.0, "Exposure in microseconds. Only writable while exposure_auto is Off.");
  declare<double>("exposure_auto_lower_limit", 100.0, "Lower exposure bound in us for the auto function.");
  declare<double>("exposure_auto_upper_limit", 20000.0, "Upper exposure bound in us for the auto function.");

  // -- gain and brightness -------------------------------------------------
  declareEnum("gain_auto", "Continuous", "GainAuto: Off, Once or Continuous.");
  declare<double>("gain", 0.0, "Gain in dB (0 to 48). Only writable while gain_auto is Off.");
  declare<double>("gain_auto_lower_limit", 0.0, "Lower gain bound in dB for the auto function.");
  declare<double>("gain_auto_upper_limit", 24.0, "Upper gain bound in dB for the auto function.");
  declare<double>("auto_target_brightness", 0.5, "Target brightness for the auto functions, 0 to 1.");
  declare<double>("black_level", 0.0, "BlackLevel in DN.");
  declare<double>("gamma", 1.0, "Gamma correction. 1.0 is linear.");

  // -- colour --------------------------------------------------------------
  declareEnum("balance_white_auto", "Continuous", "BalanceWhiteAuto: Off, Once or Continuous.");
  declare<double>("balance_ratio_red", 1.0, "Red balance ratio. Applied only while balance_white_auto is Off.");
  declare<double>("balance_ratio_green", 1.0, "Green balance ratio. Applied only while balance_white_auto is Off.");
  declare<double>("balance_ratio_blue", 1.0, "Blue balance ratio. Applied only while balance_white_auto is Off.");
  declareEnum("light_source_preset", "Daylight6500K",
    "BslLightSourcePreset: Off, Tungsten, Daylight5000K, Daylight6500K, FactoryLED6000K.");
  declareEnum("color_space", "sRgb", "BslColorSpace: Off or sRgb.");
  declare<double>("saturation", 1.0, "BslSaturation, 0 to 2.");
  declare<double>("hue", 0.0, "BslHue in degrees, -180 to 180.");
  declare<double>("contrast", 0.0, "BslContrast, -1 to 1.");
  declare<double>("brightness", 0.0, "BslBrightness, -1 to 1.");
  declare<double>("sharpness_enhancement", kLeaveAlone,
    "BslSharpnessEnhancement. Negative leaves it alone. Only writable when the "
    "camera demosaics, so a Bayer pixel_format makes this a no-op.");
  declare<double>("noise_reduction", kLeaveAlone,
    "BslNoiseReduction. Negative leaves it alone. Same camera-demosaic condition "
    "as sharpness_enhancement.");

  // -- orientation ---------------------------------------------------------
  // Two places can turn the image and they are not interchangeable. ReverseX and
  // ReverseY mirror on the sensor and cost nothing, but they shift the phase of
  // the Bayer tile without the camera saying so, which makes them wrong for a
  // Bayer pixel_format. The host-side knobs below cost one pass over the frame
  // and are always right: they relabel the Bayer pattern and rewrite CameraInfo
  // to match.
  declare<bool>("reverse_x", false,
    "Mirror the image horizontally on the camera. Free, but it corrupts the "
    "colours under a Bayer pixel_format: use flip_horizontal there instead.");
  declare<bool>("reverse_y", false,
    "Mirror the image vertically on the camera. Same Bayer caveat as reverse_x, "
    "so use flip_vertical there instead.");
  image_rotation_ = declare<int64_t>(
    "image_rotation", 0,
    "Rotate the published image clockwise by 0, 90, 180 or 270 degrees, for a "
    "camera that is not mounted upright. 90 and 270 swap width and height.");
  flip_horizontal_ = declare<bool>(
    "flip_horizontal", false,
    "Mirror the published image about its vertical axis, applied before image_rotation.");
  flip_vertical_ = declare<bool>(
    "flip_vertical", false,
    "Mirror the published image about its horizontal axis, applied before image_rotation.");
  if (!validRotation(image_rotation_)) {
    throw std::runtime_error(
      "image_rotation must be 0, 90, 180 or 270, not " + std::to_string(image_rotation_));
  }
  // Composed once, here, and read on the grab thread for the rest of the run.
  // That is why these three are not in the live-reconfigurable set: a frame
  // costs one branch on orientation_.identity(), not a lock.
  orientation_ = orientationOf(image_rotation_, flip_horizontal_, flip_vertical_);

  // -- link and transport --------------------------------------------------
  device_link_throughput_limit_ = declare<int64_t>(
    "device_link_throughput_limit", 0,
    "DeviceLinkThroughputLimit in bytes/s. 0 keeps the camera default (360 MB/s). "
    "Lower it when several cameras share one USB controller.");
  max_transfer_size_ = declare<int64_t>(
    "max_transfer_size", 0,
    "USB MaxTransferSize in bytes. 0 keeps the pylon default (262144). Raising it "
    "helps at high frame rates but needs a matching usbfs_memory_mb.");
  num_max_queued_urbs_ = declare<int64_t>(
    "num_max_queued_urbs", 0, "USB NumMaxQueuedUrbs. 0 keeps the pylon default (64).");

  // -- triggering ----------------------------------------------------------
  declareEnum("trigger_mode", "Off",
    "TriggerMode: Off for free-running acquisition, On to gate on trigger_source.");
  declare<std::string>("trigger_source", "Software",
    "TriggerSource when trigger_mode is On: Software, Line1, Line2, Line3, ...");

  // -- timestamps ----------------------------------------------------------
  timestamp_source_ = declare<std::string>(
    "timestamp_source", "ros_time",
    "ros_time stamps when the frame is retrieved on the host. camera uses the "
    "device counter, which is jitter-free but needs the resync below.");
  timestamp_resync_period_ = declare<double>(
    "timestamp_resync_period", 10.0,
    "How often to re-anchor the device counter to ROS time, in seconds. 0 anchors "
    "once at startup and lets the two clocks drift.");
  timestamp_offset_ = declare<double>(
    "timestamp_offset", 0.0,
    "Constant offset in seconds added to every stamp, to compensate a known "
    "exposure and transfer latency.");

  // -- diagnostics ---------------------------------------------------------
  stats_log_period_ = declare<double>(
    "stats_log_period", 0.0,
    "Log the measured grab rate every N seconds. This is the node's own count, so "
    "unlike `ros2 topic hz` it reports what the camera delivered rather than what "
    "survived the transport. 0 disables.");
  temperature_period_ = declare<double>(
    "temperature_publish_period", 0.0,
    "Period in seconds for publishing DeviceTemperature on ~/temperature. 0 disables.");
}


  // -------------------------------------------------- GenApi write helpers
  //
  // Every helper is best-effort: it reports what happened and returns false
  // rather than throwing, so one unsupported feature cannot abort startup.

bool Ace2CameraWrapperNode::setEnum(const char * feature, const std::string & value)
{
  if (value.empty()) {
    return false;
  }
  try {
    GenApi::CEnumerationPtr node(node_map_->GetNode(feature));
    if (!node.IsValid()) {
      RCLCPP_WARN(get_logger(), "%s: not implemented by this camera, skipped", feature);
      return false;
    }
    if (!GenApi::IsWritable(node)) {
      RCLCPP_WARN(get_logger(), "%s: not writable right now, '%s' not applied", feature, value.c_str());
      return false;
    }
    GenApi::CEnumEntryPtr entry(node->GetEntryByName(value.c_str()));
    if (!entry.IsValid() || !GenApi::IsAvailable(entry)) {
      RCLCPP_ERROR(get_logger(), "%s: '%s' is not an available value, skipped", feature, value.c_str());
      return false;
    }
    node->FromString(value.c_str());
    RCLCPP_INFO(get_logger(), "%s = %s", feature, value.c_str());
    return true;
  } catch (const Pylon::GenericException & e) {
    RCLCPP_ERROR(get_logger(), "%s: '%s' rejected: %s", feature, value.c_str(), e.GetDescription());
    return false;
  }
}


bool Ace2CameraWrapperNode::setFloat(const char * feature, double value)
{
  try {
    GenApi::CFloatPtr node(node_map_->GetNode(feature));
    if (!node.IsValid()) {
      RCLCPP_WARN(get_logger(), "%s: not implemented by this camera, skipped", feature);
      return false;
    }
    if (!GenApi::IsWritable(node)) {
      RCLCPP_WARN(get_logger(), "%s: not writable right now, %g not applied", feature, value);
      return false;
    }
    const double clamped = std::max(node->GetMin(), std::min(node->GetMax(), value));
    if (clamped != value) {
      RCLCPP_WARN(
        get_logger(), "%s: %g is outside [%g, %g], using %g",
        feature, value, node->GetMin(), node->GetMax(), clamped);
    }
    node->SetValue(clamped);
    RCLCPP_INFO(get_logger(), "%s = %g", feature, clamped);
    return true;
  } catch (const Pylon::GenericException & e) {
    RCLCPP_ERROR(get_logger(), "%s: %g rejected: %s", feature, value, e.GetDescription());
    return false;
  }
}


bool Ace2CameraWrapperNode::setInt(const char * feature, int64_t value)
{
  try {
    GenApi::CIntegerPtr node(node_map_->GetNode(feature));
    if (!node.IsValid()) {
      RCLCPP_WARN(get_logger(), "%s: not implemented by this camera, skipped", feature);
      return false;
    }
    if (!GenApi::IsWritable(node)) {
      RCLCPP_WARN(get_logger(), "%s: not writable right now, %ld not applied", feature, value);
      return false;
    }
    const int64_t clamped =
      clamp_to_increment(value, node->GetMin(), node->GetMax(), node->GetInc());
    if (clamped != value) {
      RCLCPP_WARN(
        get_logger(), "%s: %ld is outside [%ld, %ld] step %ld, using %ld",
        feature, value, node->GetMin(), node->GetMax(), node->GetInc(), clamped);
    }
    node->SetValue(clamped);
    RCLCPP_INFO(get_logger(), "%s = %ld", feature, clamped);
    return true;
  } catch (const Pylon::GenericException & e) {
    RCLCPP_ERROR(get_logger(), "%s: %ld rejected: %s", feature, value, e.GetDescription());
    return false;
  }
}


bool Ace2CameraWrapperNode::setBool(const char * feature, bool value)
{
  try {
    GenApi::CBooleanPtr node(node_map_->GetNode(feature));
    if (!node.IsValid()) {
      RCLCPP_WARN(get_logger(), "%s: not implemented by this camera, skipped", feature);
      return false;
    }
    if (!GenApi::IsWritable(node)) {
      RCLCPP_WARN(get_logger(), "%s: not writable right now, %s not applied",
                  feature, value ? "true" : "false");
      return false;
    }
    node->SetValue(value);
    RCLCPP_INFO(get_logger(), "%s = %s", feature, value ? "true" : "false");
    return true;
  } catch (const Pylon::GenericException & e) {
    RCLCPP_ERROR(get_logger(), "%s rejected: %s", feature, e.GetDescription());
    return false;
  }
}


bool Ace2CameraWrapperNode::execute(const char * feature)
{
  try {
    GenApi::CCommandPtr node(node_map_->GetNode(feature));
    if (!node.IsValid() || !GenApi::IsWritable(node)) {
      return false;
    }
    node->Execute();
    return true;
  } catch (const Pylon::GenericException & e) {
    RCLCPP_ERROR(get_logger(), "%s failed: %s", feature, e.GetDescription());
    return false;
  }
}


double Ace2CameraWrapperNode::getFloat(const char * feature, double fallback) const
{
  try {
    GenApi::CFloatPtr node(node_map_->GetNode(feature));
    if (node.IsValid() && GenApi::IsReadable(node)) {
      return node->GetValue();
    }
  } catch (const Pylon::GenericException &) {
  }
  return fallback;
}


int64_t Ace2CameraWrapperNode::getInt(const char * feature, int64_t fallback) const
{
  try {
    GenApi::CIntegerPtr node(node_map_->GetNode(feature));
    if (node.IsValid() && GenApi::IsReadable(node)) {
      return node->GetValue();
    }
  } catch (const Pylon::GenericException &) {
  }
  return fallback;
}


  // --------------------------------------------------------- camera lifecycle

  // Picks the device matching device_serial_number / device_user_id, or the first
  // camera when neither filter is set.
Pylon::CDeviceInfo Ace2CameraWrapperNode::selectDevice() const
{
  Pylon::DeviceInfoList_t devices;
  Pylon::CTlFactory::GetInstance().EnumerateDevices(devices);
  if (devices.empty()) {
    throw std::runtime_error("no Basler camera found");
  }

  for (const auto & info : devices) {
    const bool serial_ok =
      device_serial_.empty() || device_serial_ == info.GetSerialNumber().c_str();
    const bool user_id_ok =
      device_user_id_.empty() || device_user_id_ == info.GetUserDefinedName().c_str();
    if (serial_ok && user_id_ok) {
      return info;
    }
  }

  std::string available;
  for (const auto & info : devices) {
    // Pylon's String_t is GenICam's gcstring, which has no operator+ with
    // std::string, so each piece goes through c_str().
    available += std::string("\n  ") + info.GetModelName().c_str() +
      " serial=" + info.GetSerialNumber().c_str() +
      " user_id='" + info.GetUserDefinedName().c_str() + "'";
  }
  throw std::runtime_error(
    "no camera matched device_serial_number='" + device_serial_ + "' device_user_id='" +
    device_user_id_ + "'. Cameras present:" + available);
}


void Ace2CameraWrapperNode::openCamera()
{
  const Pylon::CDeviceInfo info = selectDevice();
  camera_ = std::make_unique<Pylon::CInstantCamera>(
    Pylon::CTlFactory::GetInstance().CreateDevice(info));
  camera_->Open();
  node_map_ = &camera_->GetNodeMap();

  RCLCPP_INFO(
    get_logger(), "Opened %s (serial %s, %s)",
    camera_->GetDeviceInfo().GetModelName().c_str(),
    camera_->GetDeviceInfo().GetSerialNumber().c_str(),
    camera_->GetDeviceInfo().GetDeviceClass().c_str());
}


void Ace2CameraWrapperNode::closeCamera()
{
  std::lock_guard<std::mutex> lock(camera_mutex_);
  try {
    if (camera_) {
      if (camera_->IsGrabbing()) {
        camera_->StopGrabbing();
      }
      if (camera_->IsOpen()) {
        camera_->Close();
      }
    }
  } catch (const Pylon::GenericException & e) {
    RCLCPP_WARN(get_logger(), "While closing the camera: %s", e.GetDescription());
  }
  node_map_ = nullptr;
  camera_.reset();
}


  // ------------------------------------------------------------ configuration

void Ace2CameraWrapperNode::applyConfiguration()
{
  // Start from a known state so a parameter left at its default does not
  // silently inherit whatever the previous run wrote into the camera.
  if (!startup_user_set_.empty()) {
    if (setEnum("UserSetSelector", startup_user_set_)) {
      execute("UserSetLoad");
      RCLCPP_INFO(get_logger(), "Loaded user set '%s'", startup_user_set_.c_str());
    }
  }

  // Link budget first: on this model the throughput limit is what decides the
  // achievable frame rate, so it has to be in place before the rate is set.
  if (device_link_throughput_limit_ > 0) {
    setEnum("DeviceLinkThroughputLimitMode", "On");
    setInt("DeviceLinkThroughputLimit", device_link_throughput_limit_);
  }

  configureRoi();
  const bool reverse_x = get_parameter("reverse_x").as_bool();
  const bool reverse_y = get_parameter("reverse_y").as_bool();
  setBool("ReverseX", reverse_x);
  setBool("ReverseY", reverse_y);
  if ((reverse_x || reverse_y) && pixel_format_.rfind("Bayer", 0) == 0) {
    RCLCPP_WARN(
      get_logger(),
      "reverse_x/reverse_y mirror on the sensor, which shifts the Bayer phase while the "
      "camera goes on reporting %s, so the colours come out wrong. Use flip_horizontal "
      "and flip_vertical instead: they mirror on the host and relabel the pattern.",
      pixel_format_.c_str());
  }

  setEnum("PixelFormat", pixel_format_);

  configureExposure();
  configureGain();
  configureColor();
  configureFrameRate();
  configureTrigger();

  // Optional PGI knobs. Only writable when the camera itself demosaics, which
  // a Bayer pixel_format rules out, hence the leave-alone default.
  const double sharpness = get_parameter("sharpness_enhancement").as_double();
  if (!leave_alone(sharpness)) {
    setFloat("BslSharpnessEnhancement", sharpness);
  }
  const double denoise = get_parameter("noise_reduction").as_double();
  if (!leave_alone(denoise)) {
    setFloat("BslNoiseReduction", denoise);
  }

  RCLCPP_INFO(
    get_logger(), "Camera reports ResultingFrameRate = %.2f Hz at %ldx%ld, PayloadSize %ld bytes",
    getFloat("ResultingFrameRate", 0.0), getInt("Width", 0), getInt("Height", 0),
    getInt("PayloadSize", 0));
}


void Ace2CameraWrapperNode::configureRoi()
{
  // All defaults means "leave the ROI alone". That matters on this model: the
  // loaded user set gives the nominal 2448x2048 imaging area centred on the
  // sensor, which is the resolution the datasheet quotes and the one any
  // calibration file will have been made against. The array is addressable out
  // to 2472x2064, but those extra rows and columns sit outside the specified
  // imaging area, so they have to be asked for by number rather than arrived at
  // by leaving a parameter at zero.
  if (image_width_ <= 0 && image_height_ <= 0 && offset_x_ < 0 && offset_y_ < 0) {
    RCLCPP_INFO(
      get_logger(), "Keeping the camera's default ROI: %ldx%ld at (%ld, %ld)",
      getInt("Width", 0), getInt("Height", 0), getInt("OffsetX", 0), getInt("OffsetY", 0));
    return;
  }

  // Offsets are zeroed first, because a non-zero offset caps the maximum width
  // and height that GenApi will accept.
  setInt("OffsetX", 0);
  setInt("OffsetY", 0);

  // A zero dimension here keeps the current one, so changing only the width, or
  // only the offset, does not silently resize the other axis.
  const int64_t width = (image_width_ > 0) ? image_width_ : getInt("Width", 0);
  const int64_t height = (image_height_ > 0) ? image_height_ : getInt("Height", 0);
  if (width > 0) {
    setInt("Width", width);
  }
  if (height > 0) {
    setInt("Height", height);
  }

  // -1 means centre, which the camera can do itself and does exactly.
  if (offset_x_ < 0) {
    execute("BslCenterX");
  } else {
    setInt("OffsetX", offset_x_);
  }
  if (offset_y_ < 0) {
    execute("BslCenterY");
  } else {
    setInt("OffsetY", offset_y_);
  }
}


void Ace2CameraWrapperNode::configureExposure()
{
  // ExposureTime is read-only while ExposureAuto is Continuous, so the auto
  // function is switched off, the manual value and the auto bounds are written,
  // and only then is the requested auto mode restored.
  setEnum("ExposureAuto", "Off");
  setFloat("ExposureTime", get_parameter("exposure_time").as_double());
  setFloat("AutoExposureTimeLowerLimit", get_parameter("exposure_auto_lower_limit").as_double());
  setFloat("AutoExposureTimeUpperLimit", get_parameter("exposure_auto_upper_limit").as_double());
  setEnum("ExposureAuto", enumParam("exposure_auto"));
}


void Ace2CameraWrapperNode::configureGain()
{
  setEnum("GainAuto", "Off");
  setFloat("Gain", get_parameter("gain").as_double());
  setFloat("AutoGainLowerLimit", get_parameter("gain_auto_lower_limit").as_double());
  setFloat("AutoGainUpperLimit", get_parameter("gain_auto_upper_limit").as_double());
  setFloat("AutoTargetBrightness", get_parameter("auto_target_brightness").as_double());
  setEnum("GainAuto", enumParam("gain_auto"));
}


void Ace2CameraWrapperNode::configureColor()
{
  setFloat("BlackLevel", get_parameter("black_level").as_double());
  setFloat("Gamma", get_parameter("gamma").as_double());

  // The light source preset writes the white balance and colour transformation,
  // so it goes before any explicit balance ratio.
  setEnum("BslLightSourcePreset", enumParam("light_source_preset"));
  setEnum("BslColorSpace", enumParam("color_space"));

  setEnum("BalanceWhiteAuto", "Off");
  setBalanceRatio("Red", get_parameter("balance_ratio_red").as_double());
  setBalanceRatio("Green", get_parameter("balance_ratio_green").as_double());
  setBalanceRatio("Blue", get_parameter("balance_ratio_blue").as_double());
  setEnum("BalanceWhiteAuto", enumParam("balance_white_auto"));

  setFloat("BslSaturation", get_parameter("saturation").as_double());
  setFloat("BslHue", get_parameter("hue").as_double());
  setFloat("BslContrast", get_parameter("contrast").as_double());
  setFloat("BslBrightness", get_parameter("brightness").as_double());
}


  // BalanceRatio is a single node behind BalanceRatioSelector, so the channel has
  // to be selected before each write.
bool Ace2CameraWrapperNode::setBalanceRatio(const char * channel, double value)
{
  if (!setEnum("BalanceRatioSelector", channel)) {
    return false;
  }
  return setFloat("BalanceRatio", value);
}


void Ace2CameraWrapperNode::configureFrameRate()
{
  if (frame_rate_ > 0.0) {
    setBool("AcquisitionFrameRateEnable", true);
    setFloat("AcquisitionFrameRate", frame_rate_);

    // Auto exposure will stretch past the frame period to reach its brightness
    // target, and the camera then drops the rate to match. This is the most common
    // reason a requested frame rate is not met. Warn rather than override it, since
    // trading rate for a brighter image is sometimes what was wanted.
    const double frame_period_us = 1e6 / frame_rate_;
    const double upper = get_parameter("exposure_auto_upper_limit").as_double();
    if (enumParam("exposure_auto") != "Off" && upper > frame_period_us) {
      RCLCPP_WARN(
        get_logger(),
        "exposure_auto_upper_limit is %.0f us, but %.1f fps leaves only %.0f us per frame. "
        "In a dim scene the auto exposure will hold the rate down to about %.1f fps. "
        "Lower exposure_auto_upper_limit to keep the full rate.",
        upper, frame_rate_, frame_period_us, 1e6 / upper);
    }
  } else {
    setBool("AcquisitionFrameRateEnable", false);
    RCLCPP_INFO(get_logger(), "Frame rate is free-running (limited by exposure and link budget)");
  }
}


void Ace2CameraWrapperNode::configureTrigger()
{
  setEnum("AcquisitionMode", "Continuous");
  setEnum("TriggerSelector", "FrameStart");
  const std::string mode = enumParam("trigger_mode");
  if (mode == "On") {
    setEnum("TriggerSource", get_parameter("trigger_source").as_string());
  }
  setEnum("TriggerMode", mode);
  // Remembered because a grab timeout means something completely different in
  // the two modes: a stalled stream when free-running, and simply "no trigger
  // yet" when triggered.
  triggered_ = (mode == "On");
}


  // USB transport tuning lives in the stream grabber node map, not the device's.
void Ace2CameraWrapperNode::configureTransport()
{
  camera_->MaxNumBuffer = static_cast<int>(std::max<int64_t>(1, max_num_buffer_));

  GenApi::INodeMap & stream = camera_->GetStreamGrabberNodeMap();
  const auto set_stream_int = [&](const char * feature, int64_t value) {
      if (value <= 0) {
        return;
      }
      try {
        GenApi::CIntegerPtr node(stream.GetNode(feature));
        if (node.IsValid() && GenApi::IsWritable(node)) {
          node->SetValue(clamp_to_increment(value, node->GetMin(), node->GetMax(), node->GetInc()));
          RCLCPP_INFO(get_logger(), "%s = %ld", feature, node->GetValue());
        }
      } catch (const Pylon::GenericException & e) {
        RCLCPP_WARN(get_logger(), "%s could not be set: %s", feature, e.GetDescription());
      }
    };
  set_stream_int("MaxTransferSize", max_transfer_size_);
  set_stream_int("NumMaxQueuedUrbs", num_max_queued_urbs_);
}


  // ------------------------------------------------------- output format setup

  // Startup validation, no camera needed: pixel format names resolve statically,
  // so a bad output_encoding / pixel_format pair is caught before the node claims
  // to be running.
void Ace2CameraWrapperNode::validateOutputEncoding()
{
  if (output_encoding_ != "passthrough" && output_encoding_ != "bgr8" &&
    output_encoding_ != "rgb8" && output_encoding_ != "mono8")
  {
    throw std::runtime_error(
      "unsupported output_encoding '" + output_encoding_ +
      "'. Use bgr8, rgb8, mono8 or passthrough.");
  }

  const Pylon::EPixelType requested =
    Pylon::CPixelTypeMapper::GetPylonPixelTypeByName(pixel_format_.c_str());
  if (requested == Pylon::PixelType_Undefined) {
    throw std::runtime_error("pixel_format '" + pixel_format_ + "' is not a known pylon pixel format");
  }

  std::string encoding;
  if (output_encoding_ == "passthrough" && !ros_encoding_for(requested, encoding)) {
    throw std::runtime_error(
      "output_encoding 'passthrough' cannot represent pixel_format '" + pixel_format_ +
      "' as a ROS encoding. Packed formats such as Mono10p or BayerRG12p have no ROS "
      "equivalent: pick an unpacked pixel_format, or set output_encoding to bgr8, rgb8 or mono8.");
  }
}


  // Decides, once per connection, whether frames are converted or forwarded and
  // which ROS encoding the result carries. This reads back the format the camera
  // actually accepted rather than the one that was asked for, so a rejected
  // pixel_format cannot silently mislabel the published bytes.
void Ace2CameraWrapperNode::resolveOutputFormat()
{
  Pylon::EPixelType camera_type = Pylon::PixelType_Undefined;
  try {
    GenApi::CEnumerationPtr format(node_map_->GetNode("PixelFormat"));
    if (format.IsValid() && GenApi::IsReadable(format)) {
      const std::string actual = format->ToString().c_str();
      if (actual != pixel_format_) {
        RCLCPP_WARN(
          get_logger(), "Camera is in PixelFormat '%s', not the requested '%s'",
          actual.c_str(), pixel_format_.c_str());
      }
      camera_type = Pylon::CPixelTypeMapper::GetPylonPixelTypeByName(actual.c_str());
    }
  } catch (const Pylon::GenericException & e) {
    RCLCPP_WARN(get_logger(), "Could not read back PixelFormat: %s", e.GetDescription());
  }

  if (output_encoding_ == "passthrough") {
    std::string encoding;
    if (!ros_encoding_for(camera_type, encoding)) {
      throw std::runtime_error(
        "the camera's active pixel format has no ROS equivalent, so 'passthrough' "
        "cannot be used. Set output_encoding to bgr8, rgb8 or mono8.");
    }
    setPublishedEncoding(encoding);
    convert_ = false;
    RCLCPP_INFO(get_logger(), "Publishing camera frames unconverted as '%s'", encoding.c_str());
    return;
  }

  Pylon::EPixelType target = Pylon::PixelType_Undefined;
  std::string encoding;
  if (output_encoding_ == "bgr8") {
    target = Pylon::PixelType_BGR8packed;
    encoding = sensor_msgs::image_encodings::BGR8;
  } else if (output_encoding_ == "rgb8") {
    target = Pylon::PixelType_RGB8packed;
    encoding = sensor_msgs::image_encodings::RGB8;
  } else if (output_encoding_ == "mono8") {
    target = Pylon::PixelType_Mono8;
    encoding = sensor_msgs::image_encodings::MONO8;
  } else {
    throw std::runtime_error(
      "unsupported output_encoding '" + output_encoding_ +
      "'. Use bgr8, rgb8, mono8 or passthrough.");
  }

  converter_.OutputPixelFormat = target;
  // MSB alignment scales a 10 or 12 bit source up to fill the 8 bit output
  // rather than keeping it in the bottom bits and looking almost black.
  converter_.OutputBitAlignment = Pylon::OutputBitAlignment_MsbAligned;
  // Extend must not be Clip here. Clip drops the outermost row and column that
  // demosaicing cannot reconstruct, so a 2448x2048 Bayer frame converts to
  // 2447x2047 while the Image message still advertises 2448x2048, and every
  // consumer reads a sheared picture. Extend extrapolates those edges instead
  // and keeps the converted geometry identical to the source.
  converter_.InconvertibleEdgeHandling = Pylon::InconvertibleEdgeHandling_Extend;
  convert_ = true;
  setPublishedEncoding(encoding);

  RCLCPP_INFO(
    get_logger(), "Converting camera frames to '%s' on the host", encoding.c_str());
}


  // Records the encoding the converter produces, together with the name the
  // published frame carries once the orientation has shifted a Bayer phase.
  // Both are written here on the grab thread and read there in publishFrame,
  // and orientation_ is fixed at startup, so none of it needs a lock.
void Ace2CameraWrapperNode::setPublishedEncoding(const std::string & encoding)
{
  ros_encoding_ = encoding;
  published_encoding_ = reorientEncoding(encoding, orientation_);
}


  // ------------------------------------------------------------- orientation

bool Ace2CameraWrapperNode::validRotation(int64_t rotation)
{
  return rotation == 0 || rotation == 90 || rotation == 180 || rotation == 270;
}


  // The eight ways of turning a rectangle form a group, so the mirrors and the
  // rotation collapse into a single member of it and the frame is only ever
  // walked once. Writing a rotation as "transpose, then mirror" is what makes
  // that collapse a couple of exclusive ors:
  //
  //   90 clockwise  = transpose, then mirror horizontally
  //   180           = mirror horizontally and vertically
  //   270 clockwise = transpose, then mirror vertically
  //
  // The user's mirrors happen first, in sensor coordinates, so they have to be
  // pushed through the transpose to reach the canonical order. A transpose turns
  // a horizontal mirror into a vertical one and vice versa, which is the whole
  // of the bookkeeping below.
Ace2CameraWrapperNode::Orientation Ace2CameraWrapperNode::orientationOf(
  int64_t rotation, bool flip_horizontal, bool flip_vertical)
{
  Orientation o;
  switch (rotation) {
    case 90:  o = {true, true, false}; break;
    case 180: o = {false, true, true}; break;
    case 270: o = {true, false, true}; break;
    default:  o = {false, false, false}; break;
  }

  if (o.transpose) {
    o.flip_x = (o.flip_x != flip_vertical);
    o.flip_y = (o.flip_y != flip_horizontal);
  } else {
    o.flip_x = (o.flip_x != flip_horizontal);
    o.flip_y = (o.flip_y != flip_vertical);
  }
  return o;
}


namespace
{

// The transposing half of the turn, with everything the inner loop depends on
// fixed at compile time. That matters here and nowhere else: this loop evaluates
// the flip once *per pixel*, and with the flag constant the compiler strength
// reduces it into a pointer walking by plus or minus a stride instead of
// recomputing an index. It is worth 20-38% of a 5 MP rotation, and it is the
// whole of what compile-time constants can buy - a switch reaching these
// instantiations is exactly as fast as hard-coding the orientation would be.
template<bool FLIP_X, bool FLIP_Y, size_t N>
void transpose_tiled(
  const uint8_t * src, size_t src_stride, uint32_t src_width, uint32_t src_height,
  size_t pixel_bytes, uint8_t * dst, size_t dst_stride)
{
  const size_t n = (N != 0) ? N : pixel_bytes;

  // Consecutive destination pixels come from consecutive source *rows*, so
  // walking a whole destination row at a time touches a new cache line for every
  // pixel and, on a 5 MP frame, thrashes the TLB as well. Going tile by tile
  // instead keeps 32 x 32 pixels of each buffer resident, a few kilobytes either
  // side, and roughly halves the time again.
  constexpr uint32_t kTile = 32;
  const uint32_t dst_width = src_height;
  const uint32_t dst_height = src_width;
  for (uint32_t y0 = 0; y0 < dst_height; y0 += kTile) {
    const uint32_t y1 = std::min(y0 + kTile, dst_height);
    for (uint32_t x0 = 0; x0 < dst_width; x0 += kTile) {
      const uint32_t x1 = std::min(x0 + kTile, dst_width);
      for (uint32_t y = y0; y < y1; ++y) {
        // The transpose sends destination row y to a source column and
        // destination column x to a source row; the mirrors only reverse which.
        const uint32_t src_col = FLIP_Y ? dst_height - 1 - y : y;
        uint8_t * d = dst + static_cast<size_t>(y) * dst_stride + static_cast<size_t>(x0) * n;
        for (uint32_t x = x0; x < x1; ++x, d += n) {
          const uint32_t src_row = FLIP_X ? dst_width - 1 - x : x;
          std::memcpy(
            d, src + static_cast<size_t>(src_row) * src_stride + static_cast<size_t>(src_col) * n,
            n);
        }
      }
    }
  }
}


// The pixel size is a template parameter rather than an argument because the
// inner copy is essentially the whole cost of this: with N known the compiler
// turns it into a couple of loads and stores instead of a call into memcpy, and
// a 90 degree rotation of a full 5 MP bgr8 frame drops from 8.8 ms to 4.5 ms on
// this machine. N == 0 means "take it from pixel_bytes", which is the fallback
// for a size the switch below does not name.
template<size_t N>
void orient_pixels(
  bool transpose, bool flip_x, bool flip_y,
  const uint8_t * src, size_t src_stride, uint32_t src_width, uint32_t src_height,
  size_t pixel_bytes, uint8_t * dst, size_t dst_stride)
{
  if (transpose) {
    // Lift the two flips to compile time for the loop that reads them per pixel.
    // The non-transposing path below is deliberately left with runtime flags:
    // there the flip is read once per *row*, so doing the same to it measured as
    // noise and would only have doubled the instantiations.
    const int which = (flip_x ? 1 : 0) | (flip_y ? 2 : 0);
    switch (which) {
      case 0:
        transpose_tiled<false, false, N>(
          src, src_stride, src_width, src_height, pixel_bytes, dst, dst_stride);
        return;
      case 1:
        transpose_tiled<true, false, N>(
          src, src_stride, src_width, src_height, pixel_bytes, dst, dst_stride);
        return;
      case 2:
        transpose_tiled<false, true, N>(
          src, src_stride, src_width, src_height, pixel_bytes, dst, dst_stride);
        return;
      default:
        transpose_tiled<true, true, N>(
          src, src_stride, src_width, src_height, pixel_bytes, dst, dst_stride);
        return;
    }
  }

  const size_t n = (N != 0) ? N : pixel_bytes;

  // Rows keep their length, so a destination row is one memcpy when the columns
  // are not mirrored, and a walk backwards along the source row when they are.
  // Either way this stays sequential in both buffers.
  for (uint32_t y = 0; y < src_height; ++y) {
    const uint8_t * s = src + static_cast<size_t>(flip_y ? src_height - 1 - y : y) * src_stride;
    uint8_t * d = dst + static_cast<size_t>(y) * dst_stride;
    if (!flip_x) {
      std::memcpy(d, s, static_cast<size_t>(src_width) * n);
    } else {
      const uint8_t * back = s + static_cast<size_t>(src_width - 1) * n;
      for (uint32_t x = 0; x < src_width; ++x, d += n, back -= n) {
        std::memcpy(d, back, n);
      }
    }
  }
}

}  // namespace


void Ace2CameraWrapperNode::applyOrientation(
  const Orientation & o, const uint8_t * src, size_t src_stride,
  uint32_t src_width, uint32_t src_height, size_t pixel_bytes,
  uint8_t * dst, size_t dst_stride)
{
  // Everything this node publishes is 1, 2, 3 or 4 bytes per pixel. The generic
  // case is only here so that an unexpected size is still turned correctly
  // rather than being a silent hole.
  switch (pixel_bytes) {
    case 1:
      orient_pixels<1>(
        o.transpose, o.flip_x, o.flip_y, src, src_stride, src_width, src_height,
        pixel_bytes, dst, dst_stride);
      return;
    case 2:
      orient_pixels<2>(
        o.transpose, o.flip_x, o.flip_y, src, src_stride, src_width, src_height,
        pixel_bytes, dst, dst_stride);
      return;
    case 3:
      orient_pixels<3>(
        o.transpose, o.flip_x, o.flip_y, src, src_stride, src_width, src_height,
        pixel_bytes, dst, dst_stride);
      return;
    case 4:
      orient_pixels<4>(
        o.transpose, o.flip_x, o.flip_y, src, src_stride, src_width, src_height,
        pixel_bytes, dst, dst_stride);
      return;
    default:
      orient_pixels<0>(
        o.transpose, o.flip_x, o.flip_y, src, src_stride, src_width, src_height,
        pixel_bytes, dst, dst_stride);
      return;
  }
}


std::string Ace2CameraWrapperNode::reorientEncoding(
  const std::string & encoding, const Orientation & o)
{
  // Only a Bayer name carries a phase. Every other encoding here is a whole
  // self-contained pixel per pixel and is named the same whichever way up the
  // image is.
  static const std::string kPrefix = "bayer_";
  if (o.identity() || encoding.size() < kPrefix.size() + 4 ||
    encoding.compare(0, kPrefix.size(), kPrefix) != 0)
  {
    return encoding;
  }

  // The four letters are the colours of the 2 x 2 tile in row-major order. Width
  // and Height both have inc = 2 on this camera, so a mirror shifts the phase by
  // exactly one tile position and the result is always another named pattern
  // rather than something with no ROS equivalent.
  char tile[4] = {
    encoding[kPrefix.size() + 0], encoding[kPrefix.size() + 1],
    encoding[kPrefix.size() + 2], encoding[kPrefix.size() + 3]};
  if (o.transpose) {
    std::swap(tile[1], tile[2]);
  }
  if (o.flip_x) {
    std::swap(tile[0], tile[1]);
    std::swap(tile[2], tile[3]);
  }
  if (o.flip_y) {
    std::swap(tile[0], tile[2]);
    std::swap(tile[1], tile[3]);
  }

  std::string turned = encoding;
  turned.replace(kPrefix.size(), 4, tile, 4);
  return turned;
}


  // Intrinsics for the turned image. Without this a rotated stream would carry a
  // principal point and a focal length describing an image that is no longer
  // there, and every downstream projection would be quietly wrong.
bool Ace2CameraWrapperNode::reorientCameraInfo(
  sensor_msgs::msg::CameraInfo & info, const Orientation & o)
{
  if (o.identity()) {
    return true;
  }

  // Only the two tangential distortion terms move. Every radial coefficient is a
  // function of the radius alone and does not care which way up the image is,
  // which is why an equidistant D needs nothing done to it at all.
  const bool radtan = info.d.size() >= 4 &&
    (info.distortion_model == "plumb_bob" || info.distortion_model == "rational_polynomial");
  const bool handled = radtan || info.d.empty() ||
    info.distortion_model == "equidistant" || info.distortion_model == "fisheye";

  // An all-zero roi means "the whole image" and has to stay that way.
  const bool has_roi = info.roi.width != 0 && info.roi.height != 0;

  // K is row-major fx 0 cx / 0 fy cy / 0 0 1, and P the same with a fourth
  // column Tx, Ty, 0. Tx and Ty are zero on a monocular camera, so their signs
  // below only matter to someone feeding this a stereo calibration.
  if (o.transpose) {
    std::swap(info.width, info.height);
    std::swap(info.k[0], info.k[4]);
    std::swap(info.k[2], info.k[5]);
    std::swap(info.p[0], info.p[5]);
    std::swap(info.p[2], info.p[6]);
    std::swap(info.p[3], info.p[7]);
    if (radtan) {
      std::swap(info.d[2], info.d[3]);
    }
    if (has_roi) {
      std::swap(info.roi.x_offset, info.roi.y_offset);
      std::swap(info.roi.width, info.roi.height);
    }
  }
  if (o.flip_x) {
    // Pixel index u becomes width - 1 - u, and the principal point is in those
    // same pixel-centre coordinates, so it moves with them.
    info.k[2] = static_cast<double>(info.width) - 1.0 - info.k[2];
    info.p[2] = static_cast<double>(info.width) - 1.0 - info.p[2];
    info.p[3] = -info.p[3];
    if (radtan) {
      info.d[3] = -info.d[3];   // p2, the term that multiplies x
    }
    if (has_roi) {
      info.roi.x_offset = info.width - info.roi.x_offset - info.roi.width;
    }
  }
  if (o.flip_y) {
    info.k[5] = static_cast<double>(info.height) - 1.0 - info.k[5];
    info.p[6] = static_cast<double>(info.height) - 1.0 - info.p[6];
    info.p[7] = -info.p[7];
    if (radtan) {
      info.d[2] = -info.d[2];   // p1, the term that multiplies y
    }
    if (has_roi) {
      info.roi.y_offset = info.height - info.roi.y_offset - info.roi.height;
    }
  }
  return handled;
}


  // ------------------------------------------------------------ timestamping

  // Anchors the 1 GHz device counter to ROS time. Called at stream start and then
  // every timestamp_resync_period seconds, which bounds the drift between the
  // camera's oscillator and the host clock instead of letting it accumulate.
void Ace2CameraWrapperNode::resyncCameraClock()
{
  if (!execute("TimestampLatch")) {
    RCLCPP_WARN_ONCE(
      get_logger(),
      "TimestampLatch is unavailable, falling back to host receive time for stamps");
    std::lock_guard<std::mutex> lock(cfg_mutex_);
    timestamp_source_ = "ros_time";
    return;
  }
  const int64_t ticks = getInt("TimestampLatchValue", 0);
  const int64_t now_ns = now().nanoseconds();
  const int64_t offset = now_ns - ticks;   // 1 tick == 1 ns on this device
  if (clock_offset_valid_.load()) {
    RCLCPP_DEBUG(
      get_logger(), "Camera clock resync moved the offset by %.3f ms",
      1e-6 * static_cast<double>(offset - clock_offset_ns_.load()));
  }
  clock_offset_ns_.store(offset);
  clock_offset_valid_.store(true);
  last_resync_ = std::chrono::steady_clock::now();
}


Ace2CameraWrapperNode::StampConfig Ace2CameraWrapperNode::stampConfig() const
{
  std::lock_guard<std::mutex> lock(cfg_mutex_);
  return StampConfig{
    frame_id_,
    timestamp_source_ == "camera",
    static_cast<int64_t>(std::llround(timestamp_offset_ * 1e9))};
}


rclcpp::Time Ace2CameraWrapperNode::stampFor(const Pylon::CGrabResultPtr & result, const StampConfig & cfg)
{
  if (cfg.use_camera_clock && clock_offset_valid_.load()) {
    const int64_t device_ns = static_cast<int64_t>(result->GetTimeStamp());
    return rclcpp::Time(device_ns + clock_offset_ns_.load() + cfg.offset_ns, RCL_ROS_TIME);
  }
  return now() + rclcpp::Duration(std::chrono::nanoseconds(cfg.offset_ns));
}


  // ---------------------------------------------------------------- grab loop

void Ace2CameraWrapperNode::grabThread()
{
  while (running_ && rclcpp::ok()) {
    try {
      {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        openCamera();
        applyConfiguration();
        configureTransport();
        resolveOutputFormat();
      }
      streamFrames();
    } catch (const Pylon::GenericException & e) {
      RCLCPP_ERROR(get_logger(), "Pylon error: %s", e.GetDescription());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
    }

    closeCamera();

    if (!running_ || !rclcpp::ok()) {
      break;
    }
    RCLCPP_WARN(get_logger(), "Retrying the camera in %.1f s", reconnect_delay_);
    interruptibleSleep(reconnect_delay_);
  }
  closeCamera();
}


void Ace2CameraWrapperNode::streamFrames()
{
  const auto strategy = (grab_strategy_ == "one_by_one")
    ? Pylon::GrabStrategy_OneByOne
    : Pylon::GrabStrategy_LatestImageOnly;
  if (grab_strategy_ != "one_by_one" && grab_strategy_ != "latest_image_only") {
    RCLCPP_WARN(
      get_logger(), "Unknown grab_strategy '%s', using latest_image_only",
      grab_strategy_.c_str());
  }

  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (useCameraClock()) {
      resyncCameraClock();
    }
    camera_->StartGrabbing(strategy);
  }
  RCLCPP_INFO(get_logger(), "Streaming started");

  uint64_t published = 0;
  uint64_t consecutive_failures = 0;
  uint64_t failed_total = 0;
  uint64_t published_at_last_stats = 0;
  auto last_stats = std::chrono::steady_clock::now();

  while (running_ && rclcpp::ok()) {
    Pylon::CGrabResultPtr result;
    bool got_result = false;
    try {
      // RetrieveResult is deliberately called without camera_mutex_ held: it
      // blocks for up to grab_timeout_ms, and holding the lock that long would
      // stall the temperature timer and the software-trigger service.
      got_result = camera_->RetrieveResult(
        static_cast<unsigned int>(grab_timeout_ms_), result, Pylon::TimeoutHandling_Return);
    } catch (const Pylon::GenericException & e) {
      RCLCPP_ERROR(get_logger(), "RetrieveResult failed: %s", e.GetDescription());
      return;
    }

    if (camera_->IsCameraDeviceRemoved()) {
      RCLCPP_ERROR(get_logger(), "Camera was removed from the bus");
      return;
    }

    if (!got_result) {
      if (triggered_) {
        // Idle by design between triggers, so this is not evidence of anything
        // wrong and must not count towards the stall threshold. Treating it as a
        // stall would put a quiet triggered camera into a reconnect loop.
        RCLCPP_DEBUG(
          get_logger(), "No frame within %ld ms, waiting for a trigger (%lu published so far)",
          grab_timeout_ms_, static_cast<unsigned long>(published));
        continue;
      }
      RCLCPP_WARN(
        get_logger(), "No frame within %ld ms (%lu published so far)",
        grab_timeout_ms_, static_cast<unsigned long>(published));
      if (++consecutive_failures >= 3) {
        RCLCPP_ERROR(get_logger(), "Stream appears stalled, reconnecting");
        return;
      }
      continue;
    }

    if (!result->GrabSucceeded()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Grab failed: %s",
        result->GetErrorDescription().c_str());
      ++failed_total;
      if (++consecutive_failures >= 100) {
        RCLCPP_ERROR(get_logger(), "Too many consecutive failed grabs, reconnecting");
        return;
      }
      continue;
    }
    consecutive_failures = 0;

    publishFrame(result);
    ++published;

    if (stats_log_period_ > 0.0) {
      const double since =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_stats).count();
      if (since >= stats_log_period_) {
        const uint64_t n = published - published_at_last_stats;
        RCLCPP_INFO(
          get_logger(), "Grabbed %lu frames in %.2f s = %.2f fps (%lu failed grabs total)",
          static_cast<unsigned long>(n), since, static_cast<double>(n) / since,
          static_cast<unsigned long>(failed_total));
        last_stats = std::chrono::steady_clock::now();
        published_at_last_stats = published;
      }
    }

    if (useCameraClock() && timestamp_resync_period_ > 0.0) {
      const double since = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_resync_).count();
      if (since >= timestamp_resync_period_) {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        resyncCameraClock();
      }
    }
  }

  std::lock_guard<std::mutex> lock(camera_mutex_);
  if (camera_ && camera_->IsGrabbing()) {
    camera_->StopGrabbing();
  }
}


void Ace2CameraWrapperNode::publishFrame(const Pylon::CGrabResultPtr & result)
{
  const StampConfig cfg = stampConfig();

  auto image = std::make_unique<sensor_msgs::msg::Image>();
  image->header.stamp = stampFor(result, cfg);
  image->header.frame_id = cfg.frame_id;
  image->encoding = published_encoding_;
  image->is_bigendian = 0;

  // Geometry as the frame comes off the camera. The published geometry is this
  // one after the orientation, which swaps the two for a 90 or 270 rotation.
  const uint32_t src_width = static_cast<uint32_t>(result->GetWidth());
  const uint32_t src_height = static_cast<uint32_t>(result->GetHeight());

  // Where the orientation pass reads from, and the row stride it reads with.
  // With no orientation to apply neither is used: the frame goes straight into
  // the message exactly as it did before, so the common case pays nothing.
  const uint8_t * src = nullptr;
  size_t src_step = 0;

  try {
    if (convert_ && !converter_.ImageHasDestinationFormat(result)) {
      const size_t size = converter_.GetBufferSizeForConversion(result);

      // Because the converter extends rather than clips the inconvertible edge,
      // the output has the same row count as the source and the stride divides
      // exactly. If that ever stops holding, the geometry in the header would no
      // longer describe the bytes, so drop the frame instead of publishing a
      // picture that only looks plausible.
      if (src_height == 0 || size % src_height != 0) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Converted buffer of %zu bytes does not divide into %u rows, dropping the frame",
          size, src_height);
        return;
      }
      src_step = size / src_height;

      if (orientation_.identity()) {
        image->data.resize(size);
        // Convert straight into the message buffer, so debayering costs one pass
        // and no extra copy.
        converter_.Convert(image->data.data(), size, result);
      } else {
        // The orientation pass has to read from somewhere other than what it
        // writes, so the conversion lands in a buffer that is reused frame to
        // frame and the turn is the copy into the message.
        convert_buffer_.resize(size);
        converter_.Convert(convert_buffer_.data(), size, result);
        src = convert_buffer_.data();
      }
    } else {
      // Either passthrough, or the camera already delivers the target format.
      size_t stride = 0;
      if (!result->GetStride(stride)) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Frame has no row stride (packed pixel format), dropping it");
        return;
      }
      src_step = stride;
      if (orientation_.identity()) {
        const size_t size = stride * src_height;
        image->data.resize(size);
        std::memcpy(image->data.data(), result->GetBuffer(), size);
      } else {
        // Nothing was copied yet, so the turn reads the grab buffer directly and
        // this path costs no more than the plain memcpy it replaces.
        src = static_cast<const uint8_t *>(result->GetBuffer());
      }
    }
  } catch (const Pylon::GenericException & e) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "Frame conversion failed: %s", e.GetDescription());
    return;
  }

  if (orientation_.identity()) {
    image->width = src_width;
    image->height = src_height;
    image->step = static_cast<uint32_t>(src_step);
  } else {
    // Every encoding this node publishes is a whole number of bytes per pixel and
    // the orientation moves whole pixels, so a stride that does not divide by the
    // width would mean the buffer is not the shape we think it is.
    if (src_width == 0 || src_step % src_width != 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Row stride of %zu bytes does not divide into %u pixels, dropping the frame",
        src_step, src_width);
      return;
    }
    const size_t pixel_bytes = src_step / src_width;
    image->width = orientation_.transpose ? src_height : src_width;
    image->height = orientation_.transpose ? src_width : src_height;
    image->step = static_cast<uint32_t>(image->width * pixel_bytes);
    image->data.resize(static_cast<size_t>(image->step) * image->height);
    applyOrientation(
      orientation_, src, src_step, src_width, src_height, pixel_bytes,
      image->data.data(), image->step);
  }

  auto info = std::make_unique<sensor_msgs::msg::CameraInfo>(info_manager_->getCameraInfo());
  info->header = image->header;
  if (info->width == 0 || info->height == 0) {
    // Uncalibrated: publish the geometry so downstream nodes at least agree on
    // the image size, and say once why the intrinsics are empty.
    info->width = image->width;
    info->height = image->height;
    RCLCPP_WARN_ONCE(
      get_logger(),
      "No calibration loaded, CameraInfo carries only the image size. Set camera_info_url, "
      "or run camera_calibration and store the result via the set_camera_info service.");
  } else {
    // The calibration describes the sensor, so it has to be turned the same way
    // the pixels were before it describes what is being published.
    if (!reorientCameraInfo(*info, orientation_)) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "Distortion model '%s' has coefficients this node cannot rewrite for an oriented "
        "image, so K and P were turned but D was left as it is.",
        info->distortion_model.c_str());
    }
    if (info->width != image->width || info->height != image->height) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "Calibration is for %ux%u but frames are %ux%u. The intrinsics do not apply to this ROI.",
        info->width, info->height, image->width, image->height);
    }
  }

  if (use_image_transport_) {
    // image_transport dereferences and copies whatever it is given, on every
    // overload, so there is nothing to gain from moving here.
    camera_pub_.publish(*image, *info);
  } else {
    // Publishing the unique_ptr hands ownership to the middleware. With
    // intra-process comms on and the subscriber in the same container, the 15 MB
    // frame is moved rather than copied or serialised.
    image_pub_->publish(std::move(image));
    info_pub_->publish(std::move(info));
  }
}


bool Ace2CameraWrapperNode::useCameraClock() const
{
  std::lock_guard<std::mutex> lock(cfg_mutex_);
  return timestamp_source_ == "camera";
}


void Ace2CameraWrapperNode::interruptibleSleep(double seconds)
{
  std::unique_lock<std::mutex> lock(sleep_mutex_);
  sleep_cv_.wait_for(
    lock, std::chrono::duration<double>(seconds), [this] {return !running_;});
}


  // ----------------------------------------------------------------- services

void Ace2CameraWrapperNode::onSoftwareTrigger(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(camera_mutex_);
  if (!camera_ || !camera_->IsGrabbing()) {
    response->success = false;
    response->message = "camera is not grabbing";
    return;
  }
  try {
    if (!camera_->WaitForFrameTriggerReady(1000, Pylon::TimeoutHandling_Return)) {
      response->success = false;
      response->message = "camera was not ready for a frame trigger within 1000 ms";
      return;
    }
    camera_->ExecuteSoftwareTrigger();
    response->success = true;
    response->message = "software trigger executed";
  } catch (const Pylon::GenericException & e) {
    response->success = false;
    response->message = e.GetDescription();
  }
}


void Ace2CameraWrapperNode::publishTemperature()
{
  std::lock_guard<std::mutex> lock(camera_mutex_);
  if (!camera_ || !camera_->IsOpen() || node_map_ == nullptr) {
    return;
  }
  sensor_msgs::msg::Temperature msg;
  msg.header.stamp = now();
  msg.header.frame_id = frame_id_;
  msg.temperature = getFloat("DeviceTemperature", 0.0);
  msg.variance = 0.0;
  temperature_pub_->publish(msg);
}


  // ------------------------------------------------------- runtime reconfigure

  // Only the knobs worth tuning while looking at the image are live. Anything
  // that changes the buffer layout (pixel_format, ROI) or the transport is read
  // once per connection, so those are rejected here rather than half-applied.
rcl_interfaces::msg::SetParametersResult Ace2CameraWrapperNode::onSetParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  std::lock_guard<std::mutex> lock(camera_mutex_);
  const bool camera_ready = camera_ && camera_->IsOpen() && node_map_ != nullptr;

  for (const auto & p : parameters) {
    const std::string & name = p.get_name();

    // Plain node-level settings, no camera access needed.
    if (name == "frame_id") {
      std::lock_guard<std::mutex> cfg_lock(cfg_mutex_);
      frame_id_ = p.as_string();
      continue;
    }
    if (name == "timestamp_offset") {
      std::lock_guard<std::mutex> cfg_lock(cfg_mutex_);
      timestamp_offset_ = p.as_double();
      continue;
    }
    if (name == "timestamp_source") {
      const std::string value = p.as_string();
      if (value != "ros_time" && value != "camera") {
        result.successful = false;
        result.reason = "timestamp_source must be ros_time or camera";
        return result;
      }
      {
        std::lock_guard<std::mutex> cfg_lock(cfg_mutex_);
        timestamp_source_ = value;
      }
      if (value == "camera" && camera_ready) {
        resyncCameraClock();
      }
      continue;
    }

    static const std::vector<std::string> reconfigurable = {
      "exposure_auto", "exposure_time", "exposure_auto_lower_limit",
      "exposure_auto_upper_limit", "gain_auto", "gain", "gain_auto_lower_limit",
      "gain_auto_upper_limit", "auto_target_brightness", "black_level", "gamma",
      "balance_white_auto", "balance_ratio_red", "balance_ratio_green",
      "balance_ratio_blue", "light_source_preset", "color_space", "saturation",
      "hue", "contrast", "brightness", "frame_rate", "reverse_x", "reverse_y"};
    if (std::find(reconfigurable.begin(), reconfigurable.end(), name) == reconfigurable.end()) {
      result.successful = false;
      result.reason = "'" + name +
        "' is only read when the camera is opened. Relaunch the node to change it.";
      return result;
    }

    if (!camera_ready) {
      result.successful = false;
      result.reason = "camera is not open yet";
      return result;
    }

    // The dynamically typed enum parameters accept a bool or a string and
    // nothing else, so an int or a double has to be turned away here rather
    // than throwing out of as_string() below.
    static const std::vector<std::string> enum_params = {
      "exposure_auto", "gain_auto", "balance_white_auto", "light_source_preset",
      "color_space"};
    if (std::find(enum_params.begin(), enum_params.end(), name) != enum_params.end() &&
      p.get_type() != rclcpp::ParameterType::PARAMETER_STRING &&
      p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
    {
      result.successful = false;
      result.reason = "'" + name + "' must be a string (or a bool for Off/On)";
      return result;
    }

    applyReconfigured(name, p);
  }
  return result;
}


void Ace2CameraWrapperNode::applyReconfigured(const std::string & name, const rclcpp::Parameter & p)
{
  // The auto-function interlocks matter here just as they do at startup: the
  // manual value is only writable while its auto function is Off, so switching
  // a mode off has to re-assert the manual value behind it.
  if (name == "exposure_auto") {
    const std::string mode = enumValueOf(p);
    setEnum("ExposureAuto", mode);
    if (mode == "Off") {
      setFloat("ExposureTime", get_parameter("exposure_time").as_double());
    }
  } else if (name == "exposure_time") {
    setFloat("ExposureTime", p.as_double());
  } else if (name == "exposure_auto_lower_limit") {
    setFloat("AutoExposureTimeLowerLimit", p.as_double());
  } else if (name == "exposure_auto_upper_limit") {
    setFloat("AutoExposureTimeUpperLimit", p.as_double());
  } else if (name == "gain_auto") {
    const std::string mode = enumValueOf(p);
    setEnum("GainAuto", mode);
    if (mode == "Off") {
      setFloat("Gain", get_parameter("gain").as_double());
    }
  } else if (name == "gain") {
    setFloat("Gain", p.as_double());
  } else if (name == "gain_auto_lower_limit") {
    setFloat("AutoGainLowerLimit", p.as_double());
  } else if (name == "gain_auto_upper_limit") {
    setFloat("AutoGainUpperLimit", p.as_double());
  } else if (name == "auto_target_brightness") {
    setFloat("AutoTargetBrightness", p.as_double());
  } else if (name == "black_level") {
    setFloat("BlackLevel", p.as_double());
  } else if (name == "gamma") {
    setFloat("Gamma", p.as_double());
  } else if (name == "balance_white_auto") {
    setEnum("BalanceWhiteAuto", enumValueOf(p));
  } else if (name == "balance_ratio_red") {
    setBalanceRatio("Red", p.as_double());
  } else if (name == "balance_ratio_green") {
    setBalanceRatio("Green", p.as_double());
  } else if (name == "balance_ratio_blue") {
    setBalanceRatio("Blue", p.as_double());
  } else if (name == "light_source_preset") {
    setEnum("BslLightSourcePreset", enumValueOf(p));
  } else if (name == "color_space") {
    setEnum("BslColorSpace", enumValueOf(p));
  } else if (name == "saturation") {
    setFloat("BslSaturation", p.as_double());
  } else if (name == "hue") {
    setFloat("BslHue", p.as_double());
  } else if (name == "contrast") {
    setFloat("BslContrast", p.as_double());
  } else if (name == "brightness") {
    setFloat("BslBrightness", p.as_double());
  } else if (name == "reverse_x") {
    setBool("ReverseX", p.as_bool());
  } else if (name == "reverse_y") {
    setBool("ReverseY", p.as_bool());
  } else if (name == "frame_rate") {
    frame_rate_ = p.as_double();
    configureFrameRate();
  }
}

}  // namespace ace2_camera_wrapper

// Makes the component discoverable by rclcpp_components, so a container can load
// it by name and share a process (and therefore memory) with other components.
RCLCPP_COMPONENTS_REGISTER_NODE(ace2_camera_wrapper::Ace2CameraWrapperNode)
