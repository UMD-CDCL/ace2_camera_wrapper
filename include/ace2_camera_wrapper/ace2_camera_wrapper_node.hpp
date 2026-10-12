#ifndef ACE2_CAMERA_WRAPPER__ACE2_CAMERA_WRAPPER_NODE_HPP_
#define ACE2_CAMERA_WRAPPER__ACE2_CAMERA_WRAPPER_NODE_HPP_

// ROS2 component wrapping the Basler ace 2 a2A2448-75ucPRO (5 MP colour, USB3)
// through the pylon 8 C++ SDK. See README.md for the design rationale, and
// ace2_camera_wrapper_node.cpp for the implementation notes.
//
// This header pulls in the pylon SDK because the camera objects are held by
// value or by unique_ptr below. Downstream packages that only compose the
// component into a container never include this file and therefore never need
// pylon. The exported CMake target carries pylon's include directories for the
// packages that do include it.

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/camera_publisher.hpp>
#include <image_transport/image_transport.hpp>

#include <pylon/PylonIncludes.h>
#include <pylon/ImageFormatConverter.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ace2_camera_wrapper/visibility_control.h"

namespace ace2_camera_wrapper
{

class ACE2_CAMERA_WRAPPER_PUBLIC Ace2CameraWrapperNode : public rclcpp::Node
{

public:

  /// Component entry point. `options` carries the intra-process
  /// setting and any parameter overrides from the container.
  explicit Ace2CameraWrapperNode(const rclcpp::NodeOptions & options);

  ~Ace2CameraWrapperNode() override;


private:

  // ----------------------------------------------------------------- parameters

  template<typename T>
  T declare(const std::string & name, const T & default_value, const std::string & description)
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = description;
    return this->declare_parameter<T>(name, default_value, desc);
  }

  std::string declareEnum(
    const std::string & name, const std::string & default_value, const std::string & description);

  static std::string enumValueOf(const rclcpp::Parameter & p);

  std::string enumParam(const std::string & name) const;

  void declareParameters();

  bool setEnum(const char * feature, const std::string & value);

  bool setFloat(const char * feature, double value);

  bool setInt(const char * feature, int64_t value);

  bool setBool(const char * feature, bool value);

  bool execute(const char * feature);

  double getFloat(const char * feature, double fallback) const;

  int64_t getInt(const char * feature, int64_t fallback) const;

  Pylon::CDeviceInfo selectDevice() const;

  void openCamera();

  void closeCamera();

  void applyConfiguration();

  void configureRoi();

  void configureExposure();

  void configureGain();

  void configureColor();

  bool setBalanceRatio(const char * channel, double value);

  void configureFrameRate();

  void configureTrigger();

  void configureTransport();

  void validateOutputEncoding();

  void resolveOutputFormat();

  void setPublishedEncoding(const std::string & encoding);

  void resyncCameraClock();


  // ---------------------------------------------------------------- orientation

  // How a frame is turned before it is published, for a camera that is mounted
  // rotated or mirrored. image_rotation, flip_horizontal and flip_vertical fold
  // into one element of the symmetry group of the rectangle, so any combination
  // of the three costs exactly one pass over the frame rather than one pass each.
  //
  // Canonical form: transpose first, then mirror about the vertical axis, then
  // mirror about the horizontal axis. Composed once at startup and read on the
  // grab thread thereafter, so turning a frame costs one predictable branch and
  // no synchronisation.
  struct Orientation
  {
    bool transpose{false};
    bool flip_x{false};
    bool flip_y{false};

    bool identity() const {return !transpose && !flip_x && !flip_y;}
  };

  // Folds the user's "mirror in sensor coordinates, then rotate clockwise" into
  // the canonical form above.
  static Orientation orientationOf(int64_t rotation, bool flip_horizontal, bool flip_vertical);

  static bool validRotation(int64_t rotation);

  // Moves the pixels. src and dst must not overlap.
  static void applyOrientation(
    const Orientation & o, const uint8_t * src, size_t src_stride,
    uint32_t src_width, uint32_t src_height, size_t pixel_bytes,
    uint8_t * dst, size_t dst_stride);

  // A Bayer encoding names the phase of the colour tile, and turning the pixels
  // shifts that phase, so the published name has to follow. Any other encoding
  // comes back unchanged.
  static std::string reorientEncoding(const std::string & encoding, const Orientation & o);

  // Rewrites the intrinsics to describe the turned image. Returns false when the
  // distortion model has coefficients this does not know how to rewrite, in
  // which case the geometry is still correct but D is left alone.
  static bool reorientCameraInfo(sensor_msgs::msg::CameraInfo & info, const Orientation & o);


  // Snapshot of the fields the executor thread may change under us, taken once
  // per frame so the grab thread never reads a half-written std::string.
  struct StampConfig
  {
    std::string frame_id;
    bool use_camera_clock;
    int64_t offset_ns;
  };

  StampConfig stampConfig() const;

  rclcpp::Time stampFor(const Pylon::CGrabResultPtr & result, const StampConfig & cfg);

  void grabThread();

  void streamFrames();

  void publishFrame(const Pylon::CGrabResultPtr & result);

  // A small bgr8 copy of the frame on ace2/preview, at most preview_max_hz, for the
  // operator video stream (see preview.hpp). Grab thread only.
  void publishPreview(const sensor_msgs::msg::Image & image);

  bool useCameraClock() const;

  void interruptibleSleep(double seconds);

  void onSoftwareTrigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void publishTemperature();

  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  void applyReconfigured(const std::string & name, const rclcpp::Parameter & p);

  // ------------------------------------------------------------------- members

  // Parameters cached at startup because they are only read when connecting.
  std::string device_serial_;
  std::string device_user_id_;
  std::string camera_name_;
  std::string frame_id_;
  std::string camera_info_url_;
  std::string qos_reliability_;
  int64_t qos_depth_{1};
  bool use_image_transport_{false};
  std::string startup_user_set_;
  std::string pixel_format_;
  std::string output_encoding_;
  double frame_rate_{30.0};
  std::string grab_strategy_;
  int64_t grab_timeout_ms_{5000};
  int64_t max_num_buffer_{16};
  double reconnect_delay_{2.0};
  int64_t image_width_{0};
  int64_t image_height_{0};
  int64_t offset_x_{-1};
  int64_t offset_y_{-1};
  int64_t image_rotation_{0};
  bool flip_horizontal_{false};
  bool flip_vertical_{false};
  Orientation orientation_;
  int64_t device_link_throughput_limit_{0};
  int64_t max_transfer_size_{0};
  int64_t num_max_queued_urbs_{0};
  std::string timestamp_source_;
  double timestamp_resync_period_{10.0};
  double timestamp_offset_{0.0};
  double stats_log_period_{0.0};
  double temperature_period_{0.0};

  // ROS side.
  // Only one of these is created. The plain publishers are the zero-copy path.
  // camera_pub_ is the image_transport path, which buys the compressed transports
  // at the cost of a copy per frame.
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  image_transport::CameraPublisher camera_pub_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> info_manager_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr preview_pub_;
  bool publish_preview_{false};
  int64_t preview_downscale_{4};
  double preview_max_hz_{15.0};
  std::chrono::steady_clock::time_point last_preview_{};
  rclcpp::TimerBase::SharedPtr temperature_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // Pylon side. camera_mutex_ guards camera_ and node_map_ against the
  // temperature timer, the trigger service and the parameter callback, all of
  // which run on the executor thread while grab_thread_ owns the stream.
  bool pylon_initialized_{false};
  std::mutex camera_mutex_;
  std::unique_ptr<Pylon::CInstantCamera> camera_;
  GenApi::INodeMap * node_map_{nullptr};
  Pylon::CImageFormatConverter converter_;
  bool triggered_{false};
  bool convert_{true};
  std::string ros_encoding_;
  // ros_encoding_ after the orientation has shifted a Bayer phase, which is what
  // the message actually carries. Equal to ros_encoding_ for everything else.
  std::string published_encoding_;
  // Scratch for the conversion when an orientation follows it, so the converter
  // still writes once and the orientation pass reads from it. Grab thread only.
  std::vector<uint8_t> convert_buffer_;

  // Device-clock anchor for timestamp_source == "camera". Written by the grab
  // thread under camera_mutex_, read by stampFor on the same thread, so atomics
  // are enough to keep the reads well defined.
  std::atomic<int64_t> clock_offset_ns_{0};
  std::atomic<bool> clock_offset_valid_{false};
  std::chrono::steady_clock::time_point last_resync_{};

  // Guards the three settings the parameter callback can change while the grab
  // thread is reading them: frame_id_, timestamp_source_ and timestamp_offset_.
  mutable std::mutex cfg_mutex_;

  std::atomic<bool> running_{true};
  std::thread grab_thread_;
  std::mutex sleep_mutex_;
  std::condition_variable sleep_cv_;
};

}  // namespace ace2_camera_wrapper

#endif  // ACE2_CAMERA_WRAPPER__ACE2_CAMERA_WRAPPER_NODE_HPP_
