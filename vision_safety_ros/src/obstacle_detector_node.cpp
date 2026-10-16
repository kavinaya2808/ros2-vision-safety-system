// Copyright 2026 Kavinaya. MIT License.
//
// obstacle_detector: ROS 2 wrapper around the vision_safety_core perception step.
//
//   subscribes  camera/image_raw     (sensor_msgs/Image, best effort, latest only)
//               camera/camera_info   (sensor_msgs/CameraInfo -> intrinsics)
//   TF          base_frame -> image frame (-> camera height, pitch, x offset)
//   publishes   perception/obstacle      (vision_safety_interfaces/ObstacleDetection)
//               perception/debug_image   (sensor_msgs/Image, only while someone watches)
//
// Fail-safe behaviour: until intrinsics AND extrinsics are known, or whenever
// processing fails, NOTHING is published. The safety monitor then sees stale
// perception and stops the robot. Silence is never interpreted as "clear".

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "vision_safety_core/color_detector.hpp"
#include "vision_safety_core/danger_zone.hpp"
#include "vision_safety_core/debug_overlay.hpp"
#include "vision_safety_interfaces/msg/obstacle_detection.hpp"

namespace vision_safety_ros
{

using vision_safety::CameraGeometry;
using vision_safety::ColorDetector;
using vision_safety::ColorDetectorConfig;
using vision_safety::DangerZone;
using vision_safety::DangerZoneConfig;
using vision_safety::HsvRange;
using vision_safety_interfaces::msg::ObstacleDetection;

namespace
{

// Everything the node reads from parameters, in one place.
struct NodeParams
{
  ColorDetectorConfig color;
  DangerZoneConfig zone;         // camera_to_front_m is derived per frame
  double robot_front_x_m{0.15};  // front edge, in base_frame
  std::string base_frame{"base_footprint"};
  bool use_tf{true};
  double mount_height_m{0.135};  // used only when use_tf is false
  double pitch_rad{0.15};
  double camera_x_m{0.14};
  bool publish_debug_image{true};
};

using ParamGetter = std::function<rclcpp::Parameter(const std::string &)>;

NodeParams read_params(const ParamGetter & get)
{
  NodeParams p;

  const auto ranges = get("color.hsv_ranges").as_integer_array();
  if (ranges.empty() || ranges.size() % 6 != 0) {
    throw std::invalid_argument(
            "color.hsv_ranges needs 6 integers per range: h_min h_max s_min s_max v_min v_max");
  }
  p.color.ranges.clear();
  for (std::size_t i = 0; i < ranges.size(); i += 6) {
    p.color.ranges.push_back(HsvRange{
        static_cast<int>(ranges[i]), static_cast<int>(ranges[i + 1]),
        static_cast<int>(ranges[i + 2]), static_cast<int>(ranges[i + 3]),
        static_cast<int>(ranges[i + 4]), static_cast<int>(ranges[i + 5])});
  }
  p.color.min_area_ratio = get("color.min_area_ratio").as_double();
  p.color.morph_kernel_size = static_cast<int>(get("color.morph_kernel_size").as_int());

  p.zone.stop_gap_m = get("zone.stop_gap_m").as_double();
  p.zone.path_half_width_m = get("zone.path_half_width_m").as_double();
  p.zone.large_blob_area_ratio = get("zone.large_blob_area_ratio").as_double();
  p.zone.clip_margin_px = static_cast<int>(get("zone.clip_margin_px").as_int());
  p.robot_front_x_m = get("zone.robot_front_x_m").as_double();

  p.base_frame = get("base_frame").as_string();
  p.use_tf = get("camera.use_tf").as_bool();
  p.mount_height_m = get("camera.mount_height_m").as_double();
  p.pitch_rad = get("camera.pitch_rad").as_double();
  p.camera_x_m = get("camera.x_m").as_double();
  p.publish_debug_image = get("publish_debug_image").as_bool();

  // Reuse the core library's validation so the rules live in one place.
  vision_safety::validate(p.color);
  vision_safety::validate(p.zone);
  if (!p.use_tf) {
    CameraGeometry probe;
    probe.mount_height_m = p.mount_height_m;
    probe.pitch_rad = p.pitch_rad;
    vision_safety::validate(probe);
  }
  if (p.base_frame.empty()) {
    throw std::invalid_argument("base_frame must not be empty");
  }
  return p;
}

struct Extrinsics
{
  double height_m;
  double pitch_rad;
  double x_m;
};

}  // namespace

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  explicit ObstacleDetectorNode(const rclcpp::NodeOptions & options)
  : Node("obstacle_detector", options)
  {
    declare_all_parameters();

    // Validate the startup configuration; a bad YAML file stops the node here
    // with a clear message instead of misbehaving later.
    params_ = read_params([this](const std::string & n) {return get_parameter(n);});
    detector_ = std::make_shared<const ColorDetector>(params_.color);

    // Live tuning: validate first (reject bad values with a reason), apply after.
    on_set_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & changes) {return validate_changes(changes);});
    post_set_handle_ = add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &) {apply_parameters();});

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    detection_pub_ = create_publisher<ObstacleDetection>(
      "perception/obstacle", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "perception/debug_image", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    // Latest frame only: if processing ever falls behind, drop old frames
    // rather than queueing them (stale images are worse than skipped ones).
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera/camera_info", sensor_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {on_camera_info(*msg);});
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", sensor_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_image(msg);});

    RCLCPP_INFO(get_logger(), "obstacle_detector ready, waiting for camera_info and TF");
  }

private:
  // ------------------------------------------------------------------ params
  template<typename T>
  void declare(const std::string & name, const T & default_value, const std::string & description)
  {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = description;
    declare_parameter<T>(name, default_value, d);
  }

  void declare_all_parameters()
  {
    declare<std::vector<int64_t>>(
      "color.hsv_ranges", {0, 10, 120, 255, 70, 255, 170, 179, 120, 255, 70, 255},
      "HSV bands, 6 ints each: h_min h_max s_min s_max v_min v_max (H 0-179)");
    declare<double>("color.min_area_ratio", 0.002, "blobs smaller than this image fraction are noise");
    declare<int64_t>("color.morph_kernel_size", 5, "open/close kernel size, odd; 0 or 1 disables");

    declare<double>("zone.stop_gap_m", 0.6, "block if obstacle base is this close to the robot front");
    declare<double>("zone.path_half_width_m", 0.24, "half-width of the swept path (robot + margin)");
    declare<double>("zone.robot_front_x_m", 0.15, "robot front edge x in base_frame");
    declare<double>("zone.large_blob_area_ratio", 0.5, "safety net: blobs this large always block");
    declare<int64_t>("zone.clip_margin_px", 2, "bottom rows treated as 'base cut off'");

    declare<std::string>("base_frame", "base_footprint", "robot frame on the floor, x forward");
    declare<bool>("camera.use_tf", true, "read camera height/pitch/x from TF (recommended)");
    declare<double>("camera.mount_height_m", 0.135, "camera height, only if use_tf is false");
    declare<double>("camera.pitch_rad", 0.15, "camera pitch (+ down), only if use_tf is false");
    declare<double>("camera.x_m", 0.14, "camera x in base_frame, only if use_tf is false");

    declare<bool>("publish_debug_image", true, "publish the overlay when someone subscribes");
  }

  rcl_interfaces::msg::SetParametersResult validate_changes(
    const std::vector<rclcpp::Parameter> & changes)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    // Current values, overridden by the proposed changes.
    const auto get = [&](const std::string & name) -> rclcpp::Parameter {
        for (const auto & c : changes) {
          if (c.get_name() == name) {
            return c;
          }
        }
        return get_parameter(name);
      };
    try {
      const auto candidate = read_params(get);
      ColorDetector probe(candidate.color);  // throws on bad colour config
      (void)probe;
    } catch (const std::exception & e) {
      result.successful = false;
      result.reason = e.what();
    }
    return result;
  }

  void apply_parameters()
  {
    NodeParams p;
    std::shared_ptr<const ColorDetector> detector;
    try {
      p = read_params([this](const std::string & n) {return get_parameter(n);});
      detector = std::make_shared<const ColorDetector>(p.color);
    } catch (const std::exception & e) {
      // Should not happen (validated before the change), but never crash the node.
      RCLCPP_ERROR(get_logger(), "keeping previous parameters: %s", e.what());
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const bool extrinsics_source_changed =
      p.use_tf != params_.use_tf || p.base_frame != params_.base_frame ||
      p.mount_height_m != params_.mount_height_m || p.pitch_rad != params_.pitch_rad ||
      p.camera_x_m != params_.camera_x_m;
    params_ = std::move(p);
    detector_ = std::move(detector);
    if (extrinsics_source_changed) {
      extrinsics_.reset();  // re-resolve on the next frame
    }
    RCLCPP_INFO(get_logger(), "parameters updated");
  }

  // -------------------------------------------------------------- intrinsics
  void on_camera_info(const sensor_msgs::msg::CameraInfo & info)
  {
    if (info.k[0] <= 0.0 || info.k[4] <= 0.0 || info.width == 0 || info.height == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "ignoring uncalibrated camera_info");
      return;
    }
    CameraGeometry g;
    g.fx = info.k[0];
    g.fy = info.k[4];
    g.cx = info.k[2];
    g.cy = info.k[5];
    g.width = static_cast<int>(info.width);
    g.height = static_cast<int>(info.height);

    std::lock_guard<std::mutex> lock(mutex_);
    if (!intrinsics_) {
      RCLCPP_INFO(
        get_logger(), "camera intrinsics: %dx%d fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
        g.width, g.height, g.fx, g.fy, g.cx, g.cy);
    }
    intrinsics_ = g;
  }

  // -------------------------------------------------------------- extrinsics
  // Camera height, pitch and x offset come from TF (i.e. from the URDF), so
  // they are never duplicated in a config file that could drift out of sync.
  std::optional<Extrinsics> resolve_extrinsics(const NodeParams & p, const std::string & camera_frame)
  {
    if (!p.use_tf) {
      return Extrinsics{p.mount_height_m, p.pitch_rad, p.camera_x_m};
    }
    try {
      const auto tf = tf_buffer_->lookupTransform(p.base_frame, camera_frame, tf2::TimePointZero);
      const auto & q = tf.transform.rotation;
      const auto & t = tf.transform.translation;

      // Optical frame: z = viewing direction, x = image right.
      // Rotate the unit axes by the quaternion (third and first matrix columns).
      const double zx = 2.0 * (q.x * q.z + q.w * q.y);
      const double zy = 2.0 * (q.y * q.z - q.w * q.x);
      const double zz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
      const double xz = 2.0 * (q.x * q.z - q.w * q.y);

      const double pitch = std::atan2(-zz, std::hypot(zx, zy));  // + = looking down
      const double yaw = std::atan2(zy, zx);
      if (std::abs(yaw) > 0.05 || std::abs(xz) > 0.05) {
        RCLCPP_WARN(
          get_logger(),
          "camera yaw %.3f rad / roll component %.3f: the ground-plane model assumes a "
          "forward-facing camera with no roll, distances will be biased", yaw, xz);
      }
      RCLCPP_INFO(
        get_logger(), "camera extrinsics from TF (%s -> %s): height %.3f m, pitch %.3f rad, x %.3f m",
        p.base_frame.c_str(), camera_frame.c_str(), t.z, pitch, t.x);
      return Extrinsics{t.z, pitch, t.x};
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "waiting for TF %s -> %s: %s",
        p.base_frame.c_str(), camera_frame.c_str(), e.what());
      return std::nullopt;
    }
  }

  // ------------------------------------------------------------------- image
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    const auto t0 = std::chrono::steady_clock::now();

    // Snapshot shared state so parameter updates cannot change it mid-frame.
    NodeParams p;
    std::shared_ptr<const ColorDetector> detector;
    std::optional<CameraGeometry> intrinsics;
    std::optional<Extrinsics> extrinsics;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      p = params_;
      detector = detector_;
      intrinsics = intrinsics_;
      extrinsics = extrinsics_;
    }

    if (!intrinsics) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for camera_info");
      return;
    }
    if (!extrinsics) {
      extrinsics = resolve_extrinsics(p, msg->header.frame_id);
      if (!extrinsics) {
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      extrinsics_ = extrinsics;
    }
    if (static_cast<int>(msg->width) != intrinsics->width ||
      static_cast<int>(msg->height) != intrinsics->height)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "image %ux%u does not match camera_info %dx%d",
        msg->width, msg->height, intrinsics->width, intrinsics->height);
      return;
    }

    try {
      const auto cv_img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);

      CameraGeometry geom = *intrinsics;
      geom.mount_height_m = extrinsics->height_m;
      geom.pitch_rad = extrinsics->pitch_rad;
      DangerZoneConfig zone_cfg = p.zone;
      zone_cfg.camera_to_front_m = p.robot_front_x_m - extrinsics->x_m;
      const DangerZone zone(geom, zone_cfg);

      const bool want_debug = p.publish_debug_image && debug_pub_->get_subscription_count() > 0;
      cv::Mat mask;
      const auto det = detector->detect(cv_img->image, want_debug ? &mask : nullptr);
      const auto assessment = zone.assess(det);

      ObstacleDetection out;
      out.header = msg->header;  // image stamp: lets the safety monitor judge freshness
      out.detected = det.detected;
      out.in_danger_zone = assessment.in_danger_zone;
      out.bbox.x_offset = static_cast<uint32_t>(std::max(0, det.bbox.x));
      out.bbox.y_offset = static_cast<uint32_t>(std::max(0, det.bbox.y));
      out.bbox.width = static_cast<uint32_t>(std::max(0, det.bbox.width));
      out.bbox.height = static_cast<uint32_t>(std::max(0, det.bbox.height));
      out.area_ratio = static_cast<float>(det.area_ratio);
      out.gap_m = std::isfinite(assessment.gap_m) ?
        static_cast<float>(assessment.gap_m) : std::numeric_limits<float>::quiet_NaN();
      out.bottom_clipped = assessment.bottom_clipped;
      out.in_path = assessment.in_path;
      out.triggered_by_area = assessment.triggered_by_area;
      out.processing_time_ms = static_cast<float>(
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
      detection_pub_->publish(out);

      if (want_debug) {
        const auto overlay = vision_safety::render_debug_overlay(
          cv_img->image, mask, det, assessment, geom, zone_cfg);
        debug_pub_->publish(
          *cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, overlay).toImageMsg());
      }

      if (!first_frame_logged_) {
        first_frame_logged_ = true;
        RCLCPP_INFO(
          get_logger(), "perception running (first frame %.2f ms)", out.processing_time_ms);
      }
    } catch (const std::exception & e) {
      // Do not publish: the safety monitor will see stale perception and stop.
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "frame dropped: %s", e.what());
    }
  }

  // ------------------------------------------------------------------- state
  std::mutex mutex_;
  NodeParams params_;
  std::shared_ptr<const ColorDetector> detector_;
  std::optional<CameraGeometry> intrinsics_;
  std::optional<Extrinsics> extrinsics_;
  bool first_frame_logged_{false};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<ObstacleDetection>::SharedPtr detection_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  OnSetParametersCallbackHandle::SharedPtr on_set_handle_;
  PostSetParametersCallbackHandle::SharedPtr post_set_handle_;
};

}  // namespace vision_safety_ros

RCLCPP_COMPONENTS_REGISTER_NODE(vision_safety_ros::ObstacleDetectorNode)
