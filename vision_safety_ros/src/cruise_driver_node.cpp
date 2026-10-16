// Copyright 2026 Kavinaya. MIT License.
//
// cruise_driver: a deliberately "naive" upstream controller. It requests a
// constant velocity on cmd_vel_raw forever and knows nothing about obstacles.
// That makes stop tests repeatable, and it is the perfect stand-in for a
// planner that is wrong: the safety layer must stop the robot anyway.
//
// Live control:
//   ros2 param set /cruise_driver enabled false
//   ros2 param set /cruise_driver speed 0.2

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace vision_safety_ros
{

class CruiseDriverNode : public rclcpp::Node
{
public:
  explicit CruiseDriverNode(const rclcpp::NodeOptions & options)
  : Node("cruise_driver", options)
  {
    declare_parameter<bool>("enabled", true);
    declare_parameter<double>("speed", 0.3);       // m/s
    declare_parameter<double>("angular", 0.0);     // rad/s
    rcl_interfaces::msg::ParameterDescriptor ro;
    ro.read_only = true;
    const double rate_hz = declare_parameter<double>("rate_hz", 20.0, ro);

    pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_raw", rclcpp::QoS(10));
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_hz),
      [this]() {
        geometry_msgs::msg::Twist cmd;
        if (get_parameter("enabled").as_bool()) {
          cmd.linear.x = get_parameter("speed").as_double();
          cmd.angular.z = get_parameter("angular").as_double();
        }
        pub_->publish(cmd);
      });

    RCLCPP_INFO(
      get_logger(), "cruise_driver: requesting %.2f m/s on cmd_vel_raw (obstacle-blind on purpose)",
      get_parameter("speed").as_double());
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace vision_safety_ros

RCLCPP_COMPONENTS_REGISTER_NODE(vision_safety_ros::CruiseDriverNode)
