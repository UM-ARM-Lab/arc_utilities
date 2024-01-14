#include <arm_utilities/ros_helpers.hpp>

namespace ros_helpers
{


rclcpp::Time secondsToTime(const double timestamp) {
  return rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
}

builtin_interfaces::msg::Time secondsToTimeMsg(const double timestamp) {
  return rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
}

}