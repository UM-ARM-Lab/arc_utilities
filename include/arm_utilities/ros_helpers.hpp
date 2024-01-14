#pragma once

#include <rclcpp/time.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace ros_helpers {

// You can also use this to set a builtin_interfaces::msg::Time!
rclcpp::Time secondsToTime(const double timestamp);

builtin_interfaces::msg::Time secondsToTimeMsg(const double timestamp);

}
