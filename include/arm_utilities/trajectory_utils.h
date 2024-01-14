#pragma once

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace trajectory_utils {
void append(trajectory_msgs::JointTrajectory &trajectory1, trajectory_msgs::JointTrajectory const &trajectory2);
}
