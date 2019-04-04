#/bin/bash
ROS_VERSION=${1:-kinetic}
BUILD_TYPE=${2:-Release}

. /opt/ros/$ROS_VERSION/setup.bash

cd ~/catkin_ws
catkin_make_isolated -DCMAKE_BUILD_TYPE=$BUILD_TYPE
