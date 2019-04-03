#/bin/bash
BUILD_TYPE=${1:-Release}

. /opt/ros/kinetic/setup.bash

cd ~/catkin_ws
catkin_make_isolated --catkin-make-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE
