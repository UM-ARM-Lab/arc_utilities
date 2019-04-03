#/bin/bash
BUILD_TYPE=${1:-Release}

. /opt/ros/kinetic/setup.bash
. /root/catkin_ws/devel_isolated/setup.bash

cd ~/catkin_ws
catkin_make_isolated --catkin-make-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE run_tests && catkin_test_results
