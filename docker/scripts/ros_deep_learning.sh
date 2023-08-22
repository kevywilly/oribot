#!/usr/bin/env bash
# this script installs the ros_deep_learning nodes for jetson-inference

set -e
#set -x

<<<<<<< HEAD
ROS_DISTRO='iron'
=======
ROS_DISTRO='humble'
>>>>>>> v3.0
WORKSPACE="/ros_deep_learning"

echo "ROS_DISTRO = $ROS_DISTRO"

mkdir -p $WORKSPACE/src
source /ros_entrypoint.sh


cd $WORKSPACE/src
git clone --depth=1 https://github.com/dusty-nv/ros_deep_learning
echo "Building ros_deep_learning package for ROS2 $ROS_DISTRO"
cd $WORKSPACE
colcon build
source /ros_entrypoint.sh
echo "testing that ROS2 $ROS_DISTO can find ros_deep_learning package:"
ros2 pkg prefix ros_deep_learning
