#!/bin/bash
/etc/init.d/nginx start
source /oribot/install/setup.sh
AMENT_PREFIX_PATH="$AMENT_PREFIX_PATH:/ros_deep_learning/install/ros_deep_learning" ros2 launch oribot oribot.launch.xml