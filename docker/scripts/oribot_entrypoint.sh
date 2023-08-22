#!/bin/bash
/etc/init.d/nginx start
source /oribot/install/setup.sh
export LD_PRELOAD="/lib/aarch64-linux-gnu/libGLdispatch.so"
export AMENT_PREFIX_PATH="$AMENT_PREFIX_PATH:/ros_deep_learning/install/ros_deep_learning" 
ros2 launch oribot oribot.launch.xml
