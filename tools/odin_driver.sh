#!/usr/bin/env bash
# Start the Odin1 ROS2 driver (RGB + LiDAR + IMU + odometry + depth demo).
set -e

source /opt/ros/humble/setup.bash
source /home/liyh/odin1/catkin_ws/install/setup.bash

exec ros2 launch odin_ros_driver odin1_ros2.launch.py
