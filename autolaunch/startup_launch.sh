#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
export ROS_IP=raspberrypi.local
roslaunch astrocent astrocent.launch
