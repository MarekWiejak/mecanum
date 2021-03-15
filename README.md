# Hardware descripton
## Main components
- Raspberry Pi 4
- Teensy 4.0
- RPLidar A2
- Mecanum wheels platform with engines with rotation speed sensors
- pwm/voltage converter (x4)
- 12V and 5V power source

## Setup description
Raspberry is powered by 5V source via USB C cable. Teensy is powered and connected to Raspberry with USB / microUSB cable. Lidar is powered and connected to Raspberry with USB / microUSB cable. Converters are connected to Teensy pins with signal cables (detailed description below), converters are powered by 12V power cables. Engines are connected to converters with power cables. Sensors on engines are connected to Teensy pins with signal cables.

## Teensy pin connections
b

# Software description
## 



# Folders content
- teensy - contains code running on Teensy 4.0 platform
- astrocent_pkg - a ROS package, place it at ~/catkin_ws/src/
- autolaunch - a script for automatic roslaunch on reboot

# Packages used for navigation
- rplidar   http://wiki.ros.org/rplidar
- gmapping  http://wiki.ros.org/gmapping
- amcl   http://wiki.ros.org/amcl
- move_base http://wiki.ros.org/move_base?distro=noetic
