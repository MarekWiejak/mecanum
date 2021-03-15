# Hardware descripton
## Main components
- Raspberry Pi 4
- Teensy 4.0
- RPLidar A2
- Mecanum wheels platform with engines with rotation speed sensors
- pwm/voltage converter (x4)
- 12V and 5V power source

## Setup description
Raspberry is powered by 5V source via USB C cable. Teensy is powered and connected to Raspberry with USB / microUSB cable. Lidar is powered and connected to Raspberry with USB / microUSB cable. Converters are connected to Teensy pins with signal cables (detailed description below), converters are powered by 12V power cables. Engines are connected to converters with power cables. Sensors on engines are connected to Teensy pins with signal cables, addionally, they are powered with 5V (also thorough Teensy).

## Teensy pin connections
Pin connections compatible with code in teensy/teensy.ino are listed in pin_connections.txt, also power_wire_connections.jpg might be usefull for connecting converters. 

# Software description
## Folders content
- teensy - contains code running on Teensy 4.0 platform
- astrocent_pkg - a ROS package, place it at ~/catkin_ws/src/
- autolaunch - a script for automatic roslaunch on reboot

## Teensy software
Teensy microcontroller hosts a program for low level control of engines. It makes sure, that required speed is maintained and provides measured real speed value. The program acts as ROS node subscribing /cmd_vel and publishing /measured_vel topic.
To setup the program install teensy/teensy.ino.

## Raspberry software
Raspberry computer hosts high level control, like autonomous navigation and other tasks that might be added in future. For navigation ROS navigation stack is used along with custom launch and config files and auxiliary nodes.

### Packages used for navigation
- rplidar   http://wiki.ros.org/rplidar
- gmapping  http://wiki.ros.org/gmapping
- amcl   http://wiki.ros.org/amcl
- move_base http://wiki.ros.org/move_base?distro=noetic

### Astrocent_pkg
A custom package made for the robot. It's purpose is to launch standard navigation packages, provide communication with Teensy and run auxiliary custom nodes.

custom nodes:
- controller2vel_node - converts control values received from ROS-Mobile (info below) to format compatible with rest of the system. Subscribes /control_x_y and /control_z topics, publishes /cmd_vel topic
- odometry_node - provides odomery based on measurement of real wheel speed received from Teensy. Subscribes /measured_vel topic, publishes odometry TF.

launch files:
- amcl.launch - runs amcl localization
- astrocent.launch - basic launch required for robot to operate, preferably run at boot
- gmapping.launch - runs SLAM mapping algorithm
- move_base.launch - runs complete navigation algorithm (also launches amcl)

### Autolaunch
Autolaunch folder contains script that will cause ROS software to launch on boot. The setup instruction can be found in /autolaunch/README.txt file.
Also, a set of convinient aliases that can be added to ~/.bashrc can be found in the folder.

# How to contol robot
## Manual control
For manual control ROS-Mobile android application can be used, which acts as a ROS node. For compatibility, name published topics "control_x_y" and "control_z".

## Usefull commands
If you recommended aliases from autolaunch/aliases.txt are used:
- run "rs" to source custom packages
- to start mapping run "gm" and run "sm" to save recorded map
- run "rospi" to export ROS_IP 
- run "am" for localization, NOTE: currently map file path is hardcoded in astrocent_pkg/launch/amcl.launch file and code must be changed in order to use chosen map.
- run "mb" to start navigation (also need to code map file path)
- run "re" to record a rosbag file
To use localization and navigation rviz can be used.


