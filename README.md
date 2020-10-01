# Folders content
- teensy - contains code running on Teensy 4.0 platform
- astrocent_pkg - a ROS package, place it at ~/catkin_ws/src/

# important tips
## before running robot
- python must be installed on your main controll unit
- if you are not using <i/>ROS mobile</i> app, remove <i/>node pkg="astrocent" name="controller2vel" type="controller2vel_node.py"</i> line at <i/>/astrocent_pkg/launch/astrocent.launch</i>
- run <i/>catkin_make</i> and <i/>source devel/setup.bash</i> at your worskspace 
- before compiling code for teensy go through a tuturial: http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup, this creats C++ libraries necessary to compile te code
- connect sensor and motor control pins and modify their definitions inside <i/>teensy.ino</i> file, you can test connections later, when the robot is running [wheel numbering: 1-right front, 2-left front, 3-left rear, 4-right rear]
- compile and upload code to Teensy
- to start the robot run <i/>roslaunch astrocent astrocent.launch</i>

## when running robot
- run <i/>rostopic echo rpmPV</i> to see current rotation speed of the wheels (rpm1-4), (pwm1-4 - shows current control signals - it is not important for the moment), this topic is published by Teensy and shouldn't be published manually
- rotate wheels with your hand and observe <i/>rpmPV</i> readouts to confirm, that sensor pins are connected properly, rearrange wires if needed
- run <i/>rostopic pub -1 /driverPIDsettings astrocent/PIDsettings -- 0</i> to switch PID controllers to manual mode, [value "1" switches back to automatic mode]
- when in manual mode: run <i/>rostopic pub -1 /manPWM astrocent/ManualPWM -- w1 w2 w3 w4</i> to manually set pwm control signal for motors, [replace w1-4 with int values in 0-225 range; note: value 225 origins from threshold value set to 30 in teensy.ino -> 255-30=225]
- publish on <i/>/manPWM</i> topic to confirm, that motor pins are connected properly, rearrange wies if needed
- go back to PID automatic mode
- when in automatic mode: run <i/>rostopic pub -1 /velSP astrocent/Main_vels -- vx vy rz</i> to set speed vector of the robot, [vx - parallel linear, float(-500.0; 500.0)[mm/s]; vy - perpendicular linear, float(-500.0; 500.0)[mm/s]; rz - rotational, float(-3.0; 3.0)[rad/s] ]

## errors and undesired behaviour
- raspberry is in read-only mode:    
run <i/>sudo mount -o remount, rw /</i>
- <i/>[ERROR] [1601553784.575701]: Error opening serial: [Errno 2] could not open port /dev/ttyACM0: [Errno 2] No such file or directory: '/dev/ttyACM0'</i>:     
replace   <i/>node pkg="rosserial_python" name="serial_node" type="serial_node.py" args="/dev/ttyACM0"/</i>   
with    <i/>node pkg="rosserial_python" name="serial_node" type="serial_node.py" args="/dev/ttyUSB0"/</i>   
in astrocent.launch
- a wheel runs full speed while should be halted - sensor wire might be disconnected
