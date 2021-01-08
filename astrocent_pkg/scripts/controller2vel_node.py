#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

control_vels = Twist()
control_vels.linear.x = 0
control_vels.linear.y = 0
control_vels.angular.z = 0
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
last_cmd_time_x_y = rospy.Time()
last_cmd_time_z = rospy.Time()

def write_x_y(data):
    control_vels.linear.x = data.linear.x
    control_vels.linear.y = data.linear.y
    last_cmd_time_x_y.secs = rospy.get_rostime().secs
    pub.publish(control_vels)


def write_z(data):
    control_vels.angular.z = data.angular.z
    last_cmd_time_z.secs = rospy.get_rostime().secs
    pub.publish(control_vels)


def connection_control(event):
    if ((event.current_real.secs - last_cmd_time_x_y.secs) >= 1):
        control_vels.linear.x = 0.0
        control_vels.linear.y = 0.0
        pub.publish(control_vels)
    if ((event.current_real.secs - last_cmd_time_z.secs) >= 1):
        control_vels.angular.z = 0.0
        pub.publish(control_vels) 

def controller2vel():
    rospy.init_node('controller2vel')
    last_cmd_time_x_y = rospy.Time.now()
    last_cmd_time_z = rospy.Time.now()
    rospy.Subscriber('control_x_y', Twist, write_x_y)
    rospy.Subscriber('control_z', Twist, write_z)
    rospy.Timer(rospy.Duration(1), connection_control)
    rospy.spin()

if __name__=='__main__':
    try:
        controller2vel()
    except rospy.ROSInterruptException:
        pass