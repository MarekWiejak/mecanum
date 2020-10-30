#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

control_vels = Twist()
control_vels.linear.x = 0
control_vels.linear.y = 0
control_vels.angular.z = 0
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

def write_x_y(data):
    control_vels.linear.x = data.linear.x
    control_vels.linear.y = data.linear.y
    pub.publish(control_vels)

def write_z(data):
    control_vels.angular.z = data.angular.z
    pub.publish(control_vels)

def controller2vel():
    rospy.init_node('controller2vel')
    rospy.Subscriber('control_x_y', Twist, write_x_y)
    rospy.Subscriber('control_z', Twist, write_z)
    rospy.spin()

if __name__=='__main__':
    try:
        controller2vel()
    except rospy.ROSInterruptException:
        pass