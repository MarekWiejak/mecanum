#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from astrocent.msg import Main_vels

control_vels = Main_vels()
control_vels.vx = 0
control_vels.vy = 0
control_vels.rz = 0
pub = rospy.Publisher('velSP', Main_vels, queue_size=1)

def write_x_y(data):
    control_vels.vx = data.linear.x
    control_vels.vy = data.linear.y
    pub.publish(control_vels)

def write_z(data):
    control_vels.rz = data.angular.z
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