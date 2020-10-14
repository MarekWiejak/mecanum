#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from astrocent.msg import Vector4float
from geometry_msgs.msg import Twist
from math import pi as pi

wheel_r = 39
l = 180 # (half length + half width of the platform)
message = Twist()

rpm2vel_matrix = np.array([[1,-1,1/l],[1,1,-1/l],[1,-1,-1/l],[1,1,1/l]]) / 60 * (2*pi) * wheel_r/4

pub = rospy.Publisher('velPV', Twist, queue_size=10)

def callback(data):
    read_rpm = [data.m1, data.m2, data.m3, data.m4]
    vels_2b_published = np.matmul(read_rpm, rpm2vel_matrix)
    message.linear.x = vels_2b_published[0]
    message.linear.y = vels_2b_published[1]
    message.angular.z = vels_2b_published[2]
    pub.publish(message)

def rpm2vel():
    rospy.init_node("rpm2vel")
    rospy.Subscriber('rpmPV', Vector4float, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        rpm2vel()
    except rospy.ROSInterruptException:
        pass