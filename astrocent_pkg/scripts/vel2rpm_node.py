#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from astrocent.msg import Rpm
from astrocent.msg import Main_vels
from math import pi as pi
 
max_rpm = 135
max_omega = max_rpm * 2 * pi /60
wheel_r = 39
l = 180 # (half length + half width of the platform)
k = l *pi
v_max = wheel_r * max_omega # v_max in mm/s
rotation_max = wheel_r / l * max_omega # in rotations per second
message = Rpm()

vel2rpm_matrix = np.array([[1,1,1,1],[-1,1,-1,1],[l,-l,-l,l]]) * 60 / (2*pi) / wheel_r 

pub = rospy.Publisher('rpmSP', Rpm, queue_size=1)

def check_limit(rpm_values):
    amax = np.amax(np.absolute(rpm_values))
    if amax > max_rpm:
        return rpm_values * (max_rpm / amax)
    else:
        return rpm_values

def callback(data):
    set_vels = [data.vx, data.vy, data.rz]
    rpms_2b_published = np.matmul(set_vels,vel2rpm_matrix)
    rpms_2b_published = check_limit(rpms_2b_published)
    message.rpm1 = rpms_2b_published[0]
    message.rpm2 = rpms_2b_published[1]
    message.rpm3 = rpms_2b_published[2]
    message.rpm4 = rpms_2b_published[3]
    pub.publish(message)

def vel2rpm():
    rospy.init_node('vel2rpm')
    rospy.Subscriber('velSP', Main_vels, callback)
    rospy.spin()

if __name__=='__main__':
    try:
        vel2rpm()
    except rospy.ROSInterruptException: 
        pass