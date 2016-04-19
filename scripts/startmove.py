#!/usr/bin/python

import time
import sys

import rospy
from std_msgs.msg import Float64


rospy.init_node("startmove",anonymous=True)

publeft = rospy.Publisher('p3dx/left_velocity_controller/command',Float64,queue_size=100)
pubright = rospy.Publisher('p3dx/right_velocity_controller/command',Float64,queue_size=100)

rate = rospy.Rate(3) # 3Hz
while not rospy.is_shutdown():
    publeft.publish(1)
    pubright.publish(1)
    rate.sleep()


