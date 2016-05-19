#!/usr/bin/python

import time
import sys

import rospy
from std_msgs.msg import Float64


rospy.init_node("startmove",anonymous=True)

publeft = rospy.Publisher('leftmotor',Float64,queue_size=100)
pubright = rospy.Publisher('rightmotor',Float64,queue_size=100)

rate = rospy.Rate(3) # 3Hz

left = float(sys.argv[1])
right = float(sys.argv[2])

while not rospy.is_shutdown():
    publeft.publish(left)
    pubright.publish(right)
    rate.sleep()


