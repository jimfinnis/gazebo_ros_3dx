#!/usr/bin/python

import time
import sys

import rospy

from sensor_msgs.msg import Range

rospy.init_node("showsonar",anonymous=True)

