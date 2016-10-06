#!/usr/bin/python

import time
import sys

import rospy
import diamondapparatus as da

from gazebo_msgs.msg import ModelStates

rospy.init_node('diamondpublish',anonymous=True)
da.init()

# crude throttling
msgidx=0

def callback(data):
    global msgidx
    if msgidx%100==0 and 'robot_description' in data.name:
        idx=data.name.index('robot_description')
        pos = data.pose[idx].position
        da.publish("/tracker/points",(pos.x,pos.y))
#        print "%f,%f" % (pos.x,pos.y)
    msgidx=msgidx+1
    
rospy.Subscriber('gazebo/model_states',ModelStates,callback)

rospy.spin()
