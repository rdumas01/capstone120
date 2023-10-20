#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import rospy
from geometry_msgs.msg import Point

x1 = None # Global variable to store the x value

def point_to_list(data):
    global x1
    x1 = data.x
    rospy.loginfo(rospy.get_caller_id() + " I heard x: %s, y: %s, z: %s", data.x, data.y, data.z)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("pointadd", Point, point_to_list)

    rospy.spin() # This keeps python from exiting until the node is stopped

if __name__ == '__main__':
    listener()

