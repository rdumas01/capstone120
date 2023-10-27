#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import rospy
from geometry_msgs.msg import Point

def talker():
    pub = rospy.Publisher('pointadd', Point, queue_size=10) # Specified Point message type and added a queue_size
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        point = Point()  # Created a Point object
        point.x = 0.035
        point.y = -0.02
        point.z = 0.0
        pub.publish(point)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass









#rostopic pub /my_point_topic1 geometry_msgs/Point '{x: 0.035, y: -0.02, z: 0.0}' -1 &
#rostopic pub /my_point_topic2 geometry_msgs/Point '{x: 0.035, y: 0.02, z: 0.0}' -1 &


