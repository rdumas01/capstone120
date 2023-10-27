#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import rospy
from std_msgs.msg import String

def callback(msg):
    x1 = msg
    rospy.loginfo(x1)



def wait():
    rospy.init_node('wait_the_topic_action', anonymous=True)
    rospy.Subscriber('cap120/next_action', String , callback) # Specified Point message type and added a queue_size
    
    rospy.spin()
    
    

if __name__ == '__main__':
    wait()









#rostopic pub /my_point_topic1 geometry_msgs/Point '{x: 0.035, y: -0.02, z: 0.0}' -1 &
#rostopic pub /my_point_topic2 geometry_msgs/Point '{x: 0.035, y: 0.02, z: 0.0}' -1 &


