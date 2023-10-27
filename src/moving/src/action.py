#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import rospy
from std_msgs.msg import String
import sys
import time

def next_action(msg):
    action = msg
    rospy.init_node('instruction_node', anonymous=True)
    pub = rospy.Publisher('cap120/next_action', String, queue_size=10) # Specified Point message type and added a queue_size
    rate = rospy.Rate(0.08)

    time.sleep(1) 

    while not rospy.is_shutdown():
        rospy.loginfo(action)
        pub.publish(action)
        rate.sleep()

    

# if __name__ == '__main__':
#     try:
#         if len(sys.argv) > 1:
#             action_next = sys.argv[1]
#             next_action(action_next)

#     except:

#         print("Please provide an action.")
#         rospy.ROSInterruptException
#         pass



if __name__ == '__main__':
    try:
        if len(sys.argv) > 1:
            action_next = sys.argv[1]
            next_action(action_next)
        else:
            print("Please provide an action.")
    except rospy.ROSInterruptException:
        print("ROS node interrupted.")
    except Exception as e:
        print("An unexpected error occurred: %s" % str(e))




#rostopic pub /my_point_topic1 geometry_msgs/Point '{x: 0.035, y: -0.02, z: 0.0}' -1 &
#rostopic pub /my_point_topic2 geometry_msgs/Point '{x: 0.035, y: 0.02, z: 0.0}' -1 &


