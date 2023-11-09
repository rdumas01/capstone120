#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
import numpy as np



class AverageTargetPose:


    def __init__(self):

        rospy.init_node('base_coords_average', anonymous=True)

        self.history = []
        self.hist_length = 10

        self.sub = rospy.Subscriber('/cap120/object_in_base', Pose, self.callback)
        self.pub = rospy.Publisher('/cap120/object_in_base_avg', Pose, queue_size=10)

        while not rospy.is_shutdown():

            msg = Pose()
            msg.position.x = np.mean([pose.position.x for pose in self.history])
            msg.position.y = np.mean([pose.position.y for pose in self.history])
            msg.position.z = np.mean([pose.position.z for pose in self.history])

            self.pub.publish(msg)

    
    def callback(self, msg):

        self.history.append(msg)
        if len(self.history) > self.hist_length:
            self.history = self.history[-self.hist_length:]




if __name__ == '__main__':

  AverageTargetPose()
  
  rospy.spin()