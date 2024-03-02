#!/usr/bin/env python3

import rospy
import tf2_ros
import tf
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class logging:


    def __init__(self):

        rospy.init_node('logging_node')

        self.tfBuffer = tf2_ros.Buffer()
        self.tf2listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(0.1)

        self.pubs = dict()

        self.pubs['joint1'] = rospy.Publisher('/cap120/logging/joint1_state', Float64, queue_size=10)
        self.pubs['joint2'] = rospy.Publisher('/cap120/logging/joint2_state', Float64, queue_size=10)
        self.pubs['joint3'] = rospy.Publisher('/cap120/logging/joint3_state', Float64, queue_size=10)
        self.pubs['joint4'] = rospy.Publisher('/cap120/logging/joint4_state', Float64, queue_size=10)
        self.pubs['joint5'] = rospy.Publisher('/cap120/logging/joint5_state', Float64, queue_size=10)
        self.pubs['joint6'] = rospy.Publisher('/cap120/logging/joint6_state', Float64, queue_size=10)

        self.pubs['end_effector_x'] = rospy.Publisher('/cap120/logging/end_effector_x', Float64, queue_size=10)
        self.pubs['end_effector_y'] = rospy.Publisher('/cap120/logging/end_effector_y', Float64, queue_size=10)
        self.pubs['end_effector_z'] = rospy.Publisher('/cap120/logging/end_effector_z', Float64, queue_size=10)

        self.joint_sub = rospy.Subscriber('/sgr532/sagittarius_joint_states', JointState, self.joint_callback)

        rospy.spin()


    def joint_callback(self, joints):

        names = joints.name
        positions = joints.position
        for joint, angle in zip(names[:-2], positions[:-2]):
            self.pubs[joint].publish(angle)

        transform = self.tfBuffer.lookup_transform("sgr532/link_grasping_frame",
                                                    "world",
                                                    rospy.Time()).transform.translation
        self.pubs['end_effector_x'].publish(transform.x)
        self.pubs['end_effector_y'].publish(transform.y)
        self.pubs['end_effector_z'].publish(transform.z)
        



if __name__ == '__main__':
    logging()
    