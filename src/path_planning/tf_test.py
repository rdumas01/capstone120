#!/usr/bin/env python3

# Transform a given input pose from one fixed frame to another
import rospy
from geometry_msgs.msg import Pose


def publisher():

    # setup object_coordinate_in_camera_frame test val
    out = Pose()
    out.position.x = 0
    out.position.y = 0
    out.position.z = 0
    out.orientation.x = 0
    out.orientation.y = 0
    out.orientation.z = 0
    out.orientation.w = 0
    pub = rospy.Publisher('/cap120/object_in_camera', Pose, queue_size=5)

    # Create a timer object that will sleep long enough to result in
    # a 10Hz publishing rate
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub.publish(out)
        r.sleep()



if __name__ == '__main__':

    rospy.init_node('tf_test', anonymous=False)
    try:
        publisher()
    except rospy.ROSInterruptException as e: print(e)