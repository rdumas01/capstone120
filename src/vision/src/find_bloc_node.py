#!/usr/bin/env python

from color_extraction import find_object
import rospy
from cv_bridge import CvBridge
import numpy as np

import message_filters

from sensor_msgs.msg import Image         #TODO
from geometry_msgs.msg import Pose        #TODO

# Should the resulting image with found blocs be published? (False to increase speed)
pub_found_blocs = True

# Relevant topics
obj_coords_topic = "/cap120/bloc_coords"
rgb_topic = "/camera/color/image_raw"
depth_topic = "/camera/aligned_depth_to_color/image_raw"

# Camera settings
CAM_WIDTH = 1280        #TODO
CAM_HEIGHT = 720        #TODO

# Intel RealSense D435 RGB FOV: 87 degrees x 58 degrees
rgb_horizontal_FOV = np.pi/180 * 87      #TODO
rgb_vertical_FOV = np.pi/180 * 58        #TODO



def callback(rgb_img, depth_img):
    '''
    Called every time a camera frame is published on the /cap120/camera topic.
    Publishes the (x, y) coordinates of the first object found in the frame to the /cap120/bloc_coords topic.

    Args:
        rgb_img: rgb frame (ROS Image) from the RealSense
        depth_img: depth frame (ROS Image) from the RealSense, aligned to the rgb frame
    '''

    bridge = CvBridge()
    rgb_img = bridge.imgmsg_to_cv2(rgb_img, desired_encoding="passthrough")
    depth_img = bridge.imgmsg_to_cv2(depth_img, desired_encoding="passthrough")

    obj = find_object(rgb_img, publish=pub_found_blocs)
    
    if obj is not None:

        x, y = obj
        d = depth_img[x, y]

        pub = rospy.Publisher(obj_coords_topic, Pose, queue_size=1)

        obj_coords = Pose()

        obj_coords.position.x = x       #TODO
        obj_coords.position.y = y       #TODO
        obj_coords.position.z = d       #TODO

        pub.publish(obj_coords)



def listener():
    '''
    Main function of the /cap120/find_bloc node, which performs the object detection on the camera frames.
    Subscribes to the /cap120/camera topic and calls the callback function.
    '''

    rgb_sub = rospy.Subscriber(rgb_topic, Image)
    depth_sub = rospy.Subscriber(depth_topic, Image)

    RGBD_frame = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 5, 0.1)
    RGBD_frame.registerCallback(callback)
    
    rospy.spin()



if __name__ == '__main__':

    rospy.init_node('/cap120/find_bloc_node', anonymous=False)

    listener()