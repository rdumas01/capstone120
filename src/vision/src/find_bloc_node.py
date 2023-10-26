#!/usr/bin/env python3

from color_extraction import find_object
import rospy
from cv_bridge import CvBridge
import numpy as np

import message_filters

from sensor_msgs.msg import Image, CameraInfo         #TODO
from geometry_msgs.msg import Pose                    #TODO



class find_bloc_node:

    def __init__(self):
        '''
        Main function of the /cap120/find_bloc node, which performs the object detection on the camera frames.
        Subscribes to the /cap120/camera topic and calls the callback function.
        '''
        rospy.init_node('find_bloc_node', anonymous=False)

        # Should the resulting image with found blocs be published? (False to increase speed)
        self.pub_found_blocs = True

        # Relevant topics
        self.obj_coords_topic = "/cap120/bloc_coords"
        self.rgb_topic = "/camera/color/image_raw"
        self.depth_topic = "/camera/aligned_depth_to_color/image_raw"

        self.bridge = CvBridge()

        # Camera info
        self.fx = 1      # Focal length x
        self.fy = 1      # Focal length y
        self.cx = 0      # Image center x
        self.cy = 0      # Image center y

        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)

        self.rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)

        self.RGBD_frame = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 5, 0.1)
        self.RGBD_frame.registerCallback(self.RGBD_callback)
        
        rospy.spin()



    def camera_info_callback(self, msg):
        '''
        Gets the intrinsic parameters from the CameraInfo message
        '''
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cs = msg.K[2]
        self.cy = msg.K[5]



    def RGBD_callback(self, rgb_img, depth_img):
        '''
        Called every time a camera frame is published on the /cap120/camera topic.
        Publishes the (x, y) coordinates of the first object found in the frame to the /cap120/bloc_coords topic.

        Args:
            rgb_img: rgb frame (ROS Image) from the RealSense
            depth_img: depth frame (ROS Image) from the RealSense, aligned to the rgb frame
        '''
        
        try:
            rgb_img = self.bridge.imgmsg_to_cv2(rgb_img, desired_encoding="bgr8")
            depth_img = self.bridge.imgmsg_to_cv2(depth_img, desired_encoding="16UC1")

            objects = find_object(rgb_img, publish=True)
            
            if len(objects.keys()) > 0:

                obj = objects[[key for key in objects.keys()][0]]
                ox, oy = obj['center']
                depth = np.mean(np.array([depth_img[x, y] for (x, y) in zip(obj['pixels'][0], obj['pixels'][1])]))

                pub = rospy.Publisher(self.obj_coords_topic, Pose, queue_size=1)

                obj_coords = Pose()

                obj_coords.position.x = (ox - self.cx) * depth / self.fx       #TODO verify
                obj_coords.position.y = (oy - self.cy) * depth / self.fy       #TODO verify
                obj_coords.position.z = depth

                pub.publish(obj_coords)

        except Exception as e:
            print("Error:", e)






if __name__ == '__main__':

    find_bloc_node()
