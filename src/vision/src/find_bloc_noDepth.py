#!/usr/bin/env python3

from color_extraction import find_object
import rospy
from cv_bridge import CvBridge
import numpy as np

import tf
import tf2_ros

from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Point, PointStamped, TransformStamped, Quaternion



class find_bloc_node:

    def __init__(self):
        '''
        Main function of the /cap120/find_bloc node, which performs the object detection on the camera frames.
        Subscribes to the /cap120/camera topic and calls the callback function.
        '''
        rospy.init_node('find_bloc_node', anonymous=False)

        self.tfBuffer = tf2_ros.Buffer()
        self.tf2Listener = tf2_ros.TransformListener(self.tfBuffer)
        self.tf_listener = tf.TransformListener()
        self.tfbr = tf.TransformBroadcaster()

        # Should the resulting image with found blocs be published? (False to increase speed)
        self.pub_found_blocs = True

        # Relevant topics
        self.obj_coords_topic = "/cap120/object_in_camera"
        self.rgb_topic = "/camera/color/image_raw"
        self.camera_info_topic = "/camera/color/camera_info"

        self.cam_link = 'sgr532/usb_cam_link'
        self.origin_frame = 'world'

        self.obj_h = 0.01   # object height

        self.bridge = CvBridge()

        # Camera info
        self.fx = 1      # Focal length x
        self.fy = 1      # Focal length y
        self.cx = 0      # Image center x
        self.cy = 0      # Image center y

        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)

        self.rgb_sub = rospy.Subscriber(self.rgb_topic, Image, self.RGB_callback)
        
        rospy.loginfo("find_bloc_node was successfully initialized")
        rospy.spin()



    def camera_info_callback(self, msg):
        '''
        Gets the intrinsic parameters from the CameraInfo message
        '''
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]



    def RGB_callback(self, rgb_img):
        '''
        Called every time a camera frame is published on the /cap120/camera topic.
        Publishes the (x, y) coordinates of the first object found in the frame to the /cap120/bloc_coords topic.
        
        Args:
            rgb_img: rgb frame (ROS Image) from the RealSense
            depth_img: depth frame (ROS Image) from the RealSense, aligned to the rgb frame
        '''
        try:
            rgb_img = self.bridge.imgmsg_to_cv2(rgb_img, desired_encoding="bgr8")

            objects = find_object(rgb_img, publish=True)
            
            if len(objects.keys()) > 0:

                colors = [key for key in objects.keys()]
                for col in colors:
                    for i, obj in enumerate(objects[col]):

                        ox, oy = obj['center']
                        
                        # Step 1: get camera pose in ground frame
                        try:
                            camera_pose : TransformStamped = self.tfBuffer.lookup_transform(self.origin_frame, self.cam_link, rospy.Time()).transform
                            camera_position : Point = camera_pose.translation
                            camera_rotation : Quaternion = camera_pose.rotation
                            quat = [camera_rotation.x, camera_rotation.y, camera_rotation.z, camera_rotation.w]
                            camera_yaw = tf.transformations.euler_from_quaternion(quat)[2]
                        except Exception as e:
                            rospy.logerr("Could not get camera position...")
                            print(e)

                        # Step 2: get arbitrary point that simply gives the direction toward the object
                        arbitrary_depth = 1
                        arbitrary_point = Point()
                        arbitrary_point.x = (ox - self.cx) * arbitrary_depth / self.fx
                        arbitrary_point.y = (oy - self.cy) * arbitrary_depth / self.fy
                        arbitrary_point.z = arbitrary_depth

                        # Get arbitrary_point in world frame
                        try:
                            self.tf_listener.waitForTransform(self.origin_frame, self.cam_link, rospy.Time(), rospy.Duration(0.5))
                            arbitrary_point_world : Point = self.tf_listener.transformPoint(
                                                            self.origin_frame,
                                                            PointStamped(header=Header(stamp=rospy.Time(),
                                                                                    frame_id=self.cam_link),
                                                                                    point=arbitrary_point)).point
                        except Exception as e:
                            rospy.logerr("Could not express arbitrary point in world frame...")
                            print(e)

                        # Step 3: solve for missing factor
                        alpha = (camera_position.z - self.obj_h) / (camera_position.z - arbitrary_point_world.z)

                        # Step 4: get object coordinates
                        obj_coords = Pose()
                        obj_coords.position.x = (1-alpha) * camera_position.x + alpha * arbitrary_point_world.x
                        obj_coords.position.y = (1-alpha) * camera_position.y + alpha * arbitrary_point_world.y
                        obj_coords.position.z = (1-alpha) * camera_position.z + alpha * arbitrary_point_world.z

                        # Step 5: publish transform to TF
                        self.tfbr.sendTransform((obj_coords.position.x, obj_coords.position.y, obj_coords.position.z),
                                    tf.transformations.quaternion_from_euler(0, np.pi/2, camera_yaw + obj['yaw']),
                                    rospy.Time.now(),
                                    str(col) + "_bloc_" + str(i),
                                    self.origin_frame)
                        
                        pub = rospy.Publisher(self.obj_coords_topic, Pose, queue_size=1)
                        pub.publish(obj_coords)

        except Exception as e:
            rospy.logerr(e)
            pass






if __name__ == '__main__':

    find_bloc_node()
