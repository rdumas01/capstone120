#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import cv2
import os

import rospy
import ros_numpy
import tf

from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header


### /!\ ASSUMPTION: there is at most only one block of each color in frame /!\ ###


class ObjectDetector:

    def __init__(self, colors):
        print("Running...")
        rospy.init_node('object_detector', anonymous=True)

        self.colors = colors

        self.bridge = CvBridge()

        self.xyz = None
        self.rgb = None

        self.point_cloud_sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.point_cloud_callback)

        self.tf_listener = tf.TransformListener()  # Create a TransformListener object
        self.tfbr = tf.TransformBroadcaster()

        rospy.spin()


    def point_cloud_callback(self, msg):
        try:
            # print('received')
            self.process_pc(msg)

            if self.rgb is not None:
                self.process_images()

        except Exception as e:
            print("Error1:", e)


    def process_pc(self, data):
        '''
        Extracts XYZ and RGB arrays from the point cloud data.
        '''
        pc = ros_numpy.numpify(data)
        pc = ros_numpy.point_cloud2.split_rgb_field(pc)
        
        self.xyz = np.zeros((1, pc.shape[0], 3))
        self.xyz[0,:,0]=pc['x']
        self.xyz[0,:,1]=pc['y']
        self.xyz[0,:,2]=pc['z']
        
        self.rgb = np.zeros((1, pc.shape[0], 3))
        self.rgb[0,:,0]=pc['r']
        self.rgb[0,:,1]=pc['g']
        self.rgb[0,:,2]=pc['b']
        self.rgb = np.uint8(self.rgb)

        # print(self.rgb.shape)
        # print(self.rgb)
        # cv2.imshow('rgb', self.rgb[:,:280000,:].reshape((100,2800,3)))
        # cv2.waitKey(1)

        # cv2.imshow('rgb', self.rgb)
        # cv2.waitKey(3)
    

    def process_images(self):
        '''
        Extracts bloc coordinates from XYZ and RGB data extracted from the point cloud.
        '''

        for color in self.colors:
                
            lower_hsv = color['hsv'][0]
            upper_hsv = color['hsv'][1]

            hsv = cv2.cvtColor(self.rgb, cv2.COLOR_RGB2HSV)
            mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

            masked = cv2.cvtColor(cv2.bitwise_and(hsv, hsv, mask=mask), cv2.COLOR_RGB2HSV)
            # masked = np.vstack((masked, masked, masked, masked, masked, masked, masked))
            # cv2.imshow('Res', np.vstack((masked, masked, masked, masked, masked, masked, masked)))
            cv2.imshow('rgb', masked[:,:280000,:].reshape((100,2800,3)))
            cv2.waitKey(3)

            _, indices = np.nonzero(mask)

            # If there are no detected blocs, exit
            if len(indices) <= 10:
                print("No self.rgb detected. Is your color filter wrong?")
                return

            # Calculate the center of the detected region by 
            center_x = np.mean(self.xyz[0, indices, 0])
            center_y = np.mean(self.xyz[0, indices, 1])
            center_z = np.mean(self.xyz[0, indices, 2])

            camera_link_x, camera_link_y, camera_link_z = center_x, center_y, center_z

            try:
                self.tfbr.sendTransform((camera_link_x, camera_link_y, camera_link_z),
                            tf.transformations.quaternion_from_euler(0, 0, 0),
                            rospy.Time.now(),
                            color['name']+"_bloc",
                            "camera_color_optical_frame")

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print("TF Error: " + e)
                return


    def rgb_to_hsv(self, rgb_threshold):
        '''
        Convert the RGB numpy array to an HSV numpy array.
        '''
        hsv_threshold = cv2.cvtColor(np.uint8([[rgb_threshold]]), cv2.COLOR_RGB2HSV)[0][0]
        return hsv_threshold



def color_def():
    '''
    Each color is represented by dictionary with 3 keys:
        'name': name of color (string)
        'bgr': BGR value for that color (tuple)
        'hsv': HSV range for that color (list of 2 arrays)

    Returns:
        list of dictionaries
    '''

    yellow = {'name' : 'yellow', \
              'bgr' : (107, 183, 189), \
              'hsv' : [np.array([20, 80, 80]), np.array([30, 255, 255])]}
    
    blue = {'name' : 'blue', \
            'bgr' : (255, 0, 0), \
            'hsv' : [np.array([90, 70, 100]), np.array([128, 255, 255])]}
    
    green = {'name' : 'green', \
             'bgr' : (0, 255, 0), \
             'hsv' : [np.array([36, 60, 100]), np.array([90, 255, 255])]}
    
    white = {'name' : 'white', \
             'bgr' : (255, 255, 255), \
             'hsv' : [np.array([0, 0, 200]), np.array([255, 20, 255])]}
    
    red = {'name' : 'red', \
            'bgr' : (0, 0, 255), \
            'hsv' : [[np.array([180, 150, 100]), np.array([255, 255, 255])], \
                     [np.array([0, 150, 100]), np.array([10, 255, 255])]]}
                    # Red has 2 ranges on both ends of HSV spectrum

    colors = [yellow, blue, green, red]
    colors = [yellow, blue, green]

    return colors



if __name__ == '__main__':
    print('Starting bloc detection ')
    ObjectDetector(color_def())