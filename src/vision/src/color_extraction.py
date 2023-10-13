#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge

from sensor_msgs.msg import Image


def find_object(image, display=False, publish=False, print_res=False, find_all=False):
    '''
    Scans an image to find colored object and returns their coordinates.

    Args:
        image: RGB cv2 image
        display: True to draw squares around
        publish: True to publish result image to a topic
        print_res: True to print coordinates of each bloc found in terminal
        find_all: if False, only returns the first object found

    Returns:
        (xc, yc): coordinates of first object found if find_all is False
        None if find_all is True
    '''

    # Publishing topic
    drawn_rect_topic = "/cap120/found_blocs"

    ## Color Definitions
    colors = color_def()

    # Reading the image
    if isinstance(image, str):
        img = cv2.imread(image) # if opening from path
    else:
        img = image

    # Define kernel size for noise removal
    kernel = np.ones((7,7),np.uint8)

    # Convert to hsv colorspace
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


    if display or publish:
        # Copy the image to not alter it when tracing the bounding boxes
        result = img.copy()


    for color in colors:

        # Lower bound and upper bound for color 
        lower_bound = color['hsv'][0]    
        upper_bound = color['hsv'][1]

        # Find the colors within the boundaries
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Remove unnecessary noise from mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Find contours from the mask
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        if display or publish:
            colour = color['bgr'] # Frame color
            thickness = 2 # Frame thickness


        for cntr in contours:

            xc, yc, w, h = cv2.boundingRect(cntr)
            xc = xc + w//2
            yc = yc + h//2

            if print_res:
                print('Found {} object at ({}, {})'.format(color['name'], xc, yc))


            if display or publish:
                x1 = xc - w//2
                x2 = xc + w//2
                y1 = yc - h//2
                y2 = yc + h//2
            
                # Traces a rectangle around each "object":
                cv2.rectangle(result, (x1, y1), (x2, y2), colour, thickness)


            if not find_all:
                return xc, yc

    if publish:
        pub = rospy.Publisher(drawn_rect_topic, Image)
        bridge = CvBridge()
        res_frame = bridge.cv2_to_imgmsg(result, encoding="passthrough")
        pub.publish(res_frame)

    if display: # Displays the frame with rectangles on it
        cv2.imshow("Result", result)
        cv2.waitKey(0)
        cv2.destroyAllWindows()




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
            'hsv' : [np.array([90, 50, 70]), np.array([128, 255, 255])]}
    
    green = {'name' : 'green', \
             'bgr' : (0, 255, 0), \
             'hsv' : [np.array([36, 25, 25]), np.array([90, 255, 255])]}
    
    white = {'name' : 'white', \
             'bgr' : (255, 255, 255), \
             'hsv' : [np.array([0, 0, 200]), np.array([255, 20, 255])]}
    
    red1 = {'name' : 'red', \
            'bgr' : (0, 0, 255), \
            'hsv' : [np.array([0, 150, 50]), np.array([10, 255, 255])]}
    
    red2 = {'name' : 'red', \
            'bgr' : (0, 0, 255), \
            'hsv' : [np.array([170, 150, 50]), np.array([180, 255, 255])]}

    colors = [yellow, blue, green, red1, red2]

    return colors




## Tests ##

if __name__ == '__main__':

    find_object('test_bricks.jpg', display=True, print_res=True, find_all=True)