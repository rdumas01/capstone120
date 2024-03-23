#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from math import atan2

from sensor_msgs.msg import Image


def find_object(image, display=False, publish=False, print_res=False):
    '''
    Scans an image to find colored object and returns their coordinates.

    Args:
        image: RGB cv2 image
        display: True to draw squares around
        publish: True to publish result image to a topic
        print_res: True to print coordinates of each bloc found in terminal

    Returns:
        found_objects: dictionary where each key is a color, and the value is a list of dictionaries for each object found
            e.g. found_object['red'] = [list of dictionaries corresponding to red objects]
    '''

    # Fixed variables
    drawn_rect_topic = "/cap120/found_blocs"    # Publishing topic
    area_ratio = 0.2                            # Defines which contours to keep based on contour area
    threshold_area = 4000                       # Minimum area for a blob to be considered valid

    ## Color Definitions
    colors = color_def()

    # Reading the image
    if isinstance(image, str):
        img = cv2.imread(image) # if opening from path
        print('Received an image path')
    else:
        img = image

    # Define kernel size for noise removal
    kernel = np.ones((7,7),np.uint8)

    # Convert to hsv colorspace
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    result = None
    if display or publish:
        # Copy the image to not alter it when tracing the bounding boxes
        result = img.copy()

    found_objects = dict()

    for color in colors:

        found_objects[color['name']] = []

        # Lower bound and upper bound for color 
        lower_bound = color['hsv'][0]    
        upper_bound = color['hsv'][1]

        if color['name'] == 'red': # Red has 2 ranges (on both ends of hue spectrum)
            lower_bound = color['hsv'][0][0]    
            upper_bound = color['hsv'][0][1]

        # Find the colors within the boundaries
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        if color['name'] == 'red': # Taking the union of red's 2 ranges
            lower_bound2 = color['hsv'][1][0]    
            upper_bound2 = color['hsv'][1][1]
            mask2 = cv2.inRange(hsv, lower_bound2, upper_bound2)
            mask = cv2.bitwise_or(mask, mask2)
            # cv2.imshow("OR MASK", mask)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

        # Remove unnecessary noise from mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Find contours from the mask
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key = lambda x: cv2.contourArea(x), reverse=True) # Order by decreasing area

        if display or publish:
            colour = color['bgr'] # Frame color
            thickness = 2 # Frame thickness

        if len(contours) > 0:
            min_area = max(threshold_area, area_ratio * cv2.contourArea(contours[0]))

        for cntr in contours:

            if cv2.contourArea(cntr) >= min_area:
                found_object = dict()

                epsilon = .01 * cv2.arcLength(cntr, True) #play around with this value- .018 on the blueprint side
                approx = cv2.approxPolyDP(cntr, epsilon, True) #approx is a list of vertices of detected object

                if len(approx)== 3: #3 vertices = triangle
                    #calculating centroid, base and height of triangle
                    (xc,zc), w, h, angle, third_point = find_triangle_details(approx)

                    found_object['center'] = (xc, yc)

                    if print_res:
                        print('Found {} object at ({}, {})'.format(color['name'], xc, yc))

                    # Convert angle from radians to degrees, for visualization
                    angle_degrees = np.degrees(angle)

                    # Draw angle on the image
                    angle_text = "{:.2f} deg".format(angle_degrees)
                    cv2.putText(result, angle_text, (int(xc), int(zc)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)

                    # Draw the centroid on the image
                    cv2.circle(result, (int(xc), int(zc)), 5, (0, 0, 0), -1)

                    #Draw the axis - major axis only. No need to draw the minor
                    cv2.line(result, (int(xc), int(zc)), (int(third_point[0]), int(third_point[1])), (255, 255, 0), 1, cv2.LINE_AA)

                    found_object['yaw'] = angle
                    found_object['shape'] = 'triangle'

                else: #4 vertices = quadrilateral
                    xc, yc, w, h = cv2.boundingRect(cntr)
                    xc = xc + w//2
                    yc = yc + h//2

                    found_object['center'] = (xc,yc)

                    if print_res:
                        print('Found {} object at ({}, {})'.format(color['name'], xc, yc))

                    rect : cv2.RotatedRect = cv2.minAreaRect(cntr)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    angle, bloc_shape = get_orientation_and_shape(box, result)

                    found_object['yaw'] = angle
                    found_object['shape'] = bloc_shape

                    angle_degree = angle * 180/np.pi
                    angle_text = "{:.2f} deg".format(angle_degree)
                    
                    #Display Angle on Image
                    cv2.putText(result, angle_text, (int(xc), int(zc)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)

                    #Display Contours
                    cv2.drawContours(result, [box], 0, color, 2)

                    # Display block shape
                    cv2.putText(result, bloc_shape, box[1], cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 1, cv2.LINE_AA)


                #if display or publish:
                    # Traces a rectangle around each "object":
                    #cv2.drawContours(result, [box], 0, colour, 2)
                    #cv2.putText(result, bloc_shape, box[1], cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 1, cv2.LINE_AA)

                # Draw mask to get list of pixels
                cntr_mask = np.zeros(hsv.shape[:2])
                cv2.drawContours(cntr_mask, [cntr], 0, 255, thickness=-1)
                pts_in_cntr = np.nonzero(cntr_mask)
                found_object['pixels'] = pts_in_cntr

                found_objects[color['name']].append(found_object)

    if publish:
        pub = rospy.Publisher(drawn_rect_topic, Image, queue_size=5)
        bridge = CvBridge()
        res_frame = bridge.cv2_to_imgmsg(result, encoding="bgr8")
        pub.publish(res_frame)

    if display: # Displays the frame with rectangles on it
        cv2.imshow("Result", result)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    return found_objects


def get_orientation_and_shape(box, img=None):
    '''
    box: 4x2 array containing the coordinates of a box's 4 corners
    img: the img on which to draw
    '''

    p1, p2, p3, _ = box
    center = np.mean(box, 0)
    
    c1 = np.linalg.norm(p1-p2)
    c2 = np.linalg.norm(p2-p3)

    if c1 > c2:
        major_ax = p1-p2
        minor_ax = p2-p3
    else:
        major_ax = p2-p3
        minor_ax = p1-p2
    
    ratio = max(c1, c2) / min(c1, c2)
    if ratio >= 1.4:                        # TODO: adjust threshold
        if ratio >= 2.4:                    # TODO: adjust threshold
            bloc_shape = 'long'
        else:
            bloc_shape = 'rect'
    else:
        bloc_shape = 'cube'

    if img is not None:
        draw_axis(img, center, center + major_ax, (255, 255, 0), 1)
        draw_axis(img, center, center + minor_ax, (0, 0, 255), 5)

    angle = (-atan2(major_ax[1], major_ax[0]) - np.pi/2) % np.pi
    if angle >= 6/10 * np.pi:
        angle -= np.pi

    return angle, bloc_shape


def find_triangle_details(approx):
    # Extract vertices and treat the coordinates as (x, z)
    x1, z1 = approx[0][0]
    x2, z2 = approx[1][0]
    x3, z3 = approx[2][0]
    
    # Calculate the centroid (CoM)
    G_x = (x1 + x2 + x3) / 3
    G_z = (z1 + z2 + z3) / 3
    
    # calculate the distances (sides of the triangle)
    side1 = np.linalg.norm(np.array([x1, z1]) - np.array([x2, z2]))
    side2 = np.linalg.norm(np.array([x2, z2]) - np.array([x3, z3]))
    side3 = np.linalg.norm(np.array([x3, z3]) - np.array([x1, z1]))
    
    # Base is the longest of the three sides
    sides = [(side1, (x1, z1), (x2, z2)), (side2, (x2, z2), (x3, z3)), (side3, (x3, z3), (x1, z1))]
    base, base_start, base_end = max(sides, key=lambda item: item[0])
    
    # Calculate the center of the base
    base_center = ((base_start[0] + base_end[0]) / 2, (base_start[1] + base_end[1]) / 2)

    # Find the 3rd point which is not part of the base
    third_point = set([(x1, z1), (x2, z2), (x3, z3)]) - set([base_start, base_end])
    third_point = list(third_point)[0]
    
    # Calculate the angle between center-third point line and the vertical axis -- angle of the height vector
    dx = third_point[0] - base_center[0]
    dz = third_point[1] - base_center[1]
    angle = (-atan2(dz, dx) - np.pi / 2) % np.pi
    if angle >= 6 / 10 * np.pi:
        angle -= np.pi
    
    #height of the triangle
    height = np.linalg.norm(np.array(base_center) - np.array(third_point))
    
    return (G_x, G_z), base, height, angle, third_point


def draw_axis(img, p_, q_, color, scale):
    p = list(p_)
    q = list(q_)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 1, cv2.LINE_AA)


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
              'bgr' : (107*1.2, 183*1.2, 189*1.2), \
              'hsv' : [np.array([20, 60, 60]), np.array([50, 255, 255])]}
    
    blue = {'name' : 'blue', \
            'bgr' : (255, 0, 0), \
            'hsv' : [np.array([100, 85, 30]), np.array([110, 255, 255])]}
    
    green = {'name' : 'green', \
             'bgr' : (0, 255, 0), \
             'hsv' : [np.array([60, 40, 30]), np.array([90, 255, 250])]}
    
    white = {'name' : 'white', \
             'bgr' : (255, 255, 255), \
             'hsv' : [np.array([0, 0, 200]), np.array([255, 20, 255])]}
    
    red = {'name' : 'red', \
            'bgr' : (0, 0, 255), \
            'hsv' : [[np.array([170, 150, 90]), np.array([255, 255, 255])], \
                     [np.array([0, 150, 90]), np.array([15, 255, 255])]]}
                    # Red has 2 ranges on both ends of HSV spectrum

    colors = [yellow, blue, green, red]
    #colors = [green]

    return colors




## Tests ##

if __name__ == '__main__':

    find_object('test_bricks.jpg', display=True, print_res=True)