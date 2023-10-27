#!/usr/bin/env python3

from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
import tf2_ros
import rospy
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


def gen_trajectory(target_pose):

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    # Get initial position of end effector
    while not rospy.is_shutdown():
        try:
            end_effector_pose = tfBuffer.lookup_transform('world', 'sgr532/link_grasping_frame', rospy.Time())
            # print(end_effector_pose)
            break
        except:
            pass

    x1 = end_effector_pose.transform.translation.x
    y1 = end_effector_pose.transform.translation.y
    z1 = end_effector_pose.transform.translation.z

    x2 = target_pose.position.x
    y2 = target_pose.position.y
    z2 = target_pose.position.z

    waypoints = generate_bezier_waypoints(x1, y1, z1, x2, y2, z2)
    plot_trajectory(waypoints)

    return waypoints



def plot_trajectory(waypoints):

    x_vals = [point[0] for point in waypoints]
    y_vals = [point[1] for point in waypoints]
    z_vals = [point[2] for point in waypoints]

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(x_vals, y_vals, z_vals)
    plt.show()



def bezier_curve(p0, p1, p2, p3, t):
    return (1-t)**3 * p0 + 3*(1-t)**2 * t * p1 + 3*(1-t) * t**2 * p2 + t**3 * p3



def generate_bezier_waypoints(x1, y1, z1, x2, y2, z2, offset=0.1, num_points=10):
    start = np.array([x1, y1, z1])
    end = np.array([x2, y2, z2])

    # Set control points
    control1 = start
    control2 = end + offset * np.array([0, 0, 1])

    # Sample points along the curve
    t_values = np.linspace(0, 1, num_points)
    waypoints = [bezier_curve(start, control1, control2, end, t) for t in t_values]

    # Generate list of points
    waypoints_list = [[wp[0], wp[1], wp[2]] for wp in waypoints]

    return waypoints_list