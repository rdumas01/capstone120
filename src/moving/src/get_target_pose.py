#!/usr/bin/env python3

import rospy
import tf
import tf2_ros

from geometry_msgs.msg import Pose



def get_target_pose(color : str, shape : str = None):
    '''
    Gets the Pose in the world frame of the bloc of the desired color.
    ----------
    Inputs:
        - color: string ['green', 'blue', 'ref', 'yellow']
        - shape: string (not implemented yet)
    ----------
    Output:
        - geometry_msgs.msg.Pose()
    '''

    origin_frame = "world"
    # rospy.init_node("get_target_pose_node")

    tf_listener = tf.TransformListener()
    tfBuffer = tf2_ros.Buffer()
    tf2listener = tf2_ros.TransformListener(tfBuffer)
    rospy.sleep(0.1)    # Give time to fill Listener

    frames_list = tf_listener.allFramesAsString().split(' ')    # Gets list of words
    frames_list = frames_list[1::5]     # Extracts frame names
    color_frames = [frame for frame in frames_list if str(color) in frame]   # Get frames containing desired color
    # print(color_frames)

    if len(color_frames) > 0:
        try:
            target_trans : Pose = tfBuffer.lookup_transform(origin_frame, color_frames[0], rospy.Time()).transform
            target_pose = Pose()
            target_pose.position.x = target_trans.translation.x
            target_pose.position.y = target_trans.translation.y
            target_pose.position.z = target_trans.translation.z
            target_pose.orientation.x = target_trans.rotation.x
            target_pose.orientation.y = target_trans.rotation.y
            target_pose.orientation.z = target_trans.rotation.z
            target_pose.orientation.w = target_trans.rotation.w
            return target_pose
        except Exception as e:
            rospy.logerr(e)
            return None
    else:
        rospy.logerr(f"There is no {color} bloc within view...")
        return None


if __name__ == "__main__":
    target = get_target_pose("yellow")
    print(target)