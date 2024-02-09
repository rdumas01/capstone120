#!/usr/bin/env python3

import rospy
import tf
import tf2_ros

from geometry_msgs.msg import Pose, Transform



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
    tfBuffer = tf2_ros.Buffer()
    tf2listener = tf2_ros.TransformListener(tfBuffer)
    tf_listener = tf.TransformListener()

    rospy.sleep(0.1)    # Give time to fill Listener

    frames_list = tf_listener.allFramesAsString().split(' ')    # Gets list of words
    frames_list = frames_list[1::5]     # Extracts frame names

    # ============== FRAME SELECTION ===============

    color_frames = [frame for frame in frames_list if str(color) in frame]   # Get frames containing desired color
    # print(color_frames)

    # ==============================================

    if len(color_frames) > 0:
        
        try:
            target_trans : Transform = tfBuffer.lookup_transform(origin_frame, color_frames[0], rospy.Time()).transform
            target_pose = Pose()
            target_pose.position = target_trans.translation
            target_pose.orientation = target_trans.rotation
            return target_pose
        
        except Exception as e:
            rospy.logerr(e)
            return None
        
    else:
        rospy.logerr(f"There is no {color} bloc within view...")
        return None


if __name__ == "__main__":
    target = get_target_pose("green")
    print(target)