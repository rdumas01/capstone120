#!/usr/bin/env python3

import rospy
import tf2_ros
import tf
import tf2_geometry_msgs

from geometry_msgs.msg import Pose, PoseStamped
from tf2_geometry_msgs import do_transform_pose


class camera2base:


  def __init__(self):
    rospy.init_node('camera_to_base_transform', anonymous=True)

    self.pub = rospy.Publisher('/cap120/object_in_base', Pose, queue_size=10)
    self.tfBuffer = tf2_ros.Buffer()
    self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    rospy.Subscriber('/cap120/object_in_camera', Pose, self.camera2base_callback)


  def camera2base_callback(self, input_pose):

    try:
      trans_camera_to_base_link = self.tfBuffer.lookup_transform('sgr532/base_link', 'sgr532/usb_cam_link', rospy.Time())

      pose_stamped = PoseStamped()
      pose_stamped.pose = input_pose

      # Use the transform to compute the object's pose in the base_link frame
      object_in_base_link = do_transform_pose(pose_stamped, trans_camera_to_base_link)
      
      # rospy.loginfo(object_in_base_link)
      self.pub.publish(object_in_base_link.pose) #since the topic msgs type is pose, so .pose is needed

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
      rospy.logerr(e)



if __name__ == '__main__':

  camera2base()
  
  rospy.spin()
