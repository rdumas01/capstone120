#!/usr/bin/env python3
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_ros
import tf
import tf2_geometry_msgs
#import sys
#import numpy as np

#from geometry_msgs.msg import TransformStamped, PoseStamped, Twist, Point, Pose #included pose
#commented out this line since I am not sure if we will need anything other than Pose.

from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from tf2_geometry_msgs import do_transform_pose

#from trajectory import plan_curved_trajectory - we don't need this one.

#Define the method which contains the main functionality of the node.

def controller(input_pose): 

#what should be our input here? Is that the pose? "from_frame" and "then to_frame" or should we just leave it as "()"

#what kind of information is pose? Is that just x,y,z distance (and orientation) from camera.
  """
  Get Object's pose as input (in camera frame)
  Initial a bufffer and use "tflookup_transform" to get relative quaternion b/w the frames
  
  """

  ################################### YOUR CODE HERE ##############

  # Create a publisher topic, a buffer and a tf buffer, which is primed with a tf listener
  pub = rospy.Publisher('/cap120/object_in_base', Pose, queue_size=10) #publish to topic
  tfBuffer = tf2_ros.Buffer() ## TODO: initialize a buffer
  tfListener = tf2_ros.TransformListener(tfBuffer)## TODO: initialize a tf listener
  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz

  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    try:
      trans_camera_to_base_link = tfBuffer.lookup_transform('sgr532/base_link', 'sgr532/usb_cam_link', rospy.Time()) ## TODO: create a transform between camera to base link


      pose_stamped = tf2_geometry_msgs.PoseStamped()
      pose_stamped.pose = input_pose

      # Use the transform to compute the object's pose in the base_link frame
      object_in_base_link = do_transform_pose(pose_stamped, trans_camera_to_base_link) 
      


      # some debug output below
      # print(f"Current: {trans_odom_to_base_link.transform.translation.x}, {trans_odom_to_base_link.transform.translation.y}, {baselink_yaw  }")
      # print(f"Target: {waypoint}")
      
      # rospy.loginfo(object_in_base_link)
      pub.publish(object_in_base_link.pose) #since the topic msgs type is pose, so .pose is needed
      r.sleep() # Use our rate object to sleep until it is time to publish again


    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      pass

'''
#def planning_callback(msg):
  #try:
    #trajectory = plan_curved_trajectory((msg.x, msg.y)) ## TODO: What is the tuple input to this function?
    
    ## TODO: write a loop to loop over our waypoints and call the controller function on each waypoint
    #for waypoint in trajectory:
      #controller(waypoint)

  #except rospy.ROSInterruptException:
    #pass
'''      

# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method
  rospy.init_node('camera_to_base_transform', anonymous=True) #Initialize a note that will subscripe to the topic
  
  rospy.Subscriber('/cap120/object_in_camera', Pose, controller) ## TODO: what are we subscribing to here?
  
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
