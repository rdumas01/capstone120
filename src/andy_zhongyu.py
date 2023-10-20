#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
from std_msgs.msg import String
from geometry_msgs.msg import Point


class Controller:
    def point_callback(self, msg):
        self.recv_point = msg
         
        # self.construct_plan()

    def __init__(self):
        self.reference_frame = 'world'
        self.recv_point = None

        #trying to subscript the topic called "pointadd"
        rospy.Subscriber('pointadd', Point, self.point_callback)
       
        # initialize the move_group's API
        moveit_commander.roscpp_initialize(sys.argv)

        # initialize move group for robot arm
        self.arm = MoveGroupCommander('sagittarius_arm')
        self.gripper = MoveGroupCommander('sagittarius_gripper')

        # when motion planning fails, allow replanning (当运动规划失败后，允许重新规划)
        self.arm.allow_replanning(True)
       
        # set reference coordinate frame of the goal position (设置目标位置所使用的参考坐标系)

        self.arm.set_pose_reference_frame(self.reference_frame)
        self.gripper.set_pose_reference_frame(self.reference_frame) # gripper (夹爪)
               
        # set error tolerance for positions (in meter) and pose (in radian) 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.01)
        self.arm.set_goal_joint_tolerance(0.02)

        # set maximum allowed veloity and acceleration (设置允许的最大速度和加速度)
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)
       
        # get end-effector link name (获取终端link的名称)
        self.end_effector_link = self.arm.get_end_effector_link()

        # control the arm to go back to its initial position (控制机械臂先回到初始化位置)
        self.arm.set_named_target('home')
        self.arm.go()

        self.path_start_point = [0.14, 0.02, 0.04]
        self.path_end_point = [0.05, 0, 0.04]

        self.goto_recv_point_timer = rospy.Timer(rospy.Duration(5), self.goto_recv_point)



   

    def goto_recv_point(self, callback):
        if self.recv_point:
            print("self.recv_point",self.recv_point)
            target_pose = PoseStamped()
            target_pose.header.frame_id = self.reference_frame
            target_pose.header.stamp = rospy.Time.now()
           
            target_pose.pose.position.x = self.recv_point.x
            target_pose.pose.position.y = self.recv_point.y
            target_pose.pose.position.z = self.recv_point.z

            target_pose.pose.orientation.w = 1.0

            self.arm.set_pose_target(target_pose, self.end_effector_link)
            self.arm.go()
        else:
            pass


if __name__ == "__main__":
    try:
        rospy.init_node('move_points', anonymous=True)
        #rospy.wait_for_message("/start_topic", String)
        c = Controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass