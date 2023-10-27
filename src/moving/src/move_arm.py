#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, PoseStamped
from copy import deepcopy
from std_msgs.msg import String
import tf
import numpy as np

from compute_trajectory import gen_trajectory


class move_arm_node:


    def __init__(self):
        # initialize move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # initialize arm group in the manipulator controlled by the move group. (初始化需要使用move group控制的机械臂中的arm group)
        self.arm = MoveGroupCommander('sagittarius_arm')
        self.gripper = MoveGroupCommander('sagittarius_gripper')

        # when motion planning fails, allow replanning
        self.arm.allow_replanning(True)
        
        # set reference coordinate frame of the goal position
        reference_frame = 'world'
        self.arm.set_pose_reference_frame(reference_frame)
        self.gripper.set_pose_reference_frame(reference_frame) # gripper
                
        # set error tolerance for positions (in meter) and pose (in radian)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.01)
        self.arm.set_goal_joint_tolerance(0.02)

        # set maximum allowed veloity and acceleration
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)
        
        # get end-effector link name
        self.end_effector_link = self.arm.get_end_effector_link()

        # Put arm in initial pose
        self.base_position()
        target_pose_base = self.arm.get_current_pose()

        # Custom trajectory
        # target_pose = rospy.wait_for_message("/cap120/bloc_coords", Pose, timeout=5)
        target_pose = Pose()
        target_pose.position.x = 0.200
        target_pose.position.y = -0.100
        target_pose.position.z = 0.142
        points_list = gen_trajectory(target_pose)

        # Initialize waypoints
        waypoints = []

        # Waypoint poses
        for i in range(1, len(points_list) - 1, 1):
            wpose = deepcopy(target_pose_base.pose)
            wpose.position.x = points_list[i][0]
            wpose.position.y = points_list[i][1]
            wpose.position.z = points_list[i][2]
            waypoints.append(deepcopy(wpose))
        

        fraction = 0.0   # trajectory planning cover rate 路径规划覆盖率
        maxtries = 100   # maximum allowed planning tries 最大尝试规划次数
        attempts = 0     # number of tries attempted 已经尝试规划次数
        # set current robot arm state as initial state 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()
        # plan a cartesian path trajectory that pass all waypoints 尝试规划一条笛卡尔空间下的路径，依次通过所有路点，完成圆弧轨迹
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表
                                    0.01,        # eef_step，终端步进值
                                    0.0,         # jump_threshold，跳跃阈值
                                    True)        # avoid_collisions，避障规划
            # attempts + 1 
            attempts += 1
            # print trajectory planning progress 
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                     
        # if planning success (cover rate 100%), start moving 
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the self.arm.")
            self.arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        # if planning fails, print failure message 
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        rospy.sleep(1)

        # Close gripper
        self.gripper.set_joint_value_target([-0.0197, -0.0197])
        self.gripper.go()
        rospy.sleep(2)

        # Go back to base pose
        self.base_position()
        rospy.sleep(1)

        # Open gripper
        self.gripper.set_named_target('open')
        self.gripper.go()
        rospy.sleep(2)

        # let robot arm go back to initial state
        self.arm.set_named_target('sleep')
        self.arm.go()
        rospy.sleep(1)
        
        # close moveit and exit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


    def base_position(self):

        self.arm.set_pose_target([0.250, 0, 0.175, 0, 1.57, 0])
        self.arm.go()
        self.arm.stop()
        self.arm.clear_pose_targets()

        rospy.sleep(2)




if __name__ == "__main__":

    try:
        # initialize ROS nodes
        rospy.init_node('move_arm_node', anonymous=True)

        rospy.wait_for_message("/start_topic", String)
        move_arm_node()

    except rospy.ROSInterruptException:
        pass
