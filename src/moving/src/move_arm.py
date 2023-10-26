#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, PoseStamped
from copy import deepcopy
from std_msgs.msg import String

from compute_trajectory import gen_trajectory


class move_arm_node:


    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('sagittarius_arm')
        gripper = MoveGroupCommander('sagittarius_gripper')

        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'world'
        arm.set_pose_reference_frame(reference_frame)
        gripper.set_pose_reference_frame(reference_frame) #夹爪
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.01)
        arm.set_goal_joint_tolerance(0.02)

        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()

        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()

        # 控制机械臂夹爪抓住笔
        gripper.set_joint_value_target([-0.0197, -0.0197])
        gripper.go()
        rospy.sleep(2)


        # Custom trajectory
        # target_pose = rospy.wait_for_message("/cap120/bloc_coords", Pose, timeout=5)
        target_pose = Pose()
        target_pose.position.x = 0.337
        target_pose.position.y = -0.190
        target_pose.position.z = 0.142
        points_list = gen_trajectory(target_pose)


        # 设置五角星基准点的位置
        target_pose_base = PoseStamped()
        target_pose_base.header.frame_id = reference_frame
        target_pose_base.header.stamp = rospy.Time.now()
        target_pose_base.pose.orientation.w = 1.0

        # Initial pose
        wpose = deepcopy(target_pose_base.pose)
        wpose.position.x = points_list[0][0]
        wpose.position.y = points_list[0][1]
        wpose.position.z = points_list[0][2]

        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(wpose, end_effector_link)
        arm.go()
        rospy.sleep(2)

        # Initialize waypoints
        waypoints = []

        # Waypoint poses
        for i in range(1, len(points_list) - 1, 1):
            wpose = deepcopy(target_pose_base.pose)
            wpose.position.x = points_list[i][0]
            wpose.position.y = points_list[i][1]
            wpose.position.z = points_list[i][2]
            waypoints.append(deepcopy(wpose))

        # Final pose
        # wpose = deepcopy(target_pose_base.pose)
        # wpose.position.x += points_list[0][0] * star_size
        # wpose.position.y += points_list[0][1] * star_size
        # wpose.position.z += points_list[0][2] * star_size
        # waypoints.append(deepcopy(wpose))

        fraction = 0.0   #路径规划覆盖率
        maxtries = 100   #最大尝试规划次数
        attempts = 0     #已经尝试规划次数
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点，完成圆弧轨迹
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表
                                    0.01,        # eef_step，终端步进值
                                    0.0,         # jump_threshold，跳跃阈值
                                    True)        # avoid_collisions，避障规划
            # 尝试次数累加
            attempts += 1
            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                     
        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        rospy.sleep(1)

        # 控制机械臂回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)

        # 松开笔
        gripper.set_named_target('open')
        gripper.go()
        rospy.sleep(2)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('sleep')
        arm.go()
        rospy.sleep(1)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)



if __name__ == "__main__":

    try:
        # 初始化ROS节点
        rospy.init_node('move_arm_node', anonymous=True)

        rospy.wait_for_message("/start_topic", String)
        move_arm_node()

    except rospy.ROSInterruptException:
        pass
