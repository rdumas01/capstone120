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

        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = MoveGroupCommander('sagittarius_arm')
        self.gripper = MoveGroupCommander('sagittarius_gripper')

        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'world'
        self.arm.set_pose_reference_frame(reference_frame)
        self.gripper.set_pose_reference_frame(reference_frame) #夹爪
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.01)
        self.arm.set_goal_joint_tolerance(0.02)

        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)
        
        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()

        # Put arm in initial pose
        self.base_position()


        self.main()


        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)



    def main(self):

        # Main loop - waiting for action then execute it

        self.execute_action = self.actions_dict() # Dictionary of all possible actions

        rospy.loginfo('Entering main loop...')

        block_1 = [0.220, 0.222, 0.197, -0.012, 0.293, -0.004, 0.956]
        block_2 = [0.220, -0.222, 0.197, -0.012, 0.293, -0.004, 0.956]
        all_blocks = [block_1, block_2] # number of blocks
        print(len(all_blocks))
        target_1 = [0.220, 0.000, 0.197, -0.012, 0.293, -0.004, 0.956]
        blueprint = [] # list of blueprint target points

        for i in range(len(all_blocks)):
            self.pickup_bloc(all_blocks[i])
            rospy.sleep(10)

        rospy.loginfo('Exiting main loop...')



    def base_position(self):

        self.arm.set_pose_target([0.250, 0, 0.220, 0, 1.57, 0])
        self.arm.go()
        self.arm.stop()
        self.arm.clear_pose_targets()
        self.target_pose_base = self.arm.get_current_pose()

        rospy.sleep(2)
    


    def actions_dict(self):
        
        execute_action = {'pickup' : self.pickup_bloc,
                          'drop' : self.drop,
                          'lookout' : self.lookout,
                          'sleep' : self.sleep,
                          'exit' : self.exit}
        
        return execute_action
    
    def create_pose_from_list(data_list):
        pose = Pose()
        pose.position.x = data_list[0]
        pose.position.y = data_list[1]
        pose.position.z = data_list[2]
        pose.orientation.x = data_list[3]
        pose.orientation.y = data_list[4]
        pose.orientation.z = data_list[5]
        pose.orientation.w = data_list[6]
        return pose

    def pickup_bloc(self, target_list):

        try:
            # Custom trajectory
            target_pose = self.create_pose_from_list(target_list)
            # target_pose = Pose()
            # target_pose.position.x = 0.200
            # target_pose.position.y = -0.100
            # target_pose.position.z = 0.142
            target_pose.position.z = -0.08
            points_list = gen_trajectory(target_pose)

            rospy.sleep(.1)
            # Initialize waypoints
            waypoints = []

            # Waypoint poses
            for i in range(1, len(points_list) - 1, 1):
                wpose = deepcopy(self.target_pose_base.pose)
                wpose.position.x = points_list[i][0]
                wpose.position.y = points_list[i][1]
                wpose.position.z = points_list[i][2]
                waypoints.append(deepcopy(wpose))
            

            fraction = 0.0   #路径规划覆盖率
            maxtries = 100   #最大尝试规划次数
            attempts = 0     #已经尝试规划次数
            # 设置机器臂当前的状态作为运动初始状态
            self.arm.set_start_state_to_current_state()
            # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点，完成圆弧轨迹
            while fraction < 1.0 and attempts < maxtries:
                (plan, fraction) = self.arm.compute_cartesian_path (
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
                rospy.loginfo("Path computed successfully. Moving the self.arm.")
                self.arm.execute(plan)
                rospy.loginfo("Path execution complete.")
            # 如果路径规划失败，则打印失败信息
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

            rospy.sleep(1)

            # Close gripper
            self.gripper.set_joint_value_target([-0.03, -0.03])
            self.gripper.go()
            rospy.sleep(3)
        except:
            pass

        self.base_position()

        return True
    

    def drop(self):

        self.gripper.set_named_target('open')
        self.gripper.go()

        return True


    def lookout(self):
        
        self.base_position()

        return True


    def sleep(self):

        self.arm.set_named_target('sleep')
        self.arm.go()

        return True


    def exit(self):

        # Go back to base pose
        self.base_position()
        # rospy.sleep(1)

        # Open gripper
        self.gripper.set_named_target('open')
        self.gripper.go()
        # rospy.sleep(1)

        # 控制机械臂先回到初始化位置
        self.arm.set_named_target('sleep')
        self.arm.go()
        # rospy.sleep(1)

        return False




if __name__ == "__main__":

    try:
        # 初始化ROS节点
        rospy.init_node('move_arm_node', anonymous=True)

        rospy.wait_for_message("/start_topic", String)
        move_arm_node()

    except rospy.ROSInterruptException:
        pass