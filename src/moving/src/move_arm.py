#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped
from copy import deepcopy
from std_msgs.msg import String
import tf
import numpy as np
import moveit_msgs
from compute_trajectory import gen_trajectory
from moveit_msgs.msg import OrientationConstraint, Constraints
from get_target_pose import get_target_pose


class move_arm_node:


    def __init__(self):
        #######################################
        # initialize box number
        self.box_num  = 0
        self.collision_check = 0
        self.desk_height = 0.01 # 0.085
        self.pickup_offset = 0.04
        self.cube_height = 0.03

        # define a dictionary to store box shapes dimensions
        self.shapes = {
            'box': [0.03, 0.03, 0.03],
            'rectangular': [0.09, 0.03, 0.03]
        }
        #######################################

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
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.001)
        self.arm.set_goal_joint_tolerance(0.001)
        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)
        
        # 获取终端link的名称
        # self.arm.set_end_effector_link("sgr532/end_effector")
        self.end_effector_link = self.arm.get_end_effector_link()
        # rospy.logerr(self.end_effector_link)

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
        stay_in_loop = True
        while stay_in_loop:
            
            rospy.loginfo('Waiting for action...')
            action = rospy.wait_for_message('/cap120/next_action', String)

            self.gripper.set_joint_value_target([-0.007, -0.007])
            self.gripper.go()
            rospy.sleep(3)

            if action.data in self.execute_action:
                stay_in_loop = self.execute_action[action.data]()
            else:
                rospy.loginfo(f"Received invalid action '{action.data}'. Please send a valid action.")
            rospy.sleep(1) #pretty important to have a sleep here to make sure the previous action have already be executed
        
        rospy.loginfo('Exiting main loop...')

    def from_blue_print(self):
        #place_list = rospy.wait_for_message("/cap120/place_list",Pose, timeout=5)
        place_pose_1 = Pose()
        place_pose_1.position.x = 0.200
        place_pose_1.position.y = 0.200
        place_pose_1.position.z = self.cube_height/2

        # place_pose_test = Pose()
        # place_pose_test.position.x = 0.200
        # place_pose_test.position.y = 0.200
        # place_pose_test.position.z = self.desk_height

        place_pose_2 = Pose()
        place_pose_2.position.x = 0.200
        place_pose_2.position.y = 0.200
        place_pose_2.position.z = self.cube_height/2 + self.cube_height

        place_pose_3 = Pose()
        place_pose_3.position.x = 0.200
        place_pose_3.position.y = 0.200
        place_pose_3.position.z = self.cube_height/2  + 2*self.cube_height

        place_pose_4 = Pose()
        place_pose_4.position.x = 0.200
        place_pose_4.position.y = 0.200
        place_pose_4.position.z = self.cube_height + 3*self.cube_height
        place_list = [place_pose_1, place_pose_2, place_pose_3, place_pose_4]

        return place_list


    def main2(self):
        self.place_list = self.from_blue_print()

        # total_bricks = len(self.place_list) # will change to length of blueprint, right now our defualt setting is grab four bricks
        # Main loop - waiting for action then execute it

        #self.execute_action = self.actions_dict() # Dictionary of all possible actions

        rospy.loginfo('Entering main loop...')
        
        self.lookout()
        self.drop()
        for step in self.place_list:

            
            grabbed = False

            while grabbed == False:
                #if you are testing in simulation, you need to comment out this section 
                ###################################  
                target_pose = self.look_around()
                self.drop()
                self.pickup_from_above(target_pose)
                rospy.sleep(.1)
                # if grabbed, return True, then exit while loop to execute drop_bloc
                grabbed = self.check_closure()
                rospy.sleep(.1)
                ###################################

            self.drop_goal(step)
            # if(self.collision_check == 0):
            #     self.add_box(step, 'box')
            #     rospy.sleep(1)

            

        self.sleep()

        rospy.loginfo('Exiting main loop...')

    def look_around(self):
        found = None
        target_offset = 0.00

        while not found:

            self.arm.set_pose_target([0.25, -0.15 + target_offset, 0.220, 0, 1.57, 0])
            self.arm.go()
            rospy.sleep(0.1)
            self.arm.stop()
            self.arm.clear_pose_targets()
            target_pose = get_target_pose("green")

            
            if target_pose is not None:
                return target_pose

            else:
                target_offset += 0.05
                rospy.loginfo(f"No brick found at offset {target_offset}.")
          
            if target_offset > 0.3:
                rospy.logwarn("Reached maximum search offset without finding all bricks.")
                break
            

        return [0.250, 0, 0.220, 0, 1.57, 0]

    
    def constraints_add(self):
        orientation_constraint = moveit_msgs.msg.OrientationConstraint()
        target_constraints = rospy.wait_for_message("/cap120/constraints", Pose, timeout=5)

        orientation_constraint.orientation.x = target_constraints.orientation.x
        orientation_constraint.orientation.y = target_constraints.orientation.y
        orientation_constraint.orientation.z = target_constraints.orientation.z
        orientation_constraint.orientation.w = target_constraints.orientation.w
        constraints = moveit_msgs.msg.Constraints()
        constraints.orientation_constraints.append(orientation_constraint)
        self.arm.set_path_constraints(constraints)
        

        self.arm.clear_path_constraints()


    def base_position(self):

        self.arm.set_pose_target([0.250, 0, 0.220 - 0.1137 + 0.0557, 0, 1.57, 0])
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
                          'RRTpickup' : self.pickup_by_rrt,
                          'drop_goal':self.drop_goal,
                          'pickup_from_above':self.pickup_from_above,
                          'constraints_add' : self.constraints_add,
                          'main2':self.main2,
                          'exit' : self.exit}
        
        return execute_action
    
    def rotate_pose(self, pose: Pose, angle):
        
        x = pose.orientation.x
        y = pose.orientation.y
        z = pose.orientation.z
        w = pose.orientation.w

        r, p, y = tf.transformations.euler_from_quaternion([x, y, z, w])
        y += angle

        new_x, new_y, new_z, new_w = tf.transformations.quaternion_from_euler(r, p, y)
        pose.orientation.x = new_x
        pose.orientation.y = new_y
        pose.orientation.z = new_z
        pose.orientation.w = new_w

        return pose

    def pickup_from_above(self,target_pose: Pose):
        try:    
            #target_pose : Pose = get_target_pose("green")
            self.arm.set_planner_id("RRTConnect")
            # Step 1: Open gripper
            self.gripper.set_joint_value_target([0.00, 0.00])
            self.gripper.go()

            # Step 2: Place gripper above target

            target_pose = self.rotate_pose(target_pose, np.pi/2)
            target_pose.position.z += self.desk_height + self.pickup_offset
            self.arm.set_joint_value_target(target_pose, True)
            self.arm.go()

            # Step 3: Lower the gripper to target
            target_pose.position.z -= self.pickup_offset
            self.arm.set_joint_value_target(target_pose, True)
            self.arm.go()
            rospy.sleep(1)

            # Step 4: Close gripper
            self.gripper.set_joint_value_target([-0.027, -0.027])
            self.gripper.go()
            rospy.sleep(3)

            #open
            self.gripper.set_named_target("open")
            self.gripper.go()
            rospy.sleep(1)

            #change orientation and close again

            target_pose = self.rotate_pose(target_pose, -np.pi/2)
            self.arm.set_joint_value_target(target_pose, True)
            self.arm.go()
            rospy.sleep(1)

            self.gripper.set_joint_value_target([-0.027, -0.027])
            self.gripper.go()
            rospy.sleep(2)

            # Step 5: Get back up
            target_pose.position.z += self.desk_height + self.pickup_offset
            self.arm.set_joint_value_target(target_pose, True)
            self.arm.go()

            # Step 6: Return to base position
            self.base_position()

        except Exception as e:
            rospy.logerr(e)
        
        return True 



    def pickup_by_rrt(self,target_pose):
        max_attempts = 3  
        attempt = 0
        found_plan = None  
        planners = ['RRTstar','RRTConnect', 'RRT', ]  # bring more choice for solver
        while attempt < max_attempts and not found_plan:
            for planner in planners:  
                try:
                    # set plannar
                    
                    self.arm.set_planner_id(planner)  # try different planner
                    self.arm.set_planning_time(1)  # 

                    # initialize again to avoid previous action was not done
                    
                    # self.gripper.set_joint_value_target([-0.007, -0.007])
                    # self.gripper.go()
                
                    #self.arm.clear_pose_targets()
                    # target_pose = rospy.wait_for_message("/cap120/drop_goal", Pose, timeout=5)
                    
                    #target_pose_stamped = PoseStamped()
                    #target_pose_stamped.header.frame_id = "world"
                    #target_pose_stamped.header.stamp = rospy.Time.now()
                    #target_pose_stamped.pose = target_pose
                    target_pose.position.z = self.desk_height
                    #self.arm.set_pose_target(target_pose_stamped)
                    target_pose.position.z += 0.15
                    target_pose.orientation.w = 0.707
                    target_pose.orientation.x = 0
                    target_pose.orientation.y = 0.707
                    target_pose.orientation.z = 0
                    self.arm.set_joint_value_target(target_pose, True)
                    
                    #Just replace "group.set_pose_target(pose_goal)" line with "group.set_joint_value_target(pose_goal, True)" 
                    plan = self.arm.plan()

                    # check if success
                    if plan and plan[0] and len(plan[1].joint_trajectory.points) > 0:
                        found_plan = plan[1]  
                        rospy.loginfo(f"{planner} found a plan!")
                        break  
                    else:
                        rospy.loginfo(f"{planner} failed, trying next planner.")
                except rospy.ROSException as e:
                    rospy.logerr("ROS error: {}".format(e))
                    break  
                except Exception as e:
                    rospy.logerr(f"Error in the planning with {planner}: {e}")
                    break  

            attempt += 1

        if found_plan:
            rospy.loginfo("Executing found plan!")
            self.arm.execute(found_plan) 

            try:
                target_pose.position.z -= 0.15
                self.arm.set_joint_value_target(target_pose, True)
                self.arm.go()
                # if not self.arm.go():
                #     self.collision_check = 1
                
                

                
                self.gripper.set_named_target('close')
                self.gripper.go()
                rospy.sleep(1)
                target_pose.position.z += 0.15
                self.arm.set_joint_value_target(target_pose, True)
                self.arm.go()
                target_pose.position.z -=0.15
                rospy.sleep(1)

            except Exception as e:
                rospy.logerr(e)
                return False
        
        else:
            rospy.loginfo(f"Couldn't find viable planning in {max_attempts} attempts with different planners.")

        self.base_position()
        return True  


    def pickup_bloc(self):

        try:
            # Custom trajectory
            target_pose = rospy.wait_for_message("/cap120/object_in_base", Pose, timeout=5)
            #rospy.loginfo("we ar moving to ",target_pose)
            # target_pose = Pose()
            # target_pose.position.x = 0.200
            # target_pose.position.y = -0.100
            # target_pose.position.z = 0.142
            #target_pose.position.z = -0.08
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

            rospy.sleep(3)

            # Close gripper
            self.gripper.set_joint_value_target([-0.03, -0.03])
            self.gripper.go()
            rospy.sleep(3)
        except:
            pass

        self.base_position()

        return True
    
    def drop_goal(self, target_pose):
        max_attempts = 3  
        attempt = 0
        found_plan = None  
        planners = ['RRTstar','RRTConnect', 'RRT', ]  # bring more choice for solver
        while attempt < max_attempts and not found_plan:
            for planner in planners:  
                try:
                    # set plannar
                    target_pose_copy = target_pose
                    self.arm.set_planner_id(planner)  # try different planner
                    self.arm.set_planning_time(1)  # 

                    # initialize again to avoid previous action was not done
                    
                    # self.gripper.set_joint_value_target([-0.007, -0.007])
                    # self.gripper.go()
                    # target_pose = rospy.wait_for_message("/cap120/drop_goal", Pose, timeout=5)
                    
                    #target_pose_stamped = PoseStamped()
                    #target_pose_stamped.header.frame_id = "world"
                    #target_pose_stamped.header.stamp = rospy.Time.now()
                    #target_pose_stamped.pose = target_pose
                    target_pose_copy.position.z += self.desk_height
                    #self.arm.set_pose_target(target_pose_stamped)
                    target_pose_copy.position.z += self.pickup_offset/2
                    target_pose_copy.orientation.w = 0.707
                    target_pose_copy.orientation.x = 0
                    target_pose_copy.orientation.y = 0.707
                    target_pose_copy.orientation.z = 0
                    self.arm.set_joint_value_target(target_pose_copy, True)
                    
                    #Just replace "group.set_pose_target(pose_goal)" line with "group.set_joint_value_target(pose_goal, True)" 
                    plan = self.arm.plan()

                    # check if success
                    if plan and plan[0] and len(plan[1].joint_trajectory.points) > 0:
                        found_plan = plan[1]  
                        rospy.loginfo(f"{planner} found a plan!")
                        break  
                    else:
                        rospy.loginfo(f"{planner} failed, trying next planner.")
                except rospy.ROSException as e:
                    rospy.logerr("ROS error: {}".format(e))
                    break  
                except Exception as e:
                    rospy.logerr(f"Error in the planning with {planner}: {e}")
                    break  

            attempt += 1

        if found_plan:
            rospy.loginfo("Executing found plan!")
            self.arm.execute(found_plan) 

            try:
                target_pose_copy.position.z -= self.pickup_offset/2
                self.arm.set_joint_value_target(target_pose_copy, True)
                self.arm.go()
                # if not self.arm.go():
                #     self.collision_check = 1
                
                
                #go back
                rospy.sleep(.1)
                self.gripper.set_named_target('open')
                self.gripper.go()
                target_pose_copy.position.z += self.pickup_offset
                rospy.sleep(.1)
                self.arm.set_joint_value_target(target_pose, True)
                self.arm.go()
                target_pose_copy.position.z -= self.pickup_offset
                rospy.sleep(.1)

            except Exception as e:
                rospy.logerr(e)
                return False
        
        else:
            rospy.loginfo(f"Couldn't find viable planning in {max_attempts} attempts with different planners.")

        self.base_position()
        return True  
    
    def check_closure(self):
        # get gripper value for full closure checking
        closure = self.gripper.get_current_joint_values()
        
        # set error threshold
        error = 0.005
        close_val = -0.029
        try:
            if ((closure[0] - close_val)> error and (closure[1] - close_val)> error):
                return True

        except:
            pass

        return False

    def add_box(self, desired_pose = Pose(), shape_name = 'box'):
        #rospy.init_node('add_collision_object_py', anonymous=True)
        size = self.shapes.get(shape_name, self.shapes['box'])

        scene = PlanningSceneInterface()

        box_name = "box {}".format(self.box_num)
        # scene.remove_world_object(box_name)
        self.box_num += 1

        box_pose = PoseStamped()
        box_pose.header.frame_id = self.arm.get_planning_frame()
        width, length, height = size
        box_pose.pose.orientation.w = desired_pose.orientation.w
        box_pose.pose.position.x = desired_pose.position.x  # Position of the box
        box_pose.pose.position.y = desired_pose.position.y
        box_pose.pose.position.z = desired_pose.position.z

        scene.add_box(box_name, box_pose, size=(width, length, height))  # Size of the box

        rospy.sleep(.1)  # Wait for the above operations to be completed


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