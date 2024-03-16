#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped
from copy import deepcopy
from std_msgs.msg import String
import tf, tf2_ros
import numpy as np
import moveit_msgs
from compute_trajectory import gen_trajectory
from moveit_msgs.msg import OrientationConstraint, Constraints
from get_target_pose import get_target_pose
import os

from blueprint_to_coordinates import find_object, build_castle, flatten_layers, create_poses_colors_shape_list, plot_castle_structure


class move_arm_node:


    def __init__(self):
        #######################################
        # initialize box number
        self.box_num  = 0
        self.collision_check = 0
        self.desk_height = 0.01 # 0.085
        self.pickup_offset = 0.04
        self.cube_height = 0.029

        # define a dictionary to store box shapes dimensions
        self.shapes = {
            'box': [0.03, 0.03, 0.03],
            'rectangular': [0.09, 0.03, 0.03]
        }
        #######################################

        # Initialize the API of move_group
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialization requires using the move group to control the arm group in the robotic arm
        self.arm = MoveGroupCommander('sagittarius_arm')
        self.gripper = MoveGroupCommander('sagittarius_gripper')

        # Allow re-planning after motion planning fails
        self.arm.allow_replanning(True)
        
        # Set the reference coordinate system used for the target position
        reference_frame = 'world'
        self.arm.set_pose_reference_frame(reference_frame)
        self.gripper.set_pose_reference_frame(reference_frame) #夹爪
                
        # Set the allowable error for position (unit: meters) and orientation (unit: radians)
        self.arm.set_goal_position_tolerance(0.0001)
        self.arm.set_goal_orientation_tolerance(0.0001)
        self.arm.set_goal_joint_tolerance(0.0001)
        
        # Set the allowed maximum speed and acceleration
        self.arm.set_max_acceleration_scaling_factor(1)
        self.arm.set_max_velocity_scaling_factor(1)

        #set the total planning attempts
        self.arm.set_num_planning_attempts(10)
        # Get the name of the end effector link
        self.end_effector_link = self.arm.get_end_effector_link()
        # rospy.logerr(self.end_effector_link)

        # Put arm in initial pose
        self.base_position()

        self.main()

        # Shut down MoveIt
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
            rospy.sleep(0.3) #pretty important to have a sleep here to make sure the previous action have already be executed
        
        rospy.loginfo('Exiting main loop...')
    
    
    def from_blue_print(self):
        '''
        "find_object" function: Scans a png/jpg image and finds objects and colors. Dimensions are in pixel coordinates
        "build_castle" function: Outputs sequential list based on x and z coordinate of the objects - objects are differentiated by layers. dimensions are now in "m" unit.
        "flatten_layers" function : Gets rid of layers and outputs a simple list only
        "final list" variable: List from multiple images can be combined here
        "create_poses_colors_shape_List" function: Converts the information obtained and creates separate list of poses, colors and shape
        "
        '''
        # Define the relative path to the blueprint image
        blueprint_filename = "blueprint_cube_only.png"
        # blueprint_filename = "blueprint_cube_rect.png"
        # Get the directory where the script is located
        script_dir = os.path.dirname(__file__)
        # Construct the full path to the blueprint image
        blueprint_path = os.path.join(script_dir, blueprint_filename)

        # Use the constructed path in the find_object function
        found_object_results = find_object(blueprint_path)
        
        ## Very important - for the "cube_only", use pixel_to_m = .029/144. For "cube_rect", use pixel_to_m = .029/144
        sequential_blocks = build_castle(found_object_results,pixel_to_m=.029/182, y_offset=0.150)
        # sequential_blocks = build_castle(found_object_results,pixel_to_m=.029/144, y_offset=0.150)
        flattened_blocks = flatten_layers(sequential_blocks)
        final_list=flattened_blocks

        #pose_list,shape_list,color_list
        poses_colors_shape = create_poses_colors_shape_list(final_list)
        plot_castle_structure(final_list)
        # rospy.logerr(poses_colors_shape)
        #  pose_list, shape_list, color_list
        return poses_colors_shape

    
    def execute_all(self):
    # Use from_blue_print2 to get poses, shapes, and colors
        self.poses_shapes_colors = self.from_blue_print()
        rospy.loginfo('Entering main loop...')

        # Prepare the robot arm and gripper
        self.lookout()
        self.drop()

    # Iterate over the list of poses, shapes, and colors
        for item in self.poses_shapes_colors:
            pose, shape, color = item  # Unpack the tuple

            grabbed = False
            while not grabbed:
                # if you are testing in simulation, you need to comment out this section 
                ################################### 
                target_pose = self.look_around(color=color, shape=shape)
                self.drop()
                self.pickup_from_above(target_pose)
                # rospy.sleep(.1)

                # if grabbed, return True, then exit while loop to execute drop_bloc
                grabbed = self.check_closure()
                # rospy.sleep(.1)
                ##################################
            if target_pose !=None:
                self.drop_goal(pose)

            # if(self.collision_check == 0):
            #     self.add_box(step, 'box')
            #     rospy.sleep(1)
            
            rospy.sleep(0.1)

        self.sleep()

        rospy.loginfo('Exiting main loop...')
    

    def look_around(self, color : str = None, shape : str = None):
        target_pose = None
        angle = [1,2,3,4,4,3,2,1]
        found = False
        i = 0

        while not found:
            if(i == 4):
                self.rotate_joint("joint2",1.57/7)
                self.arm.go()
                rospy.sleep(.1)
                self.rotate_joint("joint3",-1.57/7)
                self.arm.go()
            
            if(i%8 == 0 and i!=0):
                self.base_position()
                
            #search near area until it found one (rotate joint1)
            self.rotate_joint("joint1", angle[i%8]*0.4884)
            self.arm.go()
            
            # rospy.sleep(.1)  
            target_pose = get_target_pose(color, shape)

            target_pose_copy = deepcopy(target_pose)  
            # rospy.sleep(0.1)
            # self.arm.stop()
            # self.arm.clear_pose_targets()
            i+=1
            
            if target_pose_copy is not None:
                return target_pose_copy

        rospy.logwarn("Cannot find a block")
     
        return None

    
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

        # rospy.sleep(.1)
    

    def actions_dict(self):
        #both execute_all2 and look_around2 are placeholders and will eventually replace execute_all and look_around
        execute_action = {'drop' : self.drop,
                          'lookout' : self.lookout,
                          'lookaround' : self.look_around,
                          'sleep' : self.sleep,
                          'RRTpickup' : self.pickup_by_rrt,
                          'drop_goal':self.drop_goal,
                          'pickup_from_above':self.pickup_from_above,
                          'constraints_add' : self.constraints_add,
                          'execute_all':self.execute_all,
                          'rotate_joint':self.rotate_joint,
                          'exit' : self.exit}
        
        return execute_action


    def rotate_joint(self, joint_name: String = "joint6", angle=np.pi/2, absolute = True): #the angle is in radian
        """
        Rotate a single joint to a specific angle.

        Parameters:
        - joint_name: The name of the joint to rotate.
        - angle: The target angle in radians.

        we have 8 joint to use

        joint1 revolute
        joint2 revolute
        joint3 revolute
        joint4 revolute
        joint5 revolute
        joint6 revolute

        joint_gripper_left prismatic
        joint_gripper_right prismatic

        """
        # get each joint current position
        joint_positions = self.arm.get_current_joint_values()
        joint_index = self.arm.get_active_joints().index(joint_name)
        if absolute:
            joint_positions[joint_index] = angle
        else:
            joint_positions[joint_index] += angle
        self.arm.set_joint_value_target(joint_positions) #only change that joint

        return True
    
    
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


    def pickup_from_above(self,target_pose: Pose = None):

        if target_pose is None:
            self.base_position()
            rospy.logerr("None is sent to pickup_from_above")
            return True
        try:    
            #target_pose : Pose = get_target_pose("green")
            self.arm.set_planner_id("RRTConnect")

            # Step 1: Open gripper
            self.gripper.set_joint_value_target([0.00, 0.00])
            self.gripper.go(wait=False)
            # rospy.sleep(.1)

            # Step 2: Place gripper above target
            target_pose = self.rotate_pose(target_pose, np.pi/2)
            target_pose.position.z += self.desk_height + self.pickup_offset
            self.arm.set_joint_value_target(target_pose, True)
            self.arm.go(wait=True)
            
            # Step 3: Lower the gripper to target
            target_pose.position.z -= self.pickup_offset
            self.arm.set_joint_value_target(target_pose, True)
            self.arm.go()
            # rospy.sleep(.1)

            # Step 4: Close gripper
            self.gripper.set_joint_value_target([-0.027, -0.027])
            self.gripper.go()
            self.gripper.stop()
            rospy.sleep(1.3)

            # open
            self.gripper.set_named_target("open")
            self.gripper.go()
            rospy.sleep(.5)

            #change orientation and close again

            # target_pose = self.rotate_pose(target_pose, -np.pi/2)
            # self.arm.set_joint_value_target(target_pose, True)
            joint_positions = self.arm.get_current_joint_values()
            joint6_index = self.arm.get_active_joints().index('joint6')
            joint2_index = self.arm.get_active_joints().index('joint2')
            
            joint_positions[joint2_index] += 0.05
            self.arm.set_joint_value_target(joint_positions)
            self.arm.go()

            joint_positions[joint6_index] -= np.pi/2
            self.arm.set_joint_value_target(joint_positions)
            self.arm.go()
            # rospy.sleep(.1)
            #rospy.sleep(1)

            self.gripper.set_joint_value_target([-0.027, -0.027])
            self.gripper.go()
            rospy.sleep(1.3)

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

    
    def drop_goal(self, target_pose):
        max_attempts = 3  
        attempt = 0
        found_plan = None  
        planners = ['"RRTConnectkConfigDefault"','RRTstar','RRTConnect', 'RRT', ]  # bring more choice for solver
        while attempt < max_attempts and not found_plan:
            for planner in planners:  
                try:
                    # set plannar
                    target_pose_copy = target_pose
                    self.arm.set_planner_id(planner)  # try different planner
                    self.arm.set_planning_time(0.5)  # 

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
                
                # go back
                # rospy.sleep(.1)
                self.gripper.set_named_target('open')
                self.gripper.go()
                target_pose_copy.position.z += self.pickup_offset*2
                # rospy.sleep(.1)
                self.arm.set_joint_value_target(target_pose_copy, True)
                self.arm.go()
                target_pose_copy.position.z -= self.pickup_offset*2
                # rospy.sleep(.1)

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
        '''
        Open gripper
        '''

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