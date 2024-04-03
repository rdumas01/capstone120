#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h" // this red~ is ok
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>  // add this line to import Odometry msg type

#include <geometry_msgs/Point.h>
#include <iostream>
#include <algorithm>
#include <limits>

using namespace UNITREE_LEGGED_SDK;

unitree_legged_msgs::HighCmd high_cmd_ros;
geometry_msgs::Pose current_pose;

// PID gains - Only Kp is used for proportional control
double Kp = 0.05; // Proportional gain for simplicity
double target_position_x = 3; // Set your target goal position

void Init_param(){
    high_cmd_ros.head[0] = 0xFE;
    high_cmd_ros.head[1] = 0xEF;
    high_cmd_ros.levelFlag = HIGHLEVEL;
    high_cmd_ros.mode = 0;
    high_cmd_ros.gaitType = 0;
    high_cmd_ros.speedLevel = 0;
    high_cmd_ros.footRaiseHeight = 0;
    high_cmd_ros.bodyHeight = 0;
    high_cmd_ros.euler[0] = 0;
    high_cmd_ros.euler[1] = 0;
    high_cmd_ros.euler[2] = 0;
    high_cmd_ros.velocity[0] = 0.0f;
    high_cmd_ros.velocity[1] = 0.0f;
    high_cmd_ros.yawSpeed = 0.0f;
    high_cmd_ros.reserve = 0;
}

void commandCallback(const std_msgs::String::ConstPtr& msg){
    std::cout << "subscribed to command_topic" << std::endl;
    
    // according message to decide go1 action
    if (msg->data == "forward"){
        //forward
        adjustVelocityTowardsGoal();
    }
    else if (msg->dat == "stop")
    {
        // stop mode
        std::cout << "go1 stop" << std::endl;
        high_cmd_ros.mode = 0; // idle, default stand
        high_cmd_ros.velocity[0] = 0.0; // stop
    }

}

void positionCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_pose.position.x = msg->pose.pose.position.x;
    current_pose.position.y = msg->pose.pose.position.y;
    current_pose.position.z = msg->pose.pose.position.z;
}

void adjustVelocityTowardsGoal() {
    const double linearVelocityConstant = 0.1;
    const double angularVelocityConstant = 0.1;
    const double goalPositionX = 5.0; // Example goal, this could be set dynamically
    const double goalPositionY = 0.0; // Assuming a 2D plane, Y goal is set to 0 for simplicity
    const double goalYaw = 0.0;
    const double tolerance = 0.1; // Goal tolerance

    double distanceToGoal = sqrt(pow(goalPositionX - current_pose.position.x, 2) + pow(goalPositionY - current_pose.position.y, 2));

    if(distanceToGoal > tolerance) {
        // Calculate the angle to the goal
        double angleToGoal = atan2(goalPositionY - current_pose.position.y, goalPositionX - current_pose.position.x);

        // Calculate proportional linear velocity
        double linearVelocity = linearVelocityConstant * distanceToGoal;
        // Limit linear velocity to a maximum value for safety
        linearVelocity = std::min(linearVelocity, 0.2);

        // Calculate angular velocity needed to turn towards the goal
        double angularVelocity = angularVelocityConstant * angleToGoal; // Simplified, assumes robot is oriented along one axis

        // Update command velocities
        high_cmd_ros.velocity[0] = linearVelocity;
        high_cmd_ros.yawSpeed = angularVelocity; // Assuming yawSpeed controls angular velocity
    } else {
        // Stop the robot if within tolerance
        high_cmd_ros.velocity[0] = 0.0;
        high_cmd_ros.yawSpeed = 0.0;
        high_cmd_ros.euler[0] = 0;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "command_listener");
    std::cin.ignore();

    ros::NodeHandle nh;

    Init_param();

    ros::Subscriber pos_sub = nh.subscribe("/uwb_odom", 1000, positionCallback);

    // define a subscriber to subscribe to "command_topic"
    ros::Subscriber sub = nh.subscribe("/command_topic", 1000, commandCallback);

    ros::Publisher pub = nh.advertise<unitree_legged_msgs::HighCmd>("/high_cmd", 1000);
    
    ros::Rate loop_rate(500);
    while (ros::ok())
    {
        adjustVelocityTowardsGoal(); // Adjust the velocity based on current and target positions
        
        pub.publish(high_cmd_ros); // Publish the command
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}