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

// global variable, stores commands
unitree_legged_msgs::HighCmd high_cmd_ros;

//global variable for "current_position". 
//This could change based on how we get current position- Confirm with Qiayuan what format the current position is received.
geometry_msgs::Point current_position;

//Initialize various paramets
double integral_error = 0.0, previous_error = 0.0;
ros::Time previous_time;
bool first_command = true;

// PID gains
double Kp = 0.05, Ki = 0.1, Kd = 0.05;

//Target position_x #ToDo- update this value with a realistic target value
double target_position_x = 3;

// init
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

// callback function to update robot's current position with the Odometry message
void positionCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // extract position info from Odomoetry
    current_position.x = msg->pose.pose.position.x;
    current_position.y = msg->pose.pose.position.y;
    current_position.z = msg->pose.pose.position.z;
}

// callback function for commands received from the topic
void commandCallback(const std_msgs::String::ConstPtr& msg)
{
    std::cout << "subscribed" << std::endl;
    
    // according message to decide go1 action
    if (msg->data == "forward") {
        std::cout << "go1 moving forward" << std::endl;
        high_cmd_ros.mode = 2;
        high_cmd_ros.gaitType = 2;


        //PID Implementation for position- Do we need to extend in this 3D for x,y and z position?

        double error = target_position_x - current_position.x;

        // Calculate time delta
        ros::Time now = ros::Time::now();
        double time_delta = (now - previous_time).toSec();
        if (first_command) {
            time_delta = 0.01; // Default to a small time step if first command
            first_command = false;
        }
        previous_time = now;

        // Update integral and derivative of the error
        integral_error += error * time_delta;
        double derivative_error = (error - previous_error) / time_delta;
        previous_error = error;

        // Calculate control signal with PID formula
        double control_signal = Kp * error + Ki * integral_error + Kd * derivative_error;

        // Limit control signal to maximum velocity- No need to go above 0.2 I think.
        double max_velocity = 0.2;
        control_signal = std::min(std::max(control_signal, -max_velocity), max_velocity);

        //Final commands
        high_cmd_ros.velocity[0] = control_signal;
        
        ///Uncomment the line below if position control not wanted
        //high_cmd_ros.velocity[0] = 0.2f; // -1 ~ 1
        // high_cmd_ros.yawSpeed = 2; // if have this, it will rotate, if not, it will walk
        high_cmd_ros.footRaiseHeight = 0.1;
        std::cout << "mode changed" << std::endl;
        std::cout << high_cmd_ros << std::endl;
       
       
        // high_cmd_ros.yawSpeed = 2; // if have this, it will rotate, if not, it will walk
    } else if (msg->data == "stop") {
        // stop mode
        std::cout << "go1 stop" << std::endl;

        // Reset PID components
        integral_error = 0.0;
        previous_error = 0.0;
        first_command = true;

        high_cmd_ros.mode = 0; // idle, default stand
        high_cmd_ros.velocity[0] = 0.0; // stop
    }
    // add other modes
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "command_listener");

    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle nh; 

    Init_param();

    previous_time = ros::Time::now(); // Initialize previous_time

    // unitree_legged_msgs::HighCmd high_cmd_ros;

    // define a subscriber to subscribe to the position topic
    ros::Subscriber pos_sub = nh.subscribe("/uwb_odom", 1000, positionCallback);
    
    // define a subscriber to subscribe to "command_topic"
    ros::Subscriber sub = nh.subscribe("command_topic", 1000, commandCallback);

    // publisher to publish command to go1 with ros_udp.cpp
    ros::Publisher pub = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1000);
    
    ros::Rate loop_rate(500); // 500hz
    while (ros::ok()) // while true
    {     
        pub.publish(high_cmd_ros); // publish rate specified by ros::Rate loop_rate(500)
        ros::spinOnce();// update info once
        loop_rate.sleep(); // stop at the rate: ros::Rate loop_rate(500)
    }

    return 0;
}