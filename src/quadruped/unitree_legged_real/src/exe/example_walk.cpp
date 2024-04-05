
#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h" // this red~ is ok
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>  // add this line to import Odometry msg type
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <algorithm>
#include <limits>
#include <cmath>

using namespace UNITREE_LEGGED_SDK;

unitree_legged_msgs::HighCmd high_cmd_ros;
geometry_msgs::Pose current_pose;
bool stop_flag = true;
bool yaw_flag = true;
bool done_flag = false;

// PID gains - Only Kp is used for proportional control
const double Kp = 0.5; // 比例增益
const double Ki = 0.00001; // 积分增益
const double Kd = 0.000;
double integral_x = 0.0; // X方向的积分项
double integral_y = 0.0; // Y方向的积分项
double prev_error_x = 0.0; // 上一次X方向的误差
double prev_error_y = 0.0; // 上一次Y方向的误差


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

double wrapToPi(double angle) {
    // 将角度限制在 [-π, π] 范围内
    angle = fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0)
        angle += 2 * M_PI;
    return angle - M_PI;
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    //std::cout << "position subscribed" << std::endl;
    current_pose.position = msg->pose.pose.position;
    current_pose.orientation = msg->pose.pose.orientation;
    // std::cout << "current_pose.orientation" << current_pose.orientation <<std::endl;
    
}

double getYawFromQuaternion(const geometry_msgs::Quaternion& quat) {
    std::cout<< "quat "<< "\n" << quat <<std::endl;
    if (quat.x == 0 && quat.y == 0 && quat.z == 0 && quat.w == 0) {
        std::cout << "Invalid quaternion, using default value." << std::endl;
        return 0.1; // return initialization
    }
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}


// void positionCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
//     //std::cout << "position subscribed" << std::endl;
//     current_pose.position.x = msg->point.x;
//     current_pose.position.y = msg->point.y;
//     current_pose.position.z = msg->point.z;
// }

void adjustYawTowardsGoal() {
    const double ang_tolerance = M_PI/1000; // Goal tolerance
    const double targetyaw = M_PI; // 示例目标偏航角

    double currentyaw = getYawFromQuaternion(current_pose.orientation);
    std::cout << "currentyaw" << currentyaw << std::endl;

    // 计算当前偏航角与目标偏航角之间的差值，并应用 wrapToPi 确保其在 [-π, π] 范围内
    double yawtogo = wrapToPi(currentyaw - targetyaw);

    const double angular_velocity = 0.5;
    
    if(std::abs(yawtogo) > ang_tolerance) {
        high_cmd_ros.mode = 2;
        high_cmd_ros.gaitType = 2;
        high_cmd_ros.velocity[0] = 0.0;
        high_cmd_ros.velocity[1] = 0.0;
        if(yawtogo > 0) {high_cmd_ros.yawSpeed = -angular_velocity;}
        else {high_cmd_ros.yawSpeed = angular_velocity;}
    }
    else{
        // Stop the robot if within tolerance
        yaw_flag = true;
        std::cout << "go1 stop" << std::endl;
        high_cmd_ros.mode = 0; // idle, default stand
        high_cmd_ros.velocity[0] = 0.0;
        high_cmd_ros.yawSpeed = 0.0;
        high_cmd_ros.euler[0] = 0;
    }

    std::cout << "current angular velocity" << std::to_string(angular_velocity) << std::endl;
}


void adjustVelocityTowardsGoal() {
    const double goalPositionX = 4.28; // 目标位置X
    const double goalPositionY = 2.78; // 目标位置Y
    const double tolerance = 0.02; // 容忍度

    // calculate error
    double error_x = goalPositionX - current_pose.position.x;
    double error_y = goalPositionY - current_pose.position.y;

    std::cout << "distance to goal_x " << std::to_string(error_x) << std::endl;
    std::cout << "distance to goal_y " << std::to_string(error_y) << std::endl;

    // integrate errors
    integral_x += error_x;
    integral_y += error_y;

    // 计算误差的变化率（微分项）
    double derivative_x = error_x - prev_error_x;
    double derivative_y = error_y - prev_error_y;

    // 计算PID控制下的速度
    double velocity_x = Kp * error_x + Ki * integral_x + Kd * derivative_x;
    double velocity_y = Kp * error_y + Ki * integral_y + Kd * derivative_y;

   
    
    // 如果误差在容忍度之内，则停止
    if(fabs(error_x) < tolerance && fabs(error_y) < tolerance) {
        stop_flag = true;
        high_cmd_ros.mode = 0; // 空闲，默认站立
        high_cmd_ros.velocity[0] = 0.0;
        high_cmd_ros.velocity[1] = 0.0;
        done_flag = true;
    } else {
        stop_flag = false;
        high_cmd_ros.mode = 2; // 移动模式
        high_cmd_ros.gaitType = 2; // 自定义步态
        high_cmd_ros.velocity[0] = std::min(std::max(velocity_x, -0.6), 0.6); // 限制速度范围，避免超出最大速度
        high_cmd_ros.velocity[1] = std::min(std::max(velocity_y, -0.6), 0.6); // 限制速度范围，避免超出最大速度
        
        std::cout << "velocity_x = " << velocity_x << std::endl;
        std::cout << "velocity_y = " << velocity_y << std::endl;
    }

    // 更新上一次的误差
    prev_error_x = error_x;
    prev_error_y = error_y;
}

void commandCallback(const std_msgs::String::ConstPtr& msg){
    std::cout << "subscribed to command_topic" << std::endl;
    
    // according message to decide go1 action
    if (msg->data == "forward"){
        //forward
        stop_flag = false;
        yaw_flag = false;
        // adjustVelocityTowardsGoal();
    }
    else if (msg->data == "stop")
    {   
        stop_flag = true;
        // stop mode
        std::cout << "go1 stop" << std::endl;
        high_cmd_ros.mode = 0; // idle, default stand
        high_cmd_ros.velocity[0] = 0.0; // stop
    }
    

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "command_listener");
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    std::cout << "at main function" << std::endl;
    ros::NodeHandle nh;



    Init_param();

    //receive command from move_arm.py
    // ros::Duration timeout(1000.0);
    boost::shared_ptr<std_msgs::String const> sharedPtr;
    sharedPtr = ros::topic::waitForMessage<std_msgs::String>("/pick_up_first_done");
    if (sharedPtr != NULL)
    {
        ROS_INFO("Received message: %s", sharedPtr->data.c_str());
    }
    else
    {
        ROS_INFO("No message received within the timeout period.");
    }
    //

    // ros::Subscriber pos_sub = nh.subscribe("dog_uwb_1/position", 1000, positionCallback);
    ros::Subscriber pos_sub = nh.subscribe("uwb_odom/", 500, odometryCallback);

    ros::Publisher pub = nh.advertise<unitree_legged_msgs::HighCmd>("/high_cmd", 1000);
    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("/quadruped_walk_done", 1000);
    // define a subscriber to subscribe to "command_topic"
    ros::Subscriber sub = nh.subscribe("command_topic", 500, commandCallback);

    ros::Rate loop_rate(500);
    while (ros::ok())
    {
        if (!yaw_flag) {
            std::cout << "in the yaw loop" << std::endl;  
            adjustYawTowardsGoal();
        }
        
        ros::spinOnce();

        if(stop_flag==true) {
            high_cmd_ros.mode = 0; // 空闲，默认站立
            high_cmd_ros.velocity[0] = 0.0;
            high_cmd_ros.velocity[1] = 0.0;
        }

        pub.publish(high_cmd_ros); // Publish the command
        if (!stop_flag && yaw_flag) {
            adjustVelocityTowardsGoal();
        } // Adjust the velocity based on current and target positions

        if(done_flag) {
            
            std_msgs::String msg;
            msg.data = "Hello, ROS!";

            ROS_INFO("%s", msg.data.c_str());

            chatter_pub.publish(msg);
        }

        std::cout << "spin once" << std::endl;
        
        loop_rate.sleep();
    }

    return 0;
}