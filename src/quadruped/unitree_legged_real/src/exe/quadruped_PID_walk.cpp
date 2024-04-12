
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
const double Ki = 0.0001; // 积分增益
const double Kd = 0.05;
const double aKp = 1;
const double aKi = 0.02;
const double aKd = 0.0001;

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
    angle = fmod(angle, 2 * M_PI);
    if (angle < - M_PI)
        angle += 2 * M_PI;
    return angle;
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    //std::cout << "position subscribed" << std::endl;
    current_pose.position = msg->pose.pose.position;
    current_pose.orientation = msg->pose.pose.orientation;
    // std::cout << "current_pose.orientation" << current_pose.orientation <<std::endl;
    
}

double getYawFromQuaternion(const geometry_msgs::Quaternion& quat) {
    // std::cout<< "quat "<< "\n" << quat <<std::endl;
    if (quat.x == 0 && quat.y == 0 && quat.z == 0 && quat.w == 0) {
        std::cout << "Invalid quaternion, using default value." << std::endl;


        return std::numeric_limits<double>::quiet_NaN(); // return initialization
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

void adjust_Yaw_and_velocity_TowardsGoal() {
    const double ang_tolerance = M_PI/1000; // Goal tolerance
    const double targetyaw = 9*M_PI/180; // 示例目标偏航角
    double currentyaw = getYawFromQuaternion(current_pose.orientation);
    if(std::isnan(currentyaw)) {
        
        high_cmd_ros.mode = 2; // 空闲，默认站立
        high_cmd_ros.velocity[0] = 0.0;
        high_cmd_ros.velocity[1] = 0.0;
    }else {
        // std::cout << "currentyaw" << currentyaw << std::endl;
    // std::cout << "MPI" << M_PI << std::endl;
    // std::cout << "currentyaw - MPI = " << currentyaw - M_PI<<std::endl;
    // // wrapTiPi make sure it is inside the range [-π, π]
    currentyaw += M_PI; 
    double yawtogo = wrapToPi(targetyaw - currentyaw);
    std::cout<< "yaw to go = " << yawtogo <<std::endl;
    
    double integral_ang = 0;
    double prev_error_ang = 0;

    const double goalPositionX = 4.28; // target x
    const double goalPositionY = 2.78; // target y
    const double tolerance = 0.02; // tolerance for position

    // calculate error (absolute frame)
    double error_x = goalPositionX - current_pose.position.x;
    double error_y = goalPositionY - current_pose.position.y;

    std::cout << "currentyaw" << currentyaw << std::endl;
    // std::cout << "yaw error" << yawtogo << std::endl;
    // std::cout << "distance to goal_x " << std::to_string(error_x) << std::endl;
    // std::cout << "distance to goal_y " << std::to_string(error_y) << std::endl;

    // define how much robot need to go (robot frame)
    double robot_x_error = error_x * cos(currentyaw) + error_y * sin(currentyaw);
    double robot_y_error = -error_x * sin(currentyaw) + error_y * cos(currentyaw); 

    // integrate errors
    integral_x += robot_x_error;
    integral_y += robot_y_error;
    integral_ang += yawtogo;

    // 计算误差的变化率（微分项）
    double derivative_x = robot_x_error - prev_error_x;
    double derivative_y = robot_y_error - prev_error_y;
    double derivative_ang = yawtogo - prev_error_ang;

    //record current error as next step prev_error
    prev_error_x = robot_x_error;
    prev_error_y = robot_y_error; 
    prev_error_ang = yawtogo;

    // 计算PID控制下的速度
    double velocity_x = Kp * robot_x_error + Ki * integral_x + Kd * derivative_x;
    double velocity_y = Kp * robot_y_error + Ki * integral_y + Kd * derivative_y;
    double angular_velocity = aKp * yawtogo + aKi * integral_ang + aKd * derivative_ang;
    

    // If it is inside the error
    

    
    if(fabs(robot_x_error) < tolerance && fabs(robot_y_error) < tolerance && std::abs(yawtogo) < ang_tolerance) {
        stop_flag = true;
        high_cmd_ros.mode = 0; // 空闲，默认站立
        high_cmd_ros.velocity[0] = 0.0;
        high_cmd_ros.velocity[1] = 0.0;
        done_flag = true;
        stop_flag = true;
    } else {
        
        high_cmd_ros.mode = 2; // 移动模式
        high_cmd_ros.gaitType = 2; // 自定义步态
        high_cmd_ros.yawSpeed = std::min(std::max(angular_velocity, -0.5), 0.5); // 限制速度范围，避免超出最大速度}
        std::cout << "current angular velocity" << std::to_string(angular_velocity) << std::endl;

        high_cmd_ros.velocity[0] = std::min(std::max(velocity_x, -0.6), 0.6); // 限制速度范围，避免超出最大速度
        high_cmd_ros.velocity[1] = std::min(std::max(velocity_y, -0.6), 0.6); // 限制速度范围，避免超出最大速度
        
        std::cout << "velocity_x = " << velocity_x << std::endl;
        std::cout << "velocity_y = " << velocity_y << std::endl;
    }

    }
    


    
    
}


void commandCallback(const std_msgs::String::ConstPtr& msg){
    std::cout << "subscribed to command_topic" << std::endl;
    
    // according message to decide go1 action
    if (msg->data == "forward"){
        //forward
        stop_flag = false;
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


void armCallback(const std_msgs::String::ConstPtr& msg){
    
    stop_flag = false;
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

    ros::Subscriber move_arm_sub = nh.subscribe("/pick_up_first_done", 500, armCallback);
    ros::Publisher pub_back_to_arm = nh.advertise<std_msgs::String>("/quadruped_walk_done", 1000);

    Init_param();

    // ros::Subscriber pos_sub = nh.subscribe("dog_uwb_1/position", 1000, positionCallback);
    ros::Subscriber pos_sub = nh.subscribe("uwb_odom/", 500, odometryCallback);

    ros::Publisher pub_high_command = nh.advertise<unitree_legged_msgs::HighCmd>("/high_cmd", 1000);
    
    // define a subscriber to subscribe to "command_topic"
    ros::Subscriber sub = nh.subscribe("command_topic", 500, commandCallback);

    ros::Rate loop_rate(500);
    while (ros::ok())
    {
        if (!stop_flag) {
            std::cout << "in the yaw loop" << std::endl;  
            adjust_Yaw_and_velocity_TowardsGoal();
        }
        
        ros::spinOnce();

        if(stop_flag==true) {
            std::cout << "robot stop" << std::endl;
            high_cmd_ros.mode = 0; // 空闲，默认站立
            high_cmd_ros.velocity[0] = 0.0;
            high_cmd_ros.velocity[1] = 0.0;
        }

        if(done_flag) {
            std_msgs::String msg;
            msg.data = "Hello, ROS!";

            // ROS_INFO("%s", msg.data.c_str());
            pub_back_to_arm.publish(msg);
        }

        pub_high_command.publish(high_cmd_ros); // Publish the command
        

        // std::cout << "spin once" << std::endl;
        
        loop_rate.sleep();
    }

    return 0;
}