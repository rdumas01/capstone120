#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h" // Assuming convert.h is provided for necessary conversions
#include <std_msgs/String.h>
#include <sstream>
#include <math.h>

using namespace UNITREE_LEGGED_SDK;

unitree_legged_msgs::HighCmd high_cmd_ros;
geometry_msgs::Point target_position; // 目标位置
geometry_msgs::Point current_position; // 当前位置，从/odom更新
bool moving_to_target = false; // 是否正在移动到目标

// 初始化参数
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

// 更新当前位置的回调函数
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_position = msg->pose.pose.position; // 更新当前位置
}

// 接收到命令的回调函数
void commandCallback(const std_msgs::String::ConstPtr& msg)
{
    std::stringstream ss(msg->data);
    std::string command;
    ss >> command;
    
    if (command == "move_to") {
        float x, y;
        ss >> x >> y;
        target_position.x = x;
        target_position.y = y;
        moving_to_target = true; // 设置标志，开始移动到目标
    }
}

// 计算两点之间的距离
float calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

// 主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_control");
    ros::NodeHandle nh;

    Init_param();

    ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odomCallback);
    ros::Subscriber command_sub = nh.subscribe("command_topic", 1000, commandCallback);
    ros::Publisher pub = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1000);

    ros::Rate loop_rate(10); // 控制频率
    while (ros::ok())
    {
        if (moving_to_target) {
            float distance = calculateDistance(current_position, target_position);
            
            if (distance > 0.1) { // 如果距离目标超过0.1米，则继续前进
                high_cmd_ros.mode = 2; // 设置为前进模式
                high_cmd_ros.velocity[0] = 0.1; // 设置前进速度，根据需要调整
                // 根据需要调整其他参数，如转向等
            } else {
                high_cmd_ros.mode = 0; // 到达目标，停止
                moving_to_target = false; // 重置移动标志
            }
        }

        pub.publish(high_cmd_ros); // 发布命令
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

// rostopic pub /command_topic std_msgs/String "data: 'move_to 1.0 2.0'"
