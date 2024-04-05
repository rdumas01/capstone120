#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <unitree_legged_msgs/HighCmd.h>

// 用於接收odom的定位資訊
geometry_msgs::PoseStamped current_pose;

// 定位資訊的回呼函式
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 从odom消息中提取机器人当前位置
    current_pose.pose.position.x = msg->pose.pose.position.x;
    current_pose.pose.position.y = msg->pose.pose.position.y;
    current_pose.pose.orientation = msg->pose.pose.orientation;
}

// 定义发布机器人控制指令的函数
void moveRobot(double linear_x, double linear_y, double angular_z, ros::Publisher& pub) {
    // 创建 Twist 指令消息
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear_x;
    cmd_vel.linear.y = linear_y;
    cmd_vel.angular.z = angular_z;

    // 发布 Twist 指令消息
    pub.publish(cmd_vel);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_to_goal");

    ros::NodeHandle nh;

    // 訂閱odom以獲取機器狗的定位資訊
    ros::Subscriber pose_sub = nh.subscribe("/odom", 1000, poseCallback);

    // 建立发布机器人控制指令的 Publisher
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    // 假設初始位置是(0, 0)，目標位置是(1, 1)，請更改為實際需求
    double target_x = 2.0;
    double target_y = 3.0;

    // 设定移动速度和角速度
    double linear_speed = 0.1;  // 线速度 (m/s)
    double angular_speed = 0.1; // 角速度 (rad/s)

    // 发布控制指令移动机器人到目标位置
    ros::Rate rate(10); // 控制循环频率
    while (ros::ok()) {
        // 计算机器人到目标位置的方向
        double dx = target_x - current_pose.pose.position.x;
        double dy = target_y - current_pose.pose.position.y;
        double target_yaw = atan2(dy, dx);

        // 计算机器人到目标位置的线速度和角速度
        double linear_x = linear_speed * cos(target_yaw);
        double linear_y = linear_speed * sin(target_yaw);
        double angular_z = angular_speed * atan2(sin(target_yaw - current_pose.pose.orientation.z), cos(target_yaw - current_pose.pose.orientation.z));

        // 发布控制指令移动机器人
        moveRobot(linear_x, linear_y, angular_z, pub);

        // 当机器人到达目标位置时退出循环
        if (fabs(dx) < 0.1 && fabs(dy) < 0.1) {
            ROS_INFO("Robot reached the goal!");
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
