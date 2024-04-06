#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>

// 建立 MoveBase 行動伺服器型態別名
using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

// 用於接收/uwb_odom的定位資訊
geometry_msgs::PoseStamped current_pose;

// 定位資訊的回呼函式
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_pose = msg->pose;
}

void navigationCmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // Example translation of a simple navigation command to Go1 control commands
    high_cmd_ros.velocity[0] = msg->linear.x;
    high_cmd_ros.velocity[1] = msg->linear.y;
    high_cmd_ros.yawSpeed = msg->angular.z;
    
    // Set other parameters based on your navigation strategy
    // e.g., high_cmd_ros.gaitType = determineGaitType(msg);
    
    pub.publish(high_cmd_ros); // Publish the control command
}
MoveBaseClient ac("move_base", true);

// move the go1 to the target
void moveToGoal(double x, double y, double orientation_w) {

    // wait for move_base server start
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("wait for move_base server start...");
    }

    // create MoveBase goal
    move_base_msgs::MoveBaseGoal goal;

    // set goal coordinate
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = orientation_w;

    // publish MoveBase goal
    ac.sendGoal(goal);

    // wait for result
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("go1 successfully goes to the target");
    } else {
        ROS_ERROR("go1 cannot go to target");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_base_example");

    ros::NodeHandle nh;

    // 訂閱/uwb_odom以獲取機器狗的定位資訊
    ros::Subscriber pose_sub = nh.subscribe("/uwb_odom", 1000, poseCallback);

    // 假設初始位置是(0, 0)，目標位置是(1, 1)，請更改為實際需求
    double target_x = 2.0;
    double target_y = 3.0;

    // 移動機器狗到目標位置
    moveToGoal(target_x, target_y, 1.0);

    
    ros::NodeHandle nh2;
    
    // Subscriber to navigation commands (e.g., from move_base)
    ros::Subscriber sub = nh2.subscribe("/cmd_vel", 1000, navigationCmdCallback);
    
    ros::Publisher pub; // 宣告 pub 變數

    // Publisher for Go1 control commands
    pub = nh2.advertise<unitree_legged_msgs::HighCmd>("control_topic", 1000);
    
    ros::spin();

    return 0;
}