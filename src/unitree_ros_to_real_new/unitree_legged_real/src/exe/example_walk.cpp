#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h" // this red~ is ok
#include <std_msgs/String.h>

using namespace UNITREE_LEGGED_SDK;

// global variable, stores commands
unitree_legged_msgs::HighCmd high_cmd_ros;

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

// callback function for commands received from the topic
void commandCallback(const std_msgs::String::ConstPtr& msg)
{
    std::cout << "subscribed" << std::endl;
    
    // according message to decide go1 action
    if (msg->data == "forward") {
        // move forward mode
        std::cout << "go1 moving forward" << std::endl;
        high_cmd_ros.mode = 2;
        high_cmd_ros.gaitType = 2;
        high_cmd_ros.velocity[0] = 0.2f; // -1 ~ 1
        // high_cmd_ros.yawSpeed = 2; // if have this, it will rotate, if not, it will walk
        high_cmd_ros.footRaiseHeight = 0.1;
        std::cout << "mode changed" << std::endl;
        std::cout << high_cmd_ros << std::endl;
       
    } else if (msg->data == "stop") {
        // stop mode
        std::cout << "go1 stop" << std::endl;
        high_cmd_ros.mode = 0; // idle, default stand
        high_cmd_ros.velocity[0] = 0.0; // stop

    } else if (msg->data == "walk to") {
        // walk to a position, this mode is not completed
        std::cout << "go1 move to" << std::endl;
        high_cmd_ros.mode = 3;
        high_cmd_ros.speedLevel = 0; // 0~2 for low to high

        // high_cmd_ros.position = [0, 0, 0];
    } else if (msg->data == "correct pose") {
        // equivalent to L + A, Sometimes the dog doesn't walk as told possibly due to a failure 
        // to change its posture. In such a case, it is useful to correct its posture with this mode.
        std::cout << "go1 correct pose" << std::endl;
        high_cmd_ros.mode = 6;

    } else if (msg->data == "emergency") {
        // Damping at the current position, which is equivalent to L + B. This mode is suitable for 
        // software emergency. The dog refuses any commands from UDP as well as the joystick controller 
        // unless its mode is turned to mode = 6.
        std::cout << "go1 emergency" << std::endl;
        high_cmd_ros.mode = 7;

    } else if (msg->data == "recovery stand") {
        // Recovery stand at the current position
        std::cout << "go1 recovery stand" << std::endl;
        high_cmd_ros.mode = 8;

    } else if (msg->data == "back flip") {
        // back flip?!
        std::cout << "go1 back flip" << std::endl;
        high_cmd_ros.mode = 9;

    } else if (msg->data == "rotate jump") {
        // Yaw 90 degree rotation with a jump
        std::cout << "go1 rotate and jump" << std::endl;
        high_cmd_ros.mode = 10;

    } else if (msg->data == "pray") {
        // Put upward its hands like praying
        std::cout << "go1 stand up and pray" << std::endl;
        high_cmd_ros.mode = 11;

    } else if (msg->data == "dance 1") {
        // dance mode 1
        std::cout << "go1 dance mode 1" << std::endl;
        high_cmd_ros.mode = 12;

    } else if (msg->data == "dance 2") {
        // dance mode 2
        std::cout << "go1 dance mode 2" << std::endl;
        high_cmd_ros.mode = 13;
    }
    
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

    // unitree_legged_msgs::HighCmd high_cmd_ros;

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