#include <stdio.h>

#include <ros/ros.h>
#include "robot_interface/robot_interface.h"

using namespace std;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "sawyer_controller");
    ros::NodeHandle _n("sawyer_controller");

    bool use_robot;
    _n.param<bool>("use_robot", use_robot, true);
    printf("\n");
    ROS_INFO("use_robot flag set to %s", use_robot==true?"true":"false");

    printf("\n");
    RobotInterface right_ctrl("sawyer_controller","right", use_robot);
    printf("\n");
    ROS_INFO("READY! Waiting for control messages..\n");

    ros::spin();
    return 0;
}

