#include "matplotlibcpp.h"
#include "ros/ros.h"


int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle node;
    ROS_INFO("planner_node started");
    while(ros::ok())
    {
        /*NOP*/
    }
    return 0;
}