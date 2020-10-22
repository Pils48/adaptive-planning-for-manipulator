#include "ConfigurationSpace.h"
#include "ros/ros.h"


int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle node;
    ROS_INFO("planner_node started");
    // ConfigurationSpace c_space(Point{1, 2});
    // c_space.showPlot();
    while(ros::ok())
    {
        /*NOP*/
    }
    return 0;
}