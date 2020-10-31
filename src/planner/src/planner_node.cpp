#include <tf/tf.h>

//moveit
#include <moveit/robot_model_loader/robot_model_loader.h>

//project
#include "Solver.h"
#include "ConfigurationSpace.h"
#include "ros/ros.h"


//TO DO: crash with segfault while setting to default robot_state, probably configs are malformed

int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROS_INFO("planner_node started");
    
    //Init configuration space
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model %s loaded", kinematic_model->getName().c_str());
    ROS_INFO("Building configuration space...");
    ConfigurationSpace c_space(tf::Vector3(1, 2, 3), kinematic_model);
    while(ros::ok())
    {
        /*NOP*/
    }
    return 0;
}