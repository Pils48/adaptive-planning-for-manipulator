#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include "ConfigurationSpace.h"
#include "ros/ros.h"


int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle node;
    ROS_INFO("planner_node started");
    
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model %s loaded", kinematic_model->getName().c_str());
    ConfigurationSpace c_space(Point{1, 2}, *kinematic_model);
    
    // robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    // kinematic_state->setToDefaultValues();
    // const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    // c_space.showPlot();
    while(ros::ok())
    {
        /*NOP*/
    }
    return 0;
}