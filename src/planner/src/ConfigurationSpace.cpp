//MoveIt
#include <moveit/robot_state/robot_state.h>

//ros
#include "std_msgs/Bool.h"

//Internal
#include "ConfigurationSpace.h"
#include "Solver.h"


using namespace std;
using namespace moveit::core;


ConfigurationSpace::ConfigurationSpace
(
    string robot_description,
    vector<tf::Vector3> trivial_collisions
)
    : _robot_model_loader(robot_description)
    , _robot_model(_robot_model_loader.getModel()) 
{
    ros::Publisher _space_ready_pub = _nh.advertise<std_msgs::Bool>("space_ready_topic", 10);
    addCollision(trivial_collisions);
}

void ConfigurationSpace::spin()
{   
    ROS_INFO("configuration_space_node started");
    ROS_INFO("Model %s loaded", _robot_model->getName().c_str());
    ROS_INFO("Building configuration space...");
    while(ros::ok())
    {
        std_msgs::Bool is_ready;
        is_ready.data = true;
        // space_ready_pub.publish(is_ready);
        ros::spinOnce();
        _rate.sleep();
    }
}

void ConfigurationSpace::addCollision(const vector<tf::Vector3> &trivial_collisions)
{
    _trivial_collisions.insert(_trivial_collisions.end(), trivial_collisions.begin(), trivial_collisions.end());
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(_robot_model));
    auto joint_model_group = kinematic_state->getJointModelGroup("manipulator");
    ROS_INFO("%s joint model group loaded", joint_model_group->getName().c_str());

    auto solver = createSolver(*joint_model_group);
    auto chain = solver->getSimplifiedLinksChain(*joint_model_group);
    vector<double> links_length;
    transform(chain.begin(), chain.end(), back_inserter(links_length), &getLinkLength);
    solver->solveExpandIK(trivial_collisions, links_length, false);
}
