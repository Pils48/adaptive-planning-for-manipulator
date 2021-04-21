#include "ros/ros.h"

//moveit
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include "ConfigurationSpace.h"
#include "Solver.h"


using namespace std;
using namespace moveit::core;


ConfigurationSpace::ConfigurationSpace
(
    vector<tf::Vector3> trivial_collisions,
    moveit::core::RobotModelPtr robot_model
)
    : _trivial_collisions(trivial_collisions)
    , _robot_model(robot_model) 
{
    addCollision(trivial_collisions);
}

void ConfigurationSpace::addCollision(const vector<tf::Vector3> &trivial_collisions)
{
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(_robot_model));
    auto joint_model_group = kinematic_state->getJointModelGroup("manipulator");
    ROS_INFO("%s joint model group loaded", joint_model_group->getName().c_str());

    auto solver = createSolver(*joint_model_group);
    auto chain = solver->getSimplifiedLinksChain(*joint_model_group);
    vector<double> links_length;
    transform(chain.begin(), chain.end(), back_inserter(links_length), &getLinkLength);
    solver->solveExpandIK(trivial_collisions, links_length, true);
}
