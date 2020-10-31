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
    tf::Vector3 trivial_collision,
    moveit::core::RobotModelPtr robot_model
)
    : _trivial_collision(trivial_collision)
    , _robot_model(robot_model) 
{
    addCollision(trivial_collision);
}

void ConfigurationSpace::showPlot()
{

} 

void ConfigurationSpace::addCollision(const tf::Vector3 &trivial_collision)
{
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(_robot_model));
    auto joint_model_group = kinematic_state->getJointModelGroup("manipulator");

    auto solver = createSolver(*joint_model_group);
    auto chain = solver->getSimplifiedLinksChain(*joint_model_group);
    vector<double> links_length;
    transform(chain.begin(), chain.end(), back_inserter(links_length), &getLinkLength);
    auto total_length = accumulate(links_length.begin(), links_length.end(), 0);
    for (size_t i = 0; i < total_length / STANDARD_DISCRETIZATION; ++i)
    {
        if (i > links_length.front() / STANDARD_DISCRETIZATION)
        {
             auto joints = solver->solveIK(tf::Vector3(0.15, 0.21, 1), 
                        {links_length.front(), links_length.back() - i * STANDARD_DISCRETIZATION});
        }
        else
        {
            auto joints = solver->solveIK(tf::Vector3(0.15, 0.21, 1), 
                        {links_length.front() - i * STANDARD_DISCRETIZATION + links_length.back(), 0});
        }
        
    }
    
}