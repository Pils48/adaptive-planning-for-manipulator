#include "ros/ros.h"

//moveit
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include "ConfigurationSpace.h"
#include "Solver.h"

ConfigurationSpace::ConfigurationSpace
(
    Point trivial_collision,
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

void ConfigurationSpace::addCollision(const Point &trivial_collision)
{
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(_robot_model));
    auto joint_model_group = kinematic_state->getJointModelGroup("manipulator");
    // const auto &solver = joint_model_group->getSolverInstance();

    auto solver = createSolver(tf::Vector3(1, 1, 1), *joint_model_group);
    auto joints = solver->solveIK(tf::Vector3(0.15, 0.21, 1), *joint_model_group);
    
}