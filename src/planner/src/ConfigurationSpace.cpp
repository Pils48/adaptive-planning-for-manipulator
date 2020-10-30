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
    auto joints = solver->solveIK(tf::Vector3(0.4, 0.2, 1), *joint_model_group);
    
    //Debug
    // for (const auto joint_name : joint_model_group->getJointModelNames())
    // {
    //     ROS_INFO("Joint: %s", joint_name.c_str());
    // }
    // ROS_INFO("Variable count: %d", joint_model_group->getVariableCount());


    // std::vector<double> seed, fk_values;
    // std::vector<std::vector<double>> solutions;
    // double timeout = 5.0;
    // kinematics::KinematicsQueryOptions options;
    // kinematics::KinematicsResult result;
    
    // std::vector<std::string> fk_names;
    // fk_names.push_back("gripper");

    // seed.resize(solver->getJointNames().size(), 0.0);
    // fk_values.resize(solver->getJointNames().size(), 0.0);
    // kinematic_state->setToRandomPositions(joint_model_group);
    // kinematic_state->copyJointGroupPositions(joint_model_group, fk_values);
    // std::vector<geometry_msgs::Pose> poses;
    // poses.resize(1);

    // bool fk_success = solver->getPositionFK(fk_names, fk_values, poses);
    // ROS_INFO("Success: %d", fk_success);
    // ROS_INFO("FK solution: %f, %f, %f", poses.front().position.x, poses.front().position.y, poses.front().position.z);

    // solutions.clear();
    // solver->getPositionIK(poses, fk_values, solutions, result, options);
    // if (result.kinematic_error == kinematics::KinematicErrors::OK)
    // {   
    //   ROS_INFO("Number of solutions: %d", solutions.size());
    //   for (const auto solution : solutions)
    //   {
    //       for(const auto value : solution)
    //       {
    //           ROS_INFO("Value: %f", value);
    //       }
    //       ROS_INFO("--------------");
    //   }
    // }
    // else
    // {
    //   ROS_ERROR_STREAM("getPositionIK with multiple solutions failed");
    // }


    //auto solutions = solveIK(_trivial_collision, joint_model_group);
}