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
    std::vector<tf::Vector3> trivial_collisions,
    moveit::core::RobotModelPtr robot_model
)
    : _trivial_collisions(trivial_collisions)
    , _robot_model(robot_model)
    , _cspace_plot("test") 
{
    addCollision(trivial_collisions);
}

void ConfigurationSpace::showPlot()
{

}

void ConfigurationSpace::addCollision(const std::vector<tf::Vector3> &trivial_collisions)
{
    //Setting up graphs params
    plt::title("Image of the obstacle");
    plt::xlim(-M_PI_2, M_PI);
    plt::ylim(-M_PI, M_PI);
    plt::grid(true);

    //Setting up solver params
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(_robot_model));
    auto joint_model_group = kinematic_state->getJointModelGroup("manipulator");

    auto solver = createSolver(*joint_model_group);
    auto chain = solver->getSimplifiedLinksChain(*joint_model_group);
    vector<double> links_length;
    transform(chain.begin(), chain.end(), back_inserter(links_length), &getLinkLength);
    // double total_length = accumulate(links_length.begin(), links_length.end(), 0);
    double total_length = links_length.front() + links_length.back();
    for (const auto &link_length : links_length)
    {
        ROS_INFO("link_length: %f", link_length);
    }
    vector<double> x_front, y_front, x_back, y_back; 
    for (size_t idx = 0; idx < trivial_collisions.size(); ++idx)
    {
        for (size_t i = 0; i < total_length / STANDARD_DISCRETIZATION; ++i)
        {
            if (i < links_length.back() / STANDARD_DISCRETIZATION)
            {
                auto joints = solver->solveIK(trivial_collisions[idx], 
                        {links_length.front(), links_length.back() - i * STANDARD_DISCRETIZATION});
                if (joints.size() == 2)
                {
                    x_front.emplace_back(joints.front()[0]);
                    y_front.emplace_back(joints.front()[1]);
                    x_back.emplace_back(joints.back()[0]);
                    y_back.emplace_back(joints.back()[1]);
                }
            }
            else
            {
                // auto joints = solver->solveIK(tf::Vector3(0.15, 0.15, 1), 
                //             {links_length.front() - i * STANDARD_DISCRETIZATION + links_length.back(), 0});
                // for (const auto &solution : joints)
                // {
                //     x.push_back(solution[0]);
                //     y.push_back(solution[1]);
                // }
            }
        }
        reverse(x_back.begin(), x_back.end());
        reverse(y_back.begin(), y_back.end());
        x_front.insert(x_front.end(), x_back.begin(), x_back.end());
        y_front.insert(y_front.end(), y_back.begin(), y_back.end());
        plt::plot(x_front, y_front, std::map<std::string, 
                std::string>{std::make_pair("color", "black"), std::make_pair("linewidth", "3")});
        x_front.clear();
        y_front.clear();
        x_back.clear();
        y_back.clear();
        plt::pause(0.1);
    }
}
