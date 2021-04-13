#include "ros/ros.h"

//moveit
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include "ConfigurationSpace.h"
#include "Solver.h"

#include <sstream>
#include <fstream>
#include <iostream>
#include <cstring>

using namespace std;
using namespace moveit::core;

void concatenatePoints(vector<double> &x_back, vector<double> &y_back,
                        vector<double> &x_front, vector<double> &y_front)
{
    reverse(x_back.begin(), x_back.end());
    reverse(y_back.begin(), y_back.end());
    x_front.insert(x_front.end(), x_back.begin(), x_back.end());
    y_front.insert(y_front.end(), y_back.begin(), y_back.end());
    plt::plot(x_front, y_front, 
        map<string, string>{make_pair("color", "black"), make_pair("linewidth", "3")});
    x_front.clear();
    y_front.clear();
    x_back.clear();
    y_back.clear();
}

ConfigurationSpace::ConfigurationSpace
(
    vector<tf::Vector3> trivial_collisions,
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

void ConfigurationSpace::addCollision(const vector<tf::Vector3> &trivial_collisions)
{
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(_robot_model));
    auto joint_model_group = kinematic_state->getJointModelGroup("3_dof_manipulator");
    ROS_INFO("%s joint model group loaded", joint_model_group->getName().c_str());

    auto solver = createSolver(*joint_model_group);
    auto chain = solver->getSimplifiedLinksChain(*joint_model_group);
    vector<double> links_length;
    transform(chain.begin(), chain.end(), back_inserter(links_length), &getLinkLength);
    auto total_length = accumulate(links_length.begin(), links_length.end(), 0.0);
    ROS_INFO("Total length: %f", total_length);
    for (const auto &link_length : links_length)
    {
        ROS_INFO("link_length: %f", link_length);
    }

    //Setting up graphs params
    if (joint_model_group->getActiveJointModels().size() == 2)
    {
        plt::title("Image of the obstacle");
        plt::xlim(PLOT_X_LOWER_LIMIT, PLOT_X_UPPER_LIMIT);
        plt::ylim(PLOT_Y_LOWER_LIMIT, PLOT_Y_UPPER_LIMIT);
        plt::grid(true);

        //Setting up solver params
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
                    tf::Vector3 obstacle(trivial_collisions[idx].getX(), trivial_collisions[idx].getY(), 0);
                    if (links_length.back() > obstacle.length())
                    {
                        tf::Vector3 y_ort(0, 1, 0);
                        const double abs_angle = obstacle.angle(y_ort);
                        const double angle = obstacle.cross(y_ort) > 0 ? abs_angle : -abs_angle;
                        plt::plot({angle, angle}, {PLOT_Y_LOWER_LIMIT, PLOT_Y_UPPER_LIMIT},
                            map<string, string>{make_pair("color", "black"), make_pair("linewidth", "3")});
                    }
                }
            }
            concatenatePoints(x_back, y_back, x_front, y_front);
            plt::pause(0.1);
        }
    }
    else if (joint_model_group->getActiveJointModels().size() == 3)
    {
        struct double3 { double x, y, z; }; 
        vector<double3> vertices;
        for (size_t idx = 0; idx < trivial_collisions.size(); ++idx)
        {
            for (size_t i = 0; i < total_length / STANDARD_DISCRETIZATION; ++i)
            {
                if (i < links_length.back() / STANDARD_DISCRETIZATION)
                {
                    auto joints = solver->solveIK(trivial_collisions[idx], 
                        {links_length.front(), *next(links_length.begin()), links_length.back() - i * STANDARD_DISCRETIZATION});
                    if (joints.size() == 2)
                    {
                        // vertices.push_back(vertices.end(), joints.begin(), joints.end());
                        vertices.push_back(double3{ joints.back()[0], joints.back()[1], joints.back()[2] }); 
                        vertices.push_back(double3{ joints.front()[0], joints.front()[1], joints.front()[2] }); 
                    }
                }
                else
                {
                    //TO DO: near the first link
                }
            }
        }
        // Work around with ply file
        ROS_INFO("Vertices: %lu", vertices.size());
        string filename = "test.ply"; //home/user/.ros
        filebuf fb_binary;
        fb_binary.open(filename, ios::out | ios::binary);
        ostream outstream_binary(&fb_binary);
        if (outstream_binary.fail()) throw runtime_error("failed to open " + filename);
        
        tinyply::PlyFile image_ply;

        image_ply.add_properties_to_element("vertex", { "x", "y", "z" }, 
            tinyply::Type::FLOAT64, vertices.size(), reinterpret_cast<uint8_t*>(vertices.data()), tinyply::Type::INVALID, 0);

        image_ply.get_comments().push_back("generated by tinyply 2.3");

        image_ply.write(outstream_binary, true);
    } 
}
