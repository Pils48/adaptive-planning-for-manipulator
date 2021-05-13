#include "DiplomaIKSolver.h"
#include "PlannerUtils.h"
#include "tinyply.h"

#include <sstream>
#include <fstream>
#include <iostream>
#include <cstring>


using namespace robot_model; 
using namespace std;

Solutions DiplomaIKSolver::solveIK(
    const tf::Vector3 &pose, 
    const vector<double> &links_length
)
{
    if (links_length.size() != 3)
    {
        throw runtime_error("Links number for diploma solver doesn't equal 3!");
    }
    //Calculate joint_1
    const tf::Vector3 y_ort(0, 1, 0);
    const tf::Vector3 pose_projection(pose.getX(), pose.getY(), 0);
    const double abs_joint_1 = y_ort.angle(pose_projection);
    const double joint_1 = (pose_projection.cross(y_ort) > 0) ? abs_joint_1 : -abs_joint_1;
    ROS_DEBUG("Joint 1: %f", (joint_1 - M_PI / 2) * 180 / M_PI);
    
    const double length_1 = *next(links_length.begin());
    const double length_2 = links_length.back();
    Solutions solutions;
    double joint_3 = acos((sqr(pose.getX()) + sqr(pose.getY()) + sqr(pose.getZ()) - sqr(length_1) - sqr(length_2)) / (2 * length_1 * length_2));
    double joint_2 = atan(pose.getZ() / (sqr(pose.getX()) + sqr(pose.getY()))) - atan(length_2 * sin(joint_3) / (length_1 + length_2 * cos(joint_3)));
    ROS_DEBUG("First solution:");
    ROS_DEBUG("Joint 2: %f", (joint_2 - M_PI / 2) * 180 / M_PI);
    ROS_DEBUG("Joint 3: %f", joint_3 * 180 / M_PI);
    if (!isnan(joint_2) && !isnan(joint_3))
    {
        solutions.push_back(vector<double>{joint_1, joint_2, joint_3});
    }

    joint_3 = -acos((sqr(pose.getX()) + sqr(pose.getY()) + sqr(pose.getZ()) - sqr(length_1) - sqr(length_2)) / (2 * length_1 * length_2));
    joint_2 = atan(pose.getZ() / (sqr(pose.getX()) + sqr(pose.getY()))) - atan(length_2 * sin(joint_3) / (length_1 + length_2 * cos(joint_3)));
    ROS_DEBUG("Second solution:");
    ROS_DEBUG("Joint 2: %f", (joint_2 - M_PI / 2) * 180 / M_PI);
    ROS_DEBUG("Joint 3: %f", joint_3 * 180 / M_PI);
    if (!isnan(joint_2) && !isnan(joint_3))
    {
        solutions.push_back(vector<double>{joint_1, joint_2, joint_3});
    }
    return solutions;
}

bool DiplomaIKSolver::isJointModelGroupValid(
    const moveit::core::JointModelGroup &joint_model_group
)
{
    const auto joint_models = joint_model_group.getActiveJointModels();
    const auto joints_number = joint_models.size();
    if (joints_number > 3)
    {
        ROS_ERROR("Diploma solver doesn't support more than three joints!");
        return false;
    }

    vector<const RevoluteJointModel*> revoulute_joint_models;
    transform(joint_models.begin(), joint_models.end(), back_inserter(revoulute_joint_models), &castToRevouluteModel);
    
    //Check if two last joints have the parallel axis and the first one is perpendicular
    if (((*next(revoulute_joint_models.begin()))->getAxis() == revoulute_joint_models.back()->getAxis())
        && (revoulute_joint_models.front()->getAxis().dot(revoulute_joint_models.back()->getAxis()) == 0))
    {
        ROS_INFO("Diploma chain is valid");
        return true;
    }
    else
    {
        ROS_ERROR("Invalid chain!");
        return false;
    }
    return true;
}

vector<const moveit::core::LinkModel*> DiplomaIKSolver::getSimplifiedLinksChain(
    const moveit::core::JointModelGroup &joint_model_group
)
{
    const auto joints_models = joint_model_group.getActiveJointModels();
    vector<const LinkModel*> chain;
    for (const auto *joint_model : joints_models)
    {
        ROS_INFO("LINK MODEL: %s", joint_model->getChildLinkModel()->getName().c_str());
        chain.emplace_back(joint_model->getChildLinkModel());
    }
    return chain;
}

void DiplomaIKSolver::solveExpandIK(
    const std::vector<tf::Vector3> &trivial_collisions, 
    const std::vector<double> &links_length,
    bool show
)
{
    auto total_length = accumulate(links_length.begin(), links_length.end(), 0.0);
    ROS_INFO("Total length: %f", total_length);
    for (const auto &link_length : links_length)
    {
        ROS_INFO("link_length: %f", link_length);
    }
    struct double3 { double x, y, z; }; 
    vector<double3> vertices;
    if (show) //creating 3D object for demo
    {
        for (size_t idx = 0; idx < trivial_collisions.size(); ++idx)
        {
            for (size_t i = 0; i < total_length / STANDARD_DISCRETIZATION; ++i)
            {
                if (i < links_length.back() / STANDARD_DISCRETIZATION)
                {
                    auto joints = solveIK(trivial_collisions[idx], 
                        {links_length.front(), *next(links_length.begin()), links_length.back() - i * STANDARD_DISCRETIZATION});
                    if (joints.size() == 2)
                    {
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
    ROS_INFO("Image generated");
} 