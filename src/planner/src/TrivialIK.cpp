#include "TrivialIK.h"
#include "PlannerUtils.h"

#include <moveit/robot_model/link_model.h>

using namespace robot_model; 
using namespace std;


bool TrivialIK::isJointModelGroupValid(const moveit::core::JointModelGroup &joint_model_group)
{
    const auto joint_models = joint_model_group.getActiveJointModels();
    const auto joints_number = joint_models.size();
    if (joints_number > 2)
    {
        ROS_ERROR("Trivial solver doesn't support more than two joints!");
        return false;
    }

    vector<const RevoluteJointModel*> revoulute_joint_models;
    transform(joint_models.begin(), joint_models.end(), back_inserter(revoulute_joint_models), &castToRevouluteModel);
    if (revoulute_joint_models.front()->getAxis() == revoulute_joint_models.back()->getAxis())
    {
        ROS_INFO("Trivial chain is valid");
        return true;
    }
    else
    {
        ROS_ERROR("Invalid chain!");
        return false;
    }
}

std::vector<moveit::core::LinkModel*> TrivialIK::getSimplifiedLinksChain(
    const moveit::core::JointModelGroup &joint_model_group
)
{
    const auto joints_models = joint_model_group.getActiveJointModels();
    vector<const LinkModel*> chain;
    for (const auto* joint_model : joints_models)
    {
        ROS_INFO("LINK MODEL: %s", joint_model->getChildLinkModel()->getName().c_str());
        chain.emplace_back(joint_model->getChildLinkModel());
    }
}

Solutions TrivialIK::solveIK(
    const tf::Vector3 &pose, 
    const vector<double> &links_length
    )
{
    if (links_length.size() != 2)
    {
        throw runtime_error("Links number for trivial solver doesn't equal 2!");
    }

    const double length_1 = links_length.back();
    const double length_2 = links_length.front();
    Solutions solutions;
    //First solution
    auto joint_2 = acos((sqr(pose.getX()) + sqr(pose.getY()) - sqr(length_1) - sqr(length_2)) / (2 * length_1 * length_2));
    auto joint_1 = atan(pose.getY() / pose.getX()) - atan(links_length.back() * sin(joint_2) / 
                    (links_length.front() + links_length.back() * cos(joint_2)));
    ROS_INFO("First solution:\nJoint 1: %f", (joint_1 - M_PI / 2) * 180 / M_PI);
    ROS_INFO("Joint 2: %f", joint_2 * 180 / M_PI);
    solutions.push_back(vector<double>{joint_1, joint_2});

    //Second solution
    joint_2 = -acos((sqr(pose.getX()) + sqr(pose.getY()) - sqr(length_1) - sqr(length_2)) / (2 * links_length.front() * links_length.back()));
    joint_1 = atan(pose.getY() / pose.getX()) - atan(links_length.back() * sin(joint_2) / 
                (links_length.front() + links_length.back() * cos(joint_2)));
    ROS_INFO("Second solution:\nJoint 1: %f", (joint_1 - M_PI / 2) * 180 / M_PI);
    ROS_INFO("Joint 2: %f", joint_2 * 180 / M_PI);
    solutions.push_back(vector<double>{joint_1, joint_2});
    return solutions;
}

