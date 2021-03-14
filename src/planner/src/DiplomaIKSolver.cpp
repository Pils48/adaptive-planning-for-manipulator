#include "DiplomaIKSolver.h"
#include "PlannerUtils.h"


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
    ROS_INFO("Joint 1: %f", (joint_1 - M_PI / 2) * 180 / M_PI);
    
    const double length_1 = *next(links_length.begin());
    const double length_2 = links_length.back();
    Solutions solutions;
    double joint_3 = acos((sqr(pose.getX()) + sqr(pose.getY()) + sqr(pose.getZ()) - sqr(length_1) - sqr(length_2)) / (2 * length_1 * length_2));
    double joint_2 = atan(pose.getZ() / (sqr(pose.getX()) + sqr(pose.getY()))) - atan(length_2 * sin(joint_3) / (length_1 + length_2 * cos(joint_3)));
    ROS_INFO("First solution:");
    ROS_INFO("Joint 2: %f", (joint_2 - M_PI / 2) * 180 / M_PI);
    ROS_INFO("Joint 3: %f", joint_3 * 180 / M_PI);
    if (!isnan(joint_2) && !isnan(joint_3))
    {
        solutions.push_back(vector<double>{joint_1, joint_2, joint_3});
    }

    joint_3 = -acos((sqr(pose.getX()) + sqr(pose.getY()) + sqr(pose.getZ()) - sqr(length_1) - sqr(length_2)) / (2 * length_1 * length_2));
    joint_2 = atan(pose.getZ() / (sqr(pose.getX()) + sqr(pose.getY()))) - atan(length_2 * sin(joint_3) / (length_1 + length_2 * cos(joint_3)));
    ROS_INFO("Second solution:");
    ROS_INFO("Joint 2: %f", (joint_2 - M_PI / 2) * 180 / M_PI);
    ROS_INFO("Joint 3: %f", joint_3 * 180 / M_PI);
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
    // const auto joint_models = joint_model_group.getActiveJointModels();
    // const auto joints_number = joint_models.size();
    // if (joints_number > 3)
    // {
    //     ROS_ERROR("Diploma solver doesn't support more than three joints!");
    //     return false;
    // }

    // vector<const RevoluteJointModel*> revoulute_joint_models;
    // transform(joint_models.begin(), joint_models.end(), back_inserter(revoulute_joint_models), &castToRevouluteModel);
    // if ((next(revoulute_joint_models.begin())->getAxis() == revoulute_joint_models.back()->getAxis())
    //     && (revoulute_joint_models.front()->getAxis()))
    // {
    //     ROS_INFO("Trivial chain is valid");
    //     return true;
    // }
    // else
    // {
    //     ROS_ERROR("Invalid chain!");
    //     return false;
    // }
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