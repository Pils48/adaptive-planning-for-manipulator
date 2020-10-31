#include "TrivialIK.h"
#include "PlannerUtils.h"

#include <moveit/robot_model/link_model.h>

using namespace robot_model; 
using namespace std;

const RevoluteJointModel* castToRevouluteModel(const JointModel* joint_model)
{
    try
    {
        auto revoulute_joint = dynamic_cast<const RevoluteJointModel*>(joint_model);
        return revoulute_joint;
    }
    catch(const std::bad_cast& e)
    {
        ROS_ERROR("Non revoulute joint_models");        
    }       
}

double getLinkLength(const LinkModel *link_model)
{
    auto extents = link_model->getShapeExtentsAtOrigin();
    ROS_INFO("Shape extends, %s: %f %f %f", link_model->getName().c_str(), extents.x(), extents.y(), extents.z());
    return extents.z() / 1000;
}

Solutions TrivialIK::solveIK(
    const tf::Vector3 &trivial_collision, 
    const JointModelGroup &joint_model_group
    )
{
    const auto link_models = joint_model_group.getLinkModels();
    const auto joint_models = joint_model_group.getActiveJointModels();
    const auto joints_number = joint_models.size();
    if (joints_number > 2)
    {
        ROS_ERROR("Solver doesn't support more than two joints yet!");
    }

    vector<const RevoluteJointModel*> revoulute_joint_models;
    std::transform(joint_models.begin(), joint_models.end(), std::back_inserter(revoulute_joint_models), &castToRevouluteModel);
    if (revoulute_joint_models.front()->getAxis() == revoulute_joint_models.back()->getAxis())
    {
        ROS_INFO("Trivial chain is valid");
    }

    vector<double> links_length;
    transform(link_models.begin(), link_models.end(), back_inserter(links_length), &getLinkLength);
    
    //Hardcode
    links_length.pop_back();

    Solutions solutions;
    ROS_DEBUG("(pow(trivial_collision.getX(), 2): %f", (pow(trivial_collision.getX(), 2)));
    ROS_DEBUG("(pow(trivial_collision.getY(), 2): %f", (pow(trivial_collision.getY(), 2)));
    ROS_DEBUG("pow(links_length.front(), 2): %f", pow(links_length.front(), 2));
    ROS_DEBUG("pow(links_length.back(), 2): %f", pow(links_length.back(), 2));
    ROS_DEBUG("under acos: %f", ((pow(trivial_collision.getX(), 2) + pow(trivial_collision.getY(), 2) 
                        - pow(links_length.front(), 2) - pow(links_length.back(), 2)) / 
                        (2 * links_length.front() * links_length.back())));
                        
    auto joint_2 = acos((pow(trivial_collision.getX(), 2) + pow(trivial_collision.getY(), 2) 
                        - pow(links_length.front(), 2) - pow(links_length.back(), 2)) / 
                        (2 * links_length.front() * links_length.back()));
    auto joint_1 = atan(trivial_collision.getY() / trivial_collision.getX()) - 
                    atan(links_length.back() * sin(joint_2) / (links_length.front() + links_length.back() * cos(joint_2)));
    ROS_INFO("Joint 1: %f", (joint_1 - M_PI / 2) * 180 / M_PI);
    ROS_INFO("Joint 2: %f", joint_2 * 180 / M_PI);
    solutions.push_back(vector<double>{joint_1, joint_2});

    joint_2 = -acos((pow(trivial_collision.getX(), 2) + pow(trivial_collision.getY(), 2) 
                        - pow(links_length.front(), 2) - pow(links_length.back(), 2)) / 
                        (2 * links_length.front() * links_length.back()));
    joint_1 = atan(trivial_collision.getY() / trivial_collision.getX()) - 
                    atan(links_length.back() * sin(joint_2) / (links_length.front() + links_length.back() * cos(joint_2)));
    ROS_INFO("Joint 1: %f", (joint_1 - M_PI / 2) * 180 / M_PI);
    ROS_INFO("Joint 2: %f", joint_2 * 180 / M_PI);
    solutions.push_back(vector<double>{joint_1, joint_2});
    return solutions;

}




