#include "PlannerUtils.h"


const moveit::core::RevoluteJointModel* castToRevouluteModel(const robot_model::JointModel* joint_model)
{
    try
    {
        auto revoulute_joint = dynamic_cast<const moveit::core::RevoluteJointModel*>(joint_model);
        return revoulute_joint;
    }
    catch(const std::bad_cast& e)
    {
        ROS_ERROR("Non revoulute joint_models");        
    }       
}

double getLinkLength(const robot_model::LinkModel *link_model)
{
    auto extents = link_model->getShapeExtentsAtOrigin();
    ROS_INFO("Shape extends, %s: %f %f %f", link_model->getName().c_str(), extents.x(), extents.y(), extents.z());
    return extents.z() / 1000;
}

std::vector<tf::Vector3> generateTestPointsArray()
{
    std::vector<tf::Vector3> trivial_collisions;
    for (size_t i = 0; i < 0.04 / 0.005; ++i)
    {
        for (size_t j = 0; j < 0.04 / 0.005; ++j)
        {
            trivial_collisions.emplace_back(tf::Vector3(0.12 + i * 0.005, 0.12 + j * 0.005, 1));
        } 
    }
    return trivial_collisions;
}
