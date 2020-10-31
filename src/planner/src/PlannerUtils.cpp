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