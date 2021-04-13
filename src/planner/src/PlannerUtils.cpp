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

std::vector<tf::Vector3> generateTestPointsArray(
    double x_0, double x_f, double x_step, 
    double y_0, double y_f, double y_step,
    double z_0, double z_f, double z_step
)
{
    std::vector<tf::Vector3> trivial_collisions;
    for (size_t i = 0; i < (x_f - x_0) / x_step; ++i)
    {
        for (size_t j = 0; j < (y_f - y_0) / y_step; ++j)
        {
            for (size_t k = 0; k < (z_f - z_0) / z_step; ++k)
            {
                trivial_collisions.emplace_back(tf::Vector3(x_0 + i * x_step, y_0 + j * y_step, z_0 + k * z_step));
            } 
        } 
    }
    return trivial_collisions;
}
