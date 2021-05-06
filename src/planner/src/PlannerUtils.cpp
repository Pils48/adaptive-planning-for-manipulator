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

geometry_msgs::Pose transformToMsg(const tf::Transform &transform)
{
    geometry_msgs::Pose result_pose;
    result_pose.position.x = transform.getOrigin().x();
    result_pose.position.y = transform.getOrigin().y();
    result_pose.position.z = transform.getOrigin().z();
    result_pose.orientation.w = transform.getRotation().w();
    result_pose.orientation.x = transform.getRotation().x();
    result_pose.orientation.y = transform.getRotation().y();
    result_pose.orientation.z = transform.getRotation().z();
    return result_pose;
}

void waitForSubscribers(const ros::Publisher &publisher, size_t subscribers_number)
{
    while(publisher.getNumSubscribers() < subscribers_number)
    {
        ROS_WARN("Not enough subscribers for topic %s", publisher.getTopic().c_str());
        sleep(3);
    }
}

std::vector<tf::Vector3> pointsFromRawData(const double *data, const unsigned int data_count)
{
    std::vector<tf::Vector3> result_vector;
    for (size_t i = 0; i < data_count; ++i)
    {
        tf::Vector3 vector3;
        vector3.setX(*(data + (3 * i)));
        vector3.setY(*(data + (3 * i) + 1));
        vector3.setZ(*(data + (3 * i) + 2));
        result_vector.push_back(vector3);
    }
    return result_vector;
}

