#include <moveit/robot_model/robot_model.h>

#include <tf/tf.h>
#include <vector>

template <typename T>
inline T sqr(T const& value)
{
    return value * value;
}

const moveit::core::RevoluteJointModel* castToRevouluteModel(const robot_model::JointModel* joint_model);

double getLinkLength(const robot_model::LinkModel *link_model);

std::vector<tf::Vector3> generateTestPointsArray();
