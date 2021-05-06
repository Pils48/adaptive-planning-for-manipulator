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

std::vector<tf::Vector3> generateTestPointsArray(
    double x_0, double x_f, double x_step, 
    double y_0, double y_f, double y_step,
    double z_0, double z_f, double z_step
);

geometry_msgs::Pose transformToMsg(const tf::Transform &transform);

void waitForSubscribers(const ros::Publisher &publisher, size_t subscribers_number);

std::vector<tf::Vector3> pointsFromRawData(const double *data, const unsigned int data_count);
