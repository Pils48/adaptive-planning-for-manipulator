#include <tf/tf.h>

//moveit
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

//ros
#include "ros/ros.h"

//Internal
#include "PlannerUtils.h"
#include "Matplotlibcpp.h"


class ConfigurationSpace
{
public:
    ConfigurationSpace(
        std::string robot_description,
        std::vector<tf::Vector3> _trivial_collisions
    );

    ~ConfigurationSpace() = default;

    void spin();

    void addCollision(const std::vector<tf::Vector3> &trivial_collisions);
    
private:
    ros::NodeHandle _nh;
    ros::Publisher _space_ready_pub;
    robot_model_loader::RobotModelLoader _robot_model_loader;
    moveit::core::RobotModelPtr _robot_model;
    ros::Rate _rate = ros::Rate(10);
    std::vector<tf::Vector3> _trivial_collisions;
    //vector<shapes> _collision_objects
};