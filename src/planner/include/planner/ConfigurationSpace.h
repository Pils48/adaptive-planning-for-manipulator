#include <tf/tf.h>

//moveit
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

//ros
#include "ros/ros.h"
#include <geometric_shapes/shape_operations.h>

//Internal
#include "PlannerUtils.h"
#include "Matplotlibcpp.h"


struct CollisionObject
{
    std::string id;
    shapes::Mesh *shape;
    tf::Transform pose;
};
class ConfigurationSpace
{
public:
    ConfigurationSpace(
        std::string robot_description,
        std::vector<tf::Vector3> trivial_collisions);

    ConfigurationSpace(
        std::string robot_description);

    ~ConfigurationSpace() = default;

    void spin();

    void addCollision(const std::vector<tf::Vector3> &trivial_collisions);

    void addCollision(const std::vector<CollisionObject> &collision_objects);

    void loadCollisionObjects();

private:
    ros::NodeHandle _nh;
    ros::ServiceClient _process_image_client;
    ros::Publisher _planning_scene_diff_pub;
    robot_model_loader::RobotModelLoader _robot_model_loader;
    moveit::core::RobotModelPtr _robot_model;
    ros::Rate _rate = ros::Rate(10);
    std::vector<tf::Vector3> _trivial_collisions;
    std::vector<CollisionObject> _collision_objects;
    std::string _collision_objects_dir;
};