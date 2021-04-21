#include <tf/tf.h>

//moveit
#include <moveit/robot_model/robot_model.h>

//Internal
#include "PlannerUtils.h"
#include "Matplotlibcpp.h"

class ConfigurationSpace
{
public:
    ConfigurationSpace
    (
        std::vector<tf::Vector3> trivial_collisions,
        moveit::core::RobotModelPtr robot_model
    );

    //ConfigurationSpace(RobotModel, vector<shapes>);
private:
    void addCollision(const std::vector<tf::Vector3> &trivial_collisions);

    std::vector<tf::Vector3> _trivial_collisions;
    moveit::core::RobotModelPtr _robot_model;

    //vector<shapes> _collision_objects
};