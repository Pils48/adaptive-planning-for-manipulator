#include <tf/tf.h>

//moveit
#include <moveit/robot_model/robot_model.h>

//Internal
#include "PlannerUtils.h"
#include "Matplotlibcpp.h"

#define STANDARD_DISCRETIZATION 0.0001 //1cm

namespace plt = matplotlibcpp;

class ConfigurationSpace
{
public:
    ConfigurationSpace
    (
        std::vector<tf::Vector3> trivial_collisions,
        moveit::core::RobotModelPtr robot_model
    );

    void showPlot();

    //ConfigurationSpace(RobotModel, vector<shapes>);
private:
    void addCollision(const std::vector<tf::Vector3> &trivial_collisions);

    plt::Plot _cspace_plot;
    std::vector<tf::Vector3> _trivial_collisions;
    moveit::core::RobotModelPtr _robot_model;

    //vector<shapes> _collision_objects
};