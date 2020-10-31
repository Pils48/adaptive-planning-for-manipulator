#include <tf/tf.h>

//moveit
#include <moveit/robot_model/robot_model.h>

//Internal
#include "PlannerUtils.h"
#include "Matplotlibcpp.h"

#define STANDARD_DISCRETIZATION 0.01 //1cm

namespace plt = matplotlibcpp;

class ConfigurationSpace
{
public:
    ConfigurationSpace
    (
        tf::Vector3 trivial_collision,
        moveit::core::RobotModelPtr robot_model
    ); //testing constructor

    void showPlot();

    //ConfigurationSpace(RobotModel, vector<shapes>);
private:
    void addCollision(const tf::Vector3 &trivial_collision);

    plt::Plot cspace_plot;
    tf::Vector3 _trivial_collision;
    moveit::core::RobotModelPtr _robot_model;

    //vector<shapes> _collision_objects
};