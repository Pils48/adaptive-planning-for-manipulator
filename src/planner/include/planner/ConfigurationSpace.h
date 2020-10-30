//moveit
#include <moveit/robot_model/robot_model.h>

//Internal
#include "PlannerUtils.h"
#include "Matplotlibcpp.h"


namespace plt = matplotlibcpp;

class ConfigurationSpace
{
public:
    ConfigurationSpace
    (
        Point trivial_collision,
        moveit::core::RobotModelPtr robot_model
    ); //testing constructor

    void showPlot();

    //ConfigurationSpace(RobotModel, vector<shapes>);
private:
    void addCollision(const Point &trivial_collision);

    plt::Plot cspace_plot;
    Point _trivial_collision;
    moveit::core::RobotModelPtr _robot_model;

    //vector<shapes> _collision_objects
};