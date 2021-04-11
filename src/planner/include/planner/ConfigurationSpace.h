#include <tf/tf.h>

//moveit
#include <moveit/robot_model/robot_model.h>

//Internal
#include "PlannerUtils.h"
#include "Matplotlibcpp.h"
#include "tinyply.h"

namespace plt = matplotlibcpp;

constexpr auto STANDARD_DISCRETIZATION = 0.0001; 
constexpr auto PLOT_X_LOWER_LIMIT = -M_PI;
constexpr auto PLOT_X_UPPER_LIMIT = M_PI;
constexpr auto PLOT_Y_LOWER_LIMIT = -M_PI;
constexpr auto PLOT_Y_UPPER_LIMIT = M_PI;

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