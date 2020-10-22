#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include "ConfigurationSpace.h"

ConfigurationSpace::ConfigurationSpace
(
    Point trivial_collision,
    moveit::core::RobotModel robot_model
)
    : _trivial_collision(trivial_collision)
    , _robot_model(robot_model) 
{
    addCollision(trivial_collision);
}

void ConfigurationSpace::showPlot()
{

}

void ConfigurationSpace::addCollision(const Point &trivial_collision)
{

}