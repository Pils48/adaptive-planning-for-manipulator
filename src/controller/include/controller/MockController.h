// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

//project
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

class MockController
{
    ros::NodeHandle _nh;
    ros::Subscriber _animation_listener;
    ros::Rate _rate = ros::Rate(10);

    robot_model_loader::RobotModelLoader _robot_model_loader;
    const moveit::core::RobotModelPtr _kinematic_model;
    planning_scene::PlanningScene _planning_scene;

public:
    MockController();

    ~MockController() = default;

    void animationCallback(const std_msgs::Float64MultiArray &trajectory_states);

    void spin();
};