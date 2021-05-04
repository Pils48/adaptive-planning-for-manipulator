// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

//project
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"


constexpr double INTERPOLATION_STEP = 0.001;

namespace rvt = rviz_visual_tools;
class MockController
{
    ros::NodeHandle _nh;
    ros::Subscriber _animation_listener;
    ros::Rate _rate = ros::Rate(10);

    robot_model_loader::RobotModelLoader _robot_model_loader;
    const moveit::core::RobotModelPtr _kinematic_model;
    planning_scene::PlanningScene _planning_scene;
    moveit_visual_tools::MoveItVisualTools _visual_tools;
    robot_state::RobotState _kinematic_state;

public:
    MockController();

    ~MockController() = default;

    void animationCallback(const std_msgs::Float64MultiArray &trajectory_states);

    void spin();
};