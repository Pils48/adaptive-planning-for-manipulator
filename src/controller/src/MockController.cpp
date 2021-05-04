#include "MockController.h"

MockController::MockController()
    :   _robot_model_loader("robot_description")
    ,   _kinematic_model(_robot_model_loader.getModel())
    ,   _planning_scene(_kinematic_model)
{
    _animation_listener = _nh.subscribe("animation_topic", 1000, &MockController::animationCallback, this);
}

void MockController::animationCallback(const std_msgs::Float64MultiArray &trajectory_states)
{
    ROS_INFO("Size of array, %lu", trajectory_states.data.size());
    for (const auto &state : trajectory_states.data)
    {
        ROS_INFO("Coordinate %f", state);
    }
}

void MockController::spin()
{
    ROS_INFO("Controller node started");
    while(ros::ok())
    {
        ros::spinOnce();
        _rate.sleep();
    }
}