#include "MockController.h"


std::vector<std::vector<double>> interpolate(
    const std::vector<double> &initial_state,
    const std::vector<double> &final_state
)
{
    if (initial_state.size() != final_state.size())
    {
        throw std::runtime_error("Size of joints vectors during interpolation doesn't coincide!");
    }

    std::vector<std::vector<double>> result_path;
    std::vector<double> steps_vector;
    for (size_t i = 0; i < initial_state.size(); ++i)
    {   
        steps_vector.push_back(std::abs(final_state[i] - initial_state[i]) / INTERPOLATION_STEP); //Alex mentioned that emplace is better
    }
    size_t steps = std::round(*std::max_element(steps_vector.begin(), steps_vector.end()));
    ROS_INFO("Number of steps: %lu", steps);
    for (size_t i = 1; i <= steps; ++i)
	{
        
        double percentage = (double)i / (double)steps;
        std::vector<double> waypoint;
        for (size_t j = 0; j < initial_state.size(); ++j)
        {
            waypoint.push_back((1 - percentage) * initial_state[j] + percentage * final_state[j]);
        }
        result_path.push_back(waypoint);
    }
    return result_path;
}

MockController::MockController()
    :   _robot_model_loader("robot_description")
    ,   _kinematic_model(_robot_model_loader.getModel())
    ,   _planning_scene(_kinematic_model)
    ,   _visual_tools("base_link")
    ,   _kinematic_state(_kinematic_model)
{
    _animation_listener = _nh.subscribe("animation_topic", 1000, &MockController::animationCallback, this);
}

void MockController::animationCallback(const std_msgs::Float64MultiArray &trajectory_states)
{
    ROS_DEBUG("Size of array, %lu", trajectory_states.data.size());
    for (const auto &state : trajectory_states.data)
    {
        ROS_DEBUG("Coordinate %f", state);
    }

    _visual_tools.deleteAllMarkers();
    _visual_tools.loadRemoteControl();

    const robot_state::JointModelGroup* joint_model_group_ptr = _kinematic_model->getJointModelGroup("manipulator");
    const auto joints_number = joint_model_group_ptr->getActiveJointModels().size();
    for (size_t i = 0; i < trajectory_states.data.size() - joints_number; i += joints_number)
    {
        std::vector<double> initial_state;
        std::vector<double> final_state;
        auto form_joints_vector = 
        [&trajectory_states, joints_number](const auto index)
        {
            std::vector<double> joints;
            for (size_t j = 0; j < joints_number; ++j)
            {
                ROS_DEBUG("Joint: %f", trajectory_states.data[index + j]);
                joints.push_back(trajectory_states.data[index + j]);
            }
            return joints;
        };
        initial_state = form_joints_vector(i);
        final_state = form_joints_vector(i + joints_number);
        auto path = interpolate(initial_state, final_state);
        for (const auto &waypoint : path)
        {
            ROS_DEBUG("Waypoint: {%f, %f}", waypoint[0], waypoint[1]);
           _kinematic_state.setVariablePositions(waypoint);
           _kinematic_state.update(true);
           _visual_tools.publishRobotState(_kinematic_state, rvt::colors::BLUE);
           std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
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