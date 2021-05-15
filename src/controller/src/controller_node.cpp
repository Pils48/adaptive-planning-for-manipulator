#include <tf/tf.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

//project
#include "MockController.h"


int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "controller_node");
    MockController mock_controller;
    // mock_controller.test();
    mock_controller.spin();
    return 0;
}