#include <tf/tf.h>

//moveit
#include <moveit/robot_model_loader/robot_model_loader.h>

//project
#include "ConfigurationSpace.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"


//TO DO: crash with segfault while setting to default robot_state, probably configs are malformed

int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "configuration_space_node");
    ros::NodeHandle node;
    // ros::AsyncSpinner spinner(1);
    // spinner.start();
    ROS_INFO("configuration_space_node started");
    
    //Init configuration space
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model %s loaded", kinematic_model->getName().c_str());
    ROS_INFO("Building configuration space...");
    auto trivial_collisions = generateTestPointsArray(
        0.12, 0.16, 0.01,
        0.12, 0.16, 0.01,
        0.12, 0.16, 0.01
    );
    ConfigurationSpace c_space(trivial_collisions, kinematic_model);
    ros::Publisher space_ready_pub = node.advertise<std_msgs::Bool>("space_ready_topic", 10);
    
    while(ros::ok())
    {
        std_msgs::Bool is_ready;
        is_ready.data = true;
        // space_ready_pub.publish(is_ready);
    }
    return 0;
}