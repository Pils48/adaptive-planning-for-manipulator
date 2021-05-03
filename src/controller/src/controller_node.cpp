#include <tf/tf.h>

//project
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"


void animationCallback(const std_msgs::Float64MultiArray &trajectory_states)
{
    ROS_INFO("Size of array, %lu", trajectory_states.data.size());
}

int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    ros::Subscriber animation_listener = node.subscribe("animation_topic", 1000, &animationCallback);
    spinner.start();
    ROS_INFO("Controller node started");
    while(ros::ok())
    {
        // std_msgs::Bool is_ready;
        // is_ready.data = true;
        // space_ready_pub.publish(is_ready);
    }
    return 0;
}