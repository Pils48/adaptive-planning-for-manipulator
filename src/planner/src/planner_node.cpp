//Internal
#include "ConfigurationSpace.h"


int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "configuration_space_node");
    auto trivial_collisions = generateTestPointsArray(
        0.12, 0.16, 0.01,
        0.12, 0.16, 0.01,
        0.12, 0.16, 0.01
    );
    ConfigurationSpace configuration_space("robot_description", trivial_collisions);
    configuration_space.spin();
    return 0;
}