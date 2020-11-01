#include "ros/ros.h"

//moveit
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include "ConfigurationSpace.h"
#include "Solver.h"


using namespace std;
using namespace moveit::core;

ConfigurationSpace::ConfigurationSpace
(
    tf::Vector3 trivial_collision,
    moveit::core::RobotModelPtr robot_model
)
    : _trivial_collision(trivial_collision)
    , _robot_model(robot_model) 
{
    addCollision(trivial_collision);
}

void ConfigurationSpace::showPlot()
{

}
 
//save png
void ConfigurationSpace::addCollision(const tf::Vector3 &trivial_collision)
{
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(_robot_model));
    auto joint_model_group = kinematic_state->getJointModelGroup("manipulator");

    auto solver = createSolver(*joint_model_group);
    auto chain = solver->getSimplifiedLinksChain(*joint_model_group);
    vector<double> links_length;
    transform(chain.begin(), chain.end(), back_inserter(links_length), &getLinkLength);
    // double total_length = accumulate(links_length.begin(), links_length.end(), 0);
    double total_length = links_length.front() + links_length.back();
    // ROS_INFO("total_length: %f", total_length);
    vector<double> x, y;
    for (size_t i = 0; i < total_length / STANDARD_DISCRETIZATION; ++i)
    {
        if (i < links_length.front() / STANDARD_DISCRETIZATION)
        {
            auto joints = solver->solveIK(tf::Vector3(0.15, 0.15, 1), 
                        {links_length.front(), links_length.back() - i * STANDARD_DISCRETIZATION});
            for (const auto &solution : joints)
            {
                x.push_back(solution[0]);
                y.push_back(solution[1]);
            }
        }
        else
        {
            auto joints = solver->solveIK(tf::Vector3(0.15, 0.15, 1), 
                        {links_length.front() - i * STANDARD_DISCRETIZATION + links_length.back(), 0});
            for (const auto &solution : joints)
            {
                x.push_back(solution[0]);
                y.push_back(solution[1]);
            }
        }
    }
    plt::ion()
    plt::figure();
    plt::subplot()
    // plt::grid(true);
    // plt::scatter(x, y);
    // // plt::show();
    // x.push_back(10);
    // y.push_back(10);
    // plt::scatter(x, y);
    // plt::show();
    // int n = 1000;
    // std::vector<double> x_e, y_e, z_e;
    // for(int i=0; i<n; i++) {
	// 	x_e.push_back(i*i);
	// 	y_e.push_back(sin(2*M_PI*i/360.0));
	// 	z_e.push_back(log(i));

	// 	if (i % 2 == 0) {
	// 		// Clear previous plot
	// 		plt::clf();
	// 		// Plot line from given x and y data. Color is selected automatically.
	// 		plt::scatter(x_e, y_e);
	// 		// Plot a line whose name will show up as "log(x)" in the legend.
	// 		plt::named_plot("log(x)", x_e, z_e);

	// 		// Set x-axis to interval [0,1000000]
	// 		plt::xlim(0, n*n);

	// 		// Add graph title
	// 		plt::title("Sample figure");
	// 		// Enable legend.
	// 		// plt::legend();
	// 		// Display plot continuously
	// 		plt::cla;
	// 	}
	// }
}