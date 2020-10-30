#include <tf/tf.h>

#include <moveit/robot_model/robot_model.h>

#include <Eigen/Core>

#include <vector>
#include <algorithm>
#include <memory>
#include <cmath>

class Solver
{
public:
    virtual std::vector<double> solveIK(
        const tf::Vector3 &pose, 
        const robot_model::JointModelGroup &joint_model_group
    ) = 0;
};

using SolverPtr = std::shared_ptr<Solver>;

SolverPtr createSolver(
    const tf::Vector3 &pose,
    const robot_model::JointModelGroup &joint_model_group
);