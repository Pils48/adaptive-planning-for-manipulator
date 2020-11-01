#include <tf/tf.h>

#include <moveit/robot_model/robot_model.h>

#include <Eigen/Core>

#include <vector>
#include <algorithm>
#include <memory>
#include <cmath>

using Solutions = std::vector<std::vector<double>>;
class Solver
{
public:
    virtual Solutions solveIK(
        const tf::Vector3 &pose, 
        const std::vector<double> &links_length
    ) = 0;

    virtual bool isJointModelGroupValid(
        const moveit::core::JointModelGroup &joint_model_group
    ) = 0;

    virtual std::vector<const moveit::core::LinkModel*> getSimplifiedLinksChain(
        const moveit::core::JointModelGroup &joint_model_group
    ) = 0;
};

using SolverPtr = std::shared_ptr<Solver>;

SolverPtr createSolver(const robot_model::JointModelGroup &joint_model_group);