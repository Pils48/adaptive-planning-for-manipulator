#include <tf/tf.h>

#include <moveit/robot_model/robot_model.h>

#include "Matplotlibcpp.h"

#include <Eigen/Core>

#include <vector>
#include <algorithm>
#include <memory>
#include <cmath>

constexpr auto STANDARD_DISCRETIZATION = 0.1; 
constexpr auto PLOT_X_LOWER_LIMIT = -M_PI;
constexpr auto PLOT_X_UPPER_LIMIT = M_PI;
constexpr auto PLOT_Y_LOWER_LIMIT = -M_PI;
constexpr auto PLOT_Y_UPPER_LIMIT = M_PI;
constexpr auto FIGURE_WIDTH = 800;
constexpr auto FIGURE_HEIGHT = 600;

using Solutions = std::vector<std::vector<double>>;

namespace plt = matplotlibcpp;
class Solver
{
public:
    virtual void solveExpandIK(
        const std::vector<tf::Vector3> &trivial_collisions, 
        const std::vector<double> &links_length,
        bool show
    ) = 0;

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