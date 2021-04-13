#include "TrivialIK.h"

class DiplomaIKSolver : public Solver
{
public:
    DiplomaIKSolver() = default;

    void solveExpandIK(
        const std::vector<tf::Vector3> &trivial_collisions, 
        const std::vector<double> &links_length,
        bool show
    );

    Solutions solveIK(
        const tf::Vector3 &pose, 
        const std::vector<double> &links_length
    );

    bool isJointModelGroupValid(
        const moveit::core::JointModelGroup &joint_model_group
    );

    std::vector<const moveit::core::LinkModel*> getSimplifiedLinksChain(
        const moveit::core::JointModelGroup &joint_model_group
    );
};