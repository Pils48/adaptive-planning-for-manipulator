#include "TrivialIK.h"

class DiplomaIKSolver : public Solver
{
public:
    DiplomaIKSolver() = default;

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