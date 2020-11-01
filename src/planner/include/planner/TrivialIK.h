#include "Solver.h"

struct Point;

class TrivialIK : public Solver
{
public:
    TrivialIK() = default;
    
    bool isJointModelGroupValid(const robot_model::JointModelGroup &joint_model_group);

    std::vector<const moveit::core::LinkModel*> getSimplifiedLinksChain(
        const moveit::core::JointModelGroup &joint_model_group
    );

    Solutions solveIK(
        const tf::Vector3 &trivial_collision, 
        const std::vector<double> &links_length
    );
};

