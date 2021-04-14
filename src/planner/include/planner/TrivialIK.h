#include "Solver.h"

struct Point;

class TrivialIK : public Solver
{
public:
    TrivialIK() = default;
    
    void solveExpandIK(
        const std::vector<tf::Vector3> &trivial_collisions, 
        const std::vector<double> &links_length,
        bool show
    );

    bool isJointModelGroupValid(const robot_model::JointModelGroup &joint_model_group);

    std::vector<const moveit::core::LinkModel*> getSimplifiedLinksChain(
        const moveit::core::JointModelGroup &joint_model_group
    );

    Solutions solveIK(
        const tf::Vector3 &trivial_collision, 
        const std::vector<double> &links_length
    );
};

