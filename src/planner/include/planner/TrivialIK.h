#include "Solver.h"

struct Point;

class TrivialIK : Solver
{
public:
    TrivialIK() = default;
    
    std::vector<double> solveIK(
        const tf::Vector3 &trivial_collision, 
        const robot_model::JointModelGroup &joint_model_group
    );
};

