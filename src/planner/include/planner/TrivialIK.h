#include "Solver.h"

struct Point;

class TrivialIK : public Solver
{
public:
    TrivialIK() = default;
    
    Solutions solveIK(
        const tf::Vector3 &trivial_collision, 
        const robot_model::JointModelGroup &joint_model_group
    );
};

