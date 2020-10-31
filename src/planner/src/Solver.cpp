#include "TrivialIK.h"

#define PRIMITIVE_MANIPULATOR "manipulator"
#define ROBOTIS_MANIPULATOR "robotis_manipulator"

SolverPtr createSolver(
    const tf::Vector3 &pose,
    const robot_model::JointModelGroup &joint_model_group
)
{
    if(joint_model_group.getName() == PRIMITIVE_MANIPULATOR)
    {   
        SolverPtr trivial_solver(new TrivialIK);
        return trivial_solver;
    }
    else if(joint_model_group.getName() == ROBOTIS_MANIPULATOR)
    {
        return nullptr;
    }
}