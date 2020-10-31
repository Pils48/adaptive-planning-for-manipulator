#include "TrivialIK.h"

#define PRIMITIVE_MANIPULATOR "manipulator"
#define ROBOTIS_MANIPULATOR "robotis_manipulator"


SolverPtr createSolver(const robot_model::JointModelGroup &joint_model_group)
{
    if(joint_model_group.getName() == PRIMITIVE_MANIPULATOR)
    {   
        SolverPtr trivial_solver(new TrivialIK);
        if (trivial_solver->isJointModelGroupValid(joint_model_group))
        {
            return trivial_solver;
        }
        else
        {
            throw std::runtime_error("Invalid joint model group!");
        }
    }
    else if(joint_model_group.getName() == ROBOTIS_MANIPULATOR)
    {
        return nullptr;
    }
}