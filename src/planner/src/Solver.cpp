#include "DiplomaIKSolver.h"

#define PRIMITIVE_MANIPULATOR "manipulator"
#define MANIPULATOR_3_DOF "3_dof_manipulator"
#define ROBOTIS_MANIPULATOR "robotis_manipulator"


SolverPtr createSolver(const robot_model::JointModelGroup &joint_model_group)
{
    if(joint_model_group.getActiveJointModels().size() == 2)
    {   
        SolverPtr trivial_solver(new TrivialIK);
        if (trivial_solver->isJointModelGroupValid(joint_model_group))
        {
            ROS_INFO("Trivial solver created");
            return trivial_solver;
        }
        else
        {
            throw std::runtime_error("Invalid joint model group!");
        }
    }
    else if(joint_model_group.getActiveJointModels().size() == 3)
    {
        SolverPtr diploma_solver(new DiplomaIKSolver);
        if (diploma_solver->isJointModelGroupValid(joint_model_group))
        {
            ROS_INFO("Diploma solver created");
            return diploma_solver;
        }
        else
        {
            throw std::runtime_error("Invalid joint model group!");
        }
    }
    else if(joint_model_group.getName() == ROBOTIS_MANIPULATOR)
    {
        ROS_ERROR("Still does not support ROBOTIS MANIPULATOR!");
        return nullptr;
    }
}