#include "TrivialIK.h"
#include "PlannerUtils.h"

using namespace robot_model; 
using namespace std;

const RevoluteJointModel* castToRevouluteModel(const JointModel* joint_model)
{
    try
    {
        auto revoulute_joint = dynamic_cast<const RevoluteJointModel*>(joint_model);
        return revoulute_joint;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Non revoulute joint_models");        
    }       
}

vector<double> TrivialIK::solveIK(
    const tf::Vector3 &trivial_collision, 
    const JointModelGroup &joint_model_group
    )
{
    
    const auto joint_models = joint_model_group.getActiveJointModels();
    const auto joints_number = joint_models.size();
    if (joints_number > 2)
    {
        ROS_ERROR("Solver doesn't support more than two joints yet!");
    }

    vector<const RevoluteJointModel*> revoulute_joint_models;
    std::transform(joint_models.begin(), joint_models.end(), std::back_inserter(revoulute_joint_models), &castToRevouluteModel);
    /*
    // if (std::count_if(joint_models.begin(), joint_models.end(), 
    //     [](const auto *joint_model)
    //     {
    //         try
    //         {
    //             auto revoulute_joint = dynamic_cast<RevoluteJointModel*>(joint_model);
    //             auto axis = revoulute_joint->getAxis();
    //         }
    //         catch(const std::bad_cast& e)
    //         {
    //             ROS_ERROR("Non revoulute joint_models");
    //         }
            
    //     }
    // ) != joints_number)
    // {

    // }
    */
   return {};

}



