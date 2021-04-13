#include "TrivialIK.h"
#include "PlannerUtils.h"

#include <moveit/robot_model/link_model.h>

using namespace robot_model; 
using namespace std;


void concatenatePoints(
    vector<double> &x_back, vector<double> &y_back,
    vector<double> &x_front, vector<double> &y_front
)
{
    reverse(x_back.begin(), x_back.end());
    reverse(y_back.begin(), y_back.end());
    x_front.insert(x_front.end(), x_back.begin(), x_back.end());
    y_front.insert(y_front.end(), y_back.begin(), y_back.end());
    plt::plot(x_front, y_front, 
        map<string, string>{make_pair("color", "black"), make_pair("linewidth", "3")});
    x_front.clear();
    y_front.clear();
    x_back.clear();
    y_back.clear();
}

bool TrivialIK::isJointModelGroupValid(const moveit::core::JointModelGroup &joint_model_group)
{
    const auto joint_models = joint_model_group.getActiveJointModels();
    const auto joints_number = joint_models.size();
    if (joints_number > 2)
    {
        ROS_ERROR("Trivial solver doesn't support more than two joints!");
        return false;
    }

    vector<const RevoluteJointModel*> revoulute_joint_models;
    transform(joint_models.begin(), joint_models.end(), back_inserter(revoulute_joint_models), &castToRevouluteModel);
    if (revoulute_joint_models.front()->getAxis() == revoulute_joint_models.back()->getAxis())
    {
        ROS_INFO("Trivial chain is valid");
        return true;
    }
    else
    {
        ROS_ERROR("Invalid chain!");
        return false;
    }
}

std::vector<const moveit::core::LinkModel*> TrivialIK::getSimplifiedLinksChain(
    const moveit::core::JointModelGroup &joint_model_group
)
{
    const auto joints_models = joint_model_group.getActiveJointModels();
    vector<const LinkModel*> chain;
    for (const auto *joint_model : joints_models)
    {
        ROS_INFO("LINK MODEL: %s", joint_model->getChildLinkModel()->getName().c_str());
        chain.emplace_back(joint_model->getChildLinkModel());
    }
    return chain;
}

Solutions TrivialIK::solveIK(
    const tf::Vector3 &pose, 
    const vector<double> &links_length
    )
{
    if (links_length.size() != 2)
    {
        throw runtime_error("Links number for trivial solver doesn't equal 2!");
    }

    const double length_1 = links_length.front();
    const double length_2 = links_length.back();
    Solutions solutions;
    //First solution
    double joint_2 = acos((sqr(pose.getX()) + sqr(pose.getY()) - sqr(length_1) - sqr(length_2)) / (2 * length_1 * length_2));
    double joint_1 = atan(pose.getY() / pose.getX()) - atan(length_2 * sin(joint_2) / (length_1 + length_2 * cos(joint_2)));
    ROS_DEBUG("First solution:");
    ROS_DEBUG("Joint 1: %f", (joint_1 - M_PI / 2) * 180 / M_PI);
    ROS_DEBUG("Joint 2: %f", joint_2 * 180 / M_PI);
    if (!isnan(joint_1) && !isnan(joint_2))
    {
        solutions.push_back(vector<double>{joint_1, joint_2});
    }

    //Second solution
    joint_2 = -acos((sqr(pose.getX()) + sqr(pose.getY()) - sqr(length_1) - sqr(length_2)) / (2 * length_1 * length_2));
    joint_1 = atan(pose.getY() / pose.getX()) - atan(length_2 * sin(joint_2) / (length_1 + length_2 * cos(joint_2)));
    if (!isnan(joint_1) && !isnan(joint_2))
    {
        solutions.push_back(vector<double>{joint_1, joint_2});
    }
    ROS_DEBUG("Second solution:");
    ROS_DEBUG("Joint 1: %f", (joint_1 - M_PI / 2) * 180 / M_PI);
    ROS_DEBUG("Joint 2: %f", joint_2 * 180 / M_PI);
    return solutions;
}

void TrivialIK::solveExpandIK(
    const std::vector<tf::Vector3> &trivial_collisions, 
    const std::vector<double> &links_length,
    bool show
)
{
    auto total_length = accumulate(links_length.begin(), links_length.end(), 0.0);
    ROS_INFO("Total length: %f", total_length);
    for (const auto &link_length : links_length)
    {
        ROS_INFO("link_length: %f", link_length);
    }
    if (show)
    {
        plt::title("Image of the obstacle");
        plt::xlim(PLOT_X_LOWER_LIMIT, PLOT_X_UPPER_LIMIT);
        plt::ylim(PLOT_Y_LOWER_LIMIT, PLOT_Y_UPPER_LIMIT);
        plt::grid(true);
    }

    //Setting up solver params
    vector<double> x_front, y_front, x_back, y_back; 
    for (size_t idx = 0; idx < trivial_collisions.size(); ++idx)
    {
        for (size_t i = 0; i < total_length / STANDARD_DISCRETIZATION; ++i)
        {
            if (i < links_length.back() / STANDARD_DISCRETIZATION)
            {
                auto joints = solveIK(trivial_collisions[idx], 
                    {links_length.front(), links_length.back() - i * STANDARD_DISCRETIZATION});
                if (joints.size() == 2)
                {
                    x_front.emplace_back(joints.front()[0]);
                    y_front.emplace_back(joints.front()[1]);
                    x_back.emplace_back(joints.back()[0]);
                    y_back.emplace_back(joints.back()[1]);
                    }
            }
            else
            {
                tf::Vector3 obstacle(trivial_collisions[idx].getX(), trivial_collisions[idx].getY(), 0);
                if (links_length.back() > obstacle.length())
                {
                    tf::Vector3 y_ort(0, 1, 0);
                    const double abs_angle = obstacle.angle(y_ort);
                    const double angle = obstacle.cross(y_ort) > 0 ? abs_angle : -abs_angle;
                    if(show)
                    {
                        plt::plot({angle, angle}, {PLOT_Y_LOWER_LIMIT, PLOT_Y_UPPER_LIMIT},
                            map<string, string>{make_pair("color", "black"), make_pair("linewidth", "3")});
                    } 
                }
            }
        }
        concatenatePoints(x_back, y_back, x_front, y_front);
        if (show)
        {
            plt::pause(0.1);
        }
    }
}

