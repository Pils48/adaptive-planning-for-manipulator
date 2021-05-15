//MoveIt
#include <moveit_msgs/PlanningScene.h>
#include <moveit/robot_state/robot_state.h>

//ros
#include "std_msgs/Bool.h"
#include "geometric_shapes/shape_operations.h"

//Internal
#include "ConfigurationSpace.h"
#include "Solver.h"
#include "planner/ProcessImage.h"

//boost
#include <boost/filesystem.hpp>


using namespace std;
using namespace moveit::core;


ConfigurationSpace::ConfigurationSpace
(
    string robot_description,
    vector<tf::Vector3> trivial_collisions
)
    : ConfigurationSpace(robot_description)
{
    addCollision(trivial_collisions);
}

ConfigurationSpace::ConfigurationSpace
(
    std::string robot_description
)
    : _robot_model_loader(robot_description)
    , _robot_model(_robot_model_loader.getModel())
{
    ROS_INFO("Building configuration space...");
    ROS_INFO("Model %s loaded", _robot_model->getName().c_str());
    _process_image_client = _nh.serviceClient<planner::ProcessImage>("process_image");
    ROS_INFO("Process_image_client has been created");
    _planning_scene_diff_pub = _nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 100);
    ROS_INFO("Planning_scene_diff_pub has been created");
    waitForSubscribers(_planning_scene_diff_pub, 1);

    if (_nh.hasParam("/configuration_space_node/collision_objects_dir"))
    {
        _nh.param("/configuration_space_node/collision_objects_dir", _collision_objects_dir, string{""});
    }
    else
    {
        ROS_WARN("Unspesified directory for collision objects");
    }
    ROS_INFO("Set directory for collision objects: %s", _collision_objects_dir.c_str());
    loadCollisionObjects();

}

void ConfigurationSpace::spin()
{   
    while(ros::ok())
    {
        ros::spinOnce();
        _rate.sleep();
    }
}

void ConfigurationSpace::addCollision(const vector<tf::Vector3> &trivial_collisions)
{
    _trivial_collisions.insert(_trivial_collisions.end(), trivial_collisions.begin(), trivial_collisions.end());
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(_robot_model));
    auto joint_model_group = kinematic_state->getJointModelGroup("3_dof_manipulator");
    ROS_INFO("%s joint model group has been loaded", joint_model_group->getName().c_str());

    auto solver = createSolver(*joint_model_group);
    auto chain = solver->getSimplifiedLinksChain(*joint_model_group);
    vector<double> links_length;
    transform(chain.begin(), chain.end(), back_inserter(links_length), &getLinkLength);
    solver->solveExpandIK(trivial_collisions, links_length, true);

    planner::ProcessImage srv;
    srv.request.filename = "images/test_plot.jpg"; //TO DO: do not rewrite image
    if (_process_image_client.call(srv))
    {
        ROS_INFO("Image %s has been successfully processed", srv.request.filename.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service process_image");
        throw std::runtime_error("Failed to call service process_image");
    }
}

void ConfigurationSpace::addCollision(
    const vector<CollisionObject> &collision_objects
)
{
    moveit_msgs::PlanningScene planning_scene;
    vector<moveit_msgs::CollisionObject> collision_objects_msgs;
    for(const auto &collision_object : collision_objects)
    {
        auto verticies = pointsFromRawData(collision_object.shape->vertices, collision_object.shape->vertex_count, collision_object.pose);
        for (const auto &vertex : verticies)
        {
            ROS_DEBUG("Vertex: %f %f %f", vertex.getX(), vertex.getY(), vertex.getZ());
        }
        addCollision(verticies); //Add collision on map
        _trivial_collisions.insert(_trivial_collisions.end(), verticies.begin(), verticies.end()); //add to trivial_collision and add observer
        moveit_msgs::CollisionObject collision_object_msg;
        collision_object_msg.id = collision_object.id;
        collision_object_msg.header.frame_id = "base_link";
        shape_msgs::Mesh mesh;
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(collision_object.shape, mesh_msg);
        mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

        collision_object_msg.meshes.emplace_back(mesh);
        collision_object_msg.mesh_poses.emplace_back(transformToMsg(collision_object.pose));
        collision_object_msg.operation = collision_object_msg.ADD;
        collision_objects_msgs.emplace_back(collision_object_msg);
    }
    planning_scene.world.collision_objects.insert(planning_scene.world.collision_objects.end(), 
                                    collision_objects_msgs.begin(), collision_objects_msgs.end());
    planning_scene.is_diff = true;
    _planning_scene_diff_pub.publish(planning_scene);
    ROS_INFO("%lu objects have been added to scene", collision_objects.size());
    sleep(5.0); 
}

void ConfigurationSpace::loadCollisionObjects()
{
    boost::filesystem::path dir(_collision_objects_dir);
    if (!boost::filesystem::is_directory(dir))
    {
        ROS_ERROR("collision_objects_dir is not a directory!");
        throw std::runtime_error("collision_objects_dir is not a directory!");
    }
    
    std::vector<string> co_filenames;
    for (auto &entry : boost::filesystem::directory_iterator(dir))
    {
        ROS_INFO("Collision_object %s", entry.path().filename().c_str());
        co_filenames.emplace_back(entry.path().generic_string());
    }

    //Hardcode alert!!!
    for (const auto &abs_filename : co_filenames)
    {
        shapes::Mesh *mesh = shapes::createMeshFromResource("file://" + abs_filename, Eigen::Vector3d{200, 200, 200});//leak??
        shapes::ShapePtr shape_ptr(mesh);
        tf::Transform collision_object_pose(tf::Quaternion(0, 0, 1, 0), tf::Vector3(0, 150, 150)); //Hardcode
        CollisionObject collision_object {"first_object", mesh, collision_object_pose}; //Hardcode
        addCollision(vector<CollisionObject>{collision_object});
        _collision_objects.emplace_back(collision_object);
    }
    
}
