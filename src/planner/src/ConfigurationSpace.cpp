//MoveIt
#include <moveit_msgs/PlanningScene.h>
#include <moveit/robot_state/robot_state.h>

//ros
#include "std_msgs/Bool.h"
#include "geometric_shapes/shape_operations.h"

//Internal
#include "ConfigurationSpace.h"
#include "Solver.h"

//boost
#include <boost/filesystem.hpp>


using namespace std;
using namespace moveit::core;


ConfigurationSpace::ConfigurationSpace
(
    string robot_description,
    vector<tf::Vector3> trivial_collisions
)
    : _robot_model_loader(robot_description)
    , _robot_model(_robot_model_loader.getModel())
{
    _space_ready_pub = _nh.advertise<std_msgs::Bool>("space_ready_topic", 10);
    waitForSubscribers(_space_ready_pub, 1);
    _planning_scene_diff_pub = _nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 100);
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
    addCollision(trivial_collisions);
}

ConfigurationSpace::ConfigurationSpace
(
    std::string robot_description
)
    : _robot_model_loader(robot_description)
    , _robot_model(_robot_model_loader.getModel())
{
    _space_ready_pub = _nh.advertise<std_msgs::Bool>("space_ready_topic", 10);
    waitForSubscribers(_space_ready_pub, 1);
    _planning_scene_diff_pub = _nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 100);
    waitForSubscribers(_planning_scene_diff_pub, 1);
    // _collision_objects.insert(_collision_objects.end(), objects.begin(), objects.end());

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
    ROS_INFO("configuration_space_node started");
    ROS_INFO("Model %s loaded", _robot_model->getName().c_str());
    ROS_INFO("Building configuration space...");
    while(ros::ok())
    {
        std_msgs::Bool is_ready;
        is_ready.data = true;
        // space_ready_pub.publish(is_ready);
        ros::spinOnce();
        _rate.sleep();
    }
}

void ConfigurationSpace::addCollision(const vector<tf::Vector3> &trivial_collisions)
{
    _trivial_collisions.insert(_trivial_collisions.end(), trivial_collisions.begin(), trivial_collisions.end());
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(_robot_model));
    auto joint_model_group = kinematic_state->getJointModelGroup("manipulator");
    ROS_INFO("%s joint model group loaded", joint_model_group->getName().c_str());

    auto solver = createSolver(*joint_model_group);
    auto chain = solver->getSimplifiedLinksChain(*joint_model_group);
    vector<double> links_length;
    transform(chain.begin(), chain.end(), back_inserter(links_length), &getLinkLength);
    solver->solveExpandIK(trivial_collisions, links_length, false);
}

void ConfigurationSpace::addCollision(
    const vector<CollisionObject> &collision_objects
)
{
    moveit_msgs::PlanningScene planning_scene;
    vector<moveit_msgs::CollisionObject> collision_objects_msgs;
    for(const auto &collision_object : collision_objects)
    {
        // _collision_objects.push_back(ncollision_object.shape);
        auto verticies = pointsFromRawData(collision_object.shape->vertices, collision_object.shape->vertex_count);
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
        shapes::Mesh *mesh = shapes::createMeshFromResource("file://" + abs_filename);//leak??
        shapes::ShapePtr shape_ptr(mesh);
        tf::Transform collision_object_pose(tf::Quaternion(1, 0, 0, 0), tf::Vector3(100, 0, -5.5)); //Hardcode
        CollisionObject collision_object {"first_object", mesh, collision_object_pose}; //Hardcode
        addCollision(vector<CollisionObject>{collision_object});
        _collision_objects.emplace_back(collision_object);
    }
    
}
