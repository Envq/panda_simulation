#include "arm_jobs.hpp"
// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>



// Constants
static const std::string ARM_GROUP = "panda_arm";



int main(int argc, char **argv) {
    // Setup ROS
    ros::init(argc, argv, argv[0]);
    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Setup Moveit
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group(ARM_GROUP);

    // Setup time
    ros::WallDuration time(2.0);

    // Get the scene name from ROS parameter server
    std::string scene_name_file;
    if (!node_handle.getParam("scene_name_file", scene_name_file)) {
        ROS_FATAL_STREAM("Read from ROS parameter server Error: Check "
                         "'scene_name_file' field "
                         "in your launch file");
        return 0;
    }



    //** START TASK **
    ROS_INFO("**START TASK");
    time.sleep();
    group.setPlanningTime(45.0);

    // Init Objects
    ROS_INFO("**ADD COLLISION");
    arm_jobs::addCollisionObjects(planning_scene_interface, scene_name_file);
    time.sleep();

    // Pick target object
    ROS_INFO("**PICK");
    arm_jobs::pick(group);
    time.sleep();

    // Place target Object
    ROS_INFO("**PLACE");
    arm_jobs::place(group);
    time.sleep();



    // Finish
    ROS_INFO("**FINISH");
    ros::shutdown();
    return 0;
}