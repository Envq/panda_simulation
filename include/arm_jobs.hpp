#pragma once

// MyLibs
#include "my_exceptions.hpp"


// C++
#include <boost/filesystem.hpp>
#include <jsoncpp/json/json.h>
#include <set>
#include <vector>

// ROS
#include <ros/ros.h>
#include <ros/package.h>


// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ObjectColor.h>
#include <moveit_msgs/ObjectColor.h>


// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace arm_jobs {

void openGripper(trajectory_msgs::JointTrajectory &posture);

void closedGripper(trajectory_msgs::JointTrajectory &posture);

void pick(moveit::planning_interface::MoveGroupInterface &move_group);

void place(moveit::planning_interface::MoveGroupInterface &group);

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface
                             &planning_scene_interface,
                         const std::string &scene_name_file);

}  // namespace arm_jobs