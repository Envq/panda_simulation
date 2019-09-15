#pragma once

// My libs
#include "my_exceptions.hpp"

// C++
#include <boost/filesystem.hpp>
#include <jsoncpp/json/json.h>
#include <set>
#include <vector>

// Ros
#include <ros/package.h>
#include <ros/ros.h>

// Moveit
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>



// Initialize the scene with the objects contained in json files
// Note: can throw planning_error execption
moveit_msgs::PlanningScene plan_scene(std::string name_scene);