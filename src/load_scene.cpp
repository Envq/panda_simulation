#include "scene_planner.hpp"  //plan_scene()

#include <ros/ros.h>



// Constants
static const std::string NAME = "load_scene";



int main(int argc, char **argv) {
    // Setup ROS
    ROS_INFO_STREAM("START " << NAME.c_str());
    ros::init(argc, argv, NAME);
    ros::NodeHandle node_handle("~");  // private namespace
    ros::AsyncSpinner spinner(1);
    spinner.start();


    // Create a publisher and wait for subscribers
    ros::Publisher scene_publisher =
        node_handle.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    ros::WallDuration wall_time(0.5);
    ROS_INFO("Wait subscriber");
    while (scene_publisher.getNumSubscribers() < 1) {
        wall_time.sleep();
    }
    ROS_INFO("Subscriber is arrived");


    // Get the scene name from ROS parameter server
    std::string scene_name_file;
    if (!node_handle.getParam("scene_name_file", scene_name_file)) {
        ROS_FATAL_STREAM("Read from ROS parameter server Error: Check 'scene_name_file' field "
                         "in your launch file");
        return 0;
    }


    // plan scene and publish it
    try {
        auto planning_scene = plan_scene(scene_name_file);
        scene_publisher.publish(planning_scene);

    } catch (const planning_error &e) {
        ROS_ERROR_STREAM(e.what());
    }


    // Finish
    ros::shutdown();
    return 0;
}
