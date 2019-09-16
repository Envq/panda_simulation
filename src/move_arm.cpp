// C++
#include <boost/scoped_ptr.hpp>

// ROS
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/kinematic_constraints/utils.h>
// #include <moveit_msgs/DisplayTrajectory.h>
// #include <moveit_msgs/PlanningScene.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>



// Constants
static const std::string NAME = "move_arm";
static const std::string PLANNING_GROUP = "panda_arm";



int main(int argc, char **argv) {
    // ROS SETUP
    ROS_INFO_STREAM("START " << NAME.c_str());
    ros::init(argc, argv, NAME);
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();


    //** ROBOT INFORMATION SETUP
    // Create RobotModel to get robot model information
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    // Create RobotState to keep track of robot state (pose and planning group)
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));

    // Get JointModelGroup
    const robot_state::JointModelGroup *joint_model_group =
        robot_state->getJointModelGroup(PLANNING_GROUP);


    //** PLANNING SCENE SETUP
    // Create planningScene with current state of the world (including the robot)
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    // Set in planning scene the current state in a default position
    planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");


    //** PLANNING MOTION SETUP
    // Create a plannerLoader to load the planner by name using the ROS plugin library
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>>
        planner_plugin_loader;

    // Get the name of planning plugin to load from ROS parameter server
    std::string planner_plugin_name;
    if (!node_handle.getParam("planning_plugin", planner_plugin_name)) {
        ROS_FATAL_STREAM("Read from Ros parameter server Error: Check your launch file");
    }

    // Set the planning plugin from the ROS plugin library in the PlannerManager
    try {
        // CONTROLLA: in teoria posso evitare di istanziarlo e poi fare il reset
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
            "moveit_core", "planning_interface::PlannerManager"));

    } catch (pluginlib::PluginlibException &e) {
        ROS_FATAL_STREAM("Planning plugin loader Error: " << e.what());
    }

    // Create a PlannerManager instance to motion planning
    planning_interface::PlannerManagerPtr planner_instance;

    try {
        // CONTROLLA: in teoria posso evitare di istanziarlo e poi fare il reset
        // usa il plannerPluginLoader per restituire un istanza di PlannerManager per inizializzare
        // il puntatore
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));

        if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
            ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");

    } catch (pluginlib::PluginlibException &ex) {
        const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (std::size_t i = 0; i < classes.size(); ++i)
            ss << classes[i] << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name
                                                             << "': " << ex.what() << std::endl
                                                             << "Available plugins: " << ss.str());
    }



    // Finish
    ros::shutdown();
    return 0;
}
