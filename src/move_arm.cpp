// C++
#include <boost/scoped_ptr.hpp>

// ROS
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>



// Constants
static const std::string NAME = "move_arm";
static const std::string PLANNING_GROUP = "panda_arm";



int main(int argc, char **argv) {
    //**** SETTINGS ****
    // ROS SETUP
    ROS_INFO_STREAM("START " << NAME.c_str());
    ros::init(argc, argv, NAME);
    ros::NodeHandle node_handle("~");
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
    // Get the name of planning plugin to load from ROS parameter server
    std::string planner_plugin_name;
    if (!node_handle.getParam("planning_plugin", planner_plugin_name)) {
        ROS_FATAL_STREAM("Read from Ros parameter server Error: Check your launch file");
    }

    // Create a pointer to the Loader Plugin class to be created
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>>
        planner_plugin_loader;

    try {
        // Create a plannerLoader to load the planner by name using the ROS plugin library
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
            "moveit_core", "planning_interface::PlannerManager"));

    } catch (pluginlib::PluginlibException &e) {
        ROS_FATAL_STREAM("Planner plugin loader Error: " << e.what());
    }

    // Create a pointer to the Planner Manager to be created
    planning_interface::PlannerManagerPtr planner_instance;

    try {
        // Use the planner_plugin_loader to create a instance of Planner Manger
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));

        // Initialize Planner
        if (!planner_instance->initialize(robot_model, node_handle.getNamespace())) {
            ROS_FATAL_STREAM("Could not initialize planner instance");
        }
        ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");

    } catch (pluginlib::PluginlibException &ex) {
        // Get the planner classes available to view in error stream
        const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (std::size_t i = 0; i < classes.size(); ++i) {
            ss << "- " << classes[i] << "\n";
        }
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "\n--> "
                                                             << ex.what() << "\n--> "
                                                             << "Available plugins:\n"
                                                             << ss.str());
    }


    //** VISUALIZATION SETUP
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.loadRobotStatePub("/display_robot_state");
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();  // clear all old markers
    visual_tools.trigger();

    // Remote control that allows users to step through a high level script via buttons in RViz
    visual_tools.loadRemoteControl();

    /* Batch publishing is used to reduce the number of messages being sent to RViz for large
     * visualizations */
    visual_tools.trigger();

    // We can also use visual_tools to wait for user input
    visual_tools.prompt("Press 'next' 1");



    //**** START GOAL PLANNING ****
    // We will now create a motion plan request for the arm of the Panda
    // specifying the desired pose of the end-effector as input.
    // visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(),
    //                                rviz_visual_tools::PINK);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' 2");



    //**** MOTION PLANNING: POSE GOAL ****

    //** CREATE PLAINNING CONTEXT
    planning_interface::MotionPlanResponse planning_response;
    planning_interface::MotionPlanRequest planning_request;

    // Create a pose msg with goal values
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "panda_link0";
    pose.pose.position.x = 0.3;
    pose.pose.position.y = 0.4;
    pose.pose.position.z = 0.75;
    pose.pose.orientation.w = 1.0;

    // Set the tollerance of 0.01 m for position and 0.01 rad for orientation
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    // Create Constraints msg for goal pose
    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(
        "panda_link8", pose, tolerance_pose, tolerance_angle);

    // Set planning group and goal
    planning_request.group_name = PLANNING_GROUP;
    planning_request.goal_constraints.push_back(pose_goal);

    // Create planningContext (planningScene + motionPlanRequest) and solve it
    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(
        planning_scene, planning_request, planning_response.error_code_);
    context->solve(planning_response);
    if (planning_response.error_code_.val != planning_response.error_code_.SUCCESS) {
        ROS_ERROR("Planning Context Error: Could not compute plan successfully");
        return 0;
    }


    //** VISUALIZE THE RESULT
    // Create the publisher to send DisplayTrajectory msgs
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>(
        "/move_group/display_planned_path", 1, true);

    // Get the response of Motion Plan
    moveit_msgs::MotionPlanResponse response;
    planning_response.getMessage(response);

    // Create the DisplayTectory msg to
    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);

    // View the trajectory
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();

    // Publish the trajectory msg
    display_publisher.publish(display_trajectory);

    // Update the state in the planning_scene with the final state of the last plan
    robot_state->setJointGroupPositions(
        joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    planning_scene->setCurrentState(*robot_state.get());

    // Display the goal state
    visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(),
                                   rviz_visual_tools::GREEN);
    visual_tools.publishAxisLabeled(pose.pose, "goal_1");
    // visual_tools.publishText("text_pose", "Pose Goal (1)", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // We can also use visual_tools to wait for user input
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan myplan;

    // Construct a move_group plan from the planned trajectory
    myplan.trajectory_ = response.trajectory;
    move_group.execute(myplan);

    // Finish
    ros::shutdown();
    return 0;
}
