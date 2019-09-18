#include <gazebo_msgs/SetModelState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

static const std::string PLANNING_GROUP = "panda_arm";
static const double PANDA_ARM_TO_HAND_OFFSET = 0.12;
static const double PANDA_HAND_TO_FINGER_OFFSET = 0.04;
ros::Publisher gazebo_model_state_pub;
robot_model::RobotModelPtr kinematic_model;
robot_state::RobotStatePtr kinematic_state;

void jointStatesCallback(const sensor_msgs::JointState &joint_states_current);



int main(int argc, char **argv) {
    // Get name
    // const std::string NAME = std::string(argv[0]) + "_node";

    // Setup ROS
    ros::init(argc, argv, "gazebo_scene");
    ros::NodeHandle node_handle;


    // Get Robot Model and Robot State
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    kinematic_model = robot_model_loader.getModel();
    kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));

    // Create Publisher
    gazebo_model_state_pub =
        node_handle.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);



    // Define a pose (specified relative to frame_id)
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;


    // Set potision information
    // Note: MoveIt! planning scene expects the center of the object as position.
    pose.position.x = 1;
    pose.position.y = 1;
    pose.position.z = 1;



    gazebo_msgs::ModelState model_state;
    // This string results from the spawn_urdf call in the box.launch file argument: -model box
    model_state.model_name = std::string("box");
    model_state.pose = pose;
    model_state.reference_frame = std::string("world");


    gazebo_model_state_pub.publish(model_state);

    ros::spin();
    return 0;
}
