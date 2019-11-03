#include "arm_jobs.hpp"

// Constants
static const std::string PACKAGE_NAME = "panda_simulation";
static const std::string DIRECTORY_NAME = "scenes";


// PRIVATE FUNCTIONS:
moveit_msgs::CollisionObject
create_collision_object(const Json::Value &object,
                        const std::string &object_id);
moveit_msgs::ObjectColor create_object_color(const Json::Value &object,
                                             const std::string &object_id);



// PUBLIC FUNCTIONS:
namespace arm_jobs {

void setGripper(trajectory_msgs::JointTrajectory &posture, bool open) {
    // Add both finger joints of panda robot
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    // Set them as open, wide enough for the object to fit
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = open ? 0.04 : 0.00;
    posture.points[0].positions[1] = open ? 0.04 : 0.00;
    posture.points[0].time_from_start = ros::Duration(0.5);
}


void pick(moveit::planning_interface::MoveGroupInterface &move_group) {
    // Create a vector of grasps to be attempted, currently only creating single
    // grasp. This is essentially useful when using a grasp generator to
    // generate and test multiple grasps.
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);


    //** Setting grasp pose
    grasps[0].grasp_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.4;
    grasps[0].grasp_pose.pose.position.y = 0;
    grasps[0].grasp_pose.pose.position.z = 0.5;


    //** Setting pre-grasp approach
    // Defined with respect to frame_id
    grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
    // Direction is set as positive x axis
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;


    //** Setting post-grasp retreat
    // Defined with respect to frame_id
    grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
    // Direction is set as positive z axis
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;


    // Setting posture of eef before grasp
    setGripper(grasps[0].pre_grasp_posture, 1);
    // Setting posture of eef during grasp
    setGripper(grasps[0].grasp_posture, 0);


    // Set support surface as table1
    move_group.setSupportSurfaceName("table1");
    // Call pick to pick up the object using the grasps given
    move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface &group) {
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting place location pose
    place_location[0].place_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, M_PI / 2);
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    // While placing it is the exact location of the center of the object
    place_location[0].place_pose.pose.position.x = 0.0;
    place_location[0].place_pose.pose.position.y = 0.5;
    place_location[0].place_pose.pose.position.z = 0.5;


    // Setting pre-place approach
    // Defined with respect to frame_id
    place_location[0].pre_place_approach.direction.header.frame_id =
        "panda_link0";
    // Direction is set as negative z axis
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // Defined with respect to frame_id
    place_location[0].post_place_retreat.direction.header.frame_id =
        "panda_link0";
    // Direction is set as negative y axis
    place_location[0].post_place_retreat.direction.vector.y = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;


    // Setting posture of eef after placing object
    setGripper(place_location[0].post_place_posture, 1);


    // Set support surface as table2
    group.setSupportSurfaceName("table2");
    // Call place to place the object using the place locations given
    group.place("object", place_location);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface
                             &planning_scene_interface,
                         const std::string &SCENE_NAME_FILE) {
    namespace fs = boost::filesystem;

    std::set<std::string>
        objects_id_set;  // To check if the object id readed is valid
    std::vector<moveit_msgs::ObjectColor>
        objects_color;  // Colors of the objects in the scene
    std::vector<moveit_msgs::CollisionObject> collision_objects;


    // Get scene file path and check if it's correct
    fs::path scene_file = (ros::package::getPath(PACKAGE_NAME) + "/" +
                           DIRECTORY_NAME + "/" + SCENE_NAME_FILE + ".json");
    if (!fs::exists(scene_file) || !fs::is_regular_file(scene_file)) {
        throw planning_error(std::string(scene_file.c_str()) +
                             " does not exist or isn't a file");
    }


    // Create input file stream and check if it can be opened
    std::ifstream file_stream(scene_file.string(), std::ifstream::binary);
    if (!file_stream) {
        throw planning_error("could not open file " +
                             std::string(scene_file.c_str()));
    }


    // Extract json root
    Json::Value root;
    file_stream >> root;

    // Extract scene name
    const auto &scene_name = root["name"];
    ROS_INFO_STREAM("Create Scene: " << scene_name << " that contains:");

    // Extract elements from array of objects and publish it
    for (const auto &object : root["objects"]) {
        const auto &object_id = object["id"].asString();
        ROS_INFO_STREAM("--> Object: " << object_id);


        // Check if the object id is valid
        const bool id_already_used =
            objects_id_set.find(object_id) != objects_id_set.end();
        if (id_already_used) {
            ROS_ERROR("the object id is already used");
            continue;
        }


        try {
            // Create object
            auto collision_object = create_collision_object(object, object_id);

            // Add object in collision objects vector
            collision_objects.push_back(collision_object);

            // Create color object
            auto object_color = create_object_color(object, object_id);

            // Add object color to set
            objects_color.push_back(object_color);

            // Save object id in a set
            objects_id_set.insert(object_id);

        } catch (const collision_object_creation_error &e1) {
            ROS_ERROR_STREAM(e1.what());
            ROS_INFO("----> Object skipped");

        } catch (const Json::LogicError &e2) {
            ROS_ERROR_STREAM("Json Value Error: " << e2.what());
            ROS_INFO("----> Object skipped");

        } catch (const json_field_error &e3) {
            ROS_ERROR_STREAM("Json Field Error: " << e3.what());
            ROS_INFO("----> Object skipped");
        }
    }

    // Add Collision Objects in the scene
    planning_scene_interface.applyCollisionObjects(collision_objects, objects_color);
}

}  // namespace arm_jobs


// IMPLEMENTATIONS:
// Extract fron Json structure information and then create the collision object
// to return
moveit_msgs::CollisionObject
create_collision_object(const Json::Value &object,
                        const std::string &object_id) {
    // Extract fields
    const auto &type = object["type"].asString();
    const auto &dimensions = object["dimensions"];
    const auto &position = object["position"];
    const auto &orientation = object["orientation"];


    // Check correct fields:
    if (type.empty() || dimensions.empty() || position.empty() ||
        orientation.empty()) {
        throw json_field_error("missing json field");
    }


    // Create collision object to return
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "panda_link0";
    collision_object.id = object_id;


    // Define object in the world
    shape_msgs::SolidPrimitive primitive;
    if (type == "box") {
        // Set dimension information
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = dimensions["x"].asDouble();
        primitive.dimensions[1] = dimensions["y"].asDouble();
        primitive.dimensions[2] = dimensions["z"].asDouble();

    } else if (type == "sphere") {
        primitive.type = primitive.SPHERE;
        primitive.dimensions.resize(2);
        primitive.dimensions[0] = dimensions["r"].asDouble();

    } else if (type == "cylinder") {
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[0] = dimensions["h"].asDouble();
        primitive.dimensions[1] = dimensions["r"].asDouble();

    } else if (type == "cone") {
        primitive.type = primitive.CONE;
        primitive.dimensions.resize(2);
        primitive.dimensions[0] = dimensions["h"].asDouble();
        primitive.dimensions[1] = dimensions["r"].asDouble();

    } else {
        throw collision_object_creation_error(
            "the type specified in json is not valid");
    }


    // Define a pose (specified relative to frame_id)
    geometry_msgs::Pose object_pose;
    object_pose.orientation.w = orientation["w"].asDouble();
    object_pose.orientation.x = orientation["x"].asDouble();
    object_pose.orientation.y = orientation["y"].asDouble();
    object_pose.orientation.z = orientation["z"].asDouble();


    // Set potision information
    // Note: MoveIt! planning scene expects the center of the object as
    // position.
    object_pose.position.x = position["x"].asDouble();
    object_pose.position.y = position["y"].asDouble();
    object_pose.position.z = position["z"].asDouble();


    // Push information in collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(object_pose);
    collision_object.operation = collision_object.ADD;


    return std::move(collision_object);
}



// Extract fron Json structure color information and then create the colors
// vector to return
moveit_msgs::ObjectColor create_object_color(const Json::Value &object,
                                             const std::string &object_id) {
    const auto &color = object["color"];

    // Check correct fields:
    if (color.empty()) {
        throw json_field_error("missing 'color' json field");
    }


    // Set informations
    moveit_msgs::ObjectColor c;
    c.id = object_id.c_str();
    c.color.r = color["r"].asFloat();
    c.color.g = color["g"].asFloat();
    c.color.b = color["b"].asFloat();
    c.color.a = color["a"].asFloat();

    return std::move(c);
}