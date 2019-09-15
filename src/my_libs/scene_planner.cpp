#include "scene_planner.hpp"


// Constants
static const std::string PACKAGE_NAME = "panda_controller";
static const std::string DIRECTORY_NAME = "scenes";



// PRIVATE FUNCTIONS:
moveit_msgs::CollisionObject create_collision_object(const Json::Value &object,
                                                     const std::string &object_id);
moveit_msgs::ObjectColor create_object_color(const Json::Value &object,
                                             const std::string &object_id);



// PUBLIC FUNCTIONS:
moveit_msgs::PlanningScene plan_scene(std::string NAME_SCENE) {
    namespace fs = boost::filesystem;

    std::set<std::string> objects_id_set;  // To check if the object id readed is valid
    std::vector<moveit_msgs::ObjectColor> objects_color;  // Colors of the objects in the scene
    moveit_msgs::PlanningScene planning_scene;            // Planning scene


    // Get scene file path and check if it's correct
    fs::path scene_file =
        (ros::package::getPath(PACKAGE_NAME) + "/" + DIRECTORY_NAME + "/" + NAME_SCENE + ".json");
    if (!fs::exists(scene_file) || !fs::is_regular_file(scene_file)) {
        throw planning_error(std::string(scene_file.c_str()) + " does not exist or isn't a file");
    }


    // Create input file stream and check if it can be opened
    std::ifstream file_stream(scene_file.string(), std::ifstream::binary);
    if (!file_stream) {
        throw planning_error("could not open file " + std::string(scene_file.c_str()));
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
        const bool id_already_used = objects_id_set.find(object_id) != objects_id_set.end();
        if (id_already_used) {
            ROS_ERROR("the object id is already used");
            continue;
        }


        try {
            // Create object
            auto collision_object = create_collision_object(object, object_id);

            // Add object in the word of planning scene
            planning_scene.world.collision_objects.push_back(collision_object);

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


    // add colors to objects in the scene
    planning_scene.object_colors = objects_color;

    // Set that the planning scene is different
    planning_scene.is_diff = true;


    ROS_INFO("PLANNING SCENE DONE");
    return std::move(planning_scene);
}



// IMPLEMENTATIONS:
// Extract fron Json structure information and then create the collision object to return
moveit_msgs::CollisionObject create_collision_object(const Json::Value &object,
                                                     const std::string &object_id) {
    // Extract fields
    const auto &type = object["type"].asString();
    const auto &dimensions = object["dimensions"];
    const auto &position = object["position"];
    const auto &orientation = object["orientation"];

    
    // Check correct fields:
    if (type.empty() || dimensions.empty() || position.empty() || orientation.empty()) {
        throw json_field_error("missing json field");
    }


    // Create collision object to return
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "world";  // move_group.getPlanningFrame();
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
        throw collision_object_creation_error("the type specified in json is not valid");
    }


    // Define a pose (specified relative to frame_id)
    geometry_msgs::Pose object_pose;
    object_pose.orientation.w = orientation["w"].asDouble();
    object_pose.orientation.x = orientation["x"].asDouble();
    object_pose.orientation.y = orientation["y"].asDouble();
    object_pose.orientation.z = orientation["z"].asDouble();


    // Set potision information
    // Note: MoveIt! planning scene expects the center of the object as position.
    object_pose.position.x = position["x"].asDouble();  // + primitive.dimensions[0] / 2.0;
    object_pose.position.y = position["y"].asDouble();  // + primitive.dimensions[1] / 2.0;
    object_pose.position.z = position["z"].asDouble();  // + primitive.dimensions[2] / 2.0;


    // Push information in collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(object_pose);
    collision_object.operation = collision_object.ADD;


    return std::move(collision_object);
}



// Extract fron Json structure color information and then create the colors vector to return
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