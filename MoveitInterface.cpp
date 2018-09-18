#include <moveit/move_group_interface/move_group_interface.h>           // Move the Robot.
#include <moveit/planning_scene_interface/planning_scene_interface.h>   // Modify Robot's world.

#include <moveit_visual_tools/moveit_visual_tools.h>

const float OPEN_GRIPPER     = - 1.00;
const float CLOSE_GRIPPER    =   0.00;
const float DISTANCE_TO_PALM =   0.09; // Distance from last link (hand_link origin) to grapable area of the gripper.
const float GRIPPER_LENGTH   =   0.06; // From Palm to the gripper end.

float table_size_x    = 0.20;
float table_size_y    = 0.20;
float table_size_z    = 0.04;

float object_size_x   = 0.02;
float object_size_y   = 0.02;
float object_size_z   = 0.08;

float table_origin_x  = 0.40;
float table_origin_y  = 0.00;
float table_origin_z  = table_size_z / 2;

float object_origin_x = table_origin_x;
float object_origin_y = table_origin_y;
float object_origin_z = table_size_z + object_size_z / 2;

void openGripper(trajectory_msgs::JointTrajectory& posture) {
    /* Add Gripper Joints. */
    posture.joint_names.resize(6);
    posture.joint_names[0] = "gripper_right_joint";
    posture.joint_names[1] = "gripper_right_front_joint";
    posture.joint_names[2] = "gripper_right_connection_joint";
    posture.joint_names[3] = "gripper_left_joint";
    posture.joint_names[4] = "gripper_left_front_joint";
    posture.joint_names[5] = "gripper_left_connection_joint";

    /* Set them as open. */
    posture.points.resize(1);
    posture.points[0].positions.resize(6);
    posture.points[0].positions[0]    = - 1.00;
    posture.points[0].positions[1]    = - 1.00;
    posture.points[0].positions[2]    = - 1.00;
    posture.points[0].positions[3]    = - 1.00;
    posture.points[0].positions[4]    = - 1.00;
    posture.points[0].positions[5]    = - 1.00;
    posture.points[0].time_from_start = ros::Duration(5.0);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture) {
    /* Add Gripper Joints. */
    posture.joint_names.resize(6);
    posture.joint_names[0] = "gripper_right_joint";
    posture.joint_names[1] = "gripper_right_front_joint";
    posture.joint_names[2] = "gripper_right_connection_joint";
    posture.joint_names[3] = "gripper_left_joint";
    posture.joint_names[4] = "gripper_left_front_joint";
    posture.joint_names[5] = "gripper_left_connection_joint";

    /* Set them as closed. */
    posture.points.resize(1);
    posture.points[0].positions.resize(6);
    posture.points[0].positions[0]    =   0;
    posture.points[0].positions[1]    =   0;
    posture.points[0].positions[2]    =   0;
    posture.points[0].positions[3]    =   0;
    posture.points[0].positions[4]    =   0;
    posture.points[0].positions[5]    =   0;
    posture.points[0].time_from_start = ros::Duration(1.0);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group) {

    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // Position of the last link in that chain. (Connected to the End Effector.)
    grasps[0].grasp_pose.header.frame_id  = "root_link";
    grasps[0].grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    grasps[0].grasp_pose.pose.position.x = object_origin_x - DISTANCE_TO_PALM - object_size_x / 2;
    grasps[0].grasp_pose.pose.position.y = object_origin_y;
    grasps[0].grasp_pose.pose.position.z = object_origin_z;

    grasps[0].pre_grasp_approach.direction.header.frame_id = "root_link";
    grasps[0].pre_grasp_approach.direction.vector.x =   1.00;
    grasps[0].pre_grasp_approach.min_distance       =   GRIPPER_LENGTH * 1;
    grasps[0].pre_grasp_approach.desired_distance   =   GRIPPER_LENGTH * 1.2;

    grasps[0].post_grasp_retreat.direction.header.frame_id = "root_link";
    grasps[0].post_grasp_retreat.direction.vector.z =   1.00;
    grasps[0].post_grasp_retreat.min_distance       =   0.05;
    grasps[0].post_grasp_retreat.desired_distance   =   0.10;

    // Setting posture of Effector before grasp
    openGripper(grasps[0].pre_grasp_posture);
    // Setting posture of Effector during grasp
    closedGripper(grasps[0].grasp_posture);
    move_group.setSupportSurfaceName("table1");
    move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& group) {
    // Create a vector of placings to be attempted, currently only creating single place location.
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting place location pose
    place_location[0].place_pose.header.frame_id  = "root_link";
    place_location[0].place_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    place_location[0].place_pose.pose.position.x =   0.40;
    place_location[0].place_pose.pose.position.y =   0.00;
    place_location[0].place_pose.pose.position.z =   0.07;

    // Setting pre-place approach (with respect to frame_id)
    place_location[0].pre_place_approach.direction.header.frame_id = "root_link";
    place_location[0].pre_place_approach.direction.vector.z = - 1.00;
    place_location[0].pre_place_approach.min_distance       =   0.10;
    place_location[0].pre_place_approach.desired_distance   =   0.20;

    // Setting post-grasp retreat (with respect to frame_id)
    place_location[0].post_place_retreat.direction.header.frame_id = "root_link";
    place_location[0].post_place_retreat.direction.vector.x = - 1.00;
    place_location[0].post_place_retreat.min_distance       =   0.06;
    place_location[0].post_place_retreat.desired_distance   =   0.08;

    // Setting posture of eef after placing object
    openGripper(place_location[0].post_place_posture);

    // Set support surface as table1.
    group.setSupportSurfaceName("table1");
    // Call place to place the object using the place locations given.
    group.place("object", place_location);
}

/**
 *  Prints the current Robot's joint states.
 */
void printCurrentJointState(moveit::planning_interface::MoveGroupInterface& move_group) {

    moveit::core::RobotStatePtr                     current_state;
    std::vector<double>                             joint_group_positions;

    current_state = move_group.getCurrentState();
    current_state -> copyJointGroupPositions(move_group.getCurrentState() -> getJointModelGroup("boris_arm"), joint_group_positions);

    ROS_INFO("  angle1: %f", joint_group_positions[0]);   // base_rotation_joint
    ROS_INFO("  angle2: %f", joint_group_positions[1]);   // upper_arm_joint
    ROS_INFO("  angle3: %f", joint_group_positions[2]);   // lower_arm_joint
    ROS_INFO("  angle4: %f", joint_group_positions[3]);   // hand_joint
    ROS_INFO("  angle5: %f", joint_group_positions[4]);   // finger_joint

}

/**
 *  Prints the current Robot's Position Pose.
 */
void printCurrentPositionPose(moveit::planning_interface::MoveGroupInterface& move_group) {

    geometry_msgs::PoseStamped                      current_pose;
    std::vector<double>                             currentRPY;

    // Standard End Effector is the last link in that chain (finger_link).
    //move_group.setEndEffectorLink("gripper_left_link");

    current_pose = move_group.getCurrentPose();

    ROS_WARN("The end-effector is at pose:");
    ROS_INFO("  Position x: %f", current_pose.pose.position.x);
    ROS_INFO("  Position y: %f", current_pose.pose.position.y);
    ROS_INFO("  Position z: %f", current_pose.pose.position.z);
    /*
    ROS_INFO("  Orientation x: %f", current_pose.pose.orientation.x);
    ROS_INFO("  Orientation y: %f", current_pose.pose.orientation.y);
    ROS_INFO("  Orientation z: %f", current_pose.pose.orientation.z);
    ROS_INFO("  Orientation w: %f", current_pose.pose.orientation.w);
    */
    currentRPY = move_group.getCurrentRPY();
    ROS_INFO("  Roll : %f", currentRPY[0]);
    ROS_INFO("  Pitch: %f", currentRPY[1]);
    ROS_INFO("  Yaw  : %f", currentRPY[2]);

}

/**
 *  Prints the current Robot State.
 */
void printCurrentState(moveit::planning_interface::MoveGroupInterface& move_group) {

    printCurrentJointState(move_group);

    printCurrentPositionPose(move_group);

}

void addCollisionObjects() {

    moveit::planning_interface::PlanningSceneInterface  planning_scene_interface;   // Handle to the world.
    std::vector<moveit_msgs::CollisionObject>           collision_objects;          // Stores collision objects.

    // Define number of collision_objects.
    collision_objects.resize(2);

    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "root_link";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = table_size_x;
    collision_objects[0].primitives[0].dimensions[1] = table_size_y;
    collision_objects[0].primitives[0].dimensions[2] = table_size_z;
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = table_origin_x;
    collision_objects[0].primitive_poses[0].position.y = table_origin_y;
    collision_objects[0].primitive_poses[0].position.z = table_origin_z;
    collision_objects[0].operation = collision_objects[0].ADD;

    collision_objects[1].id = "object";
    collision_objects[1].header.frame_id = "root_link";
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = object_size_x;
    collision_objects[1].primitives[0].dimensions[1] = object_size_y;
    collision_objects[1].primitives[0].dimensions[2] = object_size_z;
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = object_origin_x;
    collision_objects[1].primitive_poses[0].position.y = object_origin_y;
    collision_objects[1].primitive_poses[0].position.z = object_origin_z;
    collision_objects[1].operation = collision_objects[1].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);

}

/**
 *  Plans and executes a plan by giving joint angle goals.
 *
 *  @param angle1 Goal for the base_rotation_joint
 *  @param angle2 Goal for the upper_arm_joint
 *  @param angle3 Goal for the lower_arm_joint
 *  @param angle4 Goal for the hand_joint
 *  @param angle5 Goal for the finger_joint
 */
void planJointStateGoalGripper(
    moveit::planning_interface::MoveGroupInterface& move_group,
    double angle) {

    moveit::core::RobotStatePtr                             current_state;
    moveit::planning_interface::MoveGroupInterface::Plan    my_plan;
    std::vector<double>                                     joint_group_positions;

    current_state = move_group.getCurrentState();
    current_state -> copyJointGroupPositions(move_group.getCurrentState() -> getJointModelGroup("boris_gripper"), joint_group_positions);
    
    joint_group_positions[0] = angle;
    joint_group_positions[1] = angle;
    joint_group_positions[2] = angle;
    joint_group_positions[3] = angle;
    joint_group_positions[4] = angle;
    joint_group_positions[5] = angle;
    move_group.setJointValueTarget(joint_group_positions);
    
    // plan(my_plam) and execute(my_plam), ALTERNATIVELY just move() to do both.
    if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        // Execute the planned path:
        move_group.execute(my_plan);
        ROS_INFO("Planning Done.");
    }
    else {
        ROS_INFO("Planning Failed.");
    }
    
}

/**
 *  Plans and executes a plan by giving joint angle goals.
 *
 *  @param angle1 Goal for the base_rotation_joint
 *  @param angle2 Goal for the upper_arm_joint
 *  @param angle3 Goal for the lower_arm_joint
 *  @param angle4 Goal for the hand_joint
 *  @param angle5 Goal for the finger_joint
 */
void planJointStateGoalArm(
    moveit::planning_interface::MoveGroupInterface& move_group,
    double angle1,
    double angle2,
    double angle3,
    double angle4,
    double angle5) {

    moveit::core::RobotStatePtr                             current_state;
    moveit::planning_interface::MoveGroupInterface::Plan    my_plan;
    std::vector<double>                                     joint_group_positions;

    current_state = move_group.getCurrentState();
    current_state -> copyJointGroupPositions(move_group.getCurrentState() -> getJointModelGroup("boris_arm"), joint_group_positions);
    
    joint_group_positions[0] = angle1;
    joint_group_positions[1] = angle2;
    joint_group_positions[2] = angle3;
    joint_group_positions[3] = angle4;
    joint_group_positions[4] = angle5;
    move_group.setJointValueTarget(joint_group_positions);
    
    // plan(my_plam) and execute(my_plam), ALTERNATIVELY just move() to do both.
    if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        // Execute the planned path:
        move_group.execute(my_plan);
        ROS_INFO("Planning Done.");
    }
    else {
        ROS_INFO("Planning Failed.");
    }
    
}

void planPositionPoseGoalArm(
    moveit::planning_interface::MoveGroupInterface& move_group,
    double x,
    double y,
    double z,
    double roll,
    double pitch,
    double yaw) {
    
    moveit::planning_interface::MoveGroupInterface::Plan    my_plan;
    geometry_msgs::Pose                                     target_pose;

    // Goal Pose for the end-effector.
    target_pose.position.x    =  x;
    target_pose.position.y    =  y;
    target_pose.position.z    =  z;
    target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    move_group.setPoseTarget(target_pose);

    if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        move_group.execute(my_plan);
        ROS_INFO("Planning Done.");
    }
    else {
        ROS_INFO("Planning Failed.");
    }

}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "moveit_interface_boris");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit_visual_tools::MoveItVisualTools visual_tools("root_link");
    moveit::planning_interface::MoveGroupInterface move_group("boris_arm");
    moveit::planning_interface::MoveGroupInterface move_group_gripper("boris_gripper");
    
    // Initial Position before starting:
    planJointStateGoalGripper(move_group_gripper, OPEN_GRIPPER);
    planPositionPoseGoalArm(move_group,
                            0.17,
                            0.00,
                            0.07,
                            0,
                            0,
                            0);
    addCollisionObjects();

    visual_tools.prompt("Press 'next' to start.");

    //pick(move_group);
    
    // Pre Grab Position:
    planPositionPoseGoalArm(move_group,
                            object_origin_x - DISTANCE_TO_PALM - object_size_x / 2 - GRIPPER_LENGTH * 1,
                            object_origin_y,
                            object_origin_z,
                            0,
                            0,
                            0);

    // Grasp Position:
    planPositionPoseGoalArm(move_group,
                            object_origin_x - DISTANCE_TO_PALM - object_size_x / 2,
                            object_origin_y,
                            object_origin_z,
                            0,
                            0,
                            0);

    // Post Grab Position.
    planPositionPoseGoalArm(move_group,
                            object_origin_x - DISTANCE_TO_PALM - object_size_x / 2,
                            object_origin_y,
                            object_origin_z + 0.05 * 1,
                            0,
                            0,
                            0);

    return 0;
}
