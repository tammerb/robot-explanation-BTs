#include "ur5e_nodes.h"
#include "countdown.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time

static const bool debug = false;
const double tau = 2 * M_PI;


BT_REGISTER_NODES(factory)
{
    UR5eNodes::RegisterNodes(factory);
}

namespace UR5eNodes
{

BT::NodeStatus GripperInterface::open()
{
    std::cout << "GripperInterface::open" << std::endl;

    static const std::string PLANNING_GROUP = "gripper";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    move_group_interface.setMaxVelocityScalingFactor(1.0);
    move_group_interface.setMaxAccelerationScalingFactor(1.0);
    const std::vector<std::string> named_targets = move_group_interface.getNamedTargets();
    std::map<std::string, double> positions = move_group_interface.getNamedTargetValues(named_targets[0]);

    move_group_interface.setJointValueTarget(positions);

    if (move_group_interface.move() == 1){
        _opened = true;
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }   
}

BT::NodeStatus GripperInterface::close()
{
    std::cout << "GripperInterface::close" << std::endl;

    static const std::string PLANNING_GROUP = "gripper";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    move_group_interface.setMaxVelocityScalingFactor(1.0);
    move_group_interface.setMaxAccelerationScalingFactor(1.0);
    const std::vector<std::string> named_targets = move_group_interface.getNamedTargets();
    std::map<std::string, double> positions = move_group_interface.getNamedTargetValues(named_targets[1]);

    move_group_interface.setJointValueTarget(positions);

    if (move_group_interface.move() == 1){
        _opened = false;
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus GripperInterface::grip()
{
    std::cout << "GripperInterface::grip" << std::endl;

    static const std::string PLANNING_GROUP = "gripper";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    move_group_interface.setMaxVelocityScalingFactor(1.0);
    move_group_interface.setMaxAccelerationScalingFactor(1.0);
    const std::vector<std::string> named_targets = move_group_interface.getNamedTargets();
    std::map<std::string, double> positions = move_group_interface.getNamedTargetValues(named_targets[2]);

    move_group_interface.setJointValueTarget(positions);

    if (move_group_interface.move() == 1){
        _opened = false;
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus GoHome::tick()
{
    std::cout << "GoHome: " << this->name() << std::endl;

    // setup the MoveGroupInterface
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    move_group_interface.setMaxVelocityScalingFactor(1.0);
    move_group_interface.setMaxAccelerationScalingFactor(1.0);
    const std::vector<std::string> named_targets = move_group_interface.getNamedTargets();
    std::map<std::string, double> positions = move_group_interface.getNamedTargetValues(named_targets[0]);

    move_group_interface.setJointValueTarget(positions);

    if (move_group_interface.move() == 1){
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }    
}

BT::NodeStatus GoVertical::tick()
{
    std::cout << "GoVertical: " << this->name() << std::endl;
    // setup the MoveGroupInterface
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    move_group_interface.setMaxVelocityScalingFactor(1.0);
    move_group_interface.setMaxAccelerationScalingFactor(1.0);
    const std::vector<std::string> named_targets = move_group_interface.getNamedTargets();
    std::map<std::string, double> positions = move_group_interface.getNamedTargetValues(named_targets[1]);

    move_group_interface.setJointValueTarget(positions);

    if (move_group_interface.move() == 1){
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus MoveToPose::tick()
{
    std::cout << "MoveToPose: " << this->name() << std::endl;
    // setup the MoveGroupInterface
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose current_pose = move_group_interface.getCurrentPose().pose;
    waypoints.push_back(current_pose);
    geometry_msgs::Pose target_pose1 = current_pose;
    target_pose1.position.x = -0.5;
    target_pose1.position.y = 0;
    target_pose1.position.z = 0.3;
    waypoints.push_back(target_pose1);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    // move_group_interface.execute(trajectory);

    
/*  move_group_interface.setMaxVelocityScalingFactor(1.0);
    move_group_interface.setMaxAccelerationScalingFactor(1.0);
    ROS_INFO("Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

    geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
    geometry_msgs::Pose target_pose = current_pose.pose;

    ROS_INFO("Current Pose x, y, z: %f, %f, %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
    target_pose.position.z = 0.298791;

    move_group_interface.setPoseTarget(target_pose);
*/
    if (debug || move_group_interface.execute(trajectory) == 1){
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus TranslateToPose::tick()
{
    std::cout << "TranslateToPose: " << this->name() << std::endl;
    // setup the MoveGroupInterface
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    std::vector<geometry_msgs::Pose> new_waypoints;

    geometry_msgs::Pose current_pose_t = move_group_interface.getCurrentPose().pose;
    new_waypoints.push_back(current_pose_t);
    geometry_msgs::Pose target_pose2 = current_pose_t;
    target_pose2.position.x -= 0.5;
    target_pose2.position.y += 1.0;
    target_pose2.position.z += 0.2;

    new_waypoints.push_back(target_pose2);

    moveit_msgs::RobotTrajectory trajectory2;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface.computeCartesianPath(new_waypoints, eef_step, jump_threshold, trajectory2);

    if (debug || move_group_interface.execute(trajectory2) == 1){
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus BadMove::tick()
{
    std::cout << "BadMove: " << this->name() << std::endl;
    // setup the MoveGroupInterface
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    std::vector<geometry_msgs::Pose> new_waypoints;

    geometry_msgs::Pose current_pose_t = move_group_interface.getCurrentPose().pose;
    new_waypoints.push_back(current_pose_t);
    geometry_msgs::Pose target_pose2 = current_pose_t;
    target_pose2.position.x -= 50;
    target_pose2.position.y += 1.0;
    target_pose2.position.z += 50;

    new_waypoints.push_back(target_pose2);

    moveit_msgs::RobotTrajectory trajectory2;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface.computeCartesianPath(new_waypoints, eef_step, jump_threshold, trajectory2);

    if (fraction < 1.0) {
        return BT::NodeStatus::FAILURE;
    }

    if (debug || move_group_interface.execute(trajectory2) == 1){
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}
/*
BT::NodeStatus Pick::tick()
{
    std::cout << "Pick: " << this->name() << std::endl;
    
    // setup the MoveGroupInterface
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // Setting grasp pose
    // ++++++++++++++++++++++
    // This is the pose of panda_link8. |br|
    // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
    // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
    // transform from `"panda_link8"` to the palm of the end effector.
    grasps[0].grasp_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(-tau / 4, -tau / 8, -tau / 4);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = -0.415;
    grasps[0].grasp_pose.pose.position.y = 0;
    grasps[0].grasp_pose.pose.position.z = 0.5;

    // Setting pre-grasp approach
    // ++++++++++++++++++++++++++
    // Defined with respect to frame_id //
    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    // Direction is set as positive x axis //
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    // Defined with respect to frame_id //
    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    // Direction is set as positive z axis //
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    // Setting posture of eef before grasp
    // +++++++++++++++++++++++++++++++++++
    openGripper(grasps[0].pre_grasp_posture);
    // END_SUB_TUTORIAL

    // BEGIN_SUB_TUTORIAL pick2
    // Setting posture of eef during grasp
    // +++++++++++++++++++++++++++++++++++
    closedGripper(grasps[0].grasp_posture);
    // END_SUB_TUTORIAL

    // BEGIN_SUB_TUTORIAL pick3
    // Set support surface as table1.
    move_group.setSupportSurfaceName("table1");
    // Call pick to pick up the object using the grasps given

    if (debug || move_group.pick("object", grasps) == 1){
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}
*/

}
