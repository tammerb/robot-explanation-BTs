#include "ur5e_nodes.h"
#include "countdown.h"
#include <moveit/move_group_interface/move_group_interface.h>

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
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

    move_group_interface.setMaxVelocityScalingFactor(1.0);
    move_group_interface.setMaxAccelerationScalingFactor(1.0);
    ROS_INFO("Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

    geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
    geometry_msgs::Pose target_pose = current_pose.pose;

    target_pose.position.z = 0.1;

    move_group_interface.setPoseTarget(target_pose);

    bool debug = true;
    if (move_group_interface.move() == 1 || debug){
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

}
