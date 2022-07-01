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

BT::NodeStatus GoHome::tick()
{
    std::cout << "GoHome: " << this->name() << std::endl;

    // setup the MoveGroupInterface
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
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
    const std::vector<std::string> named_targets = move_group_interface.getNamedTargets();
    std::map<std::string, double> positions = move_group_interface.getNamedTargetValues(named_targets[1]);

    move_group_interface.setJointValueTarget(positions);

    if (move_group_interface.move() == 1){
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

}
