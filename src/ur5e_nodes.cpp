#include "ur5e_nodes.h"
#include "countdown.h"
#include <moveit/move_group_interface/move_group_interface.h>


// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
BT_REGISTER_NODES(factory)
{
    DummyNodes::RegisterNodes(factory);
}

namespace DummyNodes
{

BT::NodeStatus CheckBattery()
{
    countdown(2);
    static const std::string PLANNING_GROUP = "manipulator";
    ROS_INFO("PLANNING_GROUP = \"manipulator\"");
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    ROS_INFO("MoveGroupInterface class called on PLANNING_GROUP");
    std::cout << "[ Battery: OK ]" << std::endl;

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;
    move_group_interface.setPoseTarget(target_pose1);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool plan_success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    bool move_success = 0;

    if (plan_success) {
        move_success = (move_group_interface.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }

    return (plan_success && move_success) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus GripperInterface::open()
{
    _opened = true;
    countdown(2);
    std::cout << "GripperInterface::open" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GripperInterface::close()
{
    countdown(2);
    std::cout << "GripperInterface::close" << std::endl;
    _opened = false;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachObject::tick()
{
    countdown(2);
    std::cout << "ApproachObject: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SaySomething::tick()
{
    auto msg = getInput<std::string>("message");
    if (!msg)
    {
        throw BT::RuntimeError( "missing required input [message]: ", msg.error() );
    }

    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SaySomethingSimple(BT::TreeNode &self)
{
    auto msg = self.getInput<std::string>("message");
    if (!msg)
    {
        throw BT::RuntimeError( "missing required input [message]: ", msg.error() );
    }

    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

}
