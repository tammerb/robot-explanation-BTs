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
