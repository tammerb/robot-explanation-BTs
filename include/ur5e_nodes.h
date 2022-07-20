#ifndef SIMPLE_BT_NODES_H
#define SIMPLE_BT_NODES_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace UR5eNodes
{

// BT::NodeStatus ConditionNode();

class GripperInterface
{
  public:
    GripperInterface() : _opened(true)
    {
    }

    BT::NodeStatus open();

    BT::NodeStatus close();

    BT::NodeStatus grip();

  private:
    bool _opened;
};

class GoHome : public BT::SyncActionNode
{
  public:
    GoHome(const std::string& name) :
        BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
};

class GoVertical : public BT::SyncActionNode
{
  public:
    GoVertical(const std::string& name) :
        BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
};

class MoveToPose : public BT::SyncActionNode
{
  public:
    MoveToPose(const std::string& name) :
        BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
};

class TranslateToPose : public BT::SyncActionNode
{
  public:
    TranslateToPose(const std::string& name) :
        BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
};

class BadMove : public BT::SyncActionNode
{
  public:
    BadMove(const std::string& name) :
        BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
};

class Pick : public BT::SyncActionNode
{
  public:
    Pick(const std::string& name) :
        BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
    static GripperInterface grip_singleton;

//  factory.registerSimpleCondition("ConditionNode", std::bind(ConditionNode));
    factory.registerSimpleAction("OpenGripper", std::bind(&GripperInterface::open, &grip_singleton));
    factory.registerSimpleAction("CloseGripper", std::bind(&GripperInterface::close, &grip_singleton));
    factory.registerNodeType<GoHome>("GoHome");
    factory.registerNodeType<GoVertical>("GoVertical");
    factory.registerNodeType<MoveToPose>("MoveToPose");
    factory.registerNodeType<TranslateToPose>("TranslateToPose");
    factory.registerNodeType<BadMove>("BadMove");
 //   factory.registerNodeType<Pick>("Pick");
}

} // end namespace

#endif   // SIMPLE_BT_NODES_H
