#include <ros/ros.h>
#include <behaviortree_cpp/bt_factory.h>
#include <explain_bt/ExplainableBTController.h>

// Define a Dummy Action Node for testing purposes.
class DummyAction : public BT::StatefulActionNode
{
public:
  DummyAction(const std::string &name, const BT::NodeConfig &config)
      : BT::StatefulActionNode(name, config)
  {
  }

  // Specify the input ports for this action node.
  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<bool>("succeed", false, "Whether to succeed or fail"),
        BT::InputPort<int>("msec", 0, "Amount of milliseconds to sleep")};
  }

  // Method invoked when the action starts.
  BT::NodeStatus onStart() override
  {
    bool succeed;
    int msec;

    getInput("succeed", succeed);
    getInput("msec", msec);

    if (succeed)
    {
      return_status_ = BT::NodeStatus::SUCCESS;
    }
    else
    {
      return_status_ = BT::NodeStatus::FAILURE;
    }

    if (msec <= 0)
    {
      // If no need to run for a specific duration, return the result immediately.
      return return_status_;
    }
    else
    {
      // Set a deadline for the action to run and return RUNNING.
      deadline_ = std::chrono::system_clock::now() + std::chrono::milliseconds(msec);
      return BT::NodeStatus::RUNNING;
    }
  }

  // Method invoked while the action is in the RUNNING state.
  BT::NodeStatus onRunning() override
  {
    if (std::chrono::system_clock::now() >= deadline_)
    {
      return return_status_;
    }
    else
    {
      return BT::NodeStatus::RUNNING;
    }
  }

  void onHalted() override
  {
    // Nothing to do when the action is halted.
  }

private:
  std::chrono::system_clock::time_point deadline_;
  BT::NodeStatus return_status_;
};

int main(int argc, char **argv)
{
  // Initialize ROS.
  ros::init(argc, argv, "example_bt");
  ros::NodeHandle nh;

  // Create a Behavior Tree Factory.
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<DummyAction>("DummyAction");

  // Create a behavior tree from an XML file.
  std::string xml_filename;
  nh.getParam("behavior_tree_xml", xml_filename);
  auto tree = factory.createTreeFromFile(xml_filename);

  // Create an Explainable Behavior Tree Controller.
  XBT::ExplainableBTController controller(tree, nh);
  controller.run();

  return 0;
}
