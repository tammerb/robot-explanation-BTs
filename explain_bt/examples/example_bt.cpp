#include <ros/ros.h>
#include <behaviortree_cpp/bt_factory.h>
#include <explain_bt/ExplainableBTController.h>

using namespace BT;
using namespace std::chrono;

// Example of Asynchronous node that uses StatefulActionNode as base class
class DummyAction : public BT::StatefulActionNode
{
  public:
    DummyAction(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
      // amount of milliseconds that we want to sleep
      return{ InputPort<bool>("succeed"), BT::InputPort<int>("msec") };
    }

    NodeStatus onStart() override
    {
      bool succeed = false;
      int msec = 0;
      getInput("succeed", succeed);
      getInput("msec", msec);

      if (succeed)
      {
        return_status_ = NodeStatus::SUCCESS;
      }
      else
      {
        return_status_ = NodeStatus::FAILURE;
      }

      if( msec <= 0 ) {
        // No need to go into the RUNNING state
        return return_status_;
      }
      else {
        // once the deadline is reached, we will return SUCCESS.
        deadline_ = system_clock::now() + milliseconds(msec);
        return NodeStatus::RUNNING;
      }
    }

    /// method invoked by an action in the RUNNING state.
    NodeStatus onRunning() override
    {
      if ( system_clock::now() >= deadline_ ) {
        return return_status_;
      }
      else {
        return NodeStatus::RUNNING;
      }
    }

    void onHalted() override
    {
      // nothing to do here...
      std::cout << "SleepNode interrupted" << std::endl;
    }

  private:
    system_clock::time_point deadline_;
    BT::NodeStatus return_status_;
};

std::string get_xml_filename(ros::NodeHandle &nh, std::string param)
{
    // get xml filename from ros parameter server
    std::string xml_filename;
    nh.getParam(param, xml_filename);
    ROS_INFO("Loading XML : %s", xml_filename.c_str());
    return xml_filename;
}

int main(int argc, char **argv)
{
    // setup ros
    ros::init(argc, argv, "example_bt");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // create bt factory
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<DummyAction>("DummyAction");

    // create tree from xml files
    std::string xml_filename = get_xml_filename(nh, "/behavior_tree_xml");
    auto tree = factory.createTreeFromFile(xml_filename);
    
    // create explainable bt controller
    XBT::ExplainableBTController controller(tree, nh);
    controller.run();

    return 0;
}