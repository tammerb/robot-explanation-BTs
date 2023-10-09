#ifndef EXPLAINABLE_BT_CONTROLLER_H
#define EXPLAINABLE_BT_CONTROLLER_H

#include <ros/ros.h>
#include <explain_bt/ExplainableBT.h>
#include <explain_bt/Explain.h>
#include <explain_bt/Explanations.h>
#include <std_srvs/Empty.h>

namespace XBT
{

    class ExplainableBTController
    {
    public:
        ExplainableBTController(BT::Tree &tree, ros::NodeHandle &nh);
        void run();

    private:
        ros::NodeHandle &nh_;
        std::shared_ptr<XBT::ExplainableBT> xbt_;
        bool running_flag_;
        bool start_flag_;
        ros::ServiceServer start_tree_service_;
        ros::ServiceServer stop_tree_service_;
        ros::ServiceServer reset_tree_service_;
        ros::ServiceServer explain_service_;
        ros::Publisher explanations_pub_;

        bool startTreeCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool stopTreeCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool resetTreeCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool explainCallback(explain_bt::Explain::Request &req, explain_bt::Explain::Response &res);
        void publishExplanations();
    };

} // namespace XBT

#endif // EXPLAINABLE_BT_CONTROLLER_H