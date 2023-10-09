#include <explain_bt/ExplainableBTController.h>

namespace XBT
{

  ExplainableBTController::ExplainableBTController(BT::Tree &tree, ros::NodeHandle &nh) : nh_(nh), running_flag_(false), start_flag_(false)
  {
    xbt_ = std::make_shared<XBT::ExplainableBT>(tree);

    // Create ROS services for starting, stopping, and resetting the behavior tree
    start_tree_service_ = nh.advertiseService("start_tree", &ExplainableBTController::startTreeCallback, this);
    stop_tree_service_ = nh.advertiseService("stop_tree", &ExplainableBTController::stopTreeCallback, this);
    reset_tree_service_ = nh.advertiseService("reset_tree", &ExplainableBTController::resetTreeCallback, this);

    // Create ROS service for explaining the behavior tree
    explain_service_ = nh.advertiseService("explain_tree", &ExplainableBTController::explainCallback, this);

    // Create ROS publisher for publishing explanations
    explanations_pub_ = nh.advertise<explain_bt::Explanations>("explanations", 1);
  }

  void ExplainableBTController::run()
  {
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
      if (running_flag_)
      {
        auto status = xbt_->status();
        if (status == BT::NodeStatus::RUNNING || start_flag_)
        {
          xbt_->tick();
          start_flag_ = false;
        }
      }
      publishExplanations();
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  // Start the behavior tree
  bool ExplainableBTController::startTreeCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    running_flag_ = true;
    start_flag_ = true;
    return true;
  }

  // Stop the behavior tree
  bool ExplainableBTController::stopTreeCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    running_flag_ = false;
    return true;
  }

  // Reset the behavior tree
  bool ExplainableBTController::resetTreeCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    xbt_->halt();
    return true;
  }

  // Explain the behavior tree
  bool ExplainableBTController::explainCallback(explain_bt::Explain::Request &req, explain_bt::Explain::Response &res)
  {
    uint8_t question = req.question;
    std::string answer;

    switch (question)
    {
    case explain_bt::ExplainRequest::WHAT_ARE_YOU_DOING:
      ROS_INFO_STREAM("Q: what are you doing?");
      answer = xbt_->handleWhatAreYouDoing();
      break;
    case explain_bt::ExplainRequest::WHY_ARE_YOU_DOING_THIS:
      ROS_INFO_STREAM("Q: why are you doing this?");
      answer = xbt_->handleWhyAreYouDoing();
      break;
    case explain_bt::ExplainRequest::WHAT_IS_YOUR_SUBGOAL:
      ROS_INFO_STREAM("Q: what is your subgoal?");
      answer = xbt_->handleWhatIsYourSubgoal();
      break;
    case explain_bt::ExplainRequest::HOW_DO_YOU_ACHIEVE_YOUR_SUBGOAL:
      ROS_INFO_STREAM("Q: how do you achieve your subgoal?");
      answer = xbt_->handleHowDoYouAchieveYourSubgoal();
      break;
    case explain_bt::ExplainRequest::WHAT_IS_YOUR_GOAL:
      ROS_INFO_STREAM("Q: what is your goal?");
      answer = xbt_->handleWhatIsYourGoal();
      break;
    case explain_bt::ExplainRequest::HOW_DO_YOU_ACHIEVE_YOUR_GOAL:
      ROS_INFO_STREAM("Q: how do you achieve your goal?");
      answer = xbt_->handleHowDoYouAchieveYourGoal();
      break;
    case explain_bt::ExplainRequest::WHAT_WENT_WRONG:
      ROS_INFO_STREAM("Q: what went wrong?");
      answer = xbt_->handleWhatWentWrong();
      break;
    case explain_bt::ExplainRequest::WHAT_IS_NEXT_ACTION_IF_SUCCESS:
      ROS_INFO_STREAM("Q: what is your next action if you succeed?");
      answer = xbt_->handleWhatIsNextActionIfSuccess();
      break;
    case explain_bt::ExplainRequest::WHAT_IS_NEXT_ACTION_IF_FAIL:
      ROS_INFO_STREAM("Q: what is your next action if you fail?");
      answer = xbt_->handleWhatIsNextActionIfFail();
      break;
    case explain_bt::ExplainRequest::WHAT_ARE_CURRENT_PRE_CONDITIONS:
      ROS_INFO_STREAM("Q: what are your current pre conditions?");
      answer = xbt_->handleWhatAreCurrentPreConditions();
      break;
    case explain_bt::ExplainRequest::WHAT_ARE_CURRENT_POST_CONDITIONS:
      ROS_INFO_STREAM("Q: what are your current post conditions?");
      answer = xbt_->handleWhatAreCurrentPostConditions();
      break;
    default:
      ROS_INFO_STREAM("Q: " << question << " is not a valid question.");
      answer = "Sorry, I don't understand that question.";
      break;
    }

    res.answer = answer;
    return true;
  }

  void ExplainableBTController::publishExplanations()
  {
    explain_bt::Explanations explanations_msg;

    explanations_msg.what_are_you_doing = xbt_->handleWhatAreYouDoing();
    explanations_msg.why_are_you_doing_this = xbt_->handleWhyAreYouDoing();
    explanations_msg.what_is_your_subgoal = xbt_->handleWhatIsYourSubgoal();
    explanations_msg.how_do_you_achieve_your_subgoal = xbt_->handleHowDoYouAchieveYourSubgoal();
    explanations_msg.what_is_your_goal = xbt_->handleWhatIsYourGoal();
    explanations_msg.how_do_you_achieve_your_goal = xbt_->handleHowDoYouAchieveYourGoal();
    explanations_msg.what_went_wrong = xbt_->handleWhatWentWrong();
    explanations_msg.what_is_next_action_if_success = xbt_->handleWhatIsNextActionIfSuccess();
    explanations_msg.what_is_next_action_if_fail = xbt_->handleWhatIsNextActionIfFail();
    explanations_msg.what_are_current_pre_conditions = xbt_->handleWhatAreCurrentPreConditions();
    explanations_msg.what_are_current_post_conditions = xbt_->handleWhatAreCurrentPostConditions();

    explanations_pub_.publish(explanations_msg);
  }

} // namespace XBT
