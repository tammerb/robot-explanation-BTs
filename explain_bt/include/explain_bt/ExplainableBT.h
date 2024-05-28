#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp/bt_factory.h>
#include "explain_bt/BehaviorTracker.h"
#include "explain_bt/utils.h"

namespace XBT {

/**
 * @brief Class to provide explainability for a Behavior Tree (BT).
 * 
 * The ExplainableBT class allows providing explanations for the execution
 * of a Behavior Tree (BT). It interprets the current state of the BT and
 * generates explanations for various queries related to its execution.
 * The explanations are based on the state and behavior of the BT nodes.
 */
class ExplainableBT {
public:
    /**
     * @brief Constructor to create an ExplainableBT object.
     * @param tree The Behavior Tree (BT) to be analyzed and explained.
     */
    explicit ExplainableBT(BT::Tree & tree);

    /**
     * @brief Execute one tick of the Behavior Tree (BT). 
     * @return BT::NodeStatus The status of the root node after the tick.
     */
    BT::NodeStatus tick();

    /**
     * @brief Get the status of the Behavior Tree (BT).
     * @return BT::NodeStatus The status of the root node.
    */
    BT::NodeStatus status();

    /**
     * @brief Halt the Behavior Tree (BT).
     * @return void
     */
    void halt();

    /**
     * @brief Handle the explanation for "What are you doing?" query.
     * @return std::string 
     */
    std::string handleWhatAreYouDoing();

    /**
     * @brief Handle the explanation for "Why are you doing?" query.
     * @return std::string 
     */
    std::string handleWhyAreYouDoing();

    /**
     * @brief Handle the explanation for "What is your subgoal?" query.
     * @return std::string 
     */
    std::string handleWhatIsYourSubgoal();

    /**
     * @brief Handle the explanation for "How do you achieve your subgoal?" query.
     * @return std::string 
     */
    std::string handleHowDoYouAchieveYourSubgoal();

    /**
     * @brief Handle the explanation for "What is your goal?" query.
     * @return std::string 
     */
    std::string handleWhatIsYourGoal();

    /**
     * @brief Handle the explanation for "How do you achieve your goal?" query.
     * @return std::string 
     */
    std::string handleHowDoYouAchieveYourGoal();

    /**
     * @brief Handle the explanation for "What went wrong?" query.
     * @return std::string 
     */
    std::string handleWhatWentWrong();

    /**
     * @brief Handle the explanation for "What is next action if success?" query.
     * @return std::string 
     */
    std::string handleWhatIsNextActionIfSuccess();

    /**
     * @brief Handle the explanation for "What is next action if fail?" query.
     * @return std::string 
     */
    std::string handleWhatIsNextActionIfFail();

    /**
     * @brief Handle the explanation for "What are current pre-conditions?" query.
     * @return std::string 
     */
    std::string handleWhatAreCurrentPreConditions();

    /**
     * @brief Handle the explanation for "What are current post-conditions?" query.
     * @return std::string 
     */
    std::string handleWhatAreCurrentPostConditions();

private:
    // The Behavior Tree (BT) to be analyzed and explained.
    BT::Tree &tree;
    // The Behavior Tracker to track the state and behavior of the BT nodes.
    BehaviorTracker behavior_tracker;
};

} // namespace XBT
