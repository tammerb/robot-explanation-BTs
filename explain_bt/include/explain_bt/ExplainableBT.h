#pragma once

#include <ros/ros.h>
#include <regex>
#include <stdexcept>
#include <behaviortree_cpp/bt_factory.h>
#include "explain_bt/Explain.h"
#include <boost/algorithm/string/predicate.hpp> // starts_with
#include "explain_bt/BehaviorTracker.h"
#include "explain_bt/utils/utils.h"

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
     * 
     * @param tree The Behavior Tree (BT) to be analyzed and explained.
     */
    explicit ExplainableBT(BT::Tree & tree);

    /**
     * @brief Execute one tick of the Behavior Tree (BT).
     * 
     * This method executes one tick of the BT and returns the status
     * of the root node after the execution.
     * 
     * @return BT::NodeStatus The status of the root node after the tick.
     */
    BT::NodeStatus tick();

    /**
     * @brief Get the status of the Behavior Tree (BT).
     * 
     * This method returns the status of the root node of the BT.
     * 
     * @return BT::NodeStatus The status of the root node.
    */
    BT::NodeStatus status();

    /**
     * @brief Halt the Behavior Tree (BT).
     * 
     * This method halts the execution of the BT.
     * 
     * @return void
     */
    void halt();

    /**
     * @brief Callback function to generate explanations.
     * 
     * This method is called as a callback to handle explain queries
     * from external entities. It generates explanations based on the
     * input query and returns the response with the explanation.
     * 
     * @param req The input query for the explanation.
     * @param res The response containing the generated explanation.
     * @return bool True if the explanation is generated successfully, false otherwise.
     */
    bool explain_callback(explain_bt::Explain::Request &req, explain_bt::Explain::Response &res);

private:

    /**
     * @brief Handle the explanation for "What are you doing?" query.
     */
    std::string handleWhatAreYouDoing();

    /**
     * @brief Handle the explanation for "Why are you doing this?" query.
     */
    std::string handleWhyAreYouDoing();

    /**
     * @brief Handle the explanation for "What is your subgoal?" query.
     */
    std::string handleWhatIsYourSubgoal();

    /**
     * @brief Handle the explanation for "How do you achieve your subgoal?" query.
     */
    std::string handleHowDoYouAchieveYourSubgoal();

    /**
     * @brief Handle the explanation for "What is your goal?" query.
     */
    std::string handleWhatIsYourGoal();

    /**
     * @brief Handle the explanation for "How do you achieve your goal?" query.
     */
    std::string handleHowDoYouAchieveYourGoal();

    /**
     * @brief Handle the explanation for "What went wrong?" query.
     */
    std::string handleWhatWentWrong();

    std::string handleWhatIsNextActionIfSuccess();

    std::string handleWhatIsNextActionIfFail();

    std::string handleWhatAreCurrentPreConditions();

    std::string handleWhatAreCurrentPostConditions();

    BT::Tree &tree;
    BehaviorTracker behavior_tracker;
};

} // namespace XBT
