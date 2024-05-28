#pragma once

#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <explain_bt/utils.h>

namespace XBT {

/**
 * @brief The BehaviorTracker class tracks the behavior tree execution and provides
 *        methods to retrieve various information about the tree's execution state.
 */
class BehaviorTracker : public BT::StdCoutLogger {
public:
    /**
     * @brief Constructor for BehaviorTracker.
     * @param tree The behavior tree to be tracked.
     */
    BehaviorTracker(const BT::Tree& tree);

    /**
     * @brief Callback function called whenever a node status changes in the behavior tree.
     * @param timestamp The time at which the callback is called.
     * @param node The node whose status changed.
     * @param prev_status The previous status of the node.
     * @param status The current status of the node.
     */
    void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status, BT::NodeStatus status) override;

    /**
     * @brief Get the currently running node in the behavior tree.
     * @return Pointer to the currently running node.
     */
    BT::TreeNode* get_running_node();

private:
    const BT::Tree& tree; ///< Reference to the behavior tree being tracked
    uint16_t ticking_node_uid_; ///< UID of the currently running node in the behavior tree
};

} // namespace XBT
