#include "explain_bt/BehaviorTracker.h"

namespace XBT
{
    BehaviorTracker::BehaviorTracker(const BT::Tree &tree) : tree(tree), StdCoutLogger(tree)
    {
    }

    void BehaviorTracker::callback(BT::Duration timestamp, const BT::TreeNode &node, BT::NodeStatus prev_status, BT::NodeStatus status)
    {
        ticking_node_uid_ = node.UID(); // Store the UID of the currently running node
        StdCoutLogger::callback(timestamp, node, prev_status, status);
    }

    BT::TreeNode *BehaviorTracker::get_running_node()
    {
        BT::TreeNode *running_node = nullptr;
        BT::applyRecursiveVisitor(tree.rootNode(), [this, &running_node](BT::TreeNode *node_visiting)
                                  {
            if (running_node != nullptr) { // If the running_node is already found, return immediately
                return;
            }

            if (node_visiting->UID() == ticking_node_uid_) { // Check if the UID matches the currently running node
                running_node = node_visiting; // Store the pointer to the running node
            } });


        return XBT::get_node_to_explain(running_node); // Return the node to explain
    }
} // namespace XBT
