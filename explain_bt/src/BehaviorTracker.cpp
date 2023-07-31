#include "explain_bt/BehaviorTracker.h"

namespace XBT {

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
    return running_node;
}

BT::TreeNode *BehaviorTracker::get_running_node_different_control_parent()
{
    BT::TreeNode *running_node = this->get_running_node(); // Get the currently running node

    BT::TreeNode *p = running_node->getParent();
    // std print debug statement
    std::cout << p << std::endl;
    std::cout << running_node->name() << std::endl;
    while (p->name().empty() || p->short_description() == running_node->short_description())
    {
        std::cout << p->getParent() << std::endl;

        p = p->getParent(); // Traverse up the tree until a parent with a different name is found
    }

    return p;
}

BT::TreeNode *BehaviorTracker::get_overall_goal_node()
{
    return tree.rootNode(); // Return the root node of the behavior tree
}

BT::TreeNode *BehaviorTracker::get_tree_parent()
{
    BT::TreeNode *running_node = this->get_running_node(); // Get the currently running node

    BT::TreeNode *p = running_node->getParent();

    while (p != nullptr && p->type() != BT::NodeType::SUBTREE)
    {
        p = p->getParent(); // Traverse up the tree until a parent of type SUBTREE is found
    }

    return p;
}

} // namespace XBT
