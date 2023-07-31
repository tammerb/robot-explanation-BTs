#pragma once

#include <behaviortree_cpp/bt_factory.h>

namespace XBT {

    /**
     * @brief Get the currently running node that has a different control parent
     *        (a parent node with a different name) in the behavior tree.
     * @return Pointer to the currently running node with a different control parent.
     */
    BT::TreeNode* get_diff_name_parent(BT::TreeNode *n);

    /**
     * @brief Get the parent subtree node of the currently running node in the behavior tree.
     * @return Pointer to the parent subtree node.
     */
    BT::TreeNode* get_subtree_parent(BT::TreeNode *n);

    /**
     * @brief Find the steps to achieve a given parent_node's subgoal.
     * 
     * This method finds all the nodes that need to be executed in sequence
     * to achieve the given parent_node's subgoal. It helps in generating
     * explanations related to subgoals.
     * 
     * @param parent_node The node whose subgoal steps need to be found.
     * @return std::vector<BT::TreeNode*> A vector of nodes representing the steps.
     */
    std::vector<BT::TreeNode*> get_seq_exec_nodes(BT::TreeNode* p);

    BT::TreeNode* get_next_node_on_success(BT::TreeNode* n);

    BT::TreeNode* get_next_node_on_fail(BT::TreeNode* n);
    
} // namespace XBT