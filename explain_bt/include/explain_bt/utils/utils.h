#pragma once

#include <behaviortree_cpp/bt_factory.h>

namespace XBT {

    BT::TreeNode* get_diff_name_parent(BT::TreeNode *n);

    BT::TreeNode* get_subtree_parent(BT::TreeNode *n);

    std::vector<BT::TreeNode*> get_seq_exec_nodes(BT::TreeNode* p);

    BT::TreeNode* get_next_node_on_success(BT::TreeNode* n);

    BT::TreeNode* get_next_node_on_fail(BT::TreeNode* n);
    
} // namespace XBT