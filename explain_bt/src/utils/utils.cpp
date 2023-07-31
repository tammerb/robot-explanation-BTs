#include "explain_bt/utils/utils.h"

namespace XBT
{

    BT::TreeNode *get_diff_name_parent(BT::TreeNode *n)
    {
        auto p = n->getParent();

        while (
            p != nullptr &&
            p->short_description() == n->short_description())
        {
            p = p->getParent(); // Traverse up the tree until a parent with a different name is found
        }

        // std cout debug statement
        std::cout << p << std::endl;

        return p;
    }

    BT::TreeNode *get_subtree_parent(BT::TreeNode *n)
    {
        auto p = n->getParent();
        while (
            p != nullptr &&
            p->type() != BT::NodeType::SUBTREE)
        {
            p = p->getParent(); // Traverse up until your parent is a subtree
        }

        return p;
    }

    std::vector<BT::TreeNode *> get_seq_exec_nodes(BT::TreeNode *p)
    {
        std::vector<BT::TreeNode *> steps;

        auto visitor = [&p, &steps](BT::TreeNode *node) -> bool
        {
            if (node->name().empty() || node->name() == p->name() || node->type() == BT::NodeType::DECORATOR)
            {
                return false;
            }
            else
            {
                steps.emplace_back(node);
                return true;
            }
        };

        applyRecursiveVisitorSelectively(p, visitor);

        return steps;
    }

} // namespace XBT