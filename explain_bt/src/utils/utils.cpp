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

    BT::TreeNode *get_next_node_on_success(BT::TreeNode *n)
    {
        auto p = n->getParent();
        if (!p)
        {
            return nullptr; // reached the root, no next node
        }

        if (auto control = dynamic_cast<const BT::ControlNode *>(p))
        {
            auto children = control->children();
            size_t numChildren = children.size();
            size_t currentIndex = 0;
            while (currentIndex < numChildren)
            {
                if (children[currentIndex] == n)
                {
                    break; // Found the index of the current node in its parent's children list
                }
                currentIndex++;
            }

            if (dynamic_cast<const BT::SequenceNode *>(p)) {
                if (currentIndex == numChildren - 1)
                {
                    // propagate upwards since current node is last node in sequence
                    return get_next_node_on_success(p);
                }
                else
                {
                    // Succeed: Move to the next child of its parent.
                    return children[currentIndex + 1];
                }
            }
            else if (dynamic_cast<const BT::FallbackNode *>(p)) {
                // propagate upwards since succeed on fallback
                return get_next_node_on_success(p);
            }
            else {
                // Algorithm won't work with control nodes other than sequence and fallback, throw error 
                throw std::runtime_error("XBT::get_next_node_on_success, Unexpected control node type, only sequence and fallback nodes are supported");
            }
            
        }
        else if (auto decorator = dynamic_cast<const BT::DecoratorNode *>(p))
        {
            // Propagate upwards since the decorator only has one child, which is the current node
            return get_next_node_on_success(p);
        }
        else 
        {
            // This should never happen, throw error
            throw std::runtime_error("Unexpected node type, parent should always be either a control or decorator node");
        }
    }

    BT::TreeNode* get_next_node_on_fail(BT::TreeNode* n) {
        auto p = n->getParent();
        if (!p)
        {
            return nullptr; // reached the root, no next node
        }

        if (auto control = dynamic_cast<const BT::ControlNode *>(p))
        {
            auto children = control->children();
            size_t numChildren = children.size();
            size_t currentIndex = 0;
            while (currentIndex < numChildren)
            {
                if (children[currentIndex] == n)
                {
                    break; // Found the index of the current node in its parent's children list
                }
                currentIndex++;
            }

            if (dynamic_cast<const BT::SequenceNode *>(p)) {
                // propagate upwards since fail on sequence
                return get_next_node_on_fail(p);
            }
            else if (dynamic_cast<const BT::FallbackNode *>(p)) {
                if (currentIndex == numChildren - 1)
                {
                    // propagate upwards since current node is last node in fallback
                    return get_next_node_on_fail(p);
                }
                else
                {
                    // fail on fallback, next node is right sibling
                    return children[currentIndex + 1];
                }
            }
            else {
                // Algorithm won't work with control nodes other than sequence and fallback, throw error 
                throw std::runtime_error("XBT::get_next_node_on_success, Unexpected control node type, only sequence and fallback nodes are supported");
            }
        }
        else if (auto decorator = dynamic_cast<const BT::DecoratorNode *>(p))
        {
            // Propagate upwards since the decorator only has one child, which is the current node
            return get_next_node_on_fail(p);
        }
        else 
        {
            // This should never happen, throw error
            throw std::runtime_error("Unexpected node type, parent should always be either a control or decorator node");
        }
    }

} // namespace XBT