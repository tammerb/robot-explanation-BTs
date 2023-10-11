#include "explain_bt/utils.h"

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
            if (node->name().empty() || node->short_description() == p->short_description() || node->type() == BT::NodeType::DECORATOR)
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

    // Function to get the next node on success
    BT::TreeNode *get_next_node_on_success(BT::TreeNode *n)
    {
        auto p = n->getParent();
        if (!p)
        {
            return nullptr; // Reached the root, no next node
        }

        if (auto control = dynamic_cast<const BT::ControlNode *>(p))
        {
            auto children = control->children();
            size_t numChildren = children.size();
            size_t currentIndex = 0;

            // Find the index of the current node in its parent's children list
            while (currentIndex < numChildren)
            {
                if (children[currentIndex] == n)
                {
                    break; // Found the index of the current node in its parent's children list
                }
                currentIndex++;
            }

            if (dynamic_cast<const BT::SequenceNode *>(p))
            {
                if (currentIndex == numChildren - 1)
                {
                    // Propagate upwards since the current node is the last node in the sequence
                    return get_next_node_on_success(p);
                }
                else
                {
                    // Succeed: Return the next child of its parent.
                    return children[currentIndex + 1];
                }
            }
            else if (dynamic_cast<const BT::FallbackNode *>(p))
            {
                // Propagate upwards since succeed on fallback
                return get_next_node_on_success(p);
            }
            else
            {
                // Algorithm won't work with control nodes other than sequence and fallback, throw an error
                throw std::runtime_error("XBT::get_next_node_on_success, Unexpected control node type, only sequence and fallback nodes are supported");
            }
        }
        else if (auto decorator = dynamic_cast<const BT::DecoratorNode *>(p))
        {
            if (dynamic_cast<const BT::InverterNode *>(p))
            {
                // Propagate a failure upwards since succeed on inverter
                return get_next_node_on_fail(p);
            }
            else if (auto repeat_node = dynamic_cast<const BT::RepeatNode *>(p))
            {
                if (repeat_node->do_loop())
                {
                    // Repeat node is still looping on success, so return the current node
                    return n;
                }
                else
                {
                    // Repeat node is done looping on success, so propagate upwards
                    return get_next_node_on_success(p);
                }
            }
            else if (dynamic_cast<const BT::RetryNode *>(p))
            {
                // Propagate upwards since succeed on retry
                return get_next_node_on_success(p);
            }
            else if (dynamic_cast<const BT::SubTreeNode *>(p))
            {
                // Propagate upwards since succeed on subtree
                return get_next_node_on_success(p);
            }
            else
            {
                // Algorithm won't work with decorator nodes other than inverter, repeat, retry, and subtree, throw an error
                throw std::runtime_error("XBT::get_next_node_on_success, Unexpected decorator node type, only inverter, repeat, retry, and subtree nodes are supported");
            }
        }
        else
        {
            // This should never happen, throw an error
            throw std::runtime_error("Unexpected node type, parent should always be either a control or decorator node");
        }
    }

    // Function to get the next node on failure
    BT::TreeNode *get_next_node_on_fail(BT::TreeNode *n)
    {
        auto p = n->getParent();
        if (!p)
        {
            return nullptr; // Reached the root, no next node
        }

        if (auto control = dynamic_cast<const BT::ControlNode *>(p))
        {
            auto children = control->children();
            size_t numChildren = children.size();
            size_t currentIndex = 0;

            // Find the index of the current node in its parent's children list
            while (currentIndex < numChildren)
            {
                if (children[currentIndex] == n)
                {
                    break; // Found the index of the current node in its parent's children list
                }
                currentIndex++;
            }

            if (dynamic_cast<const BT::SequenceNode *>(p))
            {
                // Propagate upwards since fail on sequence
                return get_next_node_on_fail(p);
            }
            else if (dynamic_cast<const BT::FallbackNode *>(p))
            {
                if (currentIndex == numChildren - 1)
                {
                    // Propagate upwards since the current node is the last node in fallback
                    return get_next_node_on_fail(p);
                }
                else
                {
                    // Fail: Return the next child of its parent.
                    return children[currentIndex + 1];
                }
            }
            else
            {
                // Algorithm won't work with control nodes other than sequence and fallback, throw an error
                throw std::runtime_error("XBT::get_next_node_on_fail, Unexpected control node type, only sequence and fallback nodes are supported");
            }
        }
        else if (auto decorator = dynamic_cast<const BT::DecoratorNode *>(p))
        {
            if (dynamic_cast<const BT::InverterNode *>(p))
            {
                // Propagate a succeed upwards since failure on inverter
                return get_next_node_on_success(p);
            }
            else if (dynamic_cast<const BT::RepeatNode *>(p))
            {
                // Propagate upwards since failure on repeat
                return get_next_node_on_fail(p);
            }
            else if (auto retry_node = dynamic_cast<const BT::RetryNode *>(p))
            {
                if (retry_node->do_loop())
                {
                    // Retry node is still looping on failure, so return the current node
                    return n;
                }
                else
                {
                    // Retry node is done looping on failure, so propagate upwards
                    return get_next_node_on_fail(p);
                }
            }
            else if (dynamic_cast<const BT::SubTreeNode *>(p))
            {
                // Propagate upwards since failure on subtree
                return get_next_node_on_fail(p);
            }
            else
            {
                // Algorithm won't work with decorator nodes other than inverter, repeat, retry, and subtree, throw an error
                throw std::runtime_error("XBT::get_next_node_on_fail, Unexpected decorator node type, only inverter, repeat, retry, and subtree nodes are supported");
            }
        }
        else
        {
            // This should never happen, throw an error
            throw std::runtime_error("Unexpected node type, parent should always be either a control or decorator node");
        }
    }

} // namespace XBT