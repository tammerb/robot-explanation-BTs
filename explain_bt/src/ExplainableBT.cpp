#include "explain_bt/ExplainableBT.h"

namespace XBT
{

    ExplainableBT::ExplainableBT(BT::Tree &tree) : tree(tree), behavior_tracker(tree)
    {
        printTreeRecursively(tree.rootNode());
    }

    void ExplainableBT::halt()
    {
        tree.haltTree();
    }

    BT::NodeStatus ExplainableBT::status()
    {
        return tree.rootNode()->status();
    }

    BT::NodeStatus ExplainableBT::tick()
    {
        return tree.tickOnce();
    }

    std::string ExplainableBT::handleWhatAreYouDoing()
    {
        std::string answer;
        const BT::TreeNode *n = behavior_tracker.get_running_node();

        if (n == nullptr)
            return std::string("The tree isn't running.");

        answer = "I " + n->short_description() + ".";
        return answer;
    }

    std::string ExplainableBT::handleWhyAreYouDoing()
    {
        std::string answer;
        auto n = behavior_tracker.get_running_node();

        if (n == nullptr)
            return std::string("The tree isn't running.");

        auto p = get_diff_name_parent(n);

        if (p == nullptr)
        {
            // If the parent is null, then the current node is the root node
            answer = handleWhatIsYourGoal();
        }
        else
        {
            answer = "I " + n->short_description() + " in order to " + p->short_description() + ".";
        }

        return answer;
    }

    std::string ExplainableBT::handleWhatIsYourSubgoal()
    {
        std::string answer;
        auto n = behavior_tracker.get_running_node();

        if (n == nullptr)
            return std::string("The tree isn't running.");

        auto p = get_subtree_parent(n);

        if (p == nullptr)
            answer = "Sorry. I don't have a subgoal.";
        else
        {
            answer = "My subgoal is to " + p->short_description() + ".";
        }
        return answer;
    }

    std::string ExplainableBT::handleHowDoYouAchieveYourSubgoal()
    {
        std::string answer;
        auto n = behavior_tracker.get_running_node();

        if (n == nullptr)
            return std::string("The tree isn't running.");

        auto p = get_subtree_parent(n);

        if (p == nullptr)
            answer = "Sorry. I don't have a subgoal.";
        else
        {
            auto steps = get_seq_exec_nodes(p);
            answer = "To achieve the subgoal \"" +
                     p->short_description() +
                     "\", I need to do " +
                     std::to_string(steps.size()) +
                     " steps. ";

            for (int i = 0; i < steps.size(); ++i)
            {
                answer += std::to_string(i + 1) + ". " + steps.at(i)->short_description() + ". ";
            }
        }
        return answer;
    }

    std::string ExplainableBT::handleWhatIsYourGoal()
    {
        std::string answer;
        answer = "My goal is to " + tree.rootNode()->short_description() + ".";
        return answer;
    }

    std::string ExplainableBT::handleHowDoYouAchieveYourGoal()
    {
        std::string answer;
        auto n = behavior_tracker.get_running_node();

        if (n == nullptr)
            return std::string("The tree isn't running.");

        auto root = tree.rootNode();

        std::vector<BT::TreeNode *> steps = get_seq_exec_nodes(root);
        answer = "To achieve the goal \"" +
                 root->short_description() +
                 "\", I need to do " +
                 std::to_string(steps.size()) +
                 " steps. ";

        for (int i = 0; i < steps.size(); ++i)
        {
            answer += std::to_string(i + 1) + ". " + steps.at(i)->short_description() + ". ";
        }
        return answer;
    }

    std::string ExplainableBT::handleWhatWentWrong()
    {
        std::string answer;
        BT::TreeNode *running_node = behavior_tracker.get_running_node();

        if (running_node == nullptr)
            return std::string("The tree isn't running.");

        bool is_wrong = false;
        bool is_fell_back = false;
        BT::FallbackNode *fallback_node = nullptr;

        BT::TreeNode *p = running_node->getParent();
        while (p != nullptr)
        {
            bool is_fallback_node = (dynamic_cast<BT::FallbackNode *>(p) != nullptr);
            if (is_fallback_node)
            {
                //
                // Fallback node
                //
                fallback_node = dynamic_cast<BT::FallbackNode *>(p);
                if (fallback_node->child(0)->status() == BT::NodeStatus::FAILURE)
                {
                    is_wrong = true;
                    is_fell_back = true;

                    answer = "I could not " + fallback_node->short_description() + " because ";

                    // find the failed child
                    const BT::TreeNode *failed_child = nullptr;
                    BT::applyRecursiveVisitorSelectively(fallback_node, [&failed_child](const BT::TreeNode *node) -> bool
                                                         {
                            if (node->has_failed() && (
                                        node->type() == BT::NodeType::CONDITION || 
                                        node->type() == BT::NodeType::ACTION || 
                                        node->type() == BT::NodeType::DECORATOR // Account for invert
                                    )
                                ) {
                                failed_child = node;
                                return true;
                            }
                            return false; });

                    if (failed_child->getParent() != nullptr)
                    {
                        if (failed_child->getParent()->short_description() != fallback_node->short_description())
                        {
                            answer += "I was unable to " + failed_child->getParent()->short_description() + " as ";
                        }
                    }

                    answer += failed_child->short_description() + " failed.";

                    break;
                }
            }

            bool is_retry_node = (dynamic_cast<BT::RetryNode *>(p) != nullptr);
            if (is_retry_node)
            {
                auto retry_node = dynamic_cast<BT::RetryNode *>(p);

                if (retry_node->is_retrying())
                {
                    is_wrong = true;

                    // check if have non-null parent
                    BT::TreeNode *rp = retry_node->getParent();
                    while (rp == nullptr)
                    {
                        rp = rp->getParent();
                    }

                    if (rp != nullptr)
                    {
                        answer = "I am retrying for attempt " + std::to_string(retry_node->n_th_retry()) + " to " + rp->short_description() + ". ";

                        // find the failed child
                        const BT::TreeNode *failed_child;
                        BT::applyRecursiveVisitorSelectively(retry_node, [&failed_child](const BT::TreeNode *node) -> bool
                                                             {
                                if (node->has_failed() && (node->type() == BT::NodeType::CONDITION || node->type() == BT::NodeType::ACTION)) {
                                    failed_child = node;
                                    return true;
                                }
                                return false; });

                        auto fp = failed_child->getParent();
                        while (fp->name().empty())
                        {
                            fp = fp->getParent();
                        }

                        answer += "I could not " + fp->short_description() + " because " + failed_child->short_description() + " failed.";

                        break;
                    }
                }
            }
            p = p->getParent();
        }

        if (is_fell_back)
        {
            // find if there is a parent Retry node retrying. If so, say that

            p = fallback_node->getParent();
            while (p != nullptr && p->type() != BT::NodeType::SUBTREE)
            {

                bool is_retry_node = (dynamic_cast<BT::RetryNode *>(p) != nullptr);
                if (is_retry_node)
                {
                    auto retry_node = dynamic_cast<BT::RetryNode *>(p);

                    if (retry_node->is_retrying())
                    {

                        // check if have non-null parent
                        BT::TreeNode *rp = retry_node->getParent();
                        while (rp == nullptr)
                        {
                            rp = rp->getParent();
                        }

                        if (rp != nullptr)
                        {
                            answer += " I am retrying for attempt " + std::to_string(retry_node->n_th_retry()) + " to " + rp->short_description() + ".";
                        }
                    }
                }
                p = p->getParent();
            }
        }

        if (!is_wrong)
            answer = "Nothing went wrong.";

        return answer;
    }

    std::string ExplainableBT::handleWhatIsNextActionIfSuccess()
    {
        std::string answer;
        auto n = behavior_tracker.get_running_node();

        if (n == nullptr)
            return std::string("The tree isn't running.");

        auto next_n = get_next_node_on_success(n);
        if (next_n == nullptr)
        {
            answer = "If " +
                     n->short_description() +
                     " succeeds, I will finish.";
        }
        else
        {
            answer = "If " +
                     n->short_description() +
                     " succeeds, my next action is " +
                     next_n->short_description() + ".";
        }
        return answer;
    }

    std::string ExplainableBT::handleWhatIsNextActionIfFail()
    {
        std::string answer;
        auto n = behavior_tracker.get_running_node();

        if (n == nullptr)
            return std::string("The tree isn't running.");

        auto next_n = get_next_node_on_fail(n);
        if (next_n == nullptr)
        {
            answer = "If " +
                     n->short_description() +
                     " fails, I will finish.";
        }
        else
        {
            answer = "If " +
                     n->short_description() +
                     " fails, my next action is " +
                     next_n->short_description() + ".";
        }
        return answer;
    }

    std::string ExplainableBT::handleWhatAreCurrentPreConditions()
    {
        std::string answer;
        const BT::TreeNode *n = behavior_tracker.get_running_node();

        if (n == nullptr)
            return std::string("The tree isn't running.");

        auto preconditions = n->config().pre_conditions;

        answer = "For \"" +
                 n->short_description() +
                 "\", I have " +
                 std::to_string(preconditions.size()) +
                 " preconditions. ";

        for (const auto &[cond_id, script] : preconditions)
        {
            switch (cond_id)
            {
            case BT::PreCond::SUCCESS_IF:
                answer += "Skip and succeed if " + script + " is true. ";
                break;
            case BT::PreCond::FAILURE_IF:
                answer += "Skip and fail if " + script + " is true. ";
                break;
            case BT::PreCond::SKIP_IF:
                answer += "Skip if " + script + " is true. ";
                break;
            case BT::PreCond::WHILE_TRUE:
                answer += "Skip while " + script + " is true. ";
                break;
            }
        }

        return answer;
    }

    std::string ExplainableBT::handleWhatAreCurrentPostConditions()
    {
        std::string answer;
        const BT::TreeNode *n = behavior_tracker.get_running_node();

        if (n == nullptr)
            return std::string("The tree isn't running.");

        auto postconditions = n->config().post_conditions;

        answer = "For \"" +
                 n->short_description() +
                 "\", I have " +
                 std::to_string(postconditions.size()) +
                 " postconditions. ";

        for (const auto &[cond_id, script] : postconditions)
        {
            switch (cond_id)
            {
            case BT::PostCond::ON_SUCCESS:
                answer += "If successful, " + script + ". ";
                break;
            case BT::PostCond::ON_FAILURE:
                answer += "If failed, " + script + ". ";
                break;
            case BT::PostCond::ALWAYS:
                answer += "Always " + script + ". ";
                break;
            case BT::PostCond::ON_HALTED:
                answer += "If halted, " + script + ". ";
                break;
            }
        }

        return answer;
    }

} // namespace XBT
