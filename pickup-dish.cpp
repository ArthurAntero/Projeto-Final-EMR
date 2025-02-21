#include <behaviortree_cpp_v3/action_node.h>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>

class PickUpDish : public BT::SyncActionNode
{
public:
    PickUpDish(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::unordered_map<int, int>>("meals"),
        };
    }

    BT::NodeStatus tick() override
    {
        std::unordered_map<int, int> meals;
        if (!getInput("meals", meals))
        {
            std::cerr << "[ERROR] PickUpDish: Failed to get meals list" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        if (meals.empty())
        {
            std::cerr << "[WARNING] PickUpDish: No meals available to pick up" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        auto it = meals.begin();
        int room_id = it->first;

        meals.erase(it);

        if (auto bb = config().blackboard)
        {
            bb->set("meals", meals);
        }

        std::cout << "[INFO] PickUpDish: Picking up dishes from room " << room_id << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};
