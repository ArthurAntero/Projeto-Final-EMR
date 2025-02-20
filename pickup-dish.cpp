#include <behaviortree_cpp/action_node.h>
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
            BT::InputPort<std::vector<int>>("dishes"),
        };
    }

    BT::NodeStatus tick() override
    {
        std::vector<int> dishes;
        if (!getInput("dishes", dishes))
        {
            std::cerr << "[ERROR] PickUpDish: Failed to get dishes list" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        if (dishes.empty())
        {
            std::cerr << "[WARNING] PickUpDish: No dishes available to pick up" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        int room_id = dishes.front();
        dishes.erase(dishes.begin());

        if (auto bb = config().blackboard)
            {
                bb->set("dishes", dishes);
            }

        std::cout << "[INFO] PickUpDish: Collected dirty dishes from room " << room_id << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};
