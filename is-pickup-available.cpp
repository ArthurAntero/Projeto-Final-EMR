#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>

class IsPickUpAvailable : public BT::ConditionNode
{
public:
    IsPickUpAvailable(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::unordered_map<int, bool>>("available"),
            BT::InputPort<std::unordered_map<int, int>>("meals")
        };
    }

    BT::NodeStatus tick() override
    {
        std::unordered_map<int, bool> available;
        std::unordered_map<int, int> meals;

        if (!getInput("available", available))
        {
            std::cout << "[ERROR] IsPickUpAvailable: Failed to get pickup availability map from blackboard" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        if (!getInput("meals", meals))
        {
            std::cout << "[ERROR] IsPickUpAvailable: Failed to get meals map from blackboard" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        if (meals.empty())
        {
            std::cout << "[WARNING] IsPickUpAvailable: No meals available" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        int room_id = meals.begin()->second; // Get the room ID of the first meal

        auto it = available.find(room_id);
        if (it == available.end())
        {
            std::cout << "[WARNING] IsPickUpAvailable: Room " << room_id << " not found in availability map" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        if (it->second)
        {
            std::cout << "[INFO] IsPickUpAvailable: Pickup available for room " << room_id << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        std::cout << "[WARNING] IsPickUpAvailable: Pickup not available for room " << room_id << std::endl;
        return BT::NodeStatus::FAILURE;
    }
};