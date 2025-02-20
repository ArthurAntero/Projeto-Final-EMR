#include <behaviortree_cpp/action_node.h>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>

class DeliverDishesToKitchen : public BT::SyncActionNode
{
public:
    DeliverDishesToKitchen(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::vector<int>>("dishes")
        };
    }

    BT::NodeStatus tick() override
    {
        std::vector<int> dishes;
        if (!getInput("dishes", dishes))
        {
            std::cout << "[ERROR] DeliverDishesToKitchen: Failed to get dishes list" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        if (dishes.empty())
        {
            std::cout << "[WARNING] DeliverDishesToKitchen: No dishes to deliver" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        std::cout << "Delivering dishes to kitchen..." << std::endl;

        while (!dishes.empty())
        {
            int room_id = dishes.front();
            dishes.erase(dishes.begin());
            std::cout << "Delivering dish from room " << room_id << "... Done." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        std::cout << "[INFO] DeliverDishesToKitchen: All dishes delivered to kitchen" << std::endl;

        return BT::NodeStatus::SUCCESS;
    }
};
