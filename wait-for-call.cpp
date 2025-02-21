#include <behaviortree_cpp/action_node.h>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <limits>

class WaitForCall : public BT::SyncActionNode
{
public:
    WaitForCall(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std::unordered_map<int, int>>("meals"),
            BT::OutputPort<int>("dishes_quantity")
        };
    }

    BT::NodeStatus tick() override
    {
        int dishes_quantity;
        std::unordered_map<int, int> meals;

        std::cout << "Enter the number of rooms that need dishes collected: ";
        std::cin >> dishes_quantity;

        std::cout << "Enter the room IDs (press Enter after each):" << std::endl;
        for (int i = 0; i < dishes_quantity; ++i)
        {
            int room_id;
            std::cout << "Room " << (i + 1) << ": ";
            std::cin >> room_id;
            meals[i] = room_id; 
            std::cout << "[INFO] Room " << room_id << " added to dishes list." << std::endl;
        }

        setOutput("meals", meals);
        setOutput("dishes_quantity", dishes_quantity);

        std::cout << "[INFO] WaitForCall: Finished collecting calls: " << dishes_quantity << " rooms collected" << std::endl;

        return BT::NodeStatus::SUCCESS;
    }
};