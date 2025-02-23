#include <iostream>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>

class WaitForPickUp : public BT::SyncActionNode
{
public:
    WaitForPickUp(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("pickup_time_sec")
        };
    }

    BT::NodeStatus tick() override
    {
        int pickup_time_sec;
        if (!getInput("pickup_time_sec", pickup_time_sec))
        {
            std::cout << "[ERROR] WaitForPickup: Failed to get pickup_time_sec" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[INFO] WaitForPickup: Waiting for pickup for " << pickup_time_sec << " seconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(pickup_time_sec));

        std::cout << "[INFO] WaitForPickup: Pickup wait completed." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};