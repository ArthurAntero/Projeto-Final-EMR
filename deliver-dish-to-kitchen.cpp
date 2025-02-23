#include "behaviortree_cpp/bt_factory.h"
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
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::cout << "[INFO] DeliverDishesToKitchen: All dishes delivered to the kitchen." << std::endl;

        return BT::NodeStatus::SUCCESS;
    }
};