#include <behaviortree_cpp/action_node.h>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>

class IsBatteryAbove15 : public BT::ConditionNode
{
public:
    IsBatteryAbove15(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<float>("battery_level")};
    }

    BT::NodeStatus tick() override
    {
        float battery_level;
        if (!getInput("battery_level", battery_level))
        {
            std::cerr << "[ERROR] IsBatteryAbove15: Failed to get battery level from blackboard" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        if (battery_level > 15.0)
        {
            std::cout << "[INFO] IsBatteryAbove15: Battery level is above 15%" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        std::cout << "[WARNING] IsBatteryAbove15: Battery level is below or equal to 15%" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
};
